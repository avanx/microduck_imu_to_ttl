"""飞特 SCS 协议 IMU 调试界面。

本目录自成一体，不读取固件仓库里的其它文件，整夹拷走即可。

连接舵机总线（默认 1000000 8N1、ID 200），读取地址 124 起的 20 字节，
并按时间画出姿态、陀螺仪、加速度和四元数。

调试串口是另一路：115200，只打印 imu ok，不走 SCS。

    pip install -r requirements.txt
    python scs_imu_gui.py
"""

from __future__ import annotations

import math
import queue
import struct
import sys
import threading
import time
import tkinter as tk
from collections import deque
from dataclasses import dataclass
from tkinter import messagebox, scrolledtext, ttk

try:
    import serial
    from serial.tools import list_ports
except ImportError:  # 缺依赖时仍允许 --check 跑协议自检
    serial = None
    list_ports = None

INST_PING = 0x01
INST_READ = 0x02
INST_WRITE = 0x03

ADDR_MODEL = 3
ADDR_ID = 5
ADDR_BAUD = 6
ADDR_IMU = 124
IMU_LEN = 20
TABLE_MODEL = 0x4955

GYRO_DPS_PER_LSB = 0.0175
ACCEL_G_PER_LSB = 0.000122

BAUD_TABLE = (
    (1_000_000, "1 Mbps"),
    (500_000, "500 kbps"),
    (250_000, "250 kbps"),
    (128_000, "128 kbps"),
    (115_200, "115200"),
    (76_800, "76800"),
    (57_600, "57600"),
    (38_400, "38400"),
    (19_200, "19200"),
    (14_400, "14400"),
    (9_600, "9600"),
    (4_800, "4800"),
)

NUM_FONT = ("Consolas", 10)
TICK_FONT = ("Consolas", 8)

AXIS_COLOR = {"x": "#d62728", "y": "#2ca02c", "z": "#1f77b4", "w": "#9467bd"}


def checksum(payload: bytes) -> int:
    return (~sum(payload)) & 0xFF


def build(dev_id: int, inst: int, params: bytes = b"") -> bytes:
    body = bytes((dev_id & 0xFF, len(params) + 2, inst)) + params
    return b"\xff\xff" + body + bytes((checksum(body),))


class Frame:
    __slots__ = ("dev_id", "error", "data")

    def __init__(self, dev_id: int, error: int, data: bytes):
        self.dev_id = dev_id
        self.error = error
        self.data = data


def parse_frames(buf: bytes) -> tuple[list[Frame], bytes]:
    """从字节流里取出完整 SCS 帧。字头后的多余 0xFF 继续当作字头。"""
    frames: list[Frame] = []
    i = 0
    n = len(buf)
    while i + 4 <= n:
        if buf[i] != 0xFF or buf[i + 1] != 0xFF:
            i += 1
            continue
        j = i + 2
        while j < n and buf[j] == 0xFF:
            j += 1
        if j + 1 >= n:
            break
        dev_id = buf[j]
        length = buf[j + 1]
        if length < 2 or length > 160:
            i += 1
            continue
        end = j + 2 + length
        if end > n:
            break
        body = buf[j + 2 : end]
        expect = checksum(bytes((dev_id, length)) + body[:-1])
        if expect == body[-1]:
            frames.append(Frame(dev_id, body[0], bytes(body[1:-1])))
            i = end
        else:
            i += 1
    return frames, buf[i:]


def half_to_float(bits: int) -> float:
    """IEEE 754 binary16。与 duck-control imu.rs 的 half() 一致。"""
    sign = -1.0 if bits & 0x8000 else 1.0
    exp = (bits >> 10) & 0x1F
    frac = bits & 0x3FF
    if exp == 0:
        return sign * frac * 2.0**-24
    if exp == 0x1F:
        return math.copysign(math.inf, sign) if frac == 0 else math.nan
    return sign * (1.0 + frac / 1024.0) * 2.0 ** (exp - 15)


def quat_w(x: float, y: float, z: float) -> float:
    if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
        return math.nan
    norm_sq = x * x + y * y + z * z
    if norm_sq > 1.02:
        return math.nan
    return math.sqrt(max(0.0, 1.0 - norm_sq))


def euler_deg(w: float, x: float, y: float, z: float) -> tuple[float, float, float]:
    if not all(math.isfinite(v) for v in (w, x, y, z)):
        return (math.nan, math.nan, math.nan)
    roll = math.degrees(math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y)))
    sinp = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.degrees(math.asin(sinp))
    yaw = math.degrees(math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))
    return roll, pitch, yaw


def baud_label(enum: int) -> str:
    if 0 <= enum < len(BAUD_TABLE):
        hz, name = BAUD_TABLE[enum]
        return f"{name}（枚举 {enum}）"
    return f"未知枚举 {enum}"


def looks_like_text(buf: bytes) -> str:
    if len(buf) < 4:
        return ""
    ok = sum(32 <= b < 127 or b in (9, 10, 13) for b in buf)
    if ok < len(buf) * 0.75:
        return ""
    return buf.decode("ascii", "replace").strip()


@dataclass
class Sample:
    t: float
    gyro_raw: tuple[int, int, int]
    accel_raw: tuple[int, int, int]
    quat: tuple[float, float, float]
    quat_bits: tuple[int, int, int]
    quat_w: float
    quat_ready: bool
    counter: int
    delta: int
    status: int
    raw: bytes

    @property
    def gyro_dps(self) -> tuple[float, float, float]:
        return tuple(v * GYRO_DPS_PER_LSB for v in self.gyro_raw)

    @property
    def gyro_rad(self) -> tuple[float, float, float]:
        scale = GYRO_DPS_PER_LSB * math.pi / 180.0
        return tuple(v * scale for v in self.gyro_raw)

    @property
    def accel_g(self) -> tuple[float, float, float]:
        return tuple(v * ACCEL_G_PER_LSB for v in self.accel_raw)

    @property
    def euler(self) -> tuple[float, float, float]:
        return euler_deg(self.quat_w, *self.quat)


def decode_block(raw: bytes, prev_counter: int | None) -> Sample:
    if len(raw) != IMU_LEN:
        raise ValueError(f"IMU 块长度 {len(raw)}，期望 {IMU_LEN}")
    gx, gy, gz = struct.unpack_from("<hhh", raw, 0)
    qx_b, qy_b, qz_b = struct.unpack_from("<HHH", raw, 6)
    ax, ay, az = struct.unpack_from("<hhh", raw, 12)
    counter = raw[18]
    status = raw[19]
    qx, qy, qz = half_to_float(qx_b), half_to_float(qy_b), half_to_float(qz_b)
    ready = bool(status & 0x02)
    if qx_b == qy_b == qz_b == 0 and not ready:
        w = math.nan
    else:
        w = quat_w(qx, qy, qz)
    if prev_counter is None:
        delta = 0
    else:
        delta = (counter - prev_counter) & 0xFF
    return Sample(
        t=time.monotonic(),
        gyro_raw=(gx, gy, gz),
        accel_raw=(ax, ay, az),
        quat=(qx, qy, qz),
        quat_bits=(qx_b, qy_b, qz_b),
        quat_w=w,
        quat_ready=ready,
        counter=counter,
        delta=delta,
        status=status,
        raw=bytes(raw),
    )


def status_text(status: int) -> str:
    parts = [
        "在线" if status & 0x01 else "离线",
        "有四元数" if status & 0x02 else "无四元数",
        "有采样" if status & 0x04 else "无采样",
    ]
    return "  ".join(parts) + f"   {status:#04x}"


def hex_of(data: bytes) -> str:
    return " ".join(f"{b:02X}" for b in data)


class BusWorker:
    def __init__(self, commands: queue.Queue, messages: queue.Queue, samples: deque, lock: threading.Lock):
        self.commands = commands
        self.messages = messages
        self.samples = samples
        self.lock = lock
        self.alive = True
        self.ser = None
        self.dev_id = 200
        self.period = 0.02
        self.polling = False
        self.log_frames = False
        self.imu_ready = False
        self._hold: list[tuple] = []
        self.prev_counter: int | None = None
        self.fail_streak = 0

    def run(self) -> None:
        next_poll = 0.0
        while self.alive:
            timeout = 0.02
            if self.polling and self.ser is not None:
                timeout = max(0.0, min(0.02, next_poll - time.monotonic()))
            try:
                cmd = self.commands.get(timeout=timeout)
            except queue.Empty:
                cmd = None
            if cmd:
                self._handle(cmd)
            if self.polling and self.ser is not None and time.monotonic() >= next_poll:
                self._read_imu(log=self.log_frames)
                next_poll = time.monotonic() + self.period

    def _emit(self, kind: str, *payload) -> None:
        self.messages.put((kind, *payload))

    def _handle(self, cmd: tuple) -> None:
        kind = cmd[0]
        if kind == "quit":
            self.alive = False
            self._close()
        elif kind == "open":
            self._open(cmd[1], cmd[2])
        elif kind == "close":
            self.polling = False
            self._close()
            self._emit("connected", False)
        elif kind == "set_id":
            new_id = int(cmd[1])
            if new_id != self.dev_id:
                self.dev_id = new_id
                self.imu_ready = False
                if self.polling:
                    self.polling = False
                    self._emit("polling", False)
        elif kind == "set_period":
            self.period = max(0.01, float(cmd[1]))
        elif kind == "poll":
            want = bool(cmd[1]) and self.ser is not None
            if want and not self.imu_ready:
                self._attach(self.dev_id, start_poll=True)
                return
            self.polling = want and self.imu_ready
            self._emit("polling", self.polling)
        elif kind == "log":
            self.log_frames = bool(cmd[1])
        elif kind == "discover":
            self._scan()
        elif kind == "once":
            if self.ser is None:
                return
            if not self.imu_ready and not self._attach(self.dev_id, start_poll=False):
                return
            self._read_imu(log=True)
        elif kind == "write_id":
            self._write_id(int(cmd[1]))
        elif kind == "write_baud":
            self._write_baud(int(cmd[1]))

    def _open(self, port: str, baud: int) -> None:
        self._close()
        if serial is None:
            self._emit("status", "未安装 pyserial")
            return
        try:
            ser = serial.Serial()
            ser.port = port
            ser.baudrate = baud
            ser.bytesize = serial.EIGHTBITS
            ser.parity = serial.PARITY_NONE
            ser.stopbits = serial.STOPBITS_ONE
            ser.timeout = 0
            ser.write_timeout = 0.3
            ser.dtr = False
            ser.rts = False
            ser.open()
        except Exception as exc:
            self._emit("status", f"打开失败：{exc}")
            self._emit("connected", False)
            return
        time.sleep(0.05)
        ser.reset_input_buffer()
        self.ser = ser
        self.fail_streak = 0
        self.prev_counter = None
        self._emit("connected", True)
        self._emit("log", f"已打开 {port}  {baud} 8N1")
        self._attach(self.dev_id, start_poll=True)

    def _close(self) -> None:
        ser = self.ser
        self.ser = None
        self.polling = False
        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass

    def _exchange(self, packet: bytes, accept, timeout: float, log: bool):
        ser = self.ser
        if ser is None:
            return None, b""
        try:
            ser.reset_input_buffer()
            ser.write(packet)
            ser.flush()
        except Exception as exc:
            self._emit("status", f"发送失败：{exc}")
            self._close()
            self._emit("connected", False)
            return None, b""
        if log:
            self._emit("log", "TX  " + hex_of(packet))
        buf = b""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                waiting = ser.in_waiting
                chunk = ser.read(waiting or 1)
            except Exception as exc:
                self._emit("status", f"接收失败：{exc}")
                self._close()
                self._emit("connected", False)
                return None, buf
            if not chunk:
                time.sleep(0.001)
                continue
            buf += chunk
            frames, buf = parse_frames(buf)
            for frame in frames:
                if log:
                    self._emit("log", f"RX  id={frame.dev_id} err={frame.error} {hex_of(frame.data)}")
                if accept(frame):
                    return frame, buf
        return None, buf

    def _note_silence(self, leftover: bytes) -> None:
        text = looks_like_text(leftover)
        if text:
            self._emit("log", "收到文本：" + text.replace("\r", " ").replace("\n", " | "))
            if "imu" in text.lower():
                self._emit(
                    "status",
                    "这路是调试串口（115200，打印 imu ok）。SCS 在舵机总线，默认 1000000、ID 200。",
                )
                return
        self._emit("status", "无 SCS 应答。确认端口、波特率 1000000、ID 200。")

    def _scan_interrupted(self) -> bool:
        if not self.alive or self.ser is None:
            return True
        try:
            cmd = self.commands.get_nowait()
        except queue.Empty:
            return False
        if cmd[0] in ("quit", "close", "open"):
            self._handle(cmd)
            return True
        self._hold.append(cmd)
        return False

    def _finish_held(self) -> None:
        held = self._hold
        self._hold = []
        for cmd in held:
            if self.ser is None and cmd[0] not in ("open", "quit"):
                continue
            self._handle(cmd)

    def _attach(self, dev_id: int, start_poll: bool) -> bool:
        """只单播这个 ID。型号不是 IMU 时不读后面的内存，也不轮询。"""
        self.polling = False
        self.imu_ready = False
        found = self._classify(dev_id, timeout=0.1, log=True) == "imu"
        if found and start_poll:
            self.polling = True
        self._emit("polling", self.polling)
        return found

    def _scan(self) -> None:
        if self.ser is None:
            return
        self.polling = False
        self.imu_ready = False
        self._emit("polling", False)
        order: list[int] = []
        for dev_id in (self.dev_id, 200):
            if dev_id not in order and 0 <= dev_id <= 253:
                order.append(dev_id)
        order.extend(dev_id for dev_id in range(254) if dev_id not in order)
        skipped = 0
        try:
            for index, dev_id in enumerate(order):
                if self._scan_interrupted():
                    return
                self._emit("status", f"查找 IMU {dev_id}（{index + 1}/254）。舵机只核对型号，不读数据。")
                kind = self._classify(dev_id, timeout=0.05, log=False)
                if kind == "imu":
                    self.polling = True
                    self._emit("polling", True)
                    return
                if kind == "other":
                    skipped += 1
            self._emit("status", f"没有找到型号 {TABLE_MODEL:#06x} 的 IMU，已跳过 {skipped} 个其它设备。")
        finally:
            self._finish_held()

    def _classify(self, dev_id: int, timeout: float, log: bool) -> str:
        ping = build(dev_id, INST_PING)
        frame, leftover = self._exchange(
            ping,
            lambda fr: fr.dev_id == dev_id and fr.error == 0 and fr.data == b"",
            timeout,
            log,
        )
        if frame is None:
            if log:
                self._note_silence(leftover)
            return "none"
        model_pkt = build(dev_id, INST_READ, bytes((ADDR_MODEL, 2)))
        frame, _ = self._exchange(
            model_pkt,
            lambda fr: fr.dev_id == dev_id and fr.error == 0 and len(fr.data) == 2,
            timeout,
            log,
        )
        if frame is None:
            if log:
                self._emit("status", f"ID {dev_id} 有应答，但读不到型号")
            return "none"
        model = struct.unpack_from("<H", frame.data, 0)[0]
        if model != TABLE_MODEL:
            if log:
                self._emit("log", f"ID {dev_id} 型号 {model:#06x}，不是 IMU，已跳过")
                self._emit("status", f"ID {dev_id} 型号 {model:#06x}，这不是 IMU，没有继续读")
            return "other"
        info = build(dev_id, INST_READ, bytes((ADDR_MODEL, 4)))
        frame, _ = self._exchange(
            info,
            lambda fr: fr.dev_id == dev_id and fr.error == 0 and len(fr.data) == 4,
            timeout,
            log,
        )
        if frame is None:
            stored_id, baud = dev_id, 0
        else:
            stored_id = frame.data[2]
            baud = frame.data[3]
        self.dev_id = stored_id
        self.imu_ready = True
        self.prev_counter = None
        self._emit("found", stored_id)
        self._emit("identity", model, stored_id, baud)
        self._emit("log", f"IMU ID {stored_id}，{baud_label(baud)}")
        self._emit("status", f"已连接 IMU ID {stored_id}，{baud_label(baud)}")
        return "imu"

    def _read_imu(self, log: bool) -> None:
        if not self.imu_ready:
            return
        packet = build(self.dev_id, INST_READ, bytes((ADDR_IMU, IMU_LEN)))
        dev_id = self.dev_id
        frame, leftover = self._exchange(
            packet,
            lambda fr: fr.dev_id == dev_id and fr.error == 0 and len(fr.data) == IMU_LEN,
            0.05,
            log,
        )
        if frame is None:
            self.fail_streak += 1
            self._emit("timeout", self.fail_streak)
            if self.fail_streak == 1 or self.fail_streak % 25 == 0:
                text = looks_like_text(leftover)
                if text and "imu" in text.lower():
                    self._note_silence(leftover)
                else:
                    self._emit("log", f"读 IMU 超时（连续 {self.fail_streak} 次）")
            return
        self.fail_streak = 0
        sample = decode_block(frame.data, self.prev_counter)
        self.prev_counter = sample.counter
        with self.lock:
            self.samples.append(sample)

    def _write_id(self, new_id: int) -> None:
        if self.ser is None or not 0 <= new_id <= 253:
            return
        old = self.dev_id
        packet = build(old, INST_WRITE, bytes((ADDR_ID, new_id)))
        frame, _ = self._exchange(
            packet,
            lambda fr: fr.dev_id == old and fr.error == 0 and fr.data == b"",
            0.15,
            True,
        )
        if frame is None:
            self._emit("status", "写 ID 无应答，ID 未改")
            return
        self.dev_id = new_id
        self._emit("found", new_id)
        self._emit("log", f"ID {old} → {new_id}，已写入 Flash")
        self._emit("status", f"新 ID {new_id} 从下一帧生效")

    def _write_baud(self, enum: int) -> None:
        if self.ser is None or not 0 <= enum < len(BAUD_TABLE):
            return
        packet = build(self.dev_id, INST_WRITE, bytes((ADDR_BAUD, enum)))
        frame, _ = self._exchange(
            packet,
            lambda fr: fr.dev_id == self.dev_id and fr.error == 0 and fr.data == b"",
            0.15,
            True,
        )
        if frame is None:
            self._emit("status", "写波特率无应答，波特率未改")
            return
        hz = BAUD_TABLE[enum][0]
        try:
            self.ser.baudrate = hz
        except Exception as exc:
            self._emit("status", f"设备已改波特率，本地切换失败：{exc}")
            return
        self._emit("baud", enum, hz)
        self._emit("log", f"波特率已改为 {baud_label(enum)}，本地串口已跟随")
        self._emit("status", f"波特率 {baud_label(enum)}")


def nice_step(span: float, target: int = 4) -> float:
    if span <= 0 or not math.isfinite(span):
        return 1.0
    raw = span / target
    mag = 10 ** math.floor(math.log10(raw))
    for mult in (1, 2, 2.5, 5, 10):
        step = mult * mag
        if raw <= step * 1.000001:
            return step
    return 10 * mag


def format_tick(value: float, span: float) -> str:
    if span >= 50:
        return f"{value:.0f}"
    if span >= 5:
        return f"{value:.1f}"
    if span >= 0.5:
        return f"{value:.2f}"
    return f"{value:.3f}"


class TimeChart(ttk.Frame):
    """一条时间轴上的多条数值曲线。横轴为相对当前的秒。"""

    def __init__(self, master, title: str, series: list[tuple[str, str, str]], min_span: float, include_zero: bool):
        super().__init__(master)
        self.title = title
        self.series = series
        self.min_span = min_span
        self.include_zero = include_zero
        self.window = 10.0
        self.gap = 0.25
        self.samples: deque = deque(maxlen=8000)
        self.enabled = {key: tk.BooleanVar(value=True) for key, _label, _color in series}
        self.dirty = True
        self._plot = (0, 0, 10, 10)
        self._hits: list[tuple[float, float, dict]] = []
        self._mouse_x: int | None = None
        self._cw = 0
        self._ch = 0

        head = ttk.Frame(self)
        head.pack(fill="x")
        self.title_label = ttk.Label(head, text=title)
        self.title_label.pack(side="left", padx=(2, 8))
        for key, label, color in series:
            swatch = tk.Canvas(head, width=10, height=10, bg=color, highlightthickness=0)
            swatch.pack(side="left", padx=(6, 0))
            ttk.Checkbutton(head, text=label, variable=self.enabled[key], command=self._mark).pack(side="left")
        self.hover = ttk.Label(head, text="", font=TICK_FONT)
        self.hover.pack(side="right", padx=4)

        self.canvas = tk.Canvas(self, height=150, bg="#f4f5f7", highlightthickness=1, highlightbackground="#dee2e6")
        self.canvas.pack(fill="both", expand=True)
        self.canvas.bind("<Configure>", self._on_configure)
        self.canvas.bind("<Motion>", self._on_motion)
        self.canvas.bind("<Leave>", self._on_leave)

    def _mark(self) -> None:
        self.dirty = True

    def _on_configure(self, event) -> None:
        if event.width != self._cw or event.height != self._ch:
            self._cw = event.width
            self._ch = event.height
            self.dirty = True

    def set_title(self, title: str) -> None:
        self.title = title
        self.title_label.configure(text=title)

    def replace(self, items) -> None:
        self.samples.clear()
        self.samples.extend(items)
        self.dirty = True

    def append(self, t: float, values: dict) -> None:
        self.samples.append((t, values))
        self.dirty = True

    def clear(self) -> None:
        self.samples.clear()
        self.dirty = True

    def draw(self, now: float) -> None:
        canvas = self.canvas
        width = canvas.winfo_width()
        height = canvas.winfo_height()
        if width < 40 or height < 40:
            return
        self.dirty = False
        pad_l, pad_r, pad_t, pad_b = 56, 10, 8, 22
        x0, y0 = pad_l, pad_t
        x1, y1 = width - pad_r, height - pad_b
        self._plot = (x0, y0, x1, y1)
        if x1 <= x0 + 10 or y1 <= y0 + 10:
            return

        t1 = now
        t0 = now - self.window
        shown = [(t, values) for t, values in self.samples if t >= t0]
        enabled = [key for key, _label, _color in self.series if self.enabled[key].get()]
        ys: list[float] = []
        for _t, values in shown:
            for key in enabled:
                value = values.get(key, math.nan)
                if math.isfinite(value):
                    ys.append(value)
        if self.include_zero:
            ys.append(0.0)
        if not ys:
            ymin, ymax = -self.min_span / 2.0, self.min_span / 2.0
        else:
            ymin, ymax = min(ys), max(ys)
            if ymax - ymin < self.min_span:
                mid = (ymin + ymax) / 2.0
                ymin = mid - self.min_span / 2.0
                ymax = mid + self.min_span / 2.0
            else:
                pad = (ymax - ymin) * 0.08
                ymin -= pad
                ymax += pad
        span = max(ymax - ymin, 1e-9)

        def xpix(t: float) -> float:
            return x0 + (t - t0) / self.window * (x1 - x0)

        def ypix(value: float) -> float:
            return y1 - (value - ymin) / span * (y1 - y0)

        canvas.delete("plot")
        canvas.create_rectangle(x0, y0, x1, y1, fill="#ffffff", outline="#ced4da", tags="plot")

        step = nice_step(span)
        tick = math.ceil(ymin / step - 1e-9) * step
        for _ in range(12):
            if tick > ymax + step * 0.01:
                break
            py = ypix(tick)
            if y0 - 1 <= py <= y1 + 1:
                canvas.create_line(x0, py, x1, py, fill="#e9ecef", tags="plot")
                canvas.create_text(x0 - 6, py, text=format_tick(tick, span), anchor="e", font=TICK_FONT, fill="#495057", tags="plot")
            tick += step

        if ymin < 0.0 < ymax:
            canvas.create_line(x0, ypix(0.0), x1, ypix(0.0), fill="#adb5bd", tags="plot")

        tstep = 1 if self.window <= 15 else 2 if self.window <= 40 else 5
        k = 0
        while True:
            t = t1 - k * tstep
            if t < t0 - 1e-6:
                break
            px = xpix(t)
            canvas.create_line(px, y0, px, y1, fill="#f1f3f5", tags="plot")
            label = "0" if k == 0 else f"-{k * tstep:g}s"
            canvas.create_text(px, y1 + 4, text=label, anchor="n", font=TICK_FONT, fill="#495057", tags="plot")
            k += 1

        hits: list[tuple[float, float, dict]] = []
        for t, values in shown:
            hits.append((xpix(t), t, values))
        self._hits = hits

        for key, _label, color in self.series:
            if not self.enabled[key].get():
                continue
            current: list[tuple[float, float]] = []
            lines: list[list[tuple[float, float]]] = []
            prev_t = None
            for t, values in shown:
                value = values.get(key, math.nan)
                broken = prev_t is not None and (t - prev_t) > self.gap
                if not math.isfinite(value) or broken:
                    if len(current) >= 2:
                        lines.append(current)
                    current = []
                if math.isfinite(value):
                    current.append((t, value))
                prev_t = t
            if len(current) >= 2:
                lines.append(current)
            elif len(current) == 1:
                px, py = xpix(current[0][0]), ypix(current[0][1])
                canvas.create_oval(px - 2, py - 2, px + 2, py + 2, fill=color, outline=color, tags="plot")
            for line in lines:
                pixels = self._decimate([(xpix(t), ypix(v)) for t, v in line], max(2, int(x1 - x0)))
                flat = [coord for point in pixels for coord in point]
                if len(flat) >= 4:
                    canvas.create_line(*flat, fill=color, width=1.6, tags="plot")

        self._redraw_cursor()

    def _decimate(self, pixels: list[tuple[float, float]], columns: int) -> list[tuple[float, float]]:
        if len(pixels) <= columns:
            return pixels
        buckets: dict[int, list[float]] = {}
        order: list[int] = []
        for x, y in pixels:
            key = int(x)
            if key not in buckets:
                buckets[key] = []
                order.append(key)
            buckets[key].append(y)
        out = []
        for key in order:
            group = buckets[key]
            out.append((float(key), max(group, key=abs)))
        return out

    def _on_motion(self, event) -> None:
        self._mouse_x = event.x
        self._redraw_cursor()

    def _on_leave(self, _event) -> None:
        self._mouse_x = None
        self.canvas.delete("cursor")
        self.hover.configure(text="")

    def _redraw_cursor(self) -> None:
        canvas = self.canvas
        canvas.delete("cursor")
        if self._mouse_x is None or not self._hits:
            return
        x0, y0, x1, y1 = self._plot
        if not x0 <= self._mouse_x <= x1:
            self.hover.configure(text="")
            return
        nearest = min(self._hits, key=lambda item: abs(item[0] - self._mouse_x))
        if abs(nearest[0] - self._mouse_x) > 14:
            self.hover.configure(text="")
            return
        px, t, values = nearest
        age = self._hits[-1][1] - t
        parts = [f"t=-{age:.2f}s"]
        for key, label, _color in self.series:
            if not self.enabled[key].get():
                continue
            value = values.get(key, math.nan)
            text = "—" if not math.isfinite(value) else f"{value:+.3f}"
            parts.append(f"{label} {text}")
        canvas.create_line(px, y0, px, y1, fill="#495057", dash=(2, 2), tags="cursor")
        self.hover.configure(text="   ".join(parts))


class App:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("SCS IMU 调试")
        self.root.geometry("1240x820")
        self.root.minsize(980, 680)
        style = ttk.Style(self.root)
        if "vista" in style.theme_names():
            style.theme_use("vista")
        self.root.option_add("*Font", ("Microsoft YaHei UI", 9))

        self.commands: queue.Queue = queue.Queue()
        self.messages: queue.Queue = queue.Queue()
        self.pending: deque = deque()
        self.pending_lock = threading.Lock()
        self.history: deque = deque(maxlen=8000)
        self.worker = BusWorker(self.commands, self.messages, self.pending, self.pending_lock)
        self.thread = threading.Thread(target=self.worker.run, name="scs-bus", daemon=True)
        self.thread.start()

        self.connected = False
        self.polling = False
        self.timeouts = 0
        self.model = 0
        self.baud_enum = 0
        self.unit = tk.StringVar(value="phys")
        self.port_map: dict[str, str] = {}
        self._rates: deque = deque()
        self._build()
        self.root.protocol("WM_DELETE_WINDOW", self._close)
        self.root.after(50, self._pump)
        self.refresh_ports()

    def _build(self) -> None:
        bar = ttk.Frame(self.root, padding=(8, 8, 8, 4))
        bar.pack(fill="x")

        ttk.Label(bar, text="串口").pack(side="left")
        self.port_var = tk.StringVar()
        self.port_box = ttk.Combobox(bar, textvariable=self.port_var, width=36, state="readonly")
        self.port_box.pack(side="left", padx=(4, 4))
        ttk.Button(bar, text="刷新", command=self.refresh_ports).pack(side="left")

        ttk.Label(bar, text="波特率").pack(side="left", padx=(10, 0))
        self.baud_var = tk.StringVar(value=self._baud_choice(0))
        self.baud_box = ttk.Combobox(
            bar,
            textvariable=self.baud_var,
            width=22,
            state="readonly",
            values=[self._baud_choice(i) for i in range(len(BAUD_TABLE))],
        )
        self.baud_box.pack(side="left", padx=4)
        self.baud_box.current(0)

        ttk.Label(bar, text="ID").pack(side="left", padx=(10, 0))
        self.id_var = tk.StringVar(value="200")
        self.id_spin = ttk.Spinbox(bar, from_=0, to=253, textvariable=self.id_var, width=5, command=self._push_id)
        self.id_spin.pack(side="left", padx=4)
        self.id_spin.bind("<FocusOut>", lambda _e: self._push_id())
        self.id_spin.bind("<Return>", lambda _e: self._push_id())

        self.connect_btn = ttk.Button(bar, text="连接", command=self._toggle_connect)
        self.connect_btn.pack(side="left", padx=(8, 0))

        bar2 = ttk.Frame(self.root, padding=(8, 0, 8, 4))
        bar2.pack(fill="x")
        ttk.Button(bar2, text="查找 IMU", command=lambda: self.commands.put(("discover",))).pack(side="left")
        ttk.Button(bar2, text="读一次", command=lambda: self.commands.put(("once",))).pack(side="left", padx=4)
        self.poll_btn = ttk.Button(bar2, text="开始轮询", command=self._toggle_poll)
        self.poll_btn.pack(side="left")

        ttk.Label(bar2, text="周期 ms").pack(side="left", padx=(10, 0))
        self.period_var = tk.StringVar(value="20")
        period = ttk.Spinbox(bar2, from_=10, to=500, textvariable=self.period_var, width=5, command=self._push_period)
        period.pack(side="left", padx=4)
        period.bind("<FocusOut>", lambda _e: self._push_period())
        period.bind("<Return>", lambda _e: self._push_period())

        ttk.Label(bar2, text="窗口 s").pack(side="left", padx=(8, 0))
        self.window_var = tk.StringVar(value="10")
        window = ttk.Spinbox(bar2, from_=2, to=60, textvariable=self.window_var, width=4, command=self._push_window)
        window.pack(side="left", padx=4)
        window.bind("<FocusOut>", lambda _e: self._push_window())

        ttk.Radiobutton(bar2, text="物理量", variable=self.unit, value="phys", command=self._reload_charts).pack(side="left", padx=(10, 0))
        ttk.Radiobutton(bar2, text="原始计数", variable=self.unit, value="raw", command=self._reload_charts).pack(side="left")
        self.log_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(bar2, text="轮询写入日志", variable=self.log_var, command=self._push_log).pack(side="left", padx=8)
        ttk.Button(bar2, text="清空曲线", command=self._clear_charts).pack(side="left")

        self.status = ttk.Label(self.root, text="选择 SCS 总线串口。默认 1000000、ID 200。调试口 115200 只打印 imu ok。", padding=(8, 0, 8, 4))
        self.status.pack(fill="x")

        paned = ttk.Panedwindow(self.root, orient="horizontal")
        paned.pack(fill="both", expand=True, padx=8, pady=(0, 8))
        self.paned = paned

        left = ttk.Frame(paned, padding=(0, 0, 8, 0))
        right = ttk.Frame(paned)
        paned.add(left, weight=0)
        paned.add(right, weight=1)

        self._build_readout(left)
        self._build_charts(right)
        self.root.bind_all("<MouseWheel>", self._on_wheel)
        self.root.after(80, lambda: paned.sashpos(0, 430))

    def _build_readout(self, parent) -> None:
        self.vals: dict[str, tk.StringVar] = {}

        device = ttk.LabelFrame(parent, text="设备", padding=6)
        device.pack(fill="x")
        for row, (key, label) in enumerate(
            (("model", "型号"), ("ident", "ID / 波特率"), ("flags", "状态"), ("count", "采样计数"), ("rate", "轮询"))
        ):
            ttk.Label(device, text=label).grid(row=row, column=0, sticky="w", pady=1)
            var = tk.StringVar(value="—")
            self.vals[key] = var
            ttk.Label(device, textvariable=var, font=NUM_FONT).grid(row=row, column=1, sticky="w", padx=(8, 0))

        gyro = ttk.LabelFrame(parent, text="陀螺仪   ±500 dps，17.50 mdps/LSB", padding=6)
        gyro.pack(fill="x", pady=(6, 0))
        self._axis_table(gyro, "g", ("计数", "dps", "rad/s"))

        quat = ttk.LabelFrame(parent, text="四元数   地址 130，binary16，w 由主机补", padding=6)
        quat.pack(fill="x", pady=(6, 0))
        self._axis_table(quat, "q", ("值", "半精度", ""))
        note = tk.StringVar(value="")
        self.vals["qnote"] = note
        ttk.Label(quat, textvariable=note).grid(row=4, column=0, columnspan=4, sticky="w", pady=(2, 0))

        acc = ttk.LabelFrame(parent, text="加速度   ±4 g，0.122 mg/LSB", padding=6)
        acc.pack(fill="x", pady=(6, 0))
        self._axis_table(acc, "a", ("计数", "g", ""))
        mag = tk.StringVar(value="|a| —")
        self.vals["mag"] = mag
        ttk.Label(acc, textvariable=mag, font=NUM_FONT).grid(row=4, column=0, columnspan=4, sticky="w")

        raw = ttk.LabelFrame(parent, text="最近一帧  地址 124 起 20 字节", padding=6)
        raw.pack(fill="x", pady=(6, 0))
        self.frame_text = tk.Text(raw, height=4, width=42, font=NUM_FONT, relief="flat", background="#f8f9fa")
        self.frame_text.pack(fill="x")
        self.frame_text.configure(state="disabled")

        flash = ttk.LabelFrame(parent, text="写入 Flash", padding=6)
        flash.pack(fill="x", pady=(6, 0))
        ttk.Label(flash, text="新 ID").grid(row=0, column=0, sticky="w")
        self.new_id_var = tk.StringVar(value="200")
        ttk.Spinbox(flash, from_=0, to=253, textvariable=self.new_id_var, width=6).grid(row=0, column=1, sticky="w")
        ttk.Button(flash, text="写入 ID", command=self._write_id).grid(row=0, column=2, padx=6)
        ttk.Label(flash, text="新波特率").grid(row=1, column=0, sticky="w", pady=(4, 0))
        self.new_baud_var = tk.StringVar(value=self._baud_choice(0))
        ttk.Combobox(
            flash,
            textvariable=self.new_baud_var,
            width=18,
            state="readonly",
            values=[self._baud_choice(i) for i in range(len(BAUD_TABLE))],
        ).grid(row=1, column=1, sticky="w", pady=(4, 0))
        ttk.Button(flash, text="写入波特率", command=self._write_baud).grid(row=1, column=2, padx=6, pady=(4, 0))

        log_frame = ttk.LabelFrame(parent, text="日志", padding=4)
        log_frame.pack(fill="both", expand=True, pady=(6, 0))
        self.log_box = scrolledtext.ScrolledText(log_frame, height=6, font=("Consolas", 9), relief="flat")
        self.log_box.pack(fill="both", expand=True)

    def _axis_table(self, parent, prefix: str, headers: tuple[str, str, str]) -> None:
        for col, text in enumerate(headers, start=1):
            ttk.Label(parent, text=text, font=TICK_FONT).grid(row=0, column=col, padx=4)
        names = ("X", "Y", "Z", "w") if prefix == "q" else ("X", "Y", "Z")
        keys = ("x", "y", "z", "w") if prefix == "q" else ("x", "y", "z")
        for row, (name, key) in enumerate(zip(names, keys), start=1):
            color = AXIS_COLOR[key]
            ttk.Label(parent, text=name, foreground=color).grid(row=row, column=0, sticky="w")
            for col in range(3):
                var = tk.StringVar(value="—")
                self.vals[f"{prefix}{key}{col}"] = var
                ttk.Label(parent, textvariable=var, font=NUM_FONT, width=10, anchor="e").grid(row=row, column=col + 1, sticky="e")

    def _build_charts(self, parent) -> None:
        parent.rowconfigure(0, weight=1)
        parent.rowconfigure(1, weight=1)
        parent.rowconfigure(2, weight=1)
        parent.rowconfigure(3, weight=1)
        parent.columnconfigure(0, weight=1)
        self.charts = [
            TimeChart(parent, "姿态（度，传感器坐标）", [("x", "横滚", AXIS_COLOR["x"]), ("y", "俯仰", AXIS_COLOR["y"]), ("z", "航向", AXIS_COLOR["z"])], 30.0, True),
            TimeChart(parent, "陀螺仪（dps）", [("x", "X", AXIS_COLOR["x"]), ("y", "Y", AXIS_COLOR["y"]), ("z", "Z", AXIS_COLOR["z"])], 10.0, True),
            TimeChart(parent, "加速度（g）", [("x", "X", AXIS_COLOR["x"]), ("y", "Y", AXIS_COLOR["y"]), ("z", "Z", AXIS_COLOR["z"])], 0.5, True),
            TimeChart(parent, "四元数 x y z", [("x", "x", AXIS_COLOR["x"]), ("y", "y", AXIS_COLOR["y"]), ("z", "z", AXIS_COLOR["z"]), ("w", "w", AXIS_COLOR["w"])], 0.1, True),
        ]
        self.charts[3].enabled["w"].set(False)
        for index, chart in enumerate(self.charts):
            chart.grid(row=index, column=0, sticky="nsew", pady=2)

    def _baud_choice(self, enum: int) -> str:
        hz, name = BAUD_TABLE[enum]
        return f"{hz}  {name}"

    def _selected_baud(self) -> tuple[int, int]:
        text = self.baud_var.get()
        for enum, (hz, name) in enumerate(BAUD_TABLE):
            if text.startswith(str(hz)) or name in text:
                return hz, enum
        return 1_000_000, 0

    def _selected_port(self) -> str:
        text = self.port_var.get().strip()
        return self.port_map.get(text, text.split()[0] if text else "")

    def refresh_ports(self) -> None:
        if list_ports is None:
            self._set_status("未安装 pyserial。请执行 pip install pyserial")
            return
        labels = []
        self.port_map.clear()
        for port in list_ports.comports():
            desc = port.description or ""
            label = f"{port.device}  {desc}".strip()
            self.port_map[label] = port.device
            labels.append(label)
        self.port_box.configure(values=labels)
        if labels and self.port_var.get() not in self.port_map:
            self.port_var.set(labels[0])
        if not labels:
            self.port_var.set("")
            self._set_status("没有找到串口")

    def _push_id(self) -> None:
        try:
            dev_id = int(self.id_var.get())
        except ValueError:
            return
        if 0 <= dev_id <= 253:
            self.commands.put(("set_id", dev_id))

    def _push_period(self) -> None:
        try:
            ms = float(self.period_var.get())
        except ValueError:
            return
        ms = min(500.0, max(10.0, ms))
        self.period_var.set(str(int(ms)))
        self.commands.put(("set_period", ms / 1000.0))
        gap = max(0.25, ms / 1000.0 * 3.0)
        for chart in self.charts:
            chart.gap = gap

    def _push_window(self) -> None:
        try:
            seconds = float(self.window_var.get())
        except ValueError:
            return
        seconds = min(60.0, max(2.0, seconds))
        self.window_var.set(f"{seconds:.0f}")
        for chart in self.charts:
            chart.window = seconds
            chart.dirty = True

    def _push_log(self) -> None:
        self.commands.put(("log", self.log_var.get()))

    def _on_wheel(self, event) -> None:
        widget = event.widget
        if not isinstance(widget, tk.Canvas):
            return
        try:
            seconds = float(self.window_var.get())
        except ValueError:
            seconds = 10.0
        step = 2.0 if abs(event.delta) >= 120 else 1.0
        seconds += -step if event.delta > 0 else step
        self.window_var.set(f"{min(60.0, max(2.0, seconds)):.0f}")
        self._push_window()

    def _toggle_connect(self) -> None:
        if self.connected:
            self.commands.put(("close",))
            return
        port = self._selected_port()
        if not port:
            self._set_status("先选择串口")
            return
        if serial is None:
            messagebox.showerror("缺少依赖", "请先安装 pyserial：\n\npip install pyserial")
            return
        self._push_id()
        self._push_period()
        hz, _enum = self._selected_baud()
        self._set_status(f"正在打开 {port} …")
        self.commands.put(("open", port, hz))

    def _toggle_poll(self) -> None:
        self._push_id()
        self._push_period()
        self.commands.put(("poll", not self.polling))

    def _write_id(self) -> None:
        if not self.connected:
            self._set_status("还没有连接")
            return
        try:
            new_id = int(self.new_id_var.get())
        except ValueError:
            return
        if not 0 <= new_id <= 253:
            return
        if not messagebox.askyesno("写入 ID", f"把设备 ID 改为 {new_id} 并写入 Flash？"):
            return
        self.commands.put(("write_id", new_id))

    def _write_baud(self) -> None:
        if not self.connected:
            self._set_status("还没有连接")
            return
        text = self.new_baud_var.get()
        enum = 0
        for index in range(len(BAUD_TABLE)):
            if text.startswith(str(BAUD_TABLE[index][0])):
                enum = index
                break
        if not messagebox.askyesno("写入波特率", f"把波特率改为 {baud_label(enum)} 并写入 Flash？\n应答发出后本地串口会跟着切换。"):
            return
        self.commands.put(("write_baud", enum))

    def _clear_charts(self) -> None:
        self.history.clear()
        for chart in self.charts:
            chart.clear()

    def _set_status(self, text: str) -> None:
        self.status.configure(text=text)

    def _log(self, text: str) -> None:
        self.log_box.insert("end", time.strftime("%H:%M:%S ") + text + "\n")
        lines = int(self.log_box.index("end-1c").split(".")[0])
        if lines > 400:
            self.log_box.delete("1.0", f"{lines - 400}.0")
        self.log_box.see("end")

    def _set_frame(self, text: str) -> None:
        self.frame_text.configure(state="normal")
        self.frame_text.delete("1.0", "end")
        self.frame_text.insert("1.0", text)
        self.frame_text.configure(state="disabled")

    def _chart_points(self, sample: Sample) -> list[dict]:
        roll, pitch, yaw = sample.euler
        attitude = {"x": roll, "y": pitch, "z": yaw}
        if self.unit.get() == "raw":
            gyro = {k: float(v) for k, v in zip("xyz", sample.gyro_raw)}
            accel = {k: float(v) for k, v in zip("xyz", sample.accel_raw)}
        else:
            gyro = {k: v for k, v in zip("xyz", sample.gyro_dps)}
            accel = {k: v for k, v in zip("xyz", sample.accel_g)}
        quat = {"x": sample.quat[0], "y": sample.quat[1], "z": sample.quat[2], "w": sample.quat_w}
        return [attitude, gyro, accel, quat]

    def _reload_charts(self) -> None:
        raw = self.unit.get() == "raw"
        self.charts[1].set_title("陀螺仪（LSB）" if raw else "陀螺仪（dps）")
        self.charts[1].min_span = 400.0 if raw else 10.0
        self.charts[2].set_title("加速度（LSB）" if raw else "加速度（g）")
        self.charts[2].min_span = 2000.0 if raw else 0.5
        items = [self._chart_points(sample) for sample in self.history]
        for index, chart in enumerate(self.charts):
            chart.replace((sample.t, points[index]) for sample, points in zip(self.history, items))

    def _show_sample(self, sample: Sample) -> None:
        now = time.monotonic()
        self._rates.append(now)
        while self._rates and now - self._rates[0] > 1.0:
            self._rates.popleft()
        gx, gy, gz = sample.gyro_raw
        gds = sample.gyro_dps
        grs = sample.gyro_rad
        for key, raw, dps, rad in zip("xyz", (gx, gy, gz), gds, grs):
            self.vals[f"g{key}0"].set(f"{raw:d}")
            self.vals[f"g{key}1"].set(f"{dps:+.2f}")
            self.vals[f"g{key}2"].set(f"{rad:+.4f}")
        for index, key in enumerate("xyz"):
            self.vals[f"q{key}0"].set(f"{sample.quat[index]:+.5f}")
            self.vals[f"q{key}1"].set(f"{sample.quat_bits[index]:04X}")
            self.vals[f"q{key}2"].set("")
        w_text = "—" if not math.isfinite(sample.quat_w) else f"{sample.quat_w:+.5f}"
        self.vals["qw0"].set(w_text)
        self.vals["qw1"].set("")
        self.vals["qw2"].set("")
        roll, pitch, yaw = sample.euler

        def angle(value: float) -> str:
            return "—" if not math.isfinite(value) else f"{value:+.1f}°"

        pose = f"横滚 {angle(roll)}   俯仰 {angle(pitch)}   航向 {angle(yaw)}"
        if sample.quat_bits == (0, 0, 0) and not sample.quat_ready:
            self.vals["qnote"].set(pose + "   四元数仍是 0，SFLP 还没有输出")
        elif not math.isfinite(sample.quat_w):
            self.vals["qnote"].set(pose + "   x²+y²+z² 超过 1，w 无法补")
        else:
            self.vals["qnote"].set(pose + "   传感器坐标")
        ax, ay, az = sample.accel_raw
        ag = sample.accel_g
        for key, raw, g in zip("xyz", (ax, ay, az), ag):
            self.vals[f"a{key}0"].set(f"{raw:d}")
            self.vals[f"a{key}1"].set(f"{g:+.3f}")
            self.vals[f"a{key}2"].set("")
        mag = math.sqrt(sum(v * v for v in ag))
        self.vals["mag"].set(f"|a| {mag:.3f} g")
        self.vals["flags"].set(status_text(sample.status))
        stall = "  计数停滞" if sample.delta == 0 and self.polling else ""
        self.vals["count"].set(f"{sample.counter}   Δ{sample.delta}{stall}")
        self.vals["rate"].set(f"{len(self._rates)} Hz   超时 {self.timeouts}")
        raw_bytes = sample.raw
        self._set_frame(
            f"124  {hex_of(raw_bytes[0:6])}\n"
            f"130  {hex_of(raw_bytes[6:12])}\n"
            f"136  {hex_of(raw_bytes[12:18])}\n"
            f"142  {raw_bytes[18]:02X}   {raw_bytes[19]:02X}"
        )

    def _pump(self) -> None:
        while True:
            try:
                msg = self.messages.get_nowait()
            except queue.Empty:
                break
            self._on_message(msg)
        batch = []
        with self.pending_lock:
            while self.pending:
                batch.append(self.pending.popleft())
        if batch:
            self.timeouts = 0
            for sample in batch:
                self.history.append(sample)
                points = self._chart_points(sample)
                for chart, values in zip(self.charts, points):
                    chart.append(sample.t, values)
            self._show_sample(batch[-1])
        now = time.monotonic() if self.polling else (self.history[-1].t if self.history else time.monotonic())
        if batch or self.polling or any(chart.dirty for chart in self.charts):
            for chart in self.charts:
                chart.draw(now)
        self.root.after(33, self._pump)

    def _on_message(self, msg: tuple) -> None:
        kind = msg[0]
        if kind == "log":
            self._log(msg[1])
        elif kind == "status":
            self._set_status(msg[1])
        elif kind == "connected":
            self.connected = bool(msg[1])
            self.connect_btn.configure(text="断开" if self.connected else "连接")
            self.baud_box.configure(state="disabled" if self.connected else "readonly")
            self.port_box.configure(state="disabled" if self.connected else "readonly")
            if not self.connected:
                self.polling = False
                self.poll_btn.configure(text="开始轮询")
                self._set_status("已断开")
        elif kind == "polling":
            self.polling = bool(msg[1])
            self.poll_btn.configure(text="停止轮询" if self.polling else "开始轮询")
        elif kind == "found":
            dev_id = msg[1]
            self.id_var.set(str(dev_id))
            self.new_id_var.set(str(dev_id))
        elif kind == "identity":
            model, dev_id, baud = msg[1], msg[2], msg[3]
            self.model = model
            self.baud_enum = baud
            self.vals["model"].set(f"{model:#06x}" + ("" if model == TABLE_MODEL else "  与 0x4955 不符"))
            self.vals["ident"].set(f"{dev_id}    {baud_label(baud)}")
            self.id_var.set(str(dev_id))
            self.new_id_var.set(str(dev_id))
            if 0 <= baud < len(BAUD_TABLE):
                self.new_baud_var.set(self._baud_choice(baud))
        elif kind == "baud":
            enum, _hz = msg[1], msg[2]
            self.baud_enum = enum
            choice = self._baud_choice(enum)
            self.baud_var.set(choice)
            self.new_baud_var.set(choice)
            self.vals["ident"].set(f"{self.id_var.get()}    {baud_label(enum)}")
        elif kind == "timeout":
            self.timeouts = msg[1]
            self.vals["rate"].set(f"{len(self._rates)} Hz   超时 {self.timeouts}")

    def _close(self) -> None:
        self.worker.alive = False
        self.commands.put(("quit",))
        self.root.destroy()

    def run(self) -> None:
        self.root.mainloop()


def _self_check() -> None:
    assert checksum(bytes((0x01, 0x02, 0x01))) == 0xFB
    assert build(1, INST_PING) == bytes.fromhex("ffff010201fb")
    assert build(1, INST_READ, bytes((0x38, 0x02))) == bytes.fromhex("ffff0104023802be")
    frames, rest = parse_frames(bytes.fromhex("ffff0104001805dd"))
    assert rest == b"" and frames[0].dev_id == 1 and frames[0].data == bytes.fromhex("1805")
    frames, rest = parse_frames(bytes.fromhex("ffffffffff010200fc"))
    assert frames[0].dev_id == 1 and frames[0].error == 0 and frames[0].data == b""
    assert half_to_float(0x3C00) == 1.0
    assert half_to_float(0xBC00) == -1.0
    assert half_to_float(0x3800) == 0.5
    assert abs(half_to_float(0x3555) - 0.333) < 1e-3
    raw = struct.pack("<hhh", 1000, 0, -1000)
    raw += struct.pack("<HHH", 0x3800, 0, 0)
    raw += struct.pack("<hhh", 0, 0, 8192)
    raw += bytes((9, 0x07))
    sample = decode_block(raw, None)
    assert sample.gyro_raw[0] == 1000
    assert abs(sample.gyro_dps[0] - 17.5) < 1e-9
    assert abs(sample.quat[0] - 0.5) < 1e-6
    assert abs(sample.quat_w - math.sqrt(0.75)) < 1e-6
    assert abs(sample.accel_g[2] - 8192 * ACCEL_G_PER_LSB) < 1e-9
    assert sample.counter == 9 and sample.status == 0x07
    sample2 = decode_block(raw[:18] + bytes((11, 0x07)), 9)
    assert sample2.delta == 2


def main() -> None:
    _self_check()
    if "--check" in sys.argv:
        print("ok")
        return
    if serial is None:
        root = tk.Tk()
        root.withdraw()
        messagebox.showerror("缺少依赖", "请先安装 pyserial：\n\npip install pyserial")
        return
    App().run()


if __name__ == "__main__":
    main()
