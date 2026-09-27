# IMU 板接口约定

AT32F425K8U7-4 上的 LSM6DSV16X 板，作为飞特 SCS 从机挂在舵机总线上。主机按下面的控制表读取，数据含义与 microduck `duck-control` 的 `imu.rs` / `bus.rs` 一致。

现有 `robotd` 使用 Dynamixel Protocol 2（含指令 `0x8A` fast sync read）。本板只应答飞特 SCS。主机必须改成 SCS 的 SYNC READ，否则读不到这块板。

## 硬件

外部 8 MHz 晶振，系统时钟 PLL 到 96 MHz。

| 引脚 | 名称 | 约定 |
| --- | --- | --- |
| PA2 | MCU_TX | USART2 发送，复用 `GPIO_MUX_1` |
| PA3 | MCU_RX | USART2 接收，复用 `GPIO_MUX_1` |
| PA1 | MCU_DE | USART2 硬件 `USART2_RTS_DE`，复用 `GPIO_MUX_1`。驱动器低有效，极性 `USART_DE_POLARITY_LOW`，发送期间为低 |
| PA0 | MCU_RXEN | 接收缓冲使能，高电平打开。发送前置低，发送完成后再拉高，避免本机字节回环 |
| PA9 | UART_TX | USART1 调试输出，115200 8N1，复用 `GPIO_MUX_1`。仅 `BOARD_DEBUG=1` 时编译并初始化 |
| PA10 | UART_RX | USART1 调试输入，复用 `GPIO_MUX_1`。仅 `BOARD_DEBUG=1` 时编译并初始化 |
| PA4 | IMU_CS | SPI 片选，软件控制，低有效，空闲为高 |
| PA5 | IMU_SCK | SPI1 SCK，复用 `GPIO_MUX_0` |
| PA6 | IMU_MISO | SPI1 MISO，复用 `GPIO_MUX_0` |
| PA7 | IMU_MOSI | SPI1 MOSI，复用 `GPIO_MUX_0` |
| PB0 | IMU_INT1 | FIFO 水位，推挽输入。主循环同时轮询 FIFO 状态寄存器 |
| PB2 | IMU_INT2 | 未使用，内部下拉输入 |

SPI：模式 3（CPOL=1，CPHA=1），MSB 在前，SCK 约 6 MHz（96 MHz / 16）。LSM6DSV16X 同时接受模式 0 和模式 3。

## SCS 总线

小端，与 SMS/STS（`End = 0`）以及 microduck 的小端读数一致。

- 出厂波特率 1 Mbps，8 数据位，1 停止位，无校验。枚举值 0，对应 `FTServo_Arduino` `INST.h` 的 `_1M`
- 出厂 ID **200**。合法 ID 为 0–253。254（`0xFE`）是广播，不是设备 ID
- 波特率枚举与飞特表一致：0 = 1 Mbps，1 = 500 kbps，2 = 250 kbps，3 = 128 kbps，4 = 115200，5 = 76800，6 = 57600，7 = 38400，8 = 19200，9 = 14400，10 = 9600，11 = 4800

帧格式：

```text
FF FF | ID | Length | Instruction | Parameter... | CheckSum
```

`Length = 参数个数 + 2`，计的是 Instruction、Parameter 和 CheckSum。校验为 `~(ID + Length + Instruction + Parameter...)` 的低 8 位。

应答帧把 Instruction 的位置换成状态字节。本机成功时状态为 0：

```text
FF FF | ID | Length | 0x00 | Parameter... | CheckSum
```

实现的指令：

| 指令 | 值 | 行为 |
| --- | --- | --- |
| PING | `0x01` | 应答状态，无参数。广播 PING 也应答，ID 字段填本机当前 ID |
| READ | `0x02` | 参数为起始地址、长度。只响应单播 |
| WRITE | `0x03` | 只接受地址 5（ID）和地址 6（波特率枚举）。其它地址忽略，仍回状态 0。越界的 ID（大于 253）和波特率枚举（大于 11）忽略 |
| SYNC READ | `0x82` | 指令 ID 为 `0xFE`。参数为起始地址、长度、然后是 ID 列表。本机 ID 在列表中才应答，应答格式与 READ 相同 |

广播 `0xFE` 除 PING 外不应答。SYNC READ 使用广播 ID，但是按列表逐个回自己的状态包，这是例外。

改 ID 的那一帧应答仍使用旧 ID，避免主机按发送 ID 对不上应答。新 ID 从下一帧开始生效。改波特率在该帧应答发完之后再生效。ID 和波特率写入 Flash 最后一页（`0x0800FC00`，1 KB），掉电保留。链接脚本把代码区限制在前 63 KB，这一页不会被程序占用。没有飞特 EPROM 锁，不需要先写解锁寄存器。

同步读排队：本机在 ID 列表中的序号是 `k`（从 0 起）。`k = 0` 时组帧后立即回。`k > 0` 时先等前面 `k` 个完整应答帧，或每个槽位超时 2 ms（前面的舵机没说话就跳过），再发送。这样不会和总线上的舵机同时驱动。主机按应答帧里的 ID 匹配，与 Arduino `SCS::syncReadPacketRx` 相同。

## 控制表

未列出的地址读出为 0。除地址 5 和 6 以外都是只读。

| 地址 | 长度 | 内容 |
| --- | --- | --- |
| 3 | 2 | 型号 `0x4955`，小端，字节序 `55 49` |
| 5 | 1 | ID，默认 200 |
| 6 | 1 | 波特率枚举，默认 0（1 Mbps） |
| 124 | 20 | IMU 数据块。主机每拍读的是这里开始的 **12** 字节，与 `READ_ADDR = 124`、`READ_LEN = 12` 对齐 |

地址 124 起的 20 字节，全部小端：

| 偏移 | 地址 | 内容 |
| --- | --- | --- |
| 0 | 124 | 陀螺仪 X，`i16` 原始计数 |
| 2 | 126 | 陀螺仪 Y，`i16` |
| 4 | 128 | 陀螺仪 Z，`i16` |
| 6 | 130 | SFLP 游戏旋转向量 X，IEEE754 binary16 |
| 8 | 132 | 旋转向量 Y，binary16 |
| 10 | 134 | 旋转向量 Z，binary16 |
| 12 | 136 | 加速度 X，`i16` 原始计数 |
| 14 | 138 | 加速度 Y，`i16` |
| 16 | 140 | 加速度 Z，`i16` |
| 18 | 142 | 采样计数，`u8`，每写入一块新样本加 1，到 255 回绕 |
| 19 | 143 | 状态 |

陀螺仪量程 **±500 dps**，灵敏度 **17.50 mdps/LSB**。板上不做 rad/s 换算。主机换算与 microduck 相同：`rad/s = count * 0.0175 * pi / 180`。

旋转向量是芯片 FIFO 标签 `0x13` 的 6 字节原样拷贝，不是板上算出来的浮点。`w = sqrt(max(0, 1 - x² - y² - z²))` 由主机计算。SFLP 还没有输出过样本时，这 6 字节保持 0，主机据此保留上一帧姿态。芯片若给出接近单位四元数的结果（x = y = z = 0），字节同样是 0，主机无法把它和“尚未输出”分开。这是主机解码器的行为，板子不改写成非零占位。

加速度量程 **±4 g**，灵敏度 **0.122 mg/LSB**。控制环不读这 6 字节，只给诊断读取。

状态字节：

| 位 | 含义 |
| --- | --- |
| bit0 | 传感器在线，`WHO_AM_I` 为 `0x70` 且初始化完成 |
| bit1 | 已经收到过至少一帧 SFLP 四元数 |
| bit2 | 陀螺仪或加速度已有一次有效输出 |

主机 50 Hz 的 `sync_read` 只取前 12 字节。采样计数和状态不在这 12 字节里。相邻两次总线读取是否相同，只看陀螺仪和四元数。

## IMU

芯片 LSM6DSV16X，`WHO_AM_I`（`0x0F`）= `0x70`。

| 项目 | 值 |
| --- | --- |
| 加速度 ODR | 120 Hz，高性能模式，`CTRL1 = 0x06` |
| 陀螺仪 ODR | 120 Hz，高性能模式，`CTRL2 = 0x06` |
| 陀螺仪满量程 | ±500 dps，`CTRL6 = 0x02` |
| 加速度满量程 | ±4 g，`CTRL8 = 0x01` |
| SFLP | 嵌入式页 `EMB_FUNC_EN_A` bit1 `SFLP_GAME_EN` |
| SFLP ODR | 120 Hz，`SFLP_ODR` 的 `SFLP_GAME_ODR[5:3] = 0x3`，其余固定位保持芯片复位值 |
| FIFO | 只批游戏旋转向量（`EMB_FUNC_FIFO_EN_A` bit1），连续模式。标签 `0x13`，数据 6 字节小端半精度 |

120 Hz 高于主机 50 Hz 读总线的节奏，使相邻两次 12 字节读数通常不同，避免被主机当成姿态卡住。

陀螺仪和加速度从输出寄存器读（陀螺仪 `0x22` 起 6 字节，加速度 `0x28` 起 6 字节），不进 FIFO。四元数只从 FIFO 取。总线读到的是一份当前快照：前 12 字节给控制环，后 8 字节给诊断。
