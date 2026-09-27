#include "scs.h"

#include "board.h"
#include "table.h"

#include "at32f425.h"

#define INST_PING 0x01
#define INST_READ 0x02
#define INST_WRITE 0x03
#define INST_SYNC_READ 0x82
#define BROADCAST_ID 0xFE

#define SCS_BODY_MAX 160
#define SCS_Q 4
#define SCS_REPLY_MAX (6 + TABLE_SIZE)
#define SLOT_MS 2

typedef struct {
  uint8_t id;
  uint8_t len;
  uint8_t body[SCS_BODY_MAX];
} scs_frame_t;

static scs_frame_t queue[SCS_Q];
static volatile uint8_t q_head;
static volatile uint8_t q_tail;

static uint8_t parse_state;
static uint8_t parse_id;
static uint8_t parse_len;
static uint8_t parse_got;
static uint16_t parse_skip;
static uint8_t parse_body[SCS_BODY_MAX];

static uint8_t reply_pending;
static uint8_t reply_wait;
static uint8_t reply[SCS_REPLY_MAX];
static uint8_t reply_len;
static uint32_t reply_deadline;
static int reply_baud_changed;

static volatile uint32_t rx_bytes;
static volatile uint32_t rx_overrun;
static volatile uint32_t rx_ferr;
static volatile uint32_t rx_nerr;
static volatile uint8_t rx_last;
static uint32_t frm_ok;
static uint32_t frm_bad;
static uint32_t frm_other;
static uint32_t frm_drop;
static uint32_t tx_count;
#if BOARD_DEBUG
static uint32_t stat_ms;
static uint32_t bad_log_ms;
static uint32_t read_log_ms;
static int trace_pending;
static uint8_t trace_inst;
static uint8_t trace_addr;
static uint8_t trace_n;
#endif

static void queue_push(uint8_t id, uint8_t len, const uint8_t *body)
{
  uint8_t next = (uint8_t)((q_head + 1) % SCS_Q);
  if (next == q_tail) {
    frm_drop++;
    return;
  }
  queue[q_head].id = id;
  queue[q_head].len = len;
  for (uint8_t i = 0; i < len; i++) {
    queue[q_head].body[i] = body[i];
  }
  q_head = next;
}

static int queue_pop(scs_frame_t *out)
{
  if (q_tail == q_head) {
    return 0;
  }
  *out = queue[q_tail];
  q_tail = (uint8_t)((q_tail + 1) % SCS_Q);
  return 1;
}

static void parse_byte(uint8_t byte)
{
  if (parse_state == 5) {
    if (--parse_skip == 0) {
      parse_state = 0;
    }
    return;
  }

  switch (parse_state) {
  case 0:
    parse_state = (byte == 0xFF) ? 1 : 0;
    break;
  case 1:
    parse_state = (byte == 0xFF) ? 2 : 0;
    break;
  case 2:
    if (byte == 0xFF) {
      break;
    }
    parse_id = byte;
    parse_state = 3;
    break;
  case 3:
    parse_len = byte;
    parse_got = 0;
    if (byte == 0) {
      parse_state = 0;
    } else if (byte > SCS_BODY_MAX) {
      parse_skip = byte;
      parse_state = 5;
    } else {
      parse_state = 4;
    }
    break;
  case 4:
    parse_body[parse_got++] = byte;
    if (parse_got == parse_len) {
      queue_push(parse_id, parse_len, parse_body);
      parse_state = 0;
    }
    break;
  default:
    parse_state = 0;
    break;
  }
}

#if BOARD_DEBUG
static const char *inst_name(uint8_t inst)
{
  switch (inst) {
  case INST_PING:
    return "ping";
  case INST_READ:
    return "read";
  case INST_WRITE:
    return "write";
  case INST_SYNC_READ:
    return "sync";
  default:
    return "inst";
  }
}

static int want_trace(uint8_t inst)
{
  if (inst != INST_READ) {
    return 1;
  }
  uint32_t now = board_millis();
  if ((now - read_log_ms) < 500) {
    return 0;
  }
  read_log_ms = now;
  return 1;
}

static void trace_arm(uint8_t inst, uint8_t addr, uint8_t n)
{
  if (!want_trace(inst)) {
    trace_pending = 0;
    return;
  }
  trace_inst = inst;
  trace_addr = addr;
  trace_n = n;
  trace_pending = 1;
}

static void trace_print(void)
{
  if (!trace_pending) {
    return;
  }
  trace_pending = 0;
  board_debug("scs ");
  board_debug(inst_name(trace_inst));
  board_debug(" id=");
  board_debug_u32(table_id());
  if (trace_inst != INST_PING) {
    board_debug(" addr=");
    board_debug_hex(trace_addr, 2);
    board_debug(" n=");
    board_debug_u32(trace_n);
  }
  board_debug("\r\n");
}

static void log_stat(void)
{
  uint32_t now = board_millis();
  if ((now - stat_ms) < 1000) {
    return;
  }
  stat_ms = now;
  board_debug("scs stat rx=");
  board_debug_u32(rx_bytes);
  board_debug(" frm=");
  board_debug_u32(frm_ok);
  board_debug(" bad=");
  board_debug_u32(frm_bad);
  board_debug(" other=");
  board_debug_u32(frm_other);
  board_debug(" tx=");
  board_debug_u32(tx_count);
  board_debug(" ovr=");
  board_debug_u32(rx_overrun);
  board_debug(" ferr=");
  board_debug_u32(rx_ferr);
  board_debug(" nerr=");
  board_debug_u32(rx_nerr);
  board_debug(" drop=");
  board_debug_u32(frm_drop);
  board_debug(" last=");
  board_debug_hex(rx_last, 2);
  board_debug("\r\n");
}
#else
#define trace_arm(inst, addr, n) do { if (0) { (void)(inst); (void)(addr); (void)(n); } } while (0)
#define trace_print() ((void)0)
#define log_stat() ((void)0)
#endif

static void log_bad(const scs_frame_t *frame)
{
  frm_bad++;
#if BOARD_DEBUG
  uint32_t now = board_millis();
  if ((now - bad_log_ms) < 500) {
    return;
  }
  bad_log_ms = now;
  board_debug("scs bad id=");
  board_debug_hex(frame->id, 2);
  board_debug(" len=");
  board_debug_u32(frame->len);
  board_debug("\r\n");
#else
  (void)frame;
#endif
}

static void rx_note_errors(void)
{
  if (usart_flag_get(USART2, USART_ROERR_FLAG) == SET) {
    rx_overrun++;
  }
  if (usart_flag_get(USART2, USART_FERR_FLAG) == SET) {
    rx_ferr++;
  }
  if (usart_flag_get(USART2, USART_NERR_FLAG) == SET) {
    rx_nerr++;
  }
}

static uint8_t checksum(uint8_t id, uint8_t len, const uint8_t *bytes, uint8_t n)
{
  uint16_t sum = (uint16_t)id + len;
  for (uint8_t i = 0; i < n; i++) {
    sum = (uint16_t)(sum + bytes[i]);
  }
  return (uint8_t)~sum;
}

static int frame_ok(const scs_frame_t *frame)
{
  if (frame->len < 2) {
    return 0;
  }
  uint8_t expect = checksum(frame->id, frame->len, frame->body, (uint8_t)(frame->len - 1));
  return expect == frame->body[frame->len - 1];
}

static int is_master_command(const scs_frame_t *frame)
{
  if (frame->id != BROADCAST_ID && frame->id != table_id()) {
    return 0;
  }
  switch (frame->body[0]) {
  case 0x01:
  case 0x02:
  case 0x03:
  case 0x04:
  case 0x05:
  case 0x06:
  case 0x08:
  case 0x09:
  case 0x0A:
  case 0x0B:
  case 0x82:
  case 0x83:
    return 1;
  default:
    return 0;
  }
}

static void bus_send(const uint8_t *data, uint8_t len)
{
  usart_interrupt_enable(USART2, USART_RDBF_INT, FALSE);
  gpio_bits_reset(GPIOA, GPIO_PINS_0);
  usart_receiver_enable(USART2, FALSE);
  usart_flag_clear(USART2, USART_TDC_FLAG);

  for (uint8_t i = 0; i < len; i++) {
    while (usart_flag_get(USART2, USART_TDBE_FLAG) == RESET) {
    }
    usart_data_transmit(USART2, data[i]);
  }
  while (usart_flag_get(USART2, USART_TDC_FLAG) == RESET) {
  }

  usart_receiver_enable(USART2, TRUE);
  while (usart_flag_get(USART2, USART_RDBF_FLAG) == SET) {
    (void)usart_data_receive(USART2);
  }
  gpio_bits_set(GPIOA, GPIO_PINS_0);
  usart_interrupt_enable(USART2, USART_RDBF_INT, TRUE);
}

static void arm_reply(uint8_t id, const uint8_t *param, uint8_t param_len, uint8_t wait)
{
  if ((uint16_t)param_len + 6 > SCS_REPLY_MAX) {
    param_len = SCS_REPLY_MAX - 6;
  }
  reply[0] = 0xFF;
  reply[1] = 0xFF;
  reply[2] = id;
  reply[3] = (uint8_t)(param_len + 2);
  reply[4] = 0x00;
  for (uint8_t i = 0; i < param_len; i++) {
    reply[5 + i] = param[i];
  }
  reply[5 + param_len] = checksum(id, reply[3], &reply[4], (uint8_t)(param_len + 1));
  reply_len = (uint8_t)(6 + param_len);
  reply_wait = wait;
  reply_pending = 1;
  reply_deadline = board_millis() + (wait ? SLOT_MS : 0);
}

static void flush_reply(void)
{
  uint8_t len = reply_len;
  uint8_t buf[SCS_REPLY_MAX];
  for (uint8_t i = 0; i < len; i++) {
    buf[i] = reply[i];
  }
  int change_baud = reply_baud_changed;
  reply_pending = 0;
  reply_baud_changed = 0;
  bus_send(buf, len);
  tx_count++;
  if (change_baud) {
    board_bus_set_baud(table_baud_hz());
  }
}

static void service_reply(void)
{
  if (!reply_pending) {
    return;
  }
  if (reply_wait == 0) {
    flush_reply();
    return;
  }
  if ((int32_t)(board_millis() - reply_deadline) >= 0) {
    reply_wait--;
    if (reply_wait == 0) {
      flush_reply();
    } else {
      reply_deadline = board_millis() + SLOT_MS;
    }
  }
}

static void handle_command(const scs_frame_t *frame)
{
  uint8_t inst = frame->body[0];
  uint8_t param_len = (uint8_t)(frame->len - 2);
  const uint8_t *param = &frame->body[1];
  int broadcast = frame->id == BROADCAST_ID;
  uint8_t self = table_id();

  if (!broadcast && frame->id != self) {
    frm_other++;
    return;
  }

  reply_baud_changed = 0;

  if (inst == INST_PING) {
    trace_arm(inst, 0, 0);
    arm_reply(broadcast ? self : frame->id, 0, 0, 0);
    return;
  }
  if (broadcast && inst != INST_SYNC_READ) {
    frm_other++;
    return;
  }

  if (inst == INST_READ && !broadcast && param_len >= 2) {
    uint8_t data[TABLE_SIZE];
    uint8_t n = param[1];
    if (n > TABLE_SIZE) {
      n = TABLE_SIZE;
    }
    trace_arm(inst, param[0], n);
    table_read(param[0], data, n);
    arm_reply(frame->id, data, n, 0);
    return;
  }

  if (inst == INST_WRITE && !broadcast && param_len >= 1) {
    trace_arm(inst, param[0], (uint8_t)(param_len - 1));
    reply_baud_changed = table_write(param[0], &param[1], (uint8_t)(param_len - 1));
    /* Answer with the ID the master addressed, even if the write changed it. */
    arm_reply(frame->id, 0, 0, 0);
    return;
  }

  if (inst == INST_SYNC_READ && broadcast && param_len >= 2) {
    uint8_t addr = param[0];
    uint8_t n = param[1];
    int index = -1;
    for (uint8_t i = 2; i < param_len; i++) {
      if (param[i] == self) {
        index = (int)(i - 2);
        break;
      }
    }
    if (index < 0) {
      frm_other++;
      return;
    }
    if (n > TABLE_SIZE) {
      n = TABLE_SIZE;
    }
    trace_arm(inst, addr, n);
    uint8_t data[TABLE_SIZE];
    table_read(addr, data, n);
    arm_reply(self, data, n, (uint8_t)index);
  }
}

static void on_frame(const scs_frame_t *frame)
{
  if (!frame_ok(frame)) {
    log_bad(frame);
    return;
  }
  frm_ok++;
  if (reply_pending && reply_wait > 0 && !is_master_command(frame)) {
    reply_wait--;
    if (reply_wait == 0) {
      flush_reply();
    } else {
      reply_deadline = board_millis() + SLOT_MS;
    }
    return;
  }
  if (reply_pending && is_master_command(frame)) {
    reply_pending = 0;
    reply_baud_changed = 0;
  }
  handle_command(frame);
}

void scs_init(void)
{
  for (int i = 0; i < 8; i++) {
    int error = usart_flag_get(USART2, USART_ROERR_FLAG) == SET ||
                usart_flag_get(USART2, USART_FERR_FLAG) == SET ||
                usart_flag_get(USART2, USART_NERR_FLAG) == SET;
    if (usart_flag_get(USART2, USART_RDBF_FLAG) == RESET && !error) {
      break;
    }
    (void)usart_data_receive(USART2);
  }
  usart_interrupt_enable(USART2, USART_RDBF_INT, TRUE);
  nvic_irq_enable(USART2_IRQn, 1, 0);
}

void scs_poll(void)
{
  scs_frame_t frame;
  while (queue_pop(&frame)) {
    on_frame(&frame);
    service_reply();
    trace_print();
  }
  service_reply();
  log_stat();
}

void USART2_IRQHandler(void)
{
  rx_note_errors();
  if (usart_flag_get(USART2, USART_RDBF_FLAG) != RESET) {
    uint8_t byte = (uint8_t)usart_data_receive(USART2);
    rx_bytes++;
    rx_last = byte;
    parse_byte(byte);
  } else if (usart_flag_get(USART2, USART_ROERR_FLAG) == SET ||
             usart_flag_get(USART2, USART_FERR_FLAG) == SET ||
             usart_flag_get(USART2, USART_NERR_FLAG) == SET) {
    (void)usart_data_receive(USART2);
  }
}
