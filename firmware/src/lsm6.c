#include "lsm6.h"

#include "board.h"
#include "table.h"

#include "at32f425.h"

#define REG_FUNC_CFG 0x01
#define REG_FIFO_CTRL1 0x07
#define REG_FIFO_CTRL4 0x0A
#define REG_INT1_CTRL 0x0D
#define REG_WHO_AM_I 0x0F
#define REG_CTRL1 0x10
#define REG_CTRL2 0x11
#define REG_CTRL3 0x12
#define REG_CTRL6 0x15
#define REG_CTRL8 0x17
#define REG_FIFO_STATUS1 0x1B
#define REG_STATUS 0x1E
#define REG_OUTX_L_G 0x22
#define REG_FIFO_TAG 0x78

#define REG_EMB_FUNC_EN_A 0x04
#define REG_EMB_FIFO_EN_A 0x44
#define REG_SFLP_ODR 0x5E

#define WHO_AM_I_VALUE 0x70
#define TAG_GAME_ROTATION 0x13

#define SPI_WAIT 100000u

static int online;

static int spi_wait(uint32_t flag)
{
  uint32_t left = SPI_WAIT;
  while (spi_i2s_flag_get(SPI1, flag) == RESET) {
    if (--left == 0) {
      return -1;
    }
  }
  return 0;
}

static void spi_clear(void)
{
  if (spi_i2s_flag_get(SPI1, SPI_I2S_ROERR_FLAG) == SET) {
    spi_i2s_flag_clear(SPI1, SPI_I2S_ROERR_FLAG);
  }
  if (spi_i2s_flag_get(SPI1, SPI_I2S_RDBF_FLAG) == SET) {
    (void)spi_i2s_data_receive(SPI1);
  }
}

static void cs_high(void)
{
  uint32_t left = SPI_WAIT;
  while (spi_i2s_flag_get(SPI1, SPI_I2S_BF_FLAG) == SET) {
    if (--left == 0) {
      break;
    }
  }
  gpio_bits_set(GPIOA, GPIO_PINS_4);
}

static int spi_xfer(uint8_t tx, uint8_t *rx)
{
  if (spi_wait(SPI_I2S_TDBE_FLAG) != 0) {
    return -1;
  }
  spi_i2s_data_transmit(SPI1, tx);
  if (spi_wait(SPI_I2S_RDBF_FLAG) != 0) {
    return -1;
  }
  uint8_t value = (uint8_t)spi_i2s_data_receive(SPI1);
  if (rx) {
    *rx = value;
  }
  return 0;
}

static int reg_write(uint8_t reg, uint8_t value)
{
  spi_clear();
  gpio_bits_reset(GPIOA, GPIO_PINS_4);
  int rc = spi_xfer((uint8_t)(reg & 0x7F), 0);
  if (rc == 0) {
    rc = spi_xfer(value, 0);
  }
  cs_high();
  return rc;
}

static int reg_read(uint8_t reg, uint8_t *dst, uint8_t len)
{
  spi_clear();
  gpio_bits_reset(GPIOA, GPIO_PINS_4);
  int rc = spi_xfer((uint8_t)(reg | 0x80), 0);
  for (uint8_t i = 0; rc == 0 && i < len; i++) {
    rc = spi_xfer(0x00, &dst[i]);
  }
  cs_high();
  return rc;
}

#if BOARD_DEBUG
static void debug_bytes(const uint8_t *data, uint8_t len)
{
  for (uint8_t i = 0; i < len; i++) {
    board_debug_hex(data[i], 2);
  }
}
#endif

static int bytes_differ(const uint8_t *a, const uint8_t *b, uint8_t len)
{
  for (uint8_t i = 0; i < len; i++) {
    if (a[i] != b[i]) {
      return 1;
    }
  }
  return 0;
}

static int soft_reset(void)
{
  if (reg_write(REG_CTRL3, 0x01) != 0) {
    return -1;
  }
  /* The reset bit can read as 0 while the chip is still restoring defaults. */
  board_delay_ms(15);
  uint32_t start = board_millis();
  while ((board_millis() - start) < 100) {
    uint8_t ctrl3 = 0x01;
    if (reg_read(REG_CTRL3, &ctrl3, 1) != 0) {
      return -1;
    }
    if ((ctrl3 & 0x01) == 0) {
      board_delay_ms(10);
      return 0;
    }
  }
  return -1;
}

static int sensors_on(void)
{
  if (reg_write(REG_FUNC_CFG, 0x00) != 0 || reg_write(REG_CTRL3, 0x44) != 0 ||
      reg_write(REG_CTRL1, 0x06) != 0 || reg_write(REG_CTRL2, 0x06) != 0 ||
      reg_write(REG_CTRL6, 0x02) != 0 || reg_write(REG_CTRL8, 0x01) != 0) {
    return -1;
  }
  uint8_t c1 = 0;
  uint8_t c2 = 0;
  if (reg_read(REG_CTRL1, &c1, 1) != 0 || reg_read(REG_CTRL2, &c2, 1) != 0) {
    return -1;
  }
  if (c1 != 0x06 || c2 != 0x06) {
    board_debug("imu odr ");
    board_debug_hex(c1, 2);
    board_debug(" ");
    board_debug_hex(c2, 2);
    board_debug("\r\n");
    return -1;
  }
  return 0;
}

static int sflp_enable(void)
{
  if (reg_write(REG_FUNC_CFG, 0x80) != 0) {
    return -1;
  }
  uint8_t odr = 0;
  int rc = reg_read(REG_SFLP_ODR, &odr, 1);
  if (rc == 0) {
    /* SFLP_GAME_ODR is bits [5:3]. 011 = 120 Hz. Leave the fixed bits alone. */
    odr = (uint8_t)((odr & (uint8_t)~0x38) | (uint8_t)(0x03 << 3));
    rc = reg_write(REG_SFLP_ODR, odr);
  }
  rc |= reg_write(REG_EMB_FUNC_EN_A, 0x02);
  rc |= reg_write(REG_EMB_FIFO_EN_A, 0x02);
  if (reg_write(REG_FUNC_CFG, 0x00) != 0) {
    rc = -1;
  }
  uint8_t access = 0x80;
  if (reg_read(REG_FUNC_CFG, &access, 1) != 0 || (access & 0x80) != 0) {
    rc = -1;
  }
  return rc;
}

static int fifo_stream(void)
{
  uint8_t fifo4 = 0;
  if (reg_read(REG_FIFO_CTRL4, &fifo4, 1) != 0) {
    return -1;
  }
  fifo4 = (uint8_t)((fifo4 & ~0x07) | 0x06);
  if (reg_write(REG_FIFO_CTRL1, 1) != 0 || reg_write(REG_FIFO_CTRL4, fifo4) != 0 ||
      reg_write(REG_INT1_CTRL, 0x04) != 0) {
    return -1;
  }
  return 0;
}

static int sensing_start(void)
{
  if (sensors_on() != 0) {
    board_debug("imu cfg fail\r\n");
    return -1;
  }
  if (sflp_enable() != 0) {
    board_debug("imu sflp fail\r\n");
    return -1;
  }
  if (fifo_stream() != 0) {
    board_debug("imu fifo fail\r\n");
    return -1;
  }
  return 0;
}

int lsm6_init(void)
{
  online = 0;
  uint8_t who = 0;
  int seen = 0;

  board_delay_ms(20);
  for (int attempt = 0; attempt < 5; attempt++) {
    if (reg_read(REG_WHO_AM_I, &who, 1) == 0 && who == WHO_AM_I_VALUE) {
      seen = 1;
      break;
    }
    board_delay_ms(5);
  }
  if (!seen) {
    board_debug("imu whoami fail\r\n");
    return -1;
  }
  if (soft_reset() != 0) {
    board_debug("imu reset fail\r\n");
    return -1;
  }
  if (sensing_start() != 0 && sensing_start() != 0) {
    return -1;
  }

#if BOARD_DEBUG
  board_delay_ms(50);
  uint8_t block[12];
  if (reg_read(REG_OUTX_L_G, block, 12) == 0) {
    board_debug("imu out g=");
    debug_bytes(block, 6);
    board_debug(" a=");
    debug_bytes(block + 6, 6);
    board_debug("\r\n");
  }
#endif

  online = 1;
  board_debug("imu ok\r\n");
  return 0;
}

void lsm6_poll(void)
{
  if (!online) {
    return;
  }

  static uint8_t gyro[6];
  static uint8_t accel[6];
  static uint8_t quat[6];
  static int quat_valid;
  static uint32_t last_pub;
  static uint32_t last_log;
  int fresh = 0;
  uint32_t now = board_millis();

  uint8_t block[12];
  if (reg_read(REG_OUTX_L_G, block, 12) == 0 &&
      ((now - last_pub) >= 8 || bytes_differ(block, gyro, 6) ||
       bytes_differ(block + 6, accel, 6))) {
    for (int i = 0; i < 6; i++) {
      gyro[i] = block[i];
      accel[i] = block[i + 6];
    }
    last_pub = now;
    fresh = 1;
  }

  for (int n = 0; n < 8; n++) {
    uint8_t level[2];
    if (reg_read(REG_FIFO_STATUS1, level, 2) != 0) {
      break;
    }
    if (level[0] == 0 && (level[1] & 0x01) == 0) {
      break;
    }
    uint8_t word[7];
    if (reg_read(REG_FIFO_TAG, word, 7) != 0) {
      break;
    }
    if (((word[0] >> 3) & 0x1F) == TAG_GAME_ROTATION &&
        (!quat_valid || bytes_differ(&word[1], quat, 6))) {
      for (int i = 0; i < 6; i++) {
        quat[i] = word[i + 1];
      }
      quat_valid = 1;
      fresh = 1;
    }
  }

  if (fresh) {
    table_update_sample(gyro, quat, quat_valid, accel);
  }

  if ((now - last_log) < 1000) {
    return;
  }
  last_log = now;

  uint8_t c1 = 0;
  if (reg_read(REG_CTRL1, &c1, 1) == 0 && c1 != 0x06) {
    board_debug("imu rearm\r\n");
    (void)sensing_start();
#if BOARD_DEBUG
    (void)reg_read(REG_CTRL1, &c1, 1);
#endif
  }
#if BOARD_DEBUG
  uint8_t st = 0;
  uint8_t fifo[2] = {0, 0};
  (void)reg_read(REG_STATUS, &st, 1);
  (void)reg_read(REG_FIFO_STATUS1, fifo, 2);
  board_debug("imu st=");
  board_debug_hex(st, 2);
  board_debug(" c1=");
  board_debug_hex(c1, 2);
  board_debug(" fifo=");
  board_debug_hex(fifo[0], 2);
  board_debug_hex(fifo[1], 2);
  board_debug(" q=");
  board_debug_u32((uint32_t)quat_valid);
  board_debug(" g=");
  debug_bytes(gyro, 6);
  board_debug(" a=");
  debug_bytes(accel, 6);
  board_debug("\r\n");
#endif
}
