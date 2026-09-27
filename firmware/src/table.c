#include "table.h"

#include "board.h"

#include "at32f425.h"

#include <string.h>

#define MODEL_L 0x55
#define MODEL_H 0x49
#define DEFAULT_ID 200
#define DEFAULT_BAUD 0

#define CFG_MAGIC 0x314C4D49u
#define CFG_ADDR 0x0800FC00u

#define STATUS_ONLINE 0x01
#define STATUS_QUAT 0x02
#define STATUS_SAMPLE 0x04

static const uint32_t baud_hz[12] = {
    1000000, 500000, 250000, 128000, 115200, 76800,
    57600,    38400,  19200,  14400,  9600,   4800,
};

static uint8_t mem[TABLE_SIZE];
static uint8_t status;

static void apply_identity(uint8_t id, uint8_t baud)
{
  mem[3] = MODEL_L;
  mem[4] = MODEL_H;
  mem[TABLE_ADDR_ID] = id;
  mem[TABLE_ADDR_BAUD] = baud;
}

static int cfg_load(uint8_t *id, uint8_t *baud)
{
  uint32_t magic = *(volatile uint32_t *)CFG_ADDR;
  uint32_t packed = *(volatile uint32_t *)(CFG_ADDR + 4);
  uint8_t stored_id = (uint8_t)(packed & 0xFF);
  uint8_t stored_baud = (uint8_t)((packed >> 8) & 0xFF);
  uint16_t crc = (uint16_t)(packed >> 16);
  uint16_t expect = (uint16_t)(stored_id + stored_baud + 0xA5);

  if (magic != CFG_MAGIC || crc != expect) {
    return 0;
  }
  if (stored_id > 253 || stored_baud > 11) {
    return 0;
  }
  *id = stored_id;
  *baud = stored_baud;
  return 1;
}

static void cfg_store(uint8_t id, uint8_t baud)
{
  uint16_t crc = (uint16_t)(id + baud + 0xA5);
  uint32_t packed = (uint32_t)id | ((uint32_t)baud << 8) | ((uint32_t)crc << 16);

  __disable_irq();
  flash_unlock();
  if (flash_sector_erase(CFG_ADDR) == FLASH_OPERATE_DONE) {
    flash_word_program(CFG_ADDR, CFG_MAGIC);
    flash_word_program(CFG_ADDR + 4, packed);
  }
  flash_lock();
  __enable_irq();
}

void table_init(void)
{
  uint8_t id = DEFAULT_ID;
  uint8_t baud = DEFAULT_BAUD;

  memset(mem, 0, sizeof(mem));
  int from_flash = cfg_load(&id, &baud);
  if (!from_flash) {
    id = DEFAULT_ID;
    baud = DEFAULT_BAUD;
  }
  apply_identity(id, baud);
  board_debug("scs id=");
  board_debug_u32(id);
  board_debug(" baud=");
  board_debug_u32(baud_hz[baud]);
  board_debug(from_flash ? " cfg=flash\r\n" : " cfg=default\r\n");
  status = 0;
  mem[143] = 0;
}

uint8_t table_id(void)
{
  return mem[TABLE_ADDR_ID];
}

uint32_t table_baud_hz(void)
{
  uint8_t baud = mem[TABLE_ADDR_BAUD];
  if (baud > 11) {
    baud = DEFAULT_BAUD;
  }
  return baud_hz[baud];
}

void table_read(uint8_t addr, uint8_t *dst, uint8_t len)
{
  for (uint8_t i = 0; i < len; i++) {
    uint16_t at = (uint16_t)addr + i;
    dst[i] = (at < TABLE_SIZE) ? mem[at] : 0;
  }
}

int table_write(uint8_t addr, const uint8_t *src, uint8_t len)
{
  int have_id = 0;
  int have_baud = 0;
  uint8_t new_id = mem[TABLE_ADDR_ID];
  uint8_t new_baud = mem[TABLE_ADDR_BAUD];

  for (uint8_t i = 0; i < len; i++) {
    uint16_t at = (uint16_t)addr + i;
    if (at == TABLE_ADDR_ID) {
      have_id = 1;
      new_id = src[i];
    } else if (at == TABLE_ADDR_BAUD) {
      have_baud = 1;
      new_baud = src[i];
    }
  }

  if (have_id && new_id > 253) {
    have_id = 0;
    new_id = mem[TABLE_ADDR_ID];
  }
  if (have_baud && new_baud > 11) {
    have_baud = 0;
    new_baud = mem[TABLE_ADDR_BAUD];
  }

  int baud_changed = have_baud && new_baud != mem[TABLE_ADDR_BAUD];
  int id_changed = have_id && new_id != mem[TABLE_ADDR_ID];
  if (id_changed || baud_changed) {
    mem[TABLE_ADDR_ID] = new_id;
    mem[TABLE_ADDR_BAUD] = new_baud;
    cfg_store(new_id, new_baud);
  }
  return baud_changed;
}

void table_set_online(int online)
{
  if (online) {
    status |= STATUS_ONLINE;
  } else {
    status &= (uint8_t)~STATUS_ONLINE;
  }
  mem[143] = status;
}

void table_update_sample(const uint8_t gyro[6], const uint8_t quat[6], int quat_valid,
                          const uint8_t accel[6])
{
  memcpy(&mem[TABLE_ADDR_IMU], gyro, 6);
  memcpy(&mem[TABLE_ADDR_IMU + 6], quat, 6);
  memcpy(&mem[TABLE_ADDR_IMU + 12], accel, 6);
  mem[142]++;
  status |= STATUS_SAMPLE;
  if (quat_valid) {
    status |= STATUS_QUAT;
  }
  mem[143] = status;
}
