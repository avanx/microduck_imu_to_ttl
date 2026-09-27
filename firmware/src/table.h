#ifndef TABLE_H
#define TABLE_H

#include <stdint.h>

#define TABLE_SIZE 144
#define TABLE_ADDR_ID 5
#define TABLE_ADDR_BAUD 6
#define TABLE_ADDR_IMU 124
#define TABLE_IMU_LEN 20

void table_init(void);
uint8_t table_id(void);
uint32_t table_baud_hz(void);

/* Copies `len` bytes starting at `addr`. Addresses past the table read as 0. */
void table_read(uint8_t addr, uint8_t *dst, uint8_t len);

/*
 * Applies bytes that land on the ID or baud registers. Other addresses are
 * left unchanged. Returns 1 when the baud enum actually changed.
 */
int table_write(uint8_t addr, const uint8_t *src, uint8_t len);

void table_set_online(int online);

/* Publishes one new sample and increments the counter at address 142. */
void table_update_sample(const uint8_t gyro[6], const uint8_t quat[6], int quat_valid,
                          const uint8_t accel[6]);

#endif
