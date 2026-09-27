#ifndef BOARD_H
#define BOARD_H

#include <stdint.h>

/* USART1 log. 0 removes the prints from the image; 1 enables them. */
#ifndef BOARD_DEBUG
#define BOARD_DEBUG 0
#endif

void board_init(void);
#if BOARD_DEBUG
void board_debug(const char *text);
void board_debug_u32(uint32_t value);
void board_debug_hex(uint32_t value, uint8_t digits);
#else
#define board_debug(text) do { if (0) { (void)(text); } } while (0)
#define board_debug_u32(value) do { if (0) { (void)(value); } } while (0)
#define board_debug_hex(value, digits) do { if (0) { (void)(value); (void)(digits); } } while (0)
#endif
uint32_t board_millis(void);
void board_delay_ms(uint32_t ms);
void board_bus_set_baud(uint32_t baud);

#endif
