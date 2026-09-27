#include "board.h"

#include "at32f425.h"

#define PIN_RXEN GPIO_PINS_0

static volatile uint32_t millis;

static void system_clock_config(void)
{
  crm_reset();
  flash_psr_set(FLASH_WAIT_CYCLE_2);
  crm_clock_source_enable(CRM_CLOCK_SOURCE_HEXT, TRUE);
  while (crm_hext_stable_wait() == ERROR) {
  }
  crm_pll_config(CRM_PLL_SOURCE_HEXT, CRM_PLL_MULT_12);
  crm_clock_source_enable(CRM_CLOCK_SOURCE_PLL, TRUE);
  while (crm_flag_get(CRM_PLL_STABLE_FLAG) != SET) {
  }
  crm_ahb_div_set(CRM_AHB_DIV_1);
  crm_apb2_div_set(CRM_APB2_DIV_1);
  crm_apb1_div_set(CRM_APB1_DIV_1);
  crm_sysclk_switch(CRM_SCLK_PLL);
  while (crm_sysclk_switch_status_get() != CRM_SCLK_PLL) {
  }
  system_core_clock_update();
}

static void gpio_mux(gpio_type *port, uint16_t pins, gpio_pins_source_type source, gpio_mux_sel_type mux)
{
  gpio_init_type gpio;

  gpio_default_para_init(&gpio);
  gpio.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio.gpio_mode = GPIO_MODE_MUX;
  gpio.gpio_pins = pins;
  gpio.gpio_pull = GPIO_PULL_NONE;
  gpio_init(port, &gpio);
  gpio_pin_mux_config(port, source, mux);
}

static void usart_setup(usart_type *uart, uint32_t baud)
{
  usart_init(uart, baud, USART_DATA_8BITS, USART_STOP_1_BIT);
  usart_transmitter_enable(uart, TRUE);
  usart_receiver_enable(uart, TRUE);
  usart_enable(uart, TRUE);
}

static void bus_pins(void)
{
  gpio_mux(GPIOA, GPIO_PINS_1, GPIO_PINS_SOURCE1, GPIO_MUX_1);
  gpio_mux(GPIOA, GPIO_PINS_2, GPIO_PINS_SOURCE2, GPIO_MUX_1);
  gpio_mux(GPIOA, GPIO_PINS_3, GPIO_PINS_SOURCE3, GPIO_MUX_1);

  gpio_init_type gpio;
  gpio_default_para_init(&gpio);
  gpio.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio.gpio_mode = GPIO_MODE_OUTPUT;
  gpio.gpio_pins = PIN_RXEN;
  gpio.gpio_pull = GPIO_PULL_NONE;
  gpio_init(GPIOA, &gpio);
  gpio_bits_set(GPIOA, PIN_RXEN);
}

#if BOARD_DEBUG
static void debug_pins(void)
{
  gpio_mux(GPIOA, GPIO_PINS_9, GPIO_PINS_SOURCE9, GPIO_MUX_1);
  gpio_mux(GPIOA, GPIO_PINS_10, GPIO_PINS_SOURCE10, GPIO_MUX_1);
}
#endif

static void spi_pins(void)
{
  gpio_init_type gpio;

  gpio_default_para_init(&gpio);
  gpio.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio.gpio_mode = GPIO_MODE_OUTPUT;
  gpio.gpio_pins = GPIO_PINS_4;
  gpio.gpio_pull = GPIO_PULL_UP;
  gpio_init(GPIOA, &gpio);
  gpio_bits_set(GPIOA, GPIO_PINS_4);

  gpio_mux(GPIOA, GPIO_PINS_5, GPIO_PINS_SOURCE5, GPIO_MUX_0);
  gpio_mux(GPIOA, GPIO_PINS_6, GPIO_PINS_SOURCE6, GPIO_MUX_0);
  gpio_mux(GPIOA, GPIO_PINS_7, GPIO_PINS_SOURCE7, GPIO_MUX_0);

  gpio_default_para_init(&gpio);
  gpio.gpio_mode = GPIO_MODE_INPUT;
  gpio.gpio_pull = GPIO_PULL_DOWN;
  gpio.gpio_pins = GPIO_PINS_0 | GPIO_PINS_2;
  gpio_init(GPIOB, &gpio);
}

static void spi_setup(void)
{
  spi_init_type spi;

  spi_default_para_init(&spi);
  spi.transmission_mode = SPI_TRANSMIT_FULL_DUPLEX;
  spi.master_slave_mode = SPI_MODE_MASTER;
  spi.mclk_freq_division = SPI_MCLK_DIV_16;
  spi.first_bit_transmission = SPI_FIRST_BIT_MSB;
  spi.frame_bit_num = SPI_FRAME_8BIT;
  spi.clock_polarity = SPI_CLOCK_POLARITY_HIGH;
  spi.clock_phase = SPI_CLOCK_PHASE_2EDGE;
  spi.cs_mode_selection = SPI_CS_SOFTWARE_MODE;
  spi_init(SPI1, &spi);
  spi_enable(SPI1, TRUE);
}

void board_init(void)
{
  system_clock_config();
  nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);
  SysTick_Config(system_core_clock / 1000);

  crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
#if BOARD_DEBUG
  crm_periph_clock_enable(CRM_USART1_PERIPH_CLOCK, TRUE);
#endif
  crm_periph_clock_enable(CRM_USART2_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_SPI1_PERIPH_CLOCK, TRUE);

  bus_pins();
#if BOARD_DEBUG
  debug_pins();
#endif
  spi_pins();

#if BOARD_DEBUG
  usart_setup(USART1, 115200);
#endif

  usart_setup(USART2, 1000000);
  usart_rs485_delay_time_config(USART2, 1, 1);
  usart_de_polarity_set(USART2, USART_DE_POLARITY_LOW);
  usart_rs485_mode_enable(USART2, TRUE);

  spi_setup();
}

#if BOARD_DEBUG
void board_debug(const char *text)
{
  while (*text) {
    while (usart_flag_get(USART1, USART_TDBE_FLAG) == RESET) {
    }
    usart_data_transmit(USART1, (uint8_t)*text++);
  }
}

void board_debug_u32(uint32_t value)
{
  char buf[11];
  char *end = buf + sizeof(buf) - 1;
  char *p = end;

  *p = 0;
  do {
    *--p = (char)('0' + (value % 10u));
    value /= 10u;
  } while (value != 0 && p > buf);
  board_debug(p);
}

void board_debug_hex(uint32_t value, uint8_t digits)
{
  char buf[9];
  static const char hex[] = "0123456789ABCDEF";

  if (digits > 8) {
    digits = 8;
  }
  for (uint8_t i = 0; i < digits; i++) {
    uint8_t shift = (uint8_t)((digits - 1u - i) * 4u);
    buf[i] = hex[(value >> shift) & 0xFu];
  }
  buf[digits] = 0;
  board_debug(buf);
}
#endif

uint32_t board_millis(void)
{
  return millis;
}

void board_delay_ms(uint32_t ms)
{
  uint32_t start = millis;
  while ((millis - start) < ms) {
  }
}

void board_bus_set_baud(uint32_t baud)
{
  usart_interrupt_enable(USART2, USART_RDBF_INT, FALSE);
  usart_enable(USART2, FALSE);
  usart_init(USART2, baud, USART_DATA_8BITS, USART_STOP_1_BIT);
  usart_rs485_delay_time_config(USART2, 1, 1);
  usart_de_polarity_set(USART2, USART_DE_POLARITY_LOW);
  usart_rs485_mode_enable(USART2, TRUE);
  usart_transmitter_enable(USART2, TRUE);
  usart_receiver_enable(USART2, TRUE);
  usart_enable(USART2, TRUE);
  usart_interrupt_enable(USART2, USART_RDBF_INT, TRUE);
}

void SysTick_Handler(void)
{
  millis++;
}
