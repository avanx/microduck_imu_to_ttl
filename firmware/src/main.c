#include "board.h"
#include "lsm6.h"
#include "scs.h"
#include "table.h"

int main(void)
{
  board_init();
  table_init();
  board_bus_set_baud(table_baud_hz());
  scs_init();

  if (lsm6_init() == 0) {
    table_set_online(1);
  }

  while (1) {
    scs_poll();
    lsm6_poll();
  }
}
