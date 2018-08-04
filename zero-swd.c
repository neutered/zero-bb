#include <stdlib.h>

#include "libpin.h"

static void resync(struct pinctl* pins)
{
  static const uint8_t bs[] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00 };
  pins_write(pins, bs, sizeof(bs) * 8);
}

static void jtag2swd(struct pinctl* pins)
{
  /* on the wire format:
   *  0111 1001 1110 0111
   */
  static const uint8_t bs[] = { 0x9e, 0xe7 };
  pins_write(pins, bs, sizeof(bs) * 8);
}

int main(int argc, char** argv)
{
  int rv = EXIT_FAILURE;
  struct pinctl* pins = pins_open();
  if (!pins) goto err_exit;

  /* reset-y bits for the debug port */
  resync(pins);
  jtag2swd(pins);
  resync(pins);

  /* reading id takes the debug port out of reset */
  
  pins_close(pins);
  rv = EXIT_SUCCESS;
 err_exit:
  return rv;
}
