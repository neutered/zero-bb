#include <stdlib.h>

#include "libpin.h"

int main(int argc, char** argv)
{
  int rv = EXIT_FAILURE;
  struct pinctl* pins = pins_open();
  if (!rv) goto err_exit;
  pins_close(pins);
  rv = EXIT_SUCCESS;
 err_exit:
  return rv;
}
