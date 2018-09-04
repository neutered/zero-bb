#include <assert.h>
#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <endian.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/mman.h>

#include "pinutil.h"

#define PIN_RESET 16
#define PIN_EZ_CS 12
#define PIN_EZ_CK 7
#define PIN_EZ_D  8
#define PIN_EZ_Q  25

struct ez {
  int fd;
  uint32_t* regs;
  uint32_t phase;
};

static void ez_close(struct ez* c)
{
  if (!c) return;

  if ((c->regs != MAP_FAILED) && (munmap(c->regs, REG_MAP_SIZE) != 0))
    fprintf(stderr, "%s:%d: munmap():%d:%s\n", __func__, __LINE__, errno, strerror(errno));
  if ((c->fd != -1) && (close(c->fd) != 0))
    fprintf(stderr, "%s:%d: close():%d:%s\n", __func__, __LINE__, errno, strerror(errno));
  free(c);
}

static struct ez* ez_open(unsigned phase)
{
  /* make sure the clock rate is representable w/ usleep(3) since i'm
   * lazy and linux isn't quite rtos-y. also the spec says that the max
   * clock phase time is 500us.
   */
  if (phase < 10) {
    fprintf(stderr, "%s:%d: phase:%uus not representable\n", __func__, __LINE__, phase);
    return NULL;
  } else if (phase > 500) {
    fprintf(stderr, "%s:%d: phase:%uus out of spec (max 500us)\n", __func__, __LINE__, phase);
    return NULL;
  }
  printf("%s:%d: clock phase:time %uus\n", __func__, __LINE__, phase);

  struct ez* rv = malloc(sizeof(*rv));
  if (!rv) goto err_exit;
  rv->fd = -1;
  rv->regs = MAP_FAILED;
  rv->phase = phase;

  rv->fd = open("/dev/gpiomem", O_RDWR);
  if (rv->fd == -1) {
    fprintf(stderr, "%s:%d: open():%d:%s\n", __func__, __LINE__, errno, strerror(errno));
    goto err_exit;
  }

  rv->regs = mmap(0, REG_MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, rv->fd, 0);
  if (rv->regs == MAP_FAILED) {
    fprintf(stderr, "%s:%d: mmap():%d:%s\n", __func__, __LINE__, errno, strerror(errno));
    goto err_exit;
  }

  /* reset - active low - interal pullup
   * cs - active low
   */
  PIN_CONFIG_HIZ(rv->regs, PIN_RESET, 1);
  PIN_CONFIG_HIZ(rv->regs, PIN_EZ_CS, 1);
  PIN_CONFIG_HIZ(rv->regs, PIN_EZ_CK, -1);
  PIN_CONFIG_HIZ(rv->regs, PIN_EZ_D, -1);
  PIN_CONFIG_HIZ(rv->regs, PIN_EZ_Q, -1);

  PIN_DIR(rv->regs, PIN_RESET, 1);
  PIN_DIR(rv->regs, PIN_EZ_CS, 1);
  PIN_DIR(rv->regs, PIN_EZ_CK, 1);
  PIN_DIR(rv->regs, PIN_EZ_D, 1);
  PIN_DIR(rv->regs, PIN_EZ_Q, 0);

  PIN_WRITE(rv->regs, PIN_RESET, 1);
  PIN_WRITE(rv->regs, PIN_EZ_CS, 1);
  PIN_WRITE(rv->regs, PIN_EZ_CK, 0);
  PIN_WRITE(rv->regs, PIN_EZ_D, 0);

  return rv;

 err_exit:
  ez_close(rv);
  return NULL;
}

int main(int argc, char** argv)
{
  return 0;
}
