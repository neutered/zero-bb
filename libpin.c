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
#include <sys/stat.h>

#include "libpin.h"
#include "pinutil.h"

/* swd pin definitions */
#define PIN_CLOCK 20
#define PIN_DATA 21

/* both signals are active low
 *  external - goes to a fet off the gpio to get a hard power-on reset
 *  reset - goes to the actual pin for soft reset
 */
#define PIN_RESET 16
#define PIN_EXT_POR 18

/* ez port defines */
#define PIN_EZ_CS 23
#define PIN_EZ_DO 24
#define PIN_EZ_DI 25

/* for experimenting w/ the swd data line */
#define PIN_HIZ_DATA 1

struct pinctl {
  unsigned phase; /* us */
  /* direction change requires turnaround time clockin. this is
   * inserted on the next bus op.
   */
  int last_rw_op;
  int fd;
  uint32_t* regs;
};

static void clock_out_bit(struct pinctl* c, int p, int val)
{
  PIN_WRITE(c->regs, p, val);
  PIN_WRITE(c->regs, PIN_CLOCK, 1);
  usleep(c->phase);
  PIN_WRITE(c->regs, PIN_CLOCK, 0);
  usleep(c->phase);
}

static uint8_t clock_in_bit(struct pinctl* c, int p)
{
  uint8_t rv = PIN_READ(c->regs, p);
  PIN_WRITE(c->regs, PIN_CLOCK, 1);
  usleep(c->phase);
  PIN_WRITE(c->regs, PIN_CLOCK, 0);
  usleep(c->phase);

  return rv;
}

static int clock_turnaround(struct pinctl* c, int op)
{
  if (c->last_rw_op == op)
    return 0;

  /* the pin is not supposed be driven by either host or target, but
   * when we change to input state the drive stops. dropping the pin
   * is only so that it shows up on the analyzer before the mode
   * changes to allow for the pin not to be driven.
   */
  if (c->last_rw_op)
    PIN_WRITE(c->regs, PIN_DATA, PIN_HIZ_DATA <= 0 ? 0 : 1);
  PIN_DIR(c->regs, PIN_DATA, 0);
  for (int i = 0; i < 1; i++)
    clock_in_bit(c, PIN_DATA);
  if (op)
    PIN_DIR(c->regs, PIN_DATA, 1);
  c->last_rw_op = op;

  return 1;
}

struct pinctl* pins_open(unsigned phase)
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

  struct pinctl* rv = malloc(sizeof(*rv));
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

  /* setup high-z configuration.
   *  data - up. the device is supposed to have this set?
   *  clock - down. this is always host controlled?
   */
  PIN_CONFIG_HIZ(rv->regs, PIN_CLOCK, -1);
  PIN_CONFIG_HIZ(rv->regs, PIN_DATA, PIN_HIZ_DATA);

  /* default data to output */
  rv->last_rw_op = 1;
  PIN_DIR(rv->regs, PIN_CLOCK, 1);
  PIN_DIR(rv->regs, PIN_DATA, 1);

  /* default to regular operation */
  PIN_WRITE(rv->regs, PIN_DATA, 1);
  PIN_WRITE(rv->regs, PIN_CLOCK, 0);

  /* the mchck reset pin isn't exposed (and i fucked the pad), so futz
   * w/ external power which is almost the same thing. this is
   * separate from the ezport (except for checking ez cs at reset
   * time), so is outside of that config.
   */
  PIN_CONFIG_HIZ(rv->regs, PIN_EXT_POR, 1);
  PIN_DIR(rv->regs, PIN_EXT_POR, 1);
  PIN_WRITE(rv->regs, PIN_EXT_POR, 0);

  /* the reset pin has a chip-internal pull-up, but wwhen we wait to
   * come out of reset.
   */
  PIN_CONFIG_HIZ(rv->regs, PIN_RESET, 1);
  PIN_DIR(rv->regs, PIN_RESET, 0);

#if PIN_EZ
  PIN_CONFIG_HIZ(rv->regs, PIN_EZ_CS, 1);
  PIN_CONFIG_HIZ(rv->regs, PIN_EZ_DO, 0);
  PIN_CONFIG_HIZ(rv->regs, PIN_EZ_DI, 0);

  PIN_DIR(rv->regs, PIN_EZ_CS, 1);
  PIN_DIR(rv->regs, PIN_EZ_DO, 0);
  PIN_DIR(rv->regs, PIN_EZ_DI, 1);

  PIN_WRITE(rv->regs, PIN_EZ_CS, 0);
  PIN_READ(rv->regs, PIN_EZ_DO);
  PIN_WRITE(rv->regs, PIN_EZ_DI, 0);

ez_reset(rv);
#endif /* PIN_EZ */

#if 0
usleep(200000);
  PIN_WRITE(rv->regs, PIN_EXT_POR, 1);
usleep(200000);
  PIN_WRITE(rv->regs, PIN_EXT_POR, 0);
usleep(200000);
// assert(0);
  #endif

  return rv;

 err_exit:
  pins_close(rv);
  return NULL;
}

void pins_close(struct pinctl* c)
{
  if (!c) return;

  if ((c->regs != MAP_FAILED) && (munmap(c->regs, REG_MAP_SIZE) != 0))
    fprintf(stderr, "%s:%d: munmap():%d:%s\n", __func__, __LINE__, errno, strerror(errno));
  if ((c->fd != -1) && (close(c->fd) != 0))
    fprintf(stderr, "%s:%d: close():%d:%s\n", __func__, __LINE__, errno, strerror(errno));
  free(c);
}

int pins_write(struct pinctl* c, const uint8_t* bs, int nb)
{
  assert(c != NULL);
  assert(c->regs != MAP_FAILED);

  /* turnaround time? */
  clock_turnaround(c, 1);

  int i, j;
  for (i = 0; i < nb / 8; i++)
    for (j = 0; j < 8; j++)
      clock_out_bit(c, PIN_DATA, bs[i] & (1 << j));
  for (j = 0; j < (nb & 7); j++)
    clock_out_bit(c, PIN_DATA, bs[i] & (1 << j));

  return nb;
}

int pins_read(struct pinctl* c, uint8_t* bs, int nb)
{
  assert(c != NULL);
  assert(c->regs != MAP_FAILED);

  /* turnaround time? */
  clock_turnaround(c, 0);

  int i, j;
  for (i = 0; i < nb / 8; i++) {
    bs[i] = 0;
    for (j = 0; j < 8; j++)
      bs[i] |= (clock_in_bit(c, PIN_DATA) << j);
  }
  bs[i] = 0;
  for (j = 0; j < (nb & 7); j++)
    bs[i] |= (clock_in_bit(c, PIN_DATA) << j);

  return nb;
}

int ez_write(struct pinctl* c, const uint8_t* bs, int nb)
{
  assert(c != NULL);
  assert(c->regs != MAP_FAILED);

PIN_DIR(c->regs, PIN_EZ_DO, 0);
PIN_DIR(c->regs, PIN_EZ_DI, 1);

  PIN_WRITE(c->regs, PIN_EZ_CS, 0);
  usleep(c->phase);

  /* to be as consistent as possible, ez port is msb-first, but are
   * 'complete' octets, so that's nice.
   */
  for (int i = 0; i < nb; i++)
    for (int j = 7; j >= 0; j--)
      clock_out_bit(c, PIN_EZ_DI, bs[i] & (1 << j));

  PIN_WRITE(c->regs, PIN_EZ_CS, 1);
PIN_DIR(c->regs, PIN_EZ_DO, 0);
PIN_DIR(c->regs, PIN_EZ_DI, 0);
  usleep(2 * c->phase);

  return nb;
}

int ez_read(struct pinctl* c, uint8_t* bs, int nb)
{
  assert(c != NULL);
  assert(c->regs != MAP_FAILED);

PIN_DIR(c->regs, PIN_EZ_DO, 0);
PIN_DIR(c->regs, PIN_EZ_DI, 0);
  PIN_WRITE(c->regs, PIN_EZ_CS, 0);

  /* to be as consistent as possible, ez port is msb-first, but are
   * 'complete' octets, so that's nice.
   */
  for (int i = 0; i < nb; i++) {
    uint8_t v = 0;
    for (int j = 7; j >= 0; j--)
      v |= clock_in_bit(c, PIN_EZ_DO) << j;
    bs[i] = v;
  }

  PIN_WRITE(c->regs, PIN_EZ_CS, 1);
  usleep(2 * c->phase);

  return nb;
}

int pins_reset(struct pinctl* c, int external)
{
  /* drop all the outputs so that we don't 'power' the target. */
  PIN_WRITE(c->regs, PIN_CLOCK, 0);
  PIN_WRITE(c->regs, PIN_DATA, 0);

  if (external) {
    PIN_WRITE(c->regs, PIN_EXT_POR, 1);
    usleep(1000000);
    PIN_WRITE(c->regs, PIN_EXT_POR, 0);

    PIN_DIR(c->regs, PIN_RESET, 0);
    int i, v;
    for (i = 0, v = PIN_READ(c->regs, PIN_RESET); !v; i++, v = PIN_READ(c->regs, PIN_RESET))
      usleep(1000);
    fprintf(stderr, "%s:%d: n:%dms\n", __func__, __LINE__, i);
  } else {
    PIN_WRITE(c->regs, PIN_RESET, 0);
    PIN_DIR(c->regs, PIN_RESET, 1);
    usleep(1000);
  }

  /* wait for reset to release */
  PIN_DIR(c->regs, PIN_RESET, 0);
  int i, v;
  for (i = 0, v = PIN_READ(c->regs, PIN_RESET); !v; i++, v = PIN_READ(c->regs, PIN_RESET))
    usleep(1000);
  fprintf(stderr, "%s:%d: reset release %dms\n", __func__, __LINE__, i);

  return 0;
}

int ez_reset(struct pinctl* c)
{
#if !PIN_EZ
// assert(0);
#endif

  /* drop all the outputs so that we don't 'power' the target. */
  PIN_WRITE(c->regs, PIN_CLOCK, 0);
  PIN_WRITE(c->regs, PIN_DATA, 0);

  /* enable ezport cs so that when we toggle power it's waiting */
  PIN_WRITE(c->regs, PIN_EZ_CS, 0);
usleep(10000);

  /* bounce power since i busted the reset line, and there's no
   * programmatic control if we're coming through here (eg, security
   * is enabled).
   */
  PIN_WRITE(c->regs, PIN_EXT_POR, 1);
usleep(100000);
  PIN_WRITE(c->regs, PIN_EXT_POR, 0);
usleep(100000);
PIN_WRITE(c->regs, PIN_EZ_CS, 1);
usleep(10000);

  /* check that the ezport stuff came back ok? */
  uint8_t bs[] = { 0x05 };
  ez_write(c, bs, sizeof(bs));
  ez_read(c, bs, sizeof(bs));
  fprintf(stderr, "%s:%d: status:%02x\n", __func__, __LINE__, bs[0]);
  bs[0] = 0xb9;
  ez_write(c, bs, sizeof(bs));
usleep(10000);
  bs[0] = 0xb9;
  ez_write(c, bs, sizeof(bs));
  ez_read(c, bs, sizeof(bs));
  fprintf(stderr, "%s:%d: status:%02x\n", __func__, __LINE__, bs[0]);

// assert(0);

  return !!(bs[0] & (1 << 7));
}
