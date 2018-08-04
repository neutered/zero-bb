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

/* FixMe: use sysctl to get page size or something, but the current
 * code doesn't care about the size so it doesn't really matter so we
 * go w/ 4k because that sort of works.
 */
#define REG_MAP_SIZE 0x1000

#define PIN_CLOCK 20
#define PIN_DATA 21

#if 1
#define LOAD(__p, __t) \
  __atomic_load_n(__p, __t)
#define STORE(__p, __v, __t) \
  __atomic_store_n(__p, __v, __t)
#else
#define LOAD(__p, __t) \
  *(__p)
#define STORE(__p, __v, __t) \
  *(__p) = __v
#endif

#define PIN_DIR(__r, __p, __d) do { \
assert((__p) < 54); \
unsigned i = ((__p) / 10); \
unsigned s = 3 * ((__p) % 10); \
 uint32_t v = LOAD((__r) + i, __ATOMIC_ACQUIRE); \
v &= ~(3 << s); \
v |= ((__d) << s); \
STORE((__r) + i, v, __ATOMIC_RELEASE); \
} while (0)

#define PIN_WRITE(__r, __p, __v) do { \
assert((__p) < 54); \
unsigned b = ((__v) ? 0x1c : 0x28) / 4; \
unsigned i = b + ((__p) / 32); \
unsigned s = (__p) % 32; \
STORE((__r) + i, (1 << s), __ATOMIC_RELEASE); \
} while(0)

#define PIN_READ(__r, __p) ({ \
unsigned i = (0x34 / 4) + ((__p) / 32); \
unsigned s= (__p) % 32; \
!!(LOAD((__r) + i, __ATOMIC_ACQUIRE) & (1 << s));	\
})

struct pinctl {
  unsigned cycle; /* us */
  int fd;
  uint32_t* regs;
};

/* 1 - write GPPUD (94h)
 * 2 - hold 150 cycles
 * 3 - GPPUDCLKxx (98h / 9Ch)
 * 4 - hold 150 cycles
 * 5 - clear GPPUD
 * 6 - clear GPPUDCLKxx
 *
 * FixMe: not sure if the manual's reference to cycle ere is the
 * host's clocking or the target's clock rate.
 */
static void config_hiz(struct pinctl* c, int pin, int sel)
{
  uint32_t val = (sel < 0 ? 0x01 : /* down */
		  sel > 0 ? 0x02 : /* up */
		  0x00);           /* disable */
  int i = pin / 32;
  int s = pin % 32;

  STORE(c->regs + (0x94 / 4), val, __ATOMIC_RELEASE);
usleep(1000);
  STORE(c->regs + (0x98 / 4) + i, (1 << s), __ATOMIC_RELEASE);
usleep(1000);
  STORE(c->regs + (0x94 / 4), 0, __ATOMIC_RELEASE);
usleep(1000);
}

struct pinctl* pins_open(unsigned clock)
{
  /* make sure the clock rate is representable w/ usleep(3) since i'm
   * lazy and linux isn't quite rtos-y.
   */
  unsigned cycle = 1000000 / (2 * clock);
  if (cycle < 10) {
    fprintf(stderr, "%s:%d: clock:%u not representable\n", __func__, __LINE__, clock);
    return NULL;
  }
  printf("%s:%d: cycle:%uus\n", __func__, __LINE__, cycle);
  
  struct pinctl* rv = malloc(sizeof(*rv));
  if (!rv) goto err_exit;
  rv->fd = -1;
  rv->regs = MAP_FAILED;
  rv->cycle = cycle;
  
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

  PIN_DIR(rv->regs, PIN_CLOCK, 1);
  PIN_DIR(rv->regs, PIN_DATA, 0);  

  /* setup high-z configuration.
   *  data - up. the device is supposed to have this set?
   *  clock - down. this is always host controlled?
   */
  config_hiz(rv, PIN_CLOCK, -1);
  config_hiz(rv, PIN_DATA, 1);

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

static void clock_out_bit(struct pinctl* c, int val)
{
  PIN_WRITE(c->regs, PIN_DATA, val);
  PIN_WRITE(c->regs, PIN_CLOCK, 1);
  usleep(c->cycle);
  PIN_WRITE(c->regs, PIN_CLOCK, 0);
  usleep(c->cycle);
}

static uint8_t clock_in_bit(struct pinctl* c)
{
  PIN_WRITE(c->regs, PIN_CLOCK, 1);
  usleep(c->cycle / 2);
  uint8_t rv = PIN_READ(c->regs, PIN_DATA);
  usleep(c->cycle / 2);
  PIN_WRITE(c->regs, PIN_CLOCK, 0);
  usleep(c->cycle);

  return rv;
}

int pins_write(struct pinctl* c, const uint8_t* bs, int nb)
{
  assert(c != NULL);
  assert(c->regs != MAP_FAILED);
  PIN_DIR(c->regs, PIN_DATA, 1);
  usleep(c->cycle * 2);

  int i, j;
  for (i = 0; i < nb / 8; i++)
    for (j = 0; j < 8; j++)
      clock_out_bit(c, bs[i] & (1 << j));
  for (j = 0; j < (nb & 7); j++)
    clock_out_bit(c, bs[i] & (1 << j));
  
  return nb;
}

int pins_read(struct pinctl* c, uint8_t* bs, int nb)
{
  assert(c != NULL);
  assert(c->regs != MAP_FAILED);
  PIN_WRITE(c->regs, PIN_DATA, 0);  
  PIN_DIR(c->regs, PIN_DATA, 0);
  usleep(c->cycle * 2);

  int i, j;
  for (i = 0; i < nb / 8; i++) {
    bs[i] = 0;
    for (j = 0; j < 8; j++)
      bs[i] |= (clock_in_bit(c) << j);
  }
  bs[i] = 0;
  for (j = 0; j < (nb & 7); j++)
    bs[i] |= (clock_in_bit(c) << j);
  
  return nb;
}
