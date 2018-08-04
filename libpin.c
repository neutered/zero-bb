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

#define CYCLE_TIME_US 1000

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
  int fd;
  uint32_t* regs;
};

struct pinctl* pins_open(void)
{
  struct pinctl* rv = malloc(sizeof(*rv));
  if (!rv) goto err_exit;
  rv->fd = -1;
  rv->regs = MAP_FAILED;
  
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
  PIN_WRITE(rv->regs, PIN_CLOCK, 1);
 PIN_DIR(rv->regs, PIN_DATA, 1);
  PIN_WRITE(rv->regs, PIN_DATA, 1);  

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
  usleep(CYCLE_TIME_US);
  PIN_WRITE(c->regs, PIN_CLOCK, 0);
  usleep(CYCLE_TIME_US);
}

int pins_write(struct pinctl* c, const uint8_t* bs, int nb)
{
  assert(c != NULL);
  assert(c->regs != MAP_FAILED);
  PIN_DIR(c->regs, PIN_DATA, 1);

  int i, j;
  for (i = 0; i < nb / 8; i++)
    for (j = 0; j < 8; j++)
      clock_out_bit(c, bs[i] & (1 << j));
  for (j = 0; j < (nb & 7); j++)
    clock_out_bit(c, bs[i] & (1 << j));
  
  return nb;
}
