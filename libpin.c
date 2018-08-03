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

int write_bits(const uint8_t* /* bit stream */, int /* n */);
