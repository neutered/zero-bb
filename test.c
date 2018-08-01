#include <assert.h>
#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <endian.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <sys/stat.h>

#define __dsb(__v)
#define __dmb(__v)

#define PIN_DIR(__r, __p, __d) do { \
assert((__p) < 54); \
unsigned i = ((__p) / 10); \
unsigned s = 3 * ((__p) % 10); \
__dmb(9); \
uint32_t v = (__r)[i]; \
v &= ~(3 << s); \
v |= ((__d) << s); \
(__r)[i] = v; \
__dmb(10); \
} while (0)

#define PIN_WRITE(__r, __p, __v) do { \
assert((__p) < 54); \
unsigned b = ((__v) ? 0x1c : 0x28) / 4; \
unsigned i = b + ((__p) / 32); \
unsigned s = (__p) % 32; \
(__r)[i] = (1 << s); \
__dmb(10); \
} while(0)

#define PIN_READ(__r, __p) ({ \
unsigned i = (0x34 / 4) + ((__p) / 32); \
unsigned s= (__p) % 32; \
!!((__r)[i] & (1 << s)); \
})

int main(int argc, char** argv)
{
	int rv = 0;

	int fd = open("/dev/gpiomem", O_RDWR);
	assert(fd != -1);

	int fdr = open("/dev/urandom", O_RDONLY);
	assert(fdr != -1);

	uint8_t* p = mmap(0, 0x1000, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
	if (p == MAP_FAILED)
		fprintf(stderr, "%s:%d: mmap():%d:%s\n", __func__, __LINE__, errno, strerror(errno));
	assert(p != MAP_FAILED);
	fprintf(stderr, "%s:%d: p:%p\n", __func__, __LINE__, p);

{
uint32_t* r = (uint32_t*)p;
for (int i = 0; i < 256 / 16; i += 4) {
for (int j = 0; j < 4 && i + j < 256 / 16; j++)
fprintf(stderr, "%08x ", le32toh(r[i + j]));
fprintf(stderr, "\n");
}

PIN_DIR(r, 16, 1);
PIN_DIR(r, 20, 1);
PIN_DIR(r, 21, 1);

for (int i = 0; i < 32; i++) {
uint32_t v;
ssize_t nb = read(fdr, &v, sizeof(v));
assert(nb == sizeof(v));

PIN_DIR(r, 21, 1);
for (int j = 0; j < 32; j++) {
PIN_WRITE(r, 20, 1);
PIN_WRITE(r, 21, v & (1 << j));
usleep(1000);
PIN_WRITE(r, 20, 0);
usleep(1000);
}

PIN_DIR(r, 21, 0);
PIN_WRITE(r, 16, i & 1);
for (int j = 0; j < 8; j++) {
PIN_WRITE(r, 20, 1);
int v = PIN_READ(r, 21);
assert(v == (i & 1));
usleep(1000);
PIN_WRITE(r, 20, 0);
usleep(1000);
}

usleep(100000);
}
}

	rv = munmap(p, 0x1000);
	assert(rv == 0);

	return rv;
}
