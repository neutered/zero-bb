#ifndef PINUTIL_H_
#define PINUTIL_H_

/* FixMe: use sysctl to get page size or something, but the current
 * code doesn't care about the size so it doesn't really matter so we
 * go w/ 4k because that sort of works.
 */
#define REG_MAP_SIZE 0x1000

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
!!(LOAD((__r) + i, __ATOMIC_ACQUIRE) & (1 << s)); \
})

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
#define PIN_CONFIG_HIZ(__r, __p, __s) do { \
  uint32_t v = ((__s) < 0 ? 0x01 : /* down */	 \
		(__s) > 0 ? 0x02 : /* up */	 \
		0x00);             /* disable */	 \
  int o = (__p) / 32;					 \
  int s = (__p) % 32;					 \
  STORE((__r) + (0x94 / 4), v, __ATOMIC_RELEASE); \
usleep(10000); \
  STORE((__r) + (0x98 / 4) + o, 1 << s, __ATOMIC_RELEASE); \
usleep(10000); \
  STORE((__r) + (0x98 / 4) + o, 0, __ATOMIC_RELEASE); \
usleep(10000); \
} while(0)

#endif /* PINUTIL_H_ */
