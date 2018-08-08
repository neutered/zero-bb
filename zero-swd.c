#include <assert.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "libpin.h"

#define REG_DP_IDCODE 0x00
#define REG_DP_STATUS 0x04
#define REG_DP_SELECT 0x08
#define REG_DP_RDBUFF 0x0c

#define REG_AP_IDR 0xfc

#define CSYSPWRUPACK (1 << 31)
#define CSYSPWRUPREQ (1 << 30)
#define CDBGPWRUPACK (1 << 29)
#define CDBGPWRUPREQ (1 << 28)
#define CDBGRSTACK (1 << 27)
#define CDBGRSTREQ (1 << 26)
#define WDATAERR (1 << 7)
#define READOK (1 << 6)
#define STICKYERR (1 << 5)
#define STICKYCMP (1 << 4)
#define STICKYORUN (1 << 1)
#define ORUNDETECT (1 << 0)

static void resync(struct pinctl* pins, int idle)
{
  static const uint8_t bs[] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00 };
  pins_write(pins, bs, (sizeof(bs) - !idle) * 8);
}

static void jtag2swd(struct pinctl* pins)
{
  /* on the wire format:
   *  0111 1001 1110 0111
   */
  static const uint8_t bs[] = { 0x9e, 0xe7 };
  pins_write(pins, bs, sizeof(bs) * 8);
}

/* on the wire format
 *  1 - start
 *  x - ap == 1, dp == 0
 *  y - write == 0, read == 1
 *  r - register[2]
 *  r - register[3]
 *  p - parity
 *  0 - stop
 *  1 - park
 */
static uint8_t opcode(int ap, int reg, int rw)
{
  uint8_t rv = 0x81;
  if (ap) rv |= 0x02;
  if (rw) rv |= 0x04;
  rv |= (reg & 0x0c) << 1;
  if (__builtin_popcount(rv) & 0x01) rv |= 0x20;
  return rv;
}

/* returns status
 *  [0] - ok
 *  [1] - xxx
 *  [2] - yyy
 */
static uint8_t send_op(struct pinctl* pins, int ap, int reg, int rw)
{
  uint8_t op = opcode(ap, reg, rw);
printf("%s:%d: op:%02x\n", __func__, __LINE__, op);
  pins_write(pins, &op, 8);

  uint8_t status;
  pins_read(pins, &status, 3);
printf("%s:%d: status:%02x\n", __func__, __LINE__, status);
  return status;
}

static int swd_read(struct pinctl* pins, int ap, int reg, uint32_t* val)
{
  uint8_t status = send_op(pins, ap, reg, 1);
  if (status != 1) {
    if (status == 0x04)
      return -EFAULT;
    else if (status == 0x02)
      return -EAGAIN;
    else
      fprintf(stderr, "%s:%d: unhandled status:%02x\n", __func__, __LINE__, status);
    assert(0);
  }

  /* 32-bit data + parity */
  uint8_t bs[4 + 1];
  pins_read(pins, bs, 33);
  int i, n;
  for (i = n = 0; i < 4; i++)
    n += __builtin_popcount(bs[i]);
  if ((n & 1) != (bs[4] & 0x01)) {
    fprintf(stderr, "%s:%d: parity:%02x n:%d\n", __func__, __LINE__, bs[4], n);
    return -EIO;
  }

  /* convert out of lsb-first order */
  if (val)
    *val = ((bs[3] << 24) |
	    (bs[2] << 16) |
	    (bs[1] << 8) |
	    (bs[0] << 0));
  return 0;
}

static int swd_write(struct pinctl* pins, int ap, int reg, uint32_t val)
{
  uint8_t status = send_op(pins, ap, reg, 0);
  fprintf(stderr, "%s:%d: status:%02x\n", __func__, __LINE__, status);
  if (status != 1) {
    if (status == 0x04)
      return -EFAULT;
    else if (status == 0x02)
      return -EAGAIN;
    else
      fprintf(stderr, "%s:%d: unhandled status:%02x\n", __func__, __LINE__, status);
    assert(0);
  }

  /* 32-bit data + parity */
  uint8_t bs[4 + 1];
  bs[0] = (val >> 0) & 0xff;
  bs[1] = (val >> 8) & 0xff;
  bs[2] = (val >> 16) & 0xff;
  bs[3] = (val >> 24) & 0xff;
  bs[4] = __builtin_popcount(val) & 0x01;
  pins_write(pins, bs, 33);

  return 0;
}

struct port_field_desc {
  unsigned hi, lo;
  const char* name;
};

static void dump_reg(const char* f, const struct port_field_desc* fs, unsigned nfs, uint32_t reg)
{
  printf("%s: reg:%08x:", f, reg);
  for (int i = 0; i < nfs; i++) {
    const uint32_t mask = (((1ull << (fs[i].hi + 1)) - 1) &
                           (~0ul << fs[i].lo));
    const uint32_t val = (reg & mask) >> fs[i].lo;
    if (val)
      printf("%s:%x ", fs[i].name, val);
  }
  printf("\n");
}
static void dump_dp_status(uint32_t status)
{
  static const struct port_field_desc fields[] = {
    { 31, 31, "csyspwrupack" },
    { 30, 30, "csyspwrupreq" },
    { 29, 29, "cdbgpwrupack" },
    { 28, 28, "cdbgpwrupreq" },
    { 27, 27, "cdbgrstack" },
    { 26, 26, "cdbgrstreq" },
    { 25, 24, "reserved0" },
    { 23, 12, "trncnt" },
    { 11, 8, "masklane"},
    { 7, 7, "wdataerr" },
    { 6, 6, "readok" },
    { 5, 5, "stickyerr" },
    { 4, 4, "stickycmp" },
    { 3, 2, "trnmode" },
    { 1, 1, "stickyorun" },
    { 0, 0, "orundetect" },
  };
  dump_reg(__func__, fields, sizeof(fields) / sizeof(fields[0]), status);
}

static void dump_ap_status(uint32_t status)
{
  static const struct port_field_desc fields[] = {
    { 31, 31, "dbgswenable" },
    { 30, 24, "prot" },
    { 23, 23, "spiden" },
    { 22, 16, "res0" },
    { 15, 12, "type" },
    { 11, 8, "mode" },
    { 7, 7, "trinprog" },
    { 6, 6, "deviceen" },
    { 5, 4, "addrinc" },
    { 3, 3, "res1" },
    { 2, 0, "size" },
  };
  dump_reg(__func__, fields, sizeof(fields) / sizeof(fields[0]), status);
}

static void dump_ap_idcode(uint32_t idcode)
{
  static const struct port_field_desc fields[] = {
    { 31, 28, "revision" },
    { 27, 24, "continuation" },
    { 23, 17, "identity" },
    { 16, 13, "class" },
    { 12, 8, "reserved" },
    { 7, 4, "variant" },
    { 3, 0, "type" },
  };
  dump_reg(__func__, fields, sizeof(fields) / sizeof(fields[0]), idcode);
}

static int swd_status(struct pinctl* c, uint32_t* status)
{
  int err = swd_read(c, 0, REG_DP_STATUS, status);
  if (err) {
    fprintf(stderr, "%s:%d: swd_read(DP_STATUS):%d:%s\n", __func__, __LINE__, err, strerror(err));
    goto err_exit;
  }
  dump_dp_status(*status);
err_exit:
  return err;
}

static int swd_ap_select(struct pinctl* c, uint8_t sel, uint8_t bank)
{
  uint32_t val = (sel << 24) | (bank << 4);
  return swd_write(c, 0, REG_DP_SELECT, val);
}

static int swd_ap_read(struct pinctl* c, int ap, uint8_t reg, uint32_t* out)
{
  uint8_t bank = (reg >> 4) & 0x0f;
  uint8_t offset = (reg >> 0) & 0x0f;
  int rv = swd_ap_select(c, ap, bank);
assert(rv == 0);
  return swd_read(c, 1, offset, out);
}

static int swd_ap_idcode(struct pinctl* c, int ap, uint32_t* out)
{
  int rv = swd_ap_read(c, ap, REG_AP_IDR, out);
  if (rv == 0)
    dump_ap_idcode(*out);
  return rv;
}

int main(int argc, char** argv)
{
  int rv = EXIT_FAILURE;
  unsigned clock = 1000;
  int opt;

  while ((opt = getopt(argc, argv, "c:")) != -1) {
    switch (opt) {
    case 'c':
      {
	char* end;
	clock = strtoul(optarg, &end, 0);
	assert(end != optarg);
      }
      break;

    default:
      assert(0);
    }
  }

  struct pinctl* pins = pins_open(clock);
  if (!pins) goto err_exit;

  /* reset-y bits for the debug port */
  resync(pins, 0);
  jtag2swd(pins);
  resync(pins, 1);

  /* reading id takes the debug port out of reset */
  uint32_t idcode;
  if ((opt = swd_read(pins, 0, REG_DP_IDCODE, &idcode)) != 0) {
    fprintf(stderr, "%s:%d: swd_read(IDCODE):%d:%s\n", __func__, __LINE__, opt, strerror(opt));
    goto err_exit;
  }
  printf("%s:%d: idcode:%08x\n", __func__, __LINE__, idcode);

  /* set power state */
  opt = swd_write(pins, 0, REG_DP_STATUS, CSYSPWRUPREQ | CDBGPWRUPREQ | CDBGRSTREQ);
  assert(opt == 0);
  do {
    opt = swd_status(pins, &idcode);
    printf("%s:%d: rv:%d:%s status:%08x\n", __func__, __LINE__, opt, strerror(opt), idcode);
assert(opt == 0);
  } while ((idcode & (CSYSPWRUPACK | CDBGPWRUPACK | CDBGRSTACK)) != (CSYSPWRUPACK | CDBGPWRUPACK | CDBGRSTACK));

  /* select the ap, it should be a memory access class */
  opt = swd_write(pins, 0, REG_DP_SELECT, 0 << 24);
assert(opt == 0);

  opt = swd_ap_idcode(pins, 0, &idcode);
  assert(opt == 0);
  assert(((idcode >> 13) & 0x0f) == 0x08);

  rv = EXIT_SUCCESS;
err_exit:
  pins_close(pins);
  return rv;
}
