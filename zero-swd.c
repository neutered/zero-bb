#include <assert.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <fcntl.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>

#include "libpin.h"

#define REG_DP_IDCODE 0x00
#define REG_DP_STATUS 0x04
#define REG_DP_SELECT 0x08
#define REG_DP_RDBUFF 0x0c

#define REG_AP_IDR 0xfc

#define REG_CPUID 0xe000ed00
#define REG_AIRCR 0xe000ed0c
#define REG_DHCSR 0xe000edf0
#define REG_DEMCR 0xe000edfc

#define REG_FTFL_STAT 0x40020000

#define REG_AP_MEM_CSW 0x00
#define REG_AP_MEM_TAR_LO 0x04
#define REG_AP_MEM_TAR_HI 0x08
#define REG_AP_MEM_DRW 0x0c
#define REG_AP_MEM_BD0 0x10
#define REG_AP_MEM_BD1 0x14
#define REG_AP_MEM_BD2 0x18
#define REG_AP_MEM_BD3 0x1c
#define REG_AP_MEM_MBT 0x20
#define REG_AP_MEM_BASE_HI 0xf0
#define REG_AP_MEM_CFG 0xf4
#define REG_AP_MEM_BASE_LO 0xf8

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

static int verbose;

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
static int send_op(struct pinctl* pins, int ap, int reg, int rw)
{
  uint8_t op = opcode(ap, reg, rw);
  if (verbose > 1)
    fprintf(stderr, "%s:%d: ap:%d reg:%02x rw:%d op:%02x\n", __func__, __LINE__, ap, reg, rw, op);

  for (int i = 0; i < 4; i++) {
    pins_write(pins, &op, 8);

    uint8_t status;
    pins_read(pins, &status, 3);
    if (verbose > 2)
      fprintf(stderr, "%s:%d: try:%d status:%02x\n", __func__, __LINE__, i, status);
    switch (status) {
    case 1:
      return 0;
    case 2:
      /* FixMe: what's a good delay? does it depend on the outstanding op? */
      usleep(10000);
      break;
    case 4:
      return EFAULT;
    default:
      fprintf(stderr, "%s:%d: unhandled status:%02x\n", __func__, __LINE__, status);
      assert(0);
      return EIO;
    }
  }

  return EAGAIN;
}

static int swd_read(struct pinctl* pins, int ap, int reg, uint32_t* val)
{
  int rv = send_op(pins, ap, reg, 1);
  if (rv != 0) return rv;

  /* 32-bit data + parity */
  uint8_t bs[4 + 1];
  pins_read(pins, bs, 33);
  int i, n;
  for (i = n = 0; i < 4; i++)
    n += __builtin_popcount(bs[i]);
  if ((n & 1) != (bs[4] & 0x01)) {
    fprintf(stderr, "%s:%d: parity:%02x n:%d\n", __func__, __LINE__, bs[4], n);
    return EIO;
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
  int rv = send_op(pins, ap, reg, 0);
  if (rv != 0) return rv;

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

static void dump_reg(const char* func, const char* label, const struct port_field_desc* fs, unsigned nfs, uint32_t reg)
{
  fprintf(stderr, "%s: %s:%08x:", func, label, reg);
  for (int i = 0; i < nfs; i++) {
    const uint32_t mask = (((1ull << (fs[i].hi + 1)) - 1) &
                           (~0ul << fs[i].lo));
    const uint32_t val = (reg & mask) >> fs[i].lo;
    if (val)
      fprintf(stderr, "%s:%x ", fs[i].name, val);
  }
  fprintf(stderr, "\n");
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
  dump_reg(__func__, "dp_status", fields, sizeof(fields) / sizeof(fields[0]), status);
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
  dump_reg(__func__, "ap_status", fields, sizeof(fields) / sizeof(fields[0]), status);
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
  dump_reg(__func__, "ap_idcode", fields, sizeof(fields) / sizeof(fields[0]), idcode);
}

static void dump_ap_mem_csw(uint32_t val)
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
  dump_reg(__func__, "ap_mem_csw", fields, sizeof(fields) / sizeof(fields[0]), val);
}

static void dump_ap_mem_cfg(uint32_t val)
{
  static const struct port_field_desc fields[] = {
    { 31, 3, "res0" },
    { 2, 2, "ld" },
    { 1, 1, "la" },
    { 0, 0, "be" },
  };
  dump_reg(__func__, "ap_mem_cfg", fields, sizeof(fields) / sizeof(fields[0]), val);
}

static void dump_reg_cpuid(const char* label, uint32_t val)
{
  static const struct port_field_desc fields[] = {
    { 31, 24, "implementer" },
    { 23, 20, "variant" },
    { 19, 16, "fixed" },
    { 15, 4, "partno" },
    { 3, 0, "revision" },
  };
  dump_reg(__func__, label, fields, sizeof(fields) / sizeof(fields[0]), val);
}

static void dump_reg_dhcsr(const char* label, uint32_t val)
{
  static const struct port_field_desc fields[] = {
    { 31, 26, "reserved0" }, // top part of dbg key for writes
    { 25, 25, "s_reset_st" },
    { 24, 24, "s_retire_st" },
    { 23, 18, "reserved1" },
    { 17, 17, "s_halt" },
    { 16, 16, "s_regrdy" },
    { 15, 4, "reserved2" },
    { 3, 3, "c_maskints" },
    { 2, 2, "c_step" },
    { 1, 1, "c_halt" },
    { 0, 0, "c_debugen" },
  };
  dump_reg(__func__, label, fields, sizeof(fields) / sizeof(fields[0]), val);
}

static void dump_reg_demcr(const char* label, uint32_t val)
{
  static const struct port_field_desc fields[] = {
    { 31, 25, "reserved0" },
    { 24, 24, "dwtena" },
    { 23, 11, "reseerved1" },
    { 10, 10, "vc_harderr" },
    { 9, 1, "reserved2" },
    { 0, 0, "vc_corereset" },
  };
  dump_reg(__func__, label, fields, sizeof(fields) / sizeof(fields[0]), val);
}

static void dump_reg_aircr(const char* label, uint32_t val)
{
  static const struct port_field_desc fields[] = {
    { 31, 16, "vectkey" },
    { 15, 15, "endianess" },
    { 14, 3, "reserved0" },
    { 2, 2, "sysresetreq" },
    { 1, 1, "vectclractive" },
    { 0, 0, "reserved1" },
  };
  dump_reg(__func__, label, fields, sizeof(fields) / sizeof(fields[0]), val);
}

static void hexdump(const char* tag, const uint8_t* bs, size_t nb)
{
  assert(strlen(tag) > 0);
  for (int i = 0; i < nb; i += 16) {
    fprintf(stderr, "%s %04x:", tag, i);
    for (int j = 0; j < 16 && i + j < nb; j++)
      fprintf(stderr, "%02x ", bs[i + j]);
    fprintf(stderr, "\n");
  }
}

static void dump_mem(uint64_t addr, const uint8_t* bs, size_t nb)
{
  char backing[16 + 1];
  const char* label = backing;
  void (*reg)(const char*, uint32_t) = NULL;
  if (nb == 4) {
    switch (addr) {
    case REG_CPUID: label = "cpuid"; reg = dump_reg_cpuid; break;
    case REG_AIRCR: label = "aircr"; reg = dump_reg_aircr; break;
    case REG_DHCSR: label = "dhcsr"; reg = dump_reg_dhcsr; break;
    case REG_DEMCR: label = "demcr"; reg = dump_reg_demcr; break;
    default: goto label_addr;
    }
  } else {
label_addr:
    if (addr & 0xffffffff00000000)
      sprintf(backing, "%016llx", addr);
    else
      sprintf(backing, "%08x", (uint32_t)addr);
  }
  hexdump(label, bs, nb);
  if (reg)
    (*reg)(label, *(uint32_t*)bs);
}

static int swd_status(struct pinctl* c, uint32_t* status)
{
  int err = swd_read(c, 0, REG_DP_STATUS, status);
  if (err) {
    fprintf(stderr, "%s:%d: swd_read(DP_STATUS):%d:%s\n", __func__, __LINE__, err, strerror(err));
    goto err_exit;
  }
  if (verbose)
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
  int rv;

  rv = swd_ap_select(c, ap, bank);
  assert(rv == 0);
  if (rv != 0) return rv;

  /* ap reads are posted so we issue the read and then use the dp
   * RDBUFF to get the read, to avoid possibly auto incroementing on
   * ap mem accesses.
   */
  rv = swd_read(c, 1, offset, out);
  assert(rv == 0);
  if (rv != 0) return rv;

  rv = swd_read(c, 0, REG_DP_RDBUFF, out);
  assert(rv == 0);
  if (rv != 0) return rv;

  if (verbose > 1)
    fprintf(stderr, "%s:%d: ap:%02x:%08x\n", __func__, __LINE__, reg, *out);
  return 0;
}

static int swd_ap_write(struct pinctl* c, int ap, uint8_t reg, uint32_t val)
{
  uint8_t bank = (reg >> 4) & 0x0f;
  uint8_t offset = (reg >> 0) & 0x0f;
  int rv = swd_ap_select(c, ap, bank);
  if (rv != 0) return rv;
  return swd_write(c, 1, offset, val);
}

static int swd_ap_idcode(struct pinctl* c, int ap, uint32_t* out)
{
  int rv = swd_ap_read(c, ap, REG_AP_IDR, out);
  if (verbose && (rv == 0))
    dump_ap_idcode(*out);
  return rv;
}

static int swd_ap_mem_read(struct pinctl* c, int ap, uint64_t addr, uint8_t* bs, size_t nb)
{
  const uint8_t* const orig_bs = bs;
  const size_t orig_nb = nb;

  uint32_t cfg, csw;
  int err;

  err = swd_ap_read(c, ap, REG_AP_MEM_CFG, &cfg);
  assert(err == 0);
  if (verbose)
    dump_ap_mem_cfg(cfg);
  err = swd_ap_read(c, ap, REG_AP_MEM_CSW, &csw);
  assert(err == 0);
  if (verbose)
    dump_ap_mem_csw(csw);

  /* set 32-bit transfer size and auto-increment on access */
  err = swd_ap_write(c, ap, REG_AP_MEM_CSW, (csw & ~((3 << 4) | (7 << 0))) | (1 << 4) | (2 << 0));
  assert(err == 0);

  /* the DWR access always returns 32-bits regardless of the size set
   * in CSW (ie, not a single byte if that's set). align the start/end
   * addresses and the device aligned bits regardless of the host
   * address alignment.
   */
  unsigned slop_head = (addr & 0x03);
  if (slop_head) {
    nb -= (4 - slop_head);
    addr &= ~0x03;
  }
  unsigned slop_tail = (nb & 0x03);
  assert((addr & 0x03) == 0);
  err = swd_ap_write(c, ap, REG_AP_MEM_TAR_LO, (addr >> 0) & 0xffffffff);
  assert(err == 0);
  if (cfg & 0x02) {
    err = swd_ap_write(c, ap, REG_AP_MEM_TAR_HI, (addr >> 32) & 0xffffffff);
    assert(err == 0);
  }

  uint32_t val;
  if (slop_head) {
    err = swd_ap_read(c, ap, REG_AP_MEM_DRW, &val);
    assert(err == 0);
    switch (slop_head) {
    case 1: *bs++ = (val >> 8) & 0xff;
    case 2: *bs++ = (val >> 16) & 0xff;
    case 3: *bs++ = (val >> 24) & 0xff;
      break;

    default:
      assert(0);
    }
  }

  for (int i = 0; i < nb / 4; i++) {
    err = swd_ap_read(c, ap, REG_AP_MEM_DRW, &val);
    assert(err == 0);
    *bs++ = (val >> 0) & 0xff;
    *bs++ = (val >> 8) & 0xff;
    *bs++ = (val >> 16) & 0xff;
    *bs++ = (val >> 24) & 0xff;
  }

  if (slop_tail) {
    err = swd_ap_read(c, ap, REG_AP_MEM_DRW, &val);
    assert(err == 0);
    switch (slop_tail) {
      case 3: *bs++ = (val >> 8) & 0xff;
      case 2: *bs++ = (val >> 16) & 0xff;
      case 1: *bs++ = (val >> 24) & 0xff;
        break;

      default:
        assert(0);
    }
  }

  if (verbose)
    dump_mem(addr, orig_bs, orig_nb);

  return 0;
}

/* NB: read-modify-write operations for the unaligned bits, if any. */
static int swd_ap_mem_write(struct pinctl* c, int ap, uint64_t addr, const uint8_t* bs, size_t nb)
{
  uint32_t cfg, csw;
  int err;

  err = swd_ap_read(c, ap, REG_AP_MEM_CFG, &cfg);
  assert(err == 0);
  if (verbose)
    dump_ap_mem_cfg(cfg);
  err = swd_ap_read(c, ap, REG_AP_MEM_CSW, &csw);
  assert(err == 0);
  if (verbose)
    dump_ap_mem_csw(csw);

  /* the DWR access always returns 32-bits regardless of the size set
   * in CSW (ie, not a single byte if that's set). align the start/end
   * addresses and the device aligned bits regardless of the host
   * address alignment.
   */
  unsigned slop_head = (addr & 0x03);
  if (slop_head) {
    nb -= (4 - slop_head);
    addr &= ~0x03;
  }
  unsigned slop_tail = (nb & 0x03);
  assert((addr & 0x03) == 0);
  err = swd_ap_write(c, ap, REG_AP_MEM_TAR_LO, (addr >> 0) & 0xffffffff);
  assert(err == 0);
  if (cfg & 0x02) {
    err = swd_ap_write(c, ap, REG_AP_MEM_CSW, (csw & ~((3 << 4) | (7 << 0))) | (2 << 0));
    assert(err == 0);

    err = swd_ap_write(c, ap, REG_AP_MEM_TAR_HI, (addr >> 32) & 0xffffffff);
    assert(err == 0);
  }

  uint32_t val;
  if (slop_head) {
    err = swd_ap_read(c, ap, REG_AP_MEM_DRW, &val);
    assert(err == 0);
    switch (slop_head) {
    case 1: val &= ~(0xff << 8); val |= (*bs++ << 8);
    case 2: val &= ~(0xff << 16); val |= (*bs++ << 16);
    case 3: val &= ~(0xff << 24); val |= (*bs++ << 24);
      break;

    default:
      assert(0);
    }
    err = swd_ap_write(c, ap, REG_AP_MEM_DRW, val);
    assert(err == 0);
  }

  /* set 32-bit transfer size and auto-increment on access for any
   * aligned bits.
   */
  if (nb / 4) {
    err = swd_ap_write(c, ap, REG_AP_MEM_CSW, (csw & ~((3 << 4) | (7 << 0))) | (1 << 4) | (2 << 0));
    assert(err == 0);

    for (int i = 0; i < nb / 4; i++, bs+=4) {
      /* local copy to avoid unaligned accesses on the host */
      memcpy(&val, bs, 4);
      err = swd_ap_write(c, ap, REG_AP_MEM_DRW, val);
      assert(err == 0);
    }
  }

  if (slop_tail) {
    err = swd_ap_write(c, ap, REG_AP_MEM_CSW, (csw & ~((3 << 4) | (7 << 0))) | (2 << 0));
    assert(err == 0);

    err = swd_ap_read(c, ap, REG_AP_MEM_DRW, &val);
    assert(err == 0);
    switch (slop_tail) {
    case 3: val &= ~(0xff << 8); val |= (*bs++ << 8);
    case 2: val &= ~(0xff << 16); val |= (*bs++ << 16);
    case 1: val &= ~(0xff << 24); val |= (*bs++ << 24);
      break;

    default:
      assert(0);
    }
    err = swd_ap_write(c, ap, REG_AP_MEM_DRW, val);
    assert(err == 0);
  }

  return 0;
}

static int swd_ap_mem_read_u32(struct pinctl* c, int ap, uint64_t addr, uint32_t* val)
{
  return swd_ap_mem_read(c, 0, addr, (uint8_t*)val, sizeof(*val));
}

static int swd_ap_mem_write_u32(struct pinctl* c, int ap, uint64_t addr, const uint32_t val)
{
  return swd_ap_mem_write(c, 0, addr, (uint8_t*)&val, sizeof(val));
}

/*  ftfl registers are (oddly) 8-bit, but the swd bits seem happier w/
 * 32-bit accesses (ie, always returning the word even when the config
 * is set for 8-bit accesses).
 *
 * [0] - command
 * [1] - addr[23:16]
 * [2] - addr[15:8]
 * [3] - addr[7:0]
 * [4] - param[7:0]
 * ...
 * [11] - param[63:54]
 */
static int swd_ftfl_status(struct pinctl* c, uint8_t regs[4])
{
  int err = swd_ap_mem_read(c, 0, REG_FTFL_STAT, regs, 4);
  assert(err == 0);
  assert((regs[0] & (7 << 1)) == 0);
  /* early out on the 'common' done/no-error case */
  if ((regs[0] & 0xF0) == (1 << 7))
    return 0;
  if (!(regs[0] & (1 << 7)))
    return EBUSY;
  if (regs[0] & (1 << 6))
    return EXDEV;
  if (regs[0] & (1 << 5))
    return EFAULT;
  if (regs[0] & (1 << 4))
    return EACCES;
  assert(0);
}

static int swd_ftfl_issue(struct pinctl* c, uint8_t op, uint32_t addr, uint8_t param[8])
{
  /* only 24-bits of address bits and aligned. some commands have
   * higher alignment restrictions, but this is the least common
   * denominator.
   */
  assert((addr & 0xff000000) == 0);

  int err;
  uint8_t regs[16];
  static const uint8_t fccob_map[] = {
    0x07, 0x06, 0x05, 0x04,
    0x0b, 0x0a, 0x09, 0x08,
    0x0f, 0x0e, 0x0d, 0x0c,
  };
  int fccob_read = 0;

  /* wait for FSWTAT fields ACCERR(0) / FPIOL(0) / CCIF(1). this
   * assumes that we're the only ones accessing the flash since the
   * processor should be in the halt state.
   */
  for (int i = 0; i < 10; i++) {
    err = swd_ftfl_status(c, regs);

    /* CCIF only means ready. */
    if (!err) break;
    if (verbose)
      fprintf(stderr, "%s:%d: %d err:%d:%s status:%02x\n", __func__, __LINE__, i, err, strerror(err), regs[0]);

    /* if error, clear the state by writing both teh bits regardless
     * of which one might be set.
     */
    if (regs[0] & ((1 << 5) | (1 << 4))) {
      regs[0] = ((1 << 5) | (1 << 4));
      err = swd_ap_mem_write(c, 0, REG_FTFL_STAT, regs, sizeof(regs));
      assert(err == 0);
    }
    /* we may be waiting for an other command to complete */
usleep(10000);
  }

  /* we failed to clear the error state or that last command is 'long
   * running' (erase?) so that ready never got set.
   */
  if (err) return err;

  memset(regs + 4, 0, sizeof(regs) - 4);
  regs[fccob_map[0x00]] = op;

  /* set 'addr' words for ops htat use them */
  switch (op) {
  case 0x00:
  case 0x01:
  case 0x02:
  case 0x03:
  case 0x06:
  case 0x08:
  case 0x09:
  case 0x0b:
  case 0x46:
    regs[fccob_map[0x01]] = (addr >> 16) & 0xff;
    regs[fccob_map[0x02]] = (addr >> 8) & 0xff;
    regs[fccob_map[0x03]] = (addr >> 0) & 0xff;
    break;

  default:
    fprintf(stderr, "%s:%d: unknown ftfl op:%02x\n", __func__, __LINE__, op);
    assert(0);
  case 0x40:
  case 0x41:
  case 0x43:
  case 0x44:
  case 0x45:
  case 0x80:
  case 0x81:
    break;
  }

  /* fill in the rest of the args */
  switch (op) {
  case 0x00:
    regs[fccob_map[0x04]] = param[0]; /* margin */
    break;
  case 0x01:
    regs[fccob_map[0x04]] = param[0]; /* num[15:8] */
    regs[fccob_map[0x05]] = param[1]; /* num[7:0] */
    regs[fccob_map[0x06]] = param[2]; /* margin */
    break;
  case 0x02:
    regs[fccob_map[0x04]] = param[0]; /* margin */
    regs[fccob_map[0x08]] = param[1]; /* expected[0] */
    regs[fccob_map[0x09]] = param[2]; /* expected[1] */
    regs[fccob_map[0x0a]] = param[3]; /* expected[2] */
    regs[fccob_map[0x0b]] = param[4]; /* expected[3] */
    fccob_read = 1;
    break;
  case 0x41:
    regs[fccob_map[0x01]] = param[0]; /* record index */
  case 0x03:
    regs[fccob_map[0x08]] = param[0]; /* resource selector */
    fccob_read = 1;
    break;
  case 0x06:
    regs[fccob_map[0x04]] = param[0]; /* data[0] */
    regs[fccob_map[0x05]] = param[1]; /* data[1] */
    regs[fccob_map[0x06]] = param[2]; /* data[2] */
    regs[fccob_map[0x07]] = param[3]; /* data[3] */
    break;
  case 0x43:
    regs[fccob_map[0x01]] = param[0]; /* record index */
    regs[fccob_map[0x04]] = param[1]; /* data[0] */
    regs[fccob_map[0x05]] = param[2]; /* data[1] */
    regs[fccob_map[0x06]] = param[3]; /* data[2] */
    regs[fccob_map[0x07]] = param[4]; /* data[3] */
    break;
  case 0x0b:
    regs[fccob_map[0x04]] = param[0]; /* n[15:8] */
    regs[fccob_map[0x05]] = param[1]; /* n[7:0] */
    break;
  case 0x46:
    regs[fccob_map[0x04]] = param[0]; /* swap control code */
    fccob_read = 1;
    break;
  case 0x40:
    regs[fccob_map[0x01]] = param[0]; /* margin */
    break;
  case 0x08:
  case 0x09:
  case 0x44:
    break;
  case 0x45:
    regs[fccob_map[0x04]] = param[0]; /* data[0] */
    regs[fccob_map[0x05]] = param[1]; /* data[1] */
    regs[fccob_map[0x06]] = param[2]; /* data[2] */
    regs[fccob_map[0x07]] = param[3]; /* data[3] */
    regs[fccob_map[0x08]] = param[4]; /* data[4] */
    regs[fccob_map[0x09]] = param[5]; /* data[5] */
    regs[fccob_map[0x0a]] = param[6]; /* data[6] */
    regs[fccob_map[0x0b]] = param[7]; /* data[7] */
  case 0x80:
    regs[fccob_map[0x04]] = param[0]; /* eeprom size code */
    regs[fccob_map[0x05]] = param[1]; /* partition code */
    break;
  case 0x81:
    regs[fccob_map[0x01]] = param[0]; /* function control code */
    break;
  }

  /* params and then issue by writing FSTAT[CCIF] */
  err = swd_ap_mem_write(c, 0, REG_FTFL_STAT + 4, regs + 4, sizeof(regs) - 4);
  assert(err == 0);
  err = swd_ap_mem_write(c, 0, REG_FTFL_STAT, regs + 0, 4);
  assert(err == 0);

  /* FixMe: do we need/want to wait for loner running commands?
   * currently, things that return results are fast enough
   * that the flash Txxx is taken up w/ the swd transactions.
   * that might not hold for 'erase all'.
   */
  err = swd_ftfl_status(c, regs + 0);
  assert(err == 0);
  if ((err == 0) && fccob_read) {
    switch (op) {
    case 0x03:
    case 0x41:
    case 0x46:
      err = swd_ap_mem_read(c, 0, REG_FTFL_STAT + 4, regs + 4, sizeof(regs) - 4);
      assert(err == 0);
      break;
    default:
      assert(0);
    case 0x02:
      /* we get the state already in the status register */
      break;
    }

    switch (op) {
    case 0x02:
      param[0] = regs[0] & 0x01;
      break;
    case 0x03:
    case 0x41:
      param[0] = regs[fccob_map[0x04]]; /* data[31:24] */
      param[1] = regs[fccob_map[0x05]]; /* data[23:16] */
      param[2] = regs[fccob_map[0x06]]; /* data[15:8] */
      param[3] = regs[fccob_map[0x07]]; /* data[7:0] */
      break;
    case 0x46:
      param[0] = regs[fccob_map[0x05]]; /* swap state */
      param[1] = regs[fccob_map[0x06]]; /* swap config */
      break;
    default:
      assert(0);
      break;
    }
  }

  return err;
}

int main(int argc, char** argv)
{
  int rv = EXIT_FAILURE;
  unsigned clock = 1000;
  int opt;
  uint64_t mem_addr;
  uint32_t mem_nb = 0;
  int fd_out = -1;

  while ((opt = getopt(argc, argv, "c:o:r::v")) != -1) {
    char* end;

    switch (opt) {
    case 'c':
      clock = strtoul(optarg, &end, 0);
      assert(end != optarg);
      break;
    case 'o':
      fd_out = open(optarg, O_WRONLY | O_CREAT | O_TRUNC);
      if (fd_out == -1)
        fprintf(stderr, "%s:%d: output(%s) failed:%d:%s\n", __func__, __LINE__, optarg, errno, strerror(errno));
      assert(fd_out != -1);
      break;
    case 'r':
      if (optarg == NULL) {
        mem_addr = 0;
        mem_nb = 0x1000;
      } else {
        mem_addr = strtoull(optarg, &end, 0);
        assert(end != optarg);
        assert((mem_addr & 0x03) == 0);
        if (*end == '\0') break;
        optarg = end + 1;
        mem_nb = strtoul(optarg, &end, 0);
        assert(end != optarg);
        assert((mem_nb & 0x03) == 0);
      }
      break;
    case 'v':
      verbose++;
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
  uint32_t val;
  if ((opt = swd_read(pins, 0, REG_DP_IDCODE, &val)) != 0) {
    fprintf(stderr, "%s:%d: swd_read(IDCODE):%d:%s\n", __func__, __LINE__, opt, strerror(opt));
    goto err_exit;
  }
  printf("%s:%d: idcode:%08x\n", __func__, __LINE__, val);

  /* set power state */
  opt = swd_write(pins, 0, REG_DP_STATUS, CSYSPWRUPREQ | CDBGPWRUPREQ | CDBGRSTREQ);
  assert(opt == 0);
  do {
    opt = swd_status(pins, &val);
    assert(opt == 0);
  } while ((val & (CSYSPWRUPACK | CDBGPWRUPACK | CDBGRSTACK)) != (CSYSPWRUPACK | CDBGPWRUPACK | CDBGRSTACK));

  /* it should be a memory access class */
  opt = swd_ap_idcode(pins, 0, &val);
  assert(opt == 0);
  assert(((val >> 13) & 0x0f) == 0x08);

  /* it doesn't matter about the cpuid, but it's informational */
  opt = swd_ap_mem_read_u32(pins, 0, REG_CPUID, &val);
  assert(opt == 0);

  /* halt the processor before futzing w/ flash. just for yucks, we do
   * the double read to check if it's still in reset.
   */
  opt = swd_ap_mem_read_u32(pins, 0, REG_DHCSR, &val);
  assert(opt == 0);
  if (val & (1 << 25)) {
    opt = swd_ap_mem_read_u32(pins, 0, REG_DHCSR, &val);
    assert(opt == 0);
    if (val & (1 << 25))
      printf("%s:%d: held in reset?\n", __func__, __LINE__);
  }

  /* halt and enable debug */
  if (!(val & (1 << 17))) {
    val = (val & 0x0000ffff) | (0xa05f << 16) | (1 << 1) | (1 << 0);
    opt = swd_ap_mem_write_u32(pins, 0, REG_DHCSR, val);
    assert(opt == 0);
    int i;
    for (i = 0; i < 8; i++) {
      usleep(250000);
      opt = swd_ap_mem_read_u32(pins, 0, REG_DHCSR, &val);
      assert(opt == 0);
      if (val & (1 << 17)) break;
    }
    printf("%s:%d: halt:%d dhcsr:%08x\n", __func__, __LINE__, i, val);
  }

  /* set to hold after reset and hit the reset */
  opt = swd_ap_mem_read_u32(pins, 0, REG_DEMCR, &val);
  assert(opt == 0);
  opt = swd_ap_mem_write_u32(pins, 0, REG_DEMCR, val | (1 << 0));
  assert(opt == 0);
  opt = swd_ap_mem_read_u32(pins, 0, REG_AIRCR, &val);
  assert(opt == 0);
  opt = swd_ap_mem_write_u32(pins, 0, REG_AIRCR, val | (1 << 2));
  assert(opt == 0);
  opt = swd_ap_mem_read_u32(pins, 0, REG_DHCSR, &val);
  assert(opt == 0);

  uint8_t bs[256];
  while (mem_nb > 0) {
#define min(a, b) ((a) < (b) ? (a) : (b))
    uint32_t nb = min(mem_nb, sizeof(bs));
    opt = swd_ap_mem_read(pins, 0, mem_addr, bs, nb);
    assert(opt == 0);
    if (fd_out == -1)
      hexdump("mem", bs, sizeof(bs));
    else
      write(fd_out, bs, nb);
    mem_addr += nb;
    mem_nb -= nb;
  }
  if (fd_out != -1) close(fd_out);

  for (val = 0; /**/; val++) {
    if ((val & 0xff) == 0) fprintf(stderr, "%s:%d: test val:%08x\n", __func__, __LINE__, val);
  bs[0] = 0x01;
  memcpy(bs + 1, &val, sizeof(val));
  opt = swd_ftfl_issue(pins, 0x02, 0, bs);
assert(opt == 0);
if (!bs[0]) {
  fprintf(stderr, "%s:%d: val:%08x\n", __func__, __LINE__, val);
  break;
}
  }

  rv = EXIT_SUCCESS;
err_exit:
  pins_close(pins);
  return rv;
}
