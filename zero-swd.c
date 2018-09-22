#include <assert.h>
#include <ctype.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <fcntl.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>

#include "libpin.h"
#include "sha256.h"

#define min(a, b) ((a) < (b) ? (a) : (b))
#define max(a, b) ((a) > (b) ? (a) : (b))

#define REG_DP_IDCODE 0x00
#define REG_DP_STATUS 0x04
#define REG_DP_SELECT 0x08
#define REG_DP_RDBUFF 0x0c

/* dp status register fields */
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

#define REG_AP_IDR 0xfc

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

/* arm 'memory mapped' registers */
#define REG_CPUID 0xe000ed00
#define REG_AIRCR 0xe000ed0c
#define REG_DFSR  0xe000ed30
#define REG_DHCSR 0xe000edf0
#define REG_DCRSR 0xe000edf4
#define REG_DCRDR 0xe000edf8
#define REG_DEMCR 0xe000edfc

#define REG_FTFL_STAT 0x40020000

struct memdesc {
#define DESC_WRITE (1 << 0)
#define DESC_PATH  (1 << 1)
  uint32_t flags;
  uint64_t addr;
  union {
    uint32_t val;
    size_t nb;
  };
  const char* path;
};

static struct {
  const char* s;
  size_t nb;
} reg_sel_names[] = {
#define x(s) { s, sizeof(s) - 1 }
  [ 0x0d ] = x("sp"),
  [ 0x0e ] = x("lr"),
  [ 0x0f ] = x("pc"),
  [ 0x10 ] = x("xpsr"),
  [ 0x11 ] = x("msp"),
  [ 0x12 ] = x("psp"),
  [ 0x14 ] = x("control"),
#undef x
};
#define n_reg_sel_names (sizeof(reg_sel_names) / sizeof(reg_sel_names[0]))

static int reg2sel(const char* r)
{
  char* end;
  unsigned sel = strtoul(r, &end, 0);
  if (end != r) {
    assert(sel < n_reg_sel_names);
    assert(*end == 0 || *end == ':');
    return sel;
  }
  for (int i = 0; i < n_reg_sel_names; i++) {
    if (!reg_sel_names[i].nb) continue;
    if (strncmp(r, reg_sel_names[i].s, reg_sel_names[i].nb) == 0) {
      end = (char*)(r + reg_sel_names[i].nb);
      assert(*end == 0 || *end == ':');
      return i;
    }
  }
  return -1;
}

static int verbose;

/* if dump | verbose parse status register bits */
static int swd_status(struct pinctl* c, uint32_t* status, int dump);

static void idle(struct pinctl* pins)
{
  #if 0
static const uint8_t bs[] = { 0x00, 0x00, 0x00, 0x00, };
pins_write(pins, bs, (sizeof(bs)) * 8);
#else
usleep(100000);
#endif
}

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
static int send_op(struct pinctl* c, int ap, int reg, int rw)
{
  uint8_t op = opcode(ap, reg, rw);
  if (verbose > 1)
    fprintf(stderr, "%s:%d: ap:%d reg:%02x rw:%d op:%02x\n", __func__, __LINE__, ap, reg, rw, op);

  for (int i = 0; i < 4; i++) {
    pins_write(c, &op, 8);

    uint8_t status;
    pins_read(c, &status, 3);
    if (verbose > 2)
      fprintf(stderr, "%s:%d: try:%d status:%02x\n", __func__, __LINE__, i, status);
    switch (status) {
    case 1:
      return 0;
    case 2:
      /* FixMe: what's a good delay? does it depend on the outstanding op? */
      idle(c);
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

static int internal_swd_read(struct pinctl* c, int ap, int reg, uint32_t* val)
{
  int rv = send_op(c, ap, reg, 1);
  if (rv != 0) return rv;

  /* 32-bit data + parity */
  uint8_t bs[4 + 1];
  pins_read(c, bs, 33);
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

static int internal_swd_write(struct pinctl* pins, int ap, int reg, uint32_t val)
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

static int swd_handle_fail(struct pinctl* c)
{
  uint32_t val;
  int rv = swd_status(c, &val, 1);
  assert(rv == 0);
  fprintf(stderr, "%s:%d: status:%08x\n", __func__, __LINE__, val);
  /* sticky errors? */
  uint32_t reg = 0x01;
  assert((val & 0xb2) != 0);
  if (val & 0x80) reg |= (1 << 3);
  if (val & 0x20) reg |= (1 << 2);
  if (val & 0x10) reg |= (1 << 1);
  rv = internal_swd_write(c, 0, 0, reg);
  assert(rv == 0);
  return rv;
}

static int swd_read(struct pinctl* c, int ap, int reg, uint32_t* val)
{
  int retry = 0;
  int rv;
retry:
  rv = internal_swd_read(c, ap, reg, val);
  switch (rv) {
  case EFAULT:
    fprintf(stderr, "%s:%d: %d:%02x fault retry:%d\n", __func__, __LINE__, ap, reg, retry);
    if (retry) return rv;
    retry = 1;
    rv = swd_handle_fail(c);
    assert(rv == 0);
    goto retry;

  default:
    return rv;
  }
}

static int swd_write(struct pinctl* c, int ap, int reg, uint32_t val)
{
  int retry = 0;
  int rv;
retry:
  rv = internal_swd_write(c, ap, reg, val);
  switch (rv) {
  case EFAULT:
    fprintf(stderr, "%s:%d: %d:%02x fault retry:%d\n", __func__, __LINE__, ap, reg, retry);
    if (retry) return rv;
    retry = 1;
    rv = swd_handle_fail(c);
    assert(rv == 0);
    goto retry;

  default:
    return rv;
  }
}

struct port_field_desc {
  unsigned hi, lo;
  const char* name;
};

static void dump_reg_fields(const char* func, const char* label, const struct port_field_desc* fs, unsigned nfs, uint32_t reg)
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
  dump_reg_fields(__func__, "dp_status", fields, sizeof(fields) / sizeof(fields[0]), status);
}

static void dump_dp_idcode(uint32_t val)
{
  static const struct port_field_desc fields[] = {
    { 31, 28, "revision" },
    { 27, 20, "partno" },
    { 19, 17, "reserved0" },
    { 16, 16, "min" },
    { 15, 12, "version" },
    { 11, 1, "designer" },
    { 0, 0, "rao" },
  };
  dump_reg_fields(__func__, "dp_idcode", fields, sizeof(fields) / sizeof(fields[0]), val);
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
  dump_reg_fields(__func__, "ap_idcode", fields, sizeof(fields) / sizeof(fields[0]), idcode);
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
  dump_reg_fields(__func__, "ap_mem_csw", fields, sizeof(fields) / sizeof(fields[0]), val);
}

static void dump_ap_mem_cfg(uint32_t val)
{
  static const struct port_field_desc fields[] = {
    { 31, 3, "res0" },
    { 2, 2, "ld" },
    { 1, 1, "la" },
    { 0, 0, "be" },
  };
  dump_reg_fields(__func__, "ap_mem_cfg", fields, sizeof(fields) / sizeof(fields[0]), val);
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
  dump_reg_fields(__func__, label, fields, sizeof(fields) / sizeof(fields[0]), val);
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
  dump_reg_fields(__func__, label, fields, sizeof(fields) / sizeof(fields[0]), val);
}

static void dump_reg_demcr(const char* label, uint32_t val)
{
  static const struct port_field_desc fields[] = {
    { 31, 25, "reserved0" },
    { 24, 24, "dwtena" },
    { 23, 20, "reseerved1" },
    { 19, 19, "mon_req" },
    { 18, 18, "mon_step" },
    { 17, 17, "mon_pend" },
    { 16, 16, "mon_en" },
    { 15, 11, "reserved" },
    { 10, 10, "vc_harderr" },
    { 9, 9, "vc_interr" },
    { 8, 8, "vc_buserr" },
    { 7, 7, "vc_staterr" },
    { 6, 6, "vc_chkerr" },
    { 5, 5, "vc_nocperr" },
    { 4, 4, "vc_mmerr" },
    { 3, 1, "reserved2" },
    { 0, 0, "vc_corereset" },
  };
  dump_reg_fields(__func__, label, fields, sizeof(fields) / sizeof(fields[0]), val);
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
  dump_reg_fields(__func__, label, fields, sizeof(fields) / sizeof(fields[0]), val);
}

static void hexdump(const char* tag, uint64_t cont_addr, const uint8_t* bs, size_t nb)
{
  assert(strlen(tag) > 0);
  for (int i = 0; i < nb; i += 16) {
    fprintf(stderr, "%s %08llx : ", tag, cont_addr + i);
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
  hexdump(label, addr, bs, nb);
  if (reg)
    (*reg)(label, *(uint32_t*)bs);
}

static int swd_status(struct pinctl* c, uint32_t* status, int dump)
{
  int err = swd_read(c, 0, REG_DP_STATUS, status);
  if (err) {
    fprintf(stderr, "%s:%d: swd_read(DP_STATUS):%d:%s\n", __func__, __LINE__, err, strerror(err));
    goto err_exit;
  }
  if (dump || verbose)
    dump_dp_status(*status);
err_exit:
  return err;
}

static int swd_ap_select(struct pinctl* c, uint8_t sel, uint8_t bank)
{
  static uint8_t prev_sel = -1;
  static uint8_t prev_bank = -1;
  if (verbose > 2)
    fprintf(stderr, "%s:%d: sel:%02x bank:%02x\n", __func__, __LINE__, sel, bank);
  if (prev_sel == sel && prev_bank == bank) {
    if (verbose > 2)
      fprintf(stderr, "%s:%d: repeat ap sel:%02x bank:%02x\n", __func__, __LINE__, sel, bank);
    return 0;
  }
  prev_sel = sel;
  prev_bank = bank;
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
  if (verbose > 1)
    fprintf(stderr, "%s:%d: ap:%02x reg:%02x val:%08x\n", __func__, __LINE__, ap, reg, val);
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
  if (verbose > 2)
    dump_ap_mem_cfg(cfg);
  err = swd_ap_read(c, ap, REG_AP_MEM_CSW, &csw);
  assert(err == 0);
  if (verbose > 2)
    dump_ap_mem_csw(csw);
// assert((csw >> 6) & 0x01);

  /* set 32-bit transfer size and auto-increment on access */
  if ((csw & ((3 << 4) | (7 << 0))) != ((1 << 4) | (2 << 0))) {
    err = swd_ap_write(c, ap, REG_AP_MEM_CSW, (csw & ~((3 << 4) | (7 << 0))) | (1 << 4) | (2 << 0));
    assert(err == 0);
  }

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

  /* 64-bit addressing? */
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

  for (/**/; nb >= 4; nb -= 4) {
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
    case 3: if (nb) { nb--; *bs++ = (val >> 8) & 0xff; }
    case 2: if (nb) { nb--; *bs++ = (val >> 16) & 0xff; }
    case 1: if (nb) { nb--; *bs++ = (val >> 24) & 0xff; }
      break;

    default:
      assert(0);
    }
  }
  assert(nb == 0);

  if (verbose > 1)
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
  if (verbose > 2)
    dump_ap_mem_cfg(cfg);
  err = swd_ap_read(c, ap, REG_AP_MEM_CSW, &csw);
  assert(err == 0);
  if (verbose > 2)
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

/* ftfl registers are (oddly) 8-bit, but the swd bits seem happier w/
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
    /* FixMe: the docs say that this is 0123 but the first two entries
     * for the interrupt vector table is the stack pointer for _start
     * and then _start.  checking these entries in memory gives
     * something like:
     *  0000: 00 00 00 20 85 0c 00 00 ...
     * where the code at 0x0c84 starts w/ the usual function prolog
     * (and 0x20 would be a weird stack pointer because it is in the
     * interrupt vector space).
     */
#if 0
    memcpy(regs + fccob_map[0x08], param + 1, 4);
#else
    regs[fccob_map[0x08]] = param[4]; /* expected[0] */
    regs[fccob_map[0x09]] = param[3]; /* expected[1] */
    regs[fccob_map[0x0a]] = param[2]; /* expected[2] */
    regs[fccob_map[0x0b]] = param[1]; /* expected[3] */
#endif
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

static int ftfl_flash_verify(struct pinctl* c, const char* path)
{
  int rv = 0;
  int fd = open(path, O_RDONLY);
  if (fd == -1) {
    fprintf(stderr, "%s:%d: open(%s) : %d:%s\n", __func__, __LINE__, path, errno, strerror(errno));
    rv = errno;
    goto err_exit;
  }

  struct stat buf;
  int err = fstat(fd, &buf);
  if (err == -1) {
    fprintf(stderr, "%s:%d: stat(%s) : %d:%s\n", __func__, __LINE__, path, errno, strerror(errno));
    rv = errno;
    goto err_exit;
  }
  printf("%s:%d: verify:%s nb:%zu\n", __func__, __LINE__, path, (size_t)buf.st_size);

  uint8_t bs[256];
  uint8_t params[8];
  for (uint32_t i = 0; i < buf.st_size; /**/) {
    fputc('c', stdout);
    size_t n = min(sizeof(bs), buf.st_size);
    assert((n & 3) == 0);
    ssize_t n_read = read(fd, bs, n);
    assert(n_read == n);

    for (int j = 0; j < n; j += 4, i += 4) {
      params[0] = 0x01;
      memcpy(params + 1, bs + i, 4);
      rv = swd_ftfl_issue(c, 0x02, i, params);
      if (rv) {
        fprintf(stderr, "%s:%d: ftfl:%d:%s\n", __func__, __LINE__, rv, strerror(rv));
        goto err_exit;
      }
      if (params[0])
        printf(" %04x:%02x:%02x:%02x:%02x ", i, bs[i+0], bs[i+1], bs[i+2], bs[i+3]);
    }
    fputc('\n', stdout);
    fflush(stdout);
  }

err_exit:
  if (fd != -1) close(fd);
  return rv;
}

static int ftfl_mem_write(struct pinctl* c, const struct memdesc* m)
{
  /* FixMe: doesn't handle files or flash yet */
  return swd_ap_mem_write_u32(c, 0, m->addr, m->val);
}

static int ftfl_mem_read(struct pinctl* c, const struct memdesc* m)
{
  int rv = 0;

  uint8_t bs[256];
  int fd = -1;
  if (m->path) {
    if (strcmp(m->path, "-") == 0) {
      fd = STDOUT_FILENO;
    } else {
      fd = open(m->path, O_WRONLY | O_CREAT | O_TRUNC, 0666);
      if (fd == -1) {
        fprintf(stderr, "%s:%d: output(%s) failed:%d:%s\n", __func__, __LINE__, m->path, errno, strerror(errno));
        rv = errno;
        goto err_exit;
      }
    }
  }

  SHA256_CTX ctx;
  sha256_init(&ctx);

  uint64_t addr = m->addr;
  uint32_t nb = m->nb;
  while (nb > 0) {
    uint32_t n = min(nb, sizeof(bs));
    rv = swd_ap_mem_read(c, 0, addr, bs, n);
    if (rv) goto err_exit;
    sha256_update(&ctx, bs, n);
    if (fd == -1) {
      hexdump("mem", addr, bs, n);
    } else {
      fputc('r', stdout);
      write(fd, bs, n);
    }
    addr += n;
    nb -= n;
  }

  assert(sizeof(bs) >= SHA256_BLOCK_SIZE);
  sha256_final(&ctx, bs);
  hexdump("sha", 0, bs, SHA256_BLOCK_SIZE);

  if (fd != -1 && fd != STDOUT_FILENO) {
    fputc('\n', stdout);
    fflush(stdout);

    close(fd);
  }

err_exit:
  return rv;
}

static int swd_ap_mem_reg_read(struct pinctl* c, int sel, uint32_t* out)
{
  int err = swd_ap_mem_write_u32(c, 0, REG_DCRSR, sel);
  assert(err == 0);
  int i;
  for (i = 0; i < 8; i++) {
    idle(c);
    err = swd_ap_mem_read_u32(c, 0, REG_DHCSR, out);
    assert(err == 0);
    if ((*out) & (1 << 16)) break;
  }
  assert(i < 8);
  if (i >= 8) return ETIMEDOUT;

  err = swd_ap_mem_read_u32(c, 0, REG_DCRDR, out);
  assert(err == 0);
  return 0;
}

static int write_reg(struct pinctl* c, const struct memdesc* r)
{
  int err = swd_ap_mem_write_u32(c, 0, REG_DCRDR, r->val);
  assert(err == 0);

  assert(r->addr < n_reg_sel_names);
  err = swd_ap_mem_write_u32(c, 0, REG_DCRSR, (1 << 16) | r->addr);
  assert(err == 0);

  uint32_t val;
  int i;
  for (i = 0; i < 8; i++) {
    idle(c);
    err = swd_ap_mem_read_u32(c, 0, REG_DHCSR, &val);
    assert(err == 0);
    if (val & (1 << 16)) break;
  }
  assert(i < 8);
  if (i >= 8) return ETIMEDOUT;

  return 0;
}

static void dump_reg(struct pinctl* c, const struct memdesc* r)
{
  uint32_t val;
  int err;
  err = swd_ap_mem_reg_read(c, r->addr, &val);
  assert(err == 0);
  printf("%s:%d: sel:%02llx%s%s val:%08x\n", __func__, __LINE__, r->addr, reg_sel_names[r->addr].nb ? ":" : "", reg_sel_names[r->addr].nb ? reg_sel_names[r->addr].s : "", val);
}

static void dump_regs(struct pinctl* c)
{
  struct memdesc r;
  for (r.addr = 0; r.addr < n_reg_sel_names; r.addr++) {
    /* special case skipping to avoid outputing a bogus value */
    if (r.addr == 0x13) continue;
    dump_reg(c, &r);
  }
}

static int swd_continue(struct pinctl* c)
{
  int rv = -1;

  /* if we're not halted or w/ debug enabled then there's nothing we
   * can do, but it was an error to come here at all w/o debug enabled.
   */
  uint32_t val;
  int err = swd_ap_mem_read_u32(c, 0, REG_DHCSR, &val);
  assert(err == 0);
  if (verbose)
    fprintf(stderr, "%s:%d: dhcsr:%08x\n", __func__, __LINE__, val);

  /* if debug isn't enabled then we shouldn't be halted? */
  assert((val & (1 << 0)) || ((val & (1 << 17)) == 0));

  /* if we're not halted then all is good */
  rv = (val & (1 << 17));
  if (!rv) {
    printf("%s:%d: dhcsr:%08x not halted\n", __func__, __LINE__, val);
    goto done;
  }

  /* clearing dbeug should be sufficient */
  err = swd_ap_mem_write_u32(c, 0, REG_DHCSR, (val & ~(0xffff0000 | (1 << 1) | (1 << 0))) | (0xa05f << 16));
  assert(err == 0);

  /* double check */
  err = swd_ap_mem_read_u32(c, 0, REG_DHCSR, &val);
  assert(err == 0);
  if (verbose)
    fprintf(stderr, "%s:%d: dhcsr:%08x\n", __func__, __LINE__, val);
  rv = (val & (1 << 17));

done:
  return rv;
}

static int swd_halt_wait(struct pinctl* c)
{
#define HALT_WAIT_CYCLES 8
  uint32_t val;
  int i;
  for (i = 0; i < HALT_WAIT_CYCLES; i++) {
    int err = swd_ap_mem_read_u32(c, 0, REG_DHCSR, &val);
    assert(err == 0);
    if (val & (1 << 17)) break;
    idle(c);
  }
  if (verbose)
    fprintf(stderr, "%s:%d: halt:%d dhcsr:%08x\n", __func__, __LINE__, i, val);
  return (i < HALT_WAIT_CYCLES);
}

/* returns EBUSY if teh target appears to be stuck in reset.
 */
static int swd_halt(struct pinctl* c, int sysreset)
{
  /* for yucks, we do the double read to check if it's held in
   * reset.in that case, flash accesses work, but debug stuff doesn't.
   */
  uint32_t val;
  int err = swd_ap_mem_read_u32(c, 0, REG_DHCSR, &val);
  assert(err == 0);
  if (val & (1 << 25)) {
    err = swd_ap_mem_read_u32(c, 0, REG_DHCSR, &val);
    assert(err == 0);
    if (val & (1 << 25)) {
      fprintf(stderr, "%s:%d: held in reset?\n", __func__, __LINE__);
      return EBUSY;
    }
  }
  if (verbose)
    fprintf(stderr, "%s:%d: dhcsr:%08x\n", __func__, __LINE__, val);

  /* enable debug before going to doing any fiddling w/ sysreset */
  if ((val & ((1 << 17) | (1 << 0))) != ((1 << 17) | (1 << 0))) {
    /* halt and enable debug */
    err = swd_ap_mem_write_u32(c, 0, REG_DHCSR, (val & 0x0000ffff) | (0xa05f << 16) | (1 << 1) | (1 << 0));
    assert(err == 0);

    /* if sysreset, then the halt comes again later when we reset, but
     * otherwise we wait for halt status to be asserted.
     */
    if (!sysreset)
      swd_halt_wait(c);
  }

  if (sysreset) {
    uint32_t demcr;

    /* set to hold after reset ... */
    err = swd_ap_mem_read_u32(c, 0, REG_DEMCR, &demcr);
    assert(err == 0);
    if (verbose)
      fprintf(stderr, "%s:%d: demcr:%08x\n", __func__, __LINE__, demcr);
    if (!(demcr & (1 << 0))) {
      err = swd_ap_mem_write_u32(c, 0, REG_DEMCR, demcr | (1 << 0));
      assert(err == 0);
    }

    /* assert sysreset */
    err = swd_ap_mem_read_u32(c, 0, REG_AIRCR, &val);
    assert(err == 0);
    if (verbose)
      fprintf(stderr, "%s:%d: aircr:%08x\n", __func__, __LINE__, val);
    err = swd_ap_mem_write_u32(c, 0, REG_AIRCR, (val & 0x0000ffff) | 0x05fa0000 | (1 << 2));
    assert(err == 0);

    for (int i = 0; (i < HALT_WAIT_CYCLES) && !(val & (1 << 25)); i++) {
      idle(c);

      err = swd_ap_mem_read_u32(c, 0, REG_DHCSR, &val);
      assert(err == 0);
      if (verbose)
        fprintf(stderr, "%s:%d: halt:%d dhcsr:%08x\n", __func__, __LINE__, i, val);
    }
    /* we're held in reset so another dhcsr read doesn't clear that */
    assert(val & (1 << 25));

    /* clear the hold state because it 'sticks' across the reset line
     * (it's cleared on a power-on-reset), this means that the reset
     * comes up in the halt state, which is a little surprising.
     */
    err = swd_ap_mem_write_u32(c, 0, REG_DEMCR, demcr & ~(1 << 0));
    assert(err == 0);
  }

  /* validate halt state */
  err = swd_ap_mem_read_u32(c, 0, REG_DHCSR, &val);
  assert(err == 0);
  if (verbose)
    fprintf(stderr, "%s:%d: dhcsr:%08x\n", __func__, __LINE__, val);
  assert(val & (1 << 17));

  return (val & (1 << 17)) ? 0 : ETIMEDOUT;
}

int main(int argc, char** argv)
{
  int rv = EXIT_FAILURE;
  unsigned phase = 500; /* us */
  int opt;
  uint32_t val;
  struct memdesc* regs = NULL;
  int n_regs = 0;
  struct memdesc* mems = NULL;
  int n_mems = 0;
  const char* f_verify = NULL;
  int ext_power_cycle = -1;
  int sysreset = 0;
  int syscontinue = 0;
  int n_instr = -1;

  while ((opt = getopt(argc, argv, "cg:hHn:p:r:vV:w:xX")) != -1) {
    char* end;

    switch (opt) {
    case 'c':
      syscontinue = 1;
      break;
    case 'g': {
      int sel = reg2sel(optarg);
      assert(sel != -1);
      const char* s = strchr(optarg, ':');
      int w = s != NULL;
      assert(sel != 0x13 || !w);
      if (w) {
        val = strtoul(s + 1, &end, 0);
        assert(end != s + 1);
        assert(*end == 0);
      }
      void* p = realloc(regs, sizeof(regs[0]) * (n_regs + 1));
      assert(p != NULL);
      regs = p;
      memset(regs + n_regs, 0, sizeof(regs[0]));
      regs[n_regs].addr = sel;
      if (w) {
        regs[n_regs].flags |= DESC_WRITE;
        regs[n_regs].val = val;
      }
      n_regs++;
      }
      break;
    case 'h':
      sysreset = 1;
      break;
    case 'n':
      n_instr = strtoul(optarg, &end, 0);
      assert(end != optarg && *end == 0);
      break;
    case 'p':
      phase = strtoul(optarg, &end, 0);
      assert(end != optarg);
      if (strcmp("ms", end) == 0)
        phase *= 1000;
      else if (strcmp("s", end) == 0)
        phase *= 1000000;
      else if (strcmp("hz", end) == 0)
        phase = 500000 / phase;
      else if (strcmp("khz", end) == 0)
        phase = 500000 / (phase * 1000);
      else
        assert((*end == '\0') || (strcmp("us", end) == 0));
      assert(phase > 0);
      break;
    case 'r': {
        uint64_t addr = strtoull(optarg, &end, 0);
        uint32_t nb = 16;
        char* path = NULL;
        assert(end != optarg);
        assert((addr & 0x03) == 0);
        if (*end == ':') {
          optarg = end + 1;
          nb = strtoul(optarg, &end, 0);
          assert(end != optarg);
          assert(nb > 0);
          switch (*end) {
          case 'M':
            nb *= 1024;
          case 'k':
            nb *= 1024;
            end++;
          case 0:
            break;
          default:
            assert(0);
          }
          assert(*end == 0 || *end == ':');
        }
        assert((nb & 0x03) == 0);

        if (*end == ':')
          path = end + 1;
        assert(path == NULL || *path != 0);

        void* p = realloc(mems, sizeof(mems[0]) * (n_mems + 1));
        if (!p) break;
        mems = p;
        memset(mems + n_mems, 0, sizeof(mems[0]));
        mems[n_mems].addr = addr;
        mems[n_mems].nb = nb;
        mems[n_mems].path = path;
        n_mems++;
      }
      break;
    case 'V':
      f_verify = optarg;
      break;
    case 'x':
    case 'X':
      ext_power_cycle = isupper(opt);
      break;
    case 'v':
      verbose++;
      break;
    case 'w': {
        uint64_t addr = strtoull(optarg, &end, 0);
        assert(end != optarg);
        assert((addr & 0x03) == 0);
        optarg = end + 1;
        void* p = realloc(mems, sizeof(mems[0]) * (n_mems + 1));
        if (!p) break;
        mems = p;
        memset(mems + n_mems, 0, sizeof(mems[0]));
        mems[n_mems].flags |= DESC_WRITE;
        mems[n_mems].addr = addr;
        switch (*end) {
        case ':':
          mems[n_mems].val = strtoul(optarg, &end, 0);
          assert(end != optarg);
          assert(*end == 0);
          break;
        case '-':
          assert(*optarg != 0);
          mems[n_mems].path = optarg;
          break;
        default:
          assert(*end == ':' || *end == '-');
        }
        n_mems++;
      }
      break;
    default:
      assert(0);
    }
  }

  struct pinctl* pins = pins_open(phase);
  if (!pins) goto err_exit;

  /* external reset/power-on-reset handling */
  if (ext_power_cycle >= 0)
    pins_reset(pins, ext_power_cycle);

  /* reset-y bits for the debug port */
  resync(pins, 0);
  jtag2swd(pins);
  resync(pins, 1);

  /* reading id takes the debug port out of reset */
  if ((opt = swd_read(pins, 0, REG_DP_IDCODE, &val)) != 0) {
    fprintf(stderr, "%s:%d: swd_read(IDCODE):%d:%s\n", __func__, __LINE__, opt, strerror(opt));
    goto err_exit;
  }
  dump_dp_idcode(val);
  /* at least v1, but we've already tried dp idcode so it's too late */
  assert(((val >> 12) & 0xf) >= 1);
  /* cortex-m4 has arm designer */
  assert(((val >> 1) & 0x7ff) == 0x023b);

  /* set power state */
  for (int i = 0; /**/; i++) {
    opt = swd_status(pins, &val, 1);
    assert(opt == 0);
    if ((val & (CSYSPWRUPACK | CDBGPWRUPACK)) == (CSYSPWRUPACK | CDBGPWRUPACK))
      break;
    if (i == 0) {
      opt = swd_write(pins, 0, REG_DP_STATUS, CSYSPWRUPREQ | CDBGPWRUPREQ | CDBGRSTREQ);
      assert(opt == 0);
    }
    idle(pins);
  }

  /* ap 0 should be a memory access class. the identity/continuation
   * codes on the k20 match arm.
   */
  opt = swd_ap_idcode(pins, 0, &val);
  assert(opt == 0);
  assert(((val >> 13) & 0x0f) == 0x08);
  assert(((val >> 17) & 0x7f) == 0x3b);
  assert(((val >> 24) & 0x0f) == 0x04);

  /* the k20 has an mdm ap w/ stuff like mass erase and more debug control.
   * otherwise, the code is zero if it isn't supported.
   */
  opt = swd_ap_idcode(pins, 1, &val);
  assert(opt == 0);
  printf("%s:%d: mdm-ap idr:%08x\n", __func__, __LINE__, val);
  if (val != 0) {
  assert(val == 0x001c0000);

  /* mdm ap debug overides the dhcsr setting, and security enabled
   * disables bus access to memory.
   */
  opt = swd_ap_read(pins, 1, 0x00, &val);
  assert(opt == 0);
  printf("%s:%d: mdm-ap status:%08x\n", __func__, __LINE__, val);
  assert(val & (1 << 5));
  while (val & (1 << 2)) {
    opt = swd_ap_read(pins, 1, 0x04, &val);
    assert(opt == 0);
    printf("%s:%d: mdm-ap control:%08x\n", __func__, __LINE__, val);
// erase already outstanding?
// assert(!(val & 0x01));
#if 0
    opt = swd_ap_write(pins, 1, 0x04, val | 0x01);
    assert(opt == 0);

    /* wait for the erase to be accepted */
    int i;
    for (i = 0; i < 6; i++) {
    opt = swd_ap_read(pins, 1, 0x00, &val);
    assert(opt == 0);
    printf("%s:%d: mdm-ap status:%08x\n", __func__, __LINE__, val);
    if (val & (1 << 0)) break;
sleep(10);
    }
    if (!(val & (1 << 0)))
      goto erase_fail;
    printf("\nerase ack:%d\n", i);

    /* wait for the erase */
    for (i = 0; i < 6; i++) {
      opt = swd_ap_read(pins, 1, 0x04, &val);
      assert(opt == 0);
printf("%s:%d: mdm-ap control:%08x\n", __func__, __LINE__, val);
if (!(val & (1 << 0))) break;
sleep(10);
    }
    if (val & (1 << 0))
      goto erase_fail;
    printf("\nerase:%d\n", i);
    break;
erase_fail:
#endif
    opt = ez_reset(pins);
    assert(opt == 0);
  }
  }

  /* it doesn't matter about the cpuid, but it's informational */
  opt = swd_ap_mem_read_u32(pins, 0, REG_CPUID, &val);
  assert(opt == 0);
  printf("%s:%d: cpuid:%08x\n", __func__, __LINE__, val);
  assert((((val >> 4) & 0x0fff) == 0x0c23) || /* cortex-m3 */
         (((val >> 4) & 0x0fff) == 0x0c24));  /* cortex-m4 */
  assert(((val >> 16) & 0x0f) == 0xf); /* reserved */
  assert(((val >> 24) & 0xff) == 0x41); /* arm */

  /* did we previously halt things? if we're releasing, then we're done here. */
  if (syscontinue)
    swd_continue(pins);

  /* if we have other commands to do, we have to halt the processor to
   * do anything, but skip all of this if there aren't any commands.
   */
  if ((n_instr < 0) && !n_mems && !n_regs && !f_verify)
    goto done;
  swd_halt(pins, sysreset);

  for (int i = 0; i < n_regs; i++)
    if (regs[i].flags & DESC_WRITE)
      write_reg(pins, regs + i);
    else if (regs[i].addr == 0x13)
      dump_regs(pins);
    else
      dump_reg(pins, regs + i);
  for (int i = 0; i < n_mems; i++)
    if (mems[i].flags & DESC_WRITE)
      ftfl_mem_write(pins, mems + i);
    else
      ftfl_mem_read(pins, mems + i);
  if (f_verify != NULL)
    ftfl_flash_verify(pins, f_verify);

  for (int i = 0; i < n_instr; i++) {
    opt = swd_ap_mem_write_u32(pins, 0, REG_DHCSR, (val & 0x0000fff0) | (0xa05f << 16) | (1 << 2) | (1 << 0));
    assert(opt == 0);

    /* after the single-step compltes we should be back in halt state */
    opt = swd_halt_wait(pins);
    assert(opt);
  }

done:
  rv = EXIT_SUCCESS;
err_exit:
  free(mems);
  free(regs);
  pins_close(pins);
  return rv;
}
