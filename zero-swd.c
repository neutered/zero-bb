#include <assert.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "libpin.h"

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
  rv |= (reg & 0x0f) << 1;
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
fprintf(stderr, "%s:%d: op:%02x\n", __func__, __LINE__, op);
  pins_write(pins, &op, 8);
  
  uint8_t status;
  pins_read(pins, &status, 3);

  return status;
}

static int swd_read(struct pinctl* pins, int ap, int reg, uint32_t* out)
{
  uint8_t status = send_op(pins, 0, 0, 1);
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

  /* 32-bit data + parity + turnaround? */
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
  if (out)
    *out = ((bs[3] << 24) |
	    (bs[2] << 16) |
	    (bs[1] << 8) |
	    (bs[0] << 0));
  return 0;
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
  if ((opt = swd_read(pins, 0, 0, &idcode)) != 0) {
    fprintf(stderr, "%s:%d: swd_read(IDCODE):%d:%s\n", __func__, __LINE__, opt, strerror(opt));
    goto err_exit;
  }
  printf("%s:%d: idcode:%08x\n", __func__, __LINE__, idcode);

  rv = EXIT_SUCCESS;
 err_exit:
  pins_close(pins);
  return rv;
}
