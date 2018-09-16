#ifndef LIBPIN_H_
#define LIBPIN_H_

#include <stdint.h>

struct pinctl;

struct pinctl* pins_open(unsigned /* clock rate */);
void pins_close(struct pinctl*);

int pins_write(struct pinctl*, const uint8_t* /* bit stream */, int /* n */);
int pins_read(struct pinctl*, uint8_t*, int);
int pins_reset(struct pinctl*, int);

int ez_reset(struct pinctl*);

#endif /* LIBPIN_H_*/
