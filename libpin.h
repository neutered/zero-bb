#ifndef LIBPIN_H_
#define LIBPIN_H_

#include <stdint.h>

struct pinctl;

struct pinctl* pins_open(void);
void pins_close(struct pinctl*);

int pins_write(struct pinctl*, const uint8_t* /* bit stream */, int /* n */);

#endif /* LIBPIN_H_*/
