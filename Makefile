# PREFIX=arm-cortex_a15-linux-gnueabihf
PREFIX=armv6-rpi-linux-gnueabi
PATH_PREFIX=$(HOME)/x-tools/$(PREFIX)/bin

AR=$(PATH_PREFIX)/$(PREFIX)-ar
CC=$(PATH_PREFIX)/$(PREFIX)-gcc
LD=$(PATH_PREFIX)/$(PREFIX)-ld

CFLAGS=-Wall -g

PROGS=zero-test zero-swd zero-ez
LIBS=libpin.a

all: $(PROGS)

zero-test: test.c
	$(CC) -o $@ $^

zero-swd: zero-swd.o libpin.a
	$(CC) -o $@ $^

zero-ez: zero-ez.o pinutil.h
	$(CC) -o $@ $^

libpin.a: libpin.o pinutil.h
	$(AR) cr $@ $^

libpin.i: libpin.c libpin.h
	$(CC) -E -dM -o $@  $<

clean:
	rm -f *.o *.i *~ $(PROGS) $(LIBS)
