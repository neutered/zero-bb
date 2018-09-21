# PREFIX=arm-cortex_a15-linux-gnueabihf
PREFIX=armv6-rpi-linux-gnueabi
PATH_PREFIX=$(HOME)/x-tools/$(PREFIX)/bin

AR=$(PATH_PREFIX)/$(PREFIX)-ar
CC=$(PATH_PREFIX)/$(PREFIX)-gcc
LD=$(PATH_PREFIX)/$(PREFIX)-ld

CFLAGS=-Wall -g -DPIN_EZ=0

PROGS=zero-test zero-swd
LIBS=libpin.a libsha.a

all: $(PROGS)

zero-test: test.c
	$(CC) -o $@ $^

zero-swd: zero-swd.o libpin.a libsha.a
	$(CC) -o $@ $^

libpin.a: libpin.o pinutil.h
	$(AR) cr $@ $^

libpin.i: libpin.c libpin.h
	$(CC) -E -dM -o $@  $<

libsha.a: sha256.o sha256.h
	$(AR) cr $@ $^

clean:
	rm -f *.o *.i *~ $(PROGS) $(LIBS)
