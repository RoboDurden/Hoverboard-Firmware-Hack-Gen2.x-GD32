CC ?= cc
CFLAGS  = -std=c99 -O2 -Wall -Wextra -Werror -fwrapv \
          -fsanitize=undefined,address \
          -Wdouble-promotion -Werror=float-conversion \
          -I../Inc
LDFLAGS = -fsanitize=undefined,address

SRC = ../Src/bldcFOC_core.c \
      test_main.c \
      test_sincos.c \
      test_transforms.c \
      test_bc.c \
      test_pll.c \
      test_pi.c \
      test_dtc.c \
      test_hall_cal.c

tests: $(SRC) test_helpers.h ../Inc/bldcFOC_core.h
	$(CC) $(CFLAGS) $(LDFLAGS) -o tests $(SRC) -lm

.PHONY: test clean
test: tests
	./tests

clean:
	rm -f tests
