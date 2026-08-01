CC      ?= gcc
CFLAGS  ?= -Wall -Wextra -O2 -I.

all: vlcd

vlcd: vlcd.c zstatus_proto.h lcd1602.h
	$(CC) $(CFLAGS) -o $@ vlcd.c -lm

clean:
	rm -f vlcd

.PHONY: all clean
