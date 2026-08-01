obj-m += lcd1602_drv.o

KDIR ?= /lib/modules/$(shell uname -r)/build
PWD  := $(shell pwd)

ccflags-y := -I$(src)

all:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean
	rm -f test/lcd_test

test/lcd_test: test/lcd_test.c lcd1602.h
	$(CC) -Wall -O2 -I. -o $@ $<

test: test/lcd_test

load:
	sudo insmod lcd1602_drv.ko

unload:
	sudo rmmod lcd1602_drv

.PHONY: all clean test load unload
