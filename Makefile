CROSS_COMPILE	?= 
ARCH		?= x86
KERNEL_DIR	?= /usr/src/linux

CC		:= $(CROSS_COMPILE)gcc
KERNEL_INCLUDE	:= -I$(KERNEL_DIR)/include -I$(KERNEL_DIR)/arch/$(ARCH)/include
CFLAGS		:= -W -Wall -g $(KERNEL_INCLUDE)
LDFLAGS		:= -g

all: uvc-gadget

uvc-gadget: uvc-gadget.o
	$(CC) $(LDFLAGS) -o $@ $^

clean:
	rm -f *.o
	rm -f uvc-gadget
