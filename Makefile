# SPDX-License-Identifier: GPL-2.0

ifeq ($(LINUX),)
LINUX=/path/to/linux
endif
obj-m += hard_barrier.o

all:
	        make -C $(LINUX) M=$(PWD) modules
clean:
	        make -C $(LINUX) M=$(PWD) clean
