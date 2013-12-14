# If KERNELRELEASE is defined, we've been invoked from the
# kernel build system and can use its language.
ifneq ($(KERNELRELEASE),)
        mcp2515-fen-objs := driver.o
        obj-m := driver.o

# Otherwise we were called directly from the command
# line; invoke the kernel build system.
else

		KERNELDIR ?= /lib/modules/$(shell uname -r)/build
		PWD := $(shell pwd)

default:
		make -C $(KERNELDIR) M=$(PWD) modules

clean:
		rm -rf *.ko *.ko.unsigned *.mod.c *.o modules.order Module.symvers

endif
