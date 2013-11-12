KERNEL_VERSION	:= `uname -r`
ifneq ($(KERNELRELEASE),)

obj-m := madifx.o


else

ifeq ($(BROKEN_WIP),1)
	BROKEN = -DCONFIG_SND_MADIFX_BROKEN=1
else
	BROKEN =
endif

DEBUG_CFLAGS=-g -Wall -Werror

snd-madifx-objs := madifx.o

KDIR   := /lib/modules/$(KERNEL_VERSION)/build
PWD    := $(shell pwd)
MODDIR := $(DESTDIR)/lib/modules/$(KERNEL_VERSION)/kernel/sound/pci/rme9652
BINDIR := $(DESTDIR)/usr/local/bin
INCDIR := $(DESTDIR)/usr/include/alsa/sound

default::
	$(MAKE) -C $(KDIR) SUBDIRS=$(PWD) EXTRA_CFLAGS="${DEBUG_CFLAGS} ${BROKEN}" modules

install-only:: default
	mkdir -p $(MODDIR) $(BINDIR)
	cp madifx.ko $(MODDIR)

install:: install-only
	/sbin/depmod -a
	/sbin/rmmod madifx || true
	/sbin/rmmod snd-madifx || true
	/sbin/modprobe madifx

clean::
	rm -f core .*.cmd *.o *.ko *.mod.c Module.* modules.order *.bak .\#* *~
	rm -rf .tmp_versions

endif
