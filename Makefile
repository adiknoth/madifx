ifneq ($(KERNELRELEASE),)

obj-m := madifx.o


else

ifeq ($(BROKEN_WIP),1)
	BROKEN = -DCONFIG_SND_MADIFX_BROKEN=1
else
	BROKEN =
endif

snd-madifx-objs := madifx.o

KDIR   := /lib/modules/$(shell uname -r)/build
PWD    := $(shell pwd)
MODDIR := $(DESTDIR)/lib/modules/$(shell uname -r)/kernel/sound/pci/rme9652
BINDIR := $(DESTDIR)/usr/local/bin
INCDIR := $(DESTDIR)/usr/include/alsa/sound

default::
	$(MAKE) -Wall -Wextra -C $(KDIR) SUBDIRS=$(PWD) EXTRA_CFLAGS="-g ${BROKEN}" modules

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
