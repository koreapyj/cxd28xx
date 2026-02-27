KDIR ?= /lib/modules/$(shell uname -r)/build

obj-m += cxd2878.o m88rs6060.o atsc3_alp.o tbs5530.o it930x.o

ccflags-y += -I$(src) -DCONFIG_DVB_CXD2878=1

all:
	$(MAKE) -C $(KDIR) M=$(PWD) modules
	@if pkg-config --exists libpcsclite 2>/dev/null; then \
		$(MAKE) -C pcsc; \
	fi

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean
	$(MAKE) -C pcsc clean

install:
	$(MAKE) -C $(KDIR) M=$(PWD) modules_install
	@if pkg-config --exists libpcsclite 2>/dev/null; then \
		$(MAKE) -C pcsc install; \
	fi

uninstall:
	$(MAKE) -C pcsc uninstall
