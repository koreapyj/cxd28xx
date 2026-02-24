KDIR ?= /lib/modules/$(shell uname -r)/build

obj-m += cxd2878.o m88rs6060.o tbs5530.o it930x.o

ccflags-y += -I$(src) -DCONFIG_DVB_CXD2878=1

all:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean

install:
	$(MAKE) -C $(KDIR) M=$(PWD) modules_install
