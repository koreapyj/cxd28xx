PCSC_CFLAGS := $(shell pkg-config --cflags libpcsclite 2>/dev/null)
PCSC_DRIVERDIR := $(shell pkg-config --variable=usbdropdir libpcsclite 2>/dev/null)/serial

all: libifd_it930x.so

libifd_it930x.so: ifd_it930x.c ../it930x_smartcard.h
	$(CC) -shared -fPIC -Wall -o $@ $< $(PCSC_CFLAGS)

install: libifd_it930x.so
	install -D -m 644 libifd_it930x.so $(DESTDIR)$(PCSC_DRIVERDIR)/libifd_it930x.so
	install -D -m 644 reader.conf.example $(DESTDIR)/etc/reader.conf.d/it930x.conf

uninstall:
	rm -f $(DESTDIR)$(PCSC_DRIVERDIR)/libifd_it930x.so
	rm -f $(DESTDIR)/etc/reader.conf.d/it930x.conf

clean:
	rm -f libifd_it930x.so

.PHONY: all install uninstall clean
