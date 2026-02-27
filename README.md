# CXD28xx Linux DVB Driver

Out-of-tree Linux kernel driver for the **Sony CXD28xx** series demodulator and associated tuners, supporting a wide range of broadcast standards.

## Supported Standards

- DVB-T / DVB-T2
- DVB-C / DVB-C2
- DVB-S / DVB-S2
- ISDB-T / ISDB-C / ISDB-S
- ATSC 1.0
- ATSC 3.0
- J.83B

## Supported Hardware

| Device | USB Bridge | Demod | Tuner | Frontends |
|--------|-----------|-------|-------|-----------|
| Zenview(GTMEDIA) HDTV Mate | ITE IT930x | CXD2878 | Ascot3 (internal) | 1 (terr/cable) |
| PLEX PX-MLT5U | ITE IT930x | CXD2856ER | HELENE (CXD2858ER) | 5 (terr/cable + sat) |
| PLEX PX-MLT5PE | ITE IT930x | CXD2856ER | HELENE (CXD2858ER) | 5 (terr/cable + sat) |
| PLEX PX-MLT8PE3 | ITE IT930x | CXD2856ER | HELENE (CXD2858ER) | 3 (terr/cable + sat) |
| PLEX PX-MLT8PE5 | ITE IT930x | CXD2856ER | HELENE (CXD2858ER) | 5 (terr/cable + sat) |
| Digibest ISDB6014 4TS | ITE IT930x | CXD2856ER | HELENE (CXD2858ER) | 4 (terr/cable + sat) |
| TurboSight TBS 5530 | Cypress FX2 | CXD2878 + M88RS6060 | Ascot3 + M88RS6060 | 2 (terr/cable + sat) |

TBS 5530 supports ATSC 3.0, unlike the vendor driver.

PX-MLT series (and ISDB6014) potentially support DVB-T/C/S/S2, unlike other drivers. (Not tested)

Zenview HDTV Mate has Microchip ATECC608. You can access it through i2c address `0xc8`.

## Modules

| Module | Description |
|--------|-------------|
| `cxd2878.ko` | Sony CXD2878/CXD2856 demodulator + Ascot3/HELENE tuner driver |
| `m88rs6060.ko` | Montage M88RS6060 satellite demod/tuner driver |
| `atsc3_alp.ko` | ATSC 3.0 ALP parser and network interface (shared) |
| `it930x.ko` | ITE IT930x USB bridge driver |
| `tbs5530.ko` | TurboSight TBS 5530 USB bridge driver |

## Installation (package)

Pre-built DKMS packages are available on the [GitHub Releases](https://github.com/koreapyj/cxd28xx/releases) page.

### Debian / Ubuntu

```sh
wget -O /tmp/cxd28xx-dkms.deb https://github.com/koreapyj/cxd28xx/releases/latest/download/cxd28xx-dkms_0.2_all.deb
sudo dpkg -i /tmp/cxd28xx-dkms.deb
```

To remove:

```sh
sudo dpkg -r cxd28xx-dkms
```

### Fedora / RHEL

```sh
wget -O /tmp/cxd28xx-dkms.rpm https://github.com/koreapyj/cxd28xx/releases/latest/download/cxd28xx-dkms-0.2-1.noarch.rpm
sudo dnf install /tmp/cxd28xx-dkms.rpm
```

To remove:

```sh
sudo dnf remove cxd28xx-dkms
```

## Installation (DKMS, from source)

DKMS automatically rebuilds the driver on kernel updates.

```sh
sudo dkms add /path/to/cxd28xx
sudo dkms build cxd28xx/1.0
sudo dkms install cxd28xx/1.0
```

To remove:

```sh
sudo dkms remove cxd28xx/1.0 --all
```

## Building (manual)

Requires kernel headers for the running kernel. Optionally install `libpcsclite-dev` (Debian/Ubuntu) or `pcsc-lite-devel` (Fedora) to also build the smartcard IFD handler.

```sh
make
sudo make install
sudo depmod -a
```

To remove:

```sh
sudo make uninstall
```

To build against a specific kernel:

```sh
make KDIR=/path/to/kernel/build
```

## Smartcard Reader

PX-MLT and Digibest devices have a built-in smartcard reader connected via the IT930x UART interface. The driver creates a character device `/dev/it930x_smartcardN` for each reader automatically.

### PC/SC Integration

If `libpcsclite-dev` (Debian/Ubuntu) or `pcsc-lite-devel` (Fedora) is installed when the DKMS package is built, an IFD handler library (`libifd_it930x.so`) is automatically installed for use with pcscd.

The DKMS install automatically places a reader config at `/etc/reader.conf.d/it930x.conf`. Edit this file to adjust `DEVICENAME` if your device is not `/dev/it930x_smartcard0`. Then restart pcscd:

```sh
sudo systemctl restart pcscd
```

Verify with `pcsc_scan`.

## Firmware

The following firmware files are required and should be placed in `/lib/firmware/`:

| Device | Firmware |
|--------|----------|
| IT930x (Zenview) | `dvb-usb-it9306-01.fw` |
| IT930x (PLEX, Digibest) | `it930x-firmware.bin` |
| TBS 5530 | `dvb-usb-id5530.fw` |

For PLEX and Digibest devices, refer [px4_drv](https://github.com/nns779/px4_drv) to extract from vendor drivers.

## ATSC 3.0

### State

Appears to be functional at supported devices. However, because there is currently no MMT/ROUTE demuxer available to handle the South Korean version of ATSC 3.0, the test environment remains limited.

May work in other regions. More tests are required.

### Tuning

The Linux DVB API has no `SYS_ATSC3` delivery system, and userspace tools (dvbv5-zap, etc.) do not support it. This driver works around the limitation by overloading `SYS_ATSC`:

- **ATSC 1.0** always uses 8-VSB modulation (`VSB_8`).
- If the driver sees `SYS_ATSC` with **any modulation other than `VSB_8`**, it treats the request as ATSC 3.0.

In a dvbv5 channel file, set `MODULATION` to any non-8VSB value (e.g. `QAM/AUTO`):

```
[ATSC3_STATION]
	DELIVERY_SYSTEM = ATSC
	FREQUENCY = 599000000
	MODULATION = QAM/AUTO
	BANDWIDTH_HZ = 6000000
```

Then tune normally:

```sh
dvbv5-zap -c channels.conf "ATSC3_STATION"
```

### ALP network interface

ATSC 3.0 delivers IP data via the ALP (ATSC Link-layer Protocol). The driver creates a network interface `atscN` for each adapter at device init. The interface can only be brought up while the frontend is tuned to ATSC 3.0. To receive IP traffic:

```sh
dvbv5-zap -c channels.conf "ATSC3_STATION"
sudo ip link set atsc0 up
```

The `atsc3_alp.ko` module is shared between all bridge drivers.

## License

GPL-2.0-or-later. See [LICENSE](LICENSE) for details.
