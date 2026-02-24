# CXD2878 Linux DVB Driver

Out-of-tree Linux kernel driver for the **Sony CXD2878** demodulator and associated tuners, supporting a wide range of broadcast standards.

## Supported Standards

- DVB-T / DVB-T2
- DVB-C / DVB-C2
- ISDB-T / ISDB-C
- ATSC 1.0
- ATSC 3.0 (ALP over IP)
- J.83B

## Supported Hardware

| Device | USB Bridge | Demod | Tuner | Module |
|--------|-----------|-------|-------|--------|
| Geniatech HDTV Mate | ITE IT930x | CXD2878 | Ascot3 (internal) | `it930x.ko` |
| TurboSight TBS 5530 | Cypress FX2 | CXD2878 + M88RS6060 | Ascot3 + M88RS6060 | `tbs5530.ko` |

The TBS 5530 additionally supports **DVB-S / DVB-S2 / DVB-S2X** via the Montage M88RS6060 demod/tuner.

## Modules

| Module | Description |
|--------|-------------|
| `cxd2878.ko` | Sony CXD2878 demodulator + Ascot3 tuner driver |
| `m88rs6060.ko` | Montage M88RS6060 satellite demod/tuner driver |
| `atsc3_alp.ko` | ATSC 3.0 ALP parser and network interface (shared) |
| `it930x.ko` | ITE IT930x USB bridge driver |
| `tbs5530.ko` | TurboSight TBS 5530 USB bridge driver |

## Installation (DKMS)

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

Requires kernel headers for the running kernel.

```sh
make
sudo make install
sudo depmod -a
```

To build against a specific kernel:

```sh
make KDIR=/path/to/kernel/build
```

## Firmware

The following firmware files are required and should be placed in `/lib/firmware/`:

| Device | Firmware |
|--------|----------|
| IT930x | `dvb-usb-it9306-01.fw` |
| TBS 5530 | `dvb-usb-id5530.fw` |

## ATSC 3.0

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

ATSC 3.0 delivers IP data via the ALP (ATSC Link-layer Protocol). When the driver locks an ATSC 3.0 signal, a network interface `atscN` is created. To receive IP traffic:

```sh
sudo ip link set atsc0 up
```

The `atsc3_alp.ko` module is shared between all bridge drivers.

## License

GPL-2.0-or-later. See [LICENSE](LICENSE) for details.
