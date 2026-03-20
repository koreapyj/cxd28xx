# CXD28xx Linux DVB Driver

Out-of-tree Linux kernel driver for the **Sony CXD28xx** series demodulator and associated tuners, supporting a wide range of broadcast standards.

## Supported Hardware

| Device | USB Bridge | Demod | Tuner | Frontends |
|--------|-----------|-------|-------|-----------|
| Zenview(GTMEDIA) HDTV Mate | ITE IT930x | CXD2878 | Ascot3 (internal) | 1 (terr/cable) |
| TurboSight TBS 5530 | Cypress FX2 | CXD2878 + M88RS6060 | Ascot3 + M88RS6060 | 2 (terr/cable + sat) |

Both devices support ATSC 3.0 with the ALP network interface.

Zenview HDTV Mate has a Microchip ATECC608 secure element accessible through I2C address `0xc8`.

## Modules

| Module | Description |
|--------|-------------|
| `alp.ko` | ATSC A/330 ALP protocol — generic ALP packet parser with segmentation/concatenation reassembly and IPv4 delivery via raw-ip net device (`alp%d`) |
| `cxd2878.ko` | Sony CXD2878/CXD6801/CXD6802 demodulator + Ascot3/FREIA tuner driver, includes ALP-div-TS decapsulation for ATSC 3.0 |
| `m88rs6060.ko` | Montage M88RS6060 satellite demod/tuner driver |
| `it930x.ko` | ITE IT930x USB bridge driver |
| `tbs5530.ko` | TurboSight TBS 5530 USB bridge driver |

## Installation (package)

Pre-built DKMS packages are available on the [GitHub Releases](https://github.com/koreapyj/cxd28xx/releases) page.

### Debian / Ubuntu

```sh
sudo dpkg -i cxd28xx-dkms_*.deb
```

To remove:

```sh
sudo dpkg -r cxd28xx-dkms
```

### Fedora / RHEL

```sh
sudo dnf install cxd28xx-dkms-*.rpm
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

Requires kernel headers for the running kernel.

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

## Firmware

The following firmware files are required and should be placed in `/lib/firmware/`:

| Device | Firmware |
|--------|----------|
| Zenview HDTV Mate | `dvb-usb-hdtvmate.fw` |
| TBS 5530 | `dvb-usb-id5530.fw` |

For Zenview HDTV Mate, obtain `liba3_phy_sony.so` by downloading the vendor's Android application and extracting the APK. Then use `utils/extract_firmware_hdtvmate.c` to extract the firmware blob from the library. Alternatively, `dvb-usb-it9303-01.fw` from the `linux-firmware` package can be used, but it has slightly worse performance on ATSC 3.0.

For TBS 5530, see [tbsdtv/linux_media/wiki](https://github.com/tbsdtv/linux_media/wiki#firmware).

## ATSC 3.0

### Tuning

Use the included `atsc3-zap` utility to tune ATSC 3.0 channels:

```sh
# Build the zapper
cc -o atsc3-zap utils/atsc3-zap.c

# Tune (frequency in Hz, optional PLP selection)
./atsc3-zap 701000000 --plp 0
```

### ALP Network Interface

ATSC 3.0 delivers IP data via the ALP (ATSC Link-layer Protocol). The driver creates a network interface `alp0` (or `alp1`, etc.) for each adapter at device init.

The `alp.ko` module implements the A/330 ALP protocol:

- Parses ALP single packets, segmented packets (A/330 §5.1.2.2), concatenated packets (A/330 §5.1.2.3), and header extensions (HM=1)
- Delivers IPv4 payloads to the kernel network stack via `netif_rx`
- Protocol-level statistics via `ethtool -S alp0`

Bridge drivers expose additional TS-level statistics (e.g. TEI count, reassembly errors) via sysfs under the USB device.

The ALP-div-TS decapsulation (CXD2878-specific 3-byte TS header format) is handled inside `cxd2878.ko` and feeds complete ALP packets to the generic `alp.ko` module.

### Example: South Korea ATSC 3.0

```sh
# 1. Bring up the ALP interface and assign an IP address BEFORE tuning
sudo ip link set alp0 up
sudo ip addr add 192.168.0.100/24 dev alp0

# 2. Tune to ATSC 3.0 (701 MHz, PLP 0 for non-scrambled MMT stream)
./atsc3-zap 701000000 --plp 0

# 3. Verify multicast traffic is flowing
tcpdump -i alp0
```

**Important notes:**

- It is recommended to configure the interface and IP address **before** tuning to avoid missing initial packets. `atsc3-zap` does not bring up the interface.
- A random IP address must be assigned to the interface for multicast reception to work.
- PLP 0 provides non-scrambled MMT stream in South Korea.
- Standard tools like ffplay cannot play MMT streams directly. An MMT demuxer is required.

## License

GPL-2.0-only. See [LICENSE](LICENSE) for details.
