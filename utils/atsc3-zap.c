/* SPX-License-Identifier: GPL-2.0-only */
/*
 * atsc3-zap - ATSC 3.0 tuning utility for cxd28xx driver.
 *
 * Tunes a DVB frontend to an ATSC 3.0 channel using DVBv5 ioctls
 * (bypassing dvbv5-zap which cannot handle SYS_ATSC3), brings up
 * the associated atscN network interface on lock, and keeps running
 * until interrupted.
 *
 * Usage: atsc3-zap <freq_hz> [--plp <id>] [-a <adapter>] [-f <frontend>]
 *
 * Copyright (c) 2026 Yoonji Park <koreapyj@dcmys.kr>
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <signal.h>
#include <dirent.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/dvb/frontend.h>

#ifndef SYS_ATSC3
#define SYS_ATSC3 35
#endif

#ifndef NO_STREAM_ID_FILTER
#define NO_STREAM_ID_FILTER (~0U)
#endif

static volatile int running = 1;
static char netif_name[IFNAMSIZ];
static int sock_fd = -1;

static void signal_handler(int sig)
{
	(void)sig;
	running = 0;
}

static int netif_set(const char *ifname, int up)
{
	struct ifreq ifr;

	if (sock_fd < 0)
		return -1;

	memset(&ifr, 0, sizeof(ifr));
	snprintf(ifr.ifr_name, IFNAMSIZ, "%s", ifname);

	if (ioctl(sock_fd, SIOCGIFFLAGS, &ifr) < 0)
		return -1;

	if (up)
		ifr.ifr_flags |= IFF_UP;
	else
		ifr.ifr_flags &= ~IFF_UP;

	return ioctl(sock_fd, SIOCSIFFLAGS, &ifr);
}

/*
 * Find the atscN interface belonging to this DVB adapter.
 *
 * The driver sets the netdev parent to the DVB adapter device via
 * SET_NETDEV_DEV. We compare the sysfs device symlink of each atscN
 * interface with the DVB adapter's device path.
 */
static int find_netif(int adapter)
{
	char adapter_dev[256], net_dev[256], adapter_real[PATH_MAX],
	     net_real[PATH_MAX];
	struct dirent *de;
	DIR *dir;

	snprintf(adapter_dev, sizeof(adapter_dev),
		 "/sys/class/dvb/dvb%d.frontend0/device", adapter);
	if (!realpath(adapter_dev, adapter_real))
		return -1;

	dir = opendir("/sys/class/net");
	if (!dir)
		return -1;

	while ((de = readdir(dir)) != NULL) {
		if (strncmp(de->d_name, "atsc", 4) != 0)
			continue;
		if (strlen(de->d_name) >= IFNAMSIZ)
			continue;

		snprintf(net_dev, sizeof(net_dev),
			 "/sys/class/net/%.15s/device", de->d_name);
		if (!realpath(net_dev, net_real))
			continue;

		if (strcmp(adapter_real, net_real) == 0) {
			memcpy(netif_name, de->d_name,
			       strlen(de->d_name) + 1);
			closedir(dir);
			return 0;
		}
	}

	closedir(dir);
	return -1;
}

static void usage(const char *prog)
{
	fprintf(stderr,
		"Usage: %s <freq_hz> [--plp <id>] [-a <adapter>] [-f <frontend>]\n"
		"\n"
		"  <freq_hz>       RF frequency in Hz (e.g. 599000000)\n"
		"  --plp <id>      PLP ID 0-63 (omit for all PLPs)\n"
		"  -a <adapter>    DVB adapter number (default 0)\n"
		"  -f <frontend>   Frontend number (default 0)\n",
		prog);
}

int main(int argc, char **argv)
{
	unsigned int freq = 0, bw = 6000000;
	unsigned int stream_id = NO_STREAM_ID_FILTER;
	int adapter = 0, frontend = 0;
	char fe_path[64];
	int fe_fd, i;
	int netif_up = 0;

	/* Parse arguments */
	if (argc < 2) {
		usage(argv[0]);
		return 1;
	}

	freq = strtoul(argv[1], NULL, 0);
	if (freq == 0) {
		fprintf(stderr, "Invalid frequency: %s\n", argv[1]);
		return 1;
	}

	for (i = 2; i < argc; i++) {
		if (strcmp(argv[i], "--plp") == 0 && i + 1 < argc) {
			stream_id = strtoul(argv[++i], NULL, 0);
			if (stream_id > 63) {
				fprintf(stderr, "PLP ID must be 0-63\n");
				return 1;
			}
		} else if (strcmp(argv[i], "-a") == 0 && i + 1 < argc) {
			adapter = atoi(argv[++i]);
		} else if (strcmp(argv[i], "-f") == 0 && i + 1 < argc) {
			frontend = atoi(argv[++i]);
		} else {
			usage(argv[0]);
			return 1;
		}
	}

	/* Open frontend */
	snprintf(fe_path, sizeof(fe_path),
		 "/dev/dvb/adapter%d/frontend%d", adapter, frontend);
	fe_fd = open(fe_path, O_RDWR);
	if (fe_fd < 0) {
		fprintf(stderr, "Cannot open %s: %s\n", fe_path, strerror(errno));
		return 1;
	}

	/* Socket for netif control */
	sock_fd = socket(AF_INET, SOCK_DGRAM, 0);

	/* Setup signal handlers */
	signal(SIGINT, signal_handler);
	signal(SIGTERM, signal_handler);

	/* Clear frontend state */
	{
		struct dtv_property p = { .cmd = DTV_CLEAR };
		struct dtv_properties cmd = { .num = 1, .props = &p };

		if (ioctl(fe_fd, FE_SET_PROPERTY, &cmd) < 0) {
			perror("FE_SET_PROPERTY DTV_CLEAR");
			goto out;
		}
	}

	/* Tune ATSC 3.0 */
	{
		struct dtv_property props[6];
		struct dtv_properties cmd;
		int n = 0;

		memset(props, 0, sizeof(props));

		props[n].cmd = DTV_DELIVERY_SYSTEM;
		props[n].u.data = SYS_ATSC3;
		n++;

		props[n].cmd = DTV_FREQUENCY;
		props[n].u.data = freq;
		n++;

		props[n].cmd = DTV_BANDWIDTH_HZ;
		props[n].u.data = bw;
		n++;

		if (stream_id != NO_STREAM_ID_FILTER) {
			props[n].cmd = DTV_STREAM_ID;
			props[n].u.data = stream_id;
			n++;
		}

		props[n].cmd = DTV_TUNE;
		n++;

		cmd.num = n;
		cmd.props = props;

		if (ioctl(fe_fd, FE_SET_PROPERTY, &cmd) < 0) {
			perror("FE_SET_PROPERTY DTV_TUNE");
			goto out;
		}
	}

	if (stream_id != NO_STREAM_ID_FILTER)
		printf("Tuning %u Hz, PLP %u ...\n", freq, stream_id);
	else
		printf("Tuning %u Hz, all PLPs ...\n", freq);

	/* Poll for lock */
	while (running) {
		enum fe_status status = 0;

		if (ioctl(fe_fd, FE_READ_STATUS, &status) < 0) {
			perror("FE_READ_STATUS");
			break;
		}

		if (status & FE_HAS_LOCK) {
			if (!netif_up) {
				printf("Locked.\n");

				/* Bring up atsc netif */
				if (find_netif(adapter) == 0) {
					if (netif_set(netif_name, 1) == 0) {
						printf("Interface %s up.\n",
						       netif_name);
						netif_up = 1;
					} else {
						fprintf(stderr,
							"Failed to bring up %s: %s\n",
							netif_name,
							strerror(errno));
					}
				} else {
					fprintf(stderr,
						"No atsc interface found for adapter %d\n",
						adapter);
				}
			}
			usleep(500000);
		} else {
			if (netif_up) {
				printf("Lock lost.\n");
				netif_set(netif_name, 0);
				netif_up = 0;
			}
			usleep(100000);
		}
	}

	/* Cleanup */
	printf("\nStopping.\n");
	if (netif_up)
		netif_set(netif_name, 0);

out:
	if (sock_fd >= 0)
		close(sock_fd);
	close(fe_fd);
	return 0;
}
