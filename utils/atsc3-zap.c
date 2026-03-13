/* SPX-License-Identifier: GPL-2.0-only */
/*
 * atsc3-zap - ATSC 3.0 tuning utility for cxd28xx driver.
 *
 * Tunes a DVB frontend to an ATSC 3.0 channel using DVBv5 ioctls
 * (bypassing dvbv5-zap which cannot handle SYS_ATSC3), brings up
 * the associated atscN network interface on lock, and keeps running
 * until interrupted.
 *
 * Usage: atsc3-zap <freq_hz> [--plp <id>] [-a <adapter>] [-f <frontend>] [-r]
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
#include <poll.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/dvb/frontend.h>
#include <linux/dvb/dmx.h>

#ifndef SYS_ATSC3
#define SYS_ATSC3 35
#endif

#ifndef NO_STREAM_ID_FILTER
#define NO_STREAM_ID_FILTER (~0U)
#endif

static volatile int running = 1;

static void signal_handler(int sig)
{
	(void)sig;
	running = 0;
}

static void usage(const char *prog)
{
	fprintf(stderr,
		"Usage: %s <freq_hz> [--plp <id>] [-a <adapter>] [-f <frontend>] [-r]\n"
		"\n"
		"  <freq_hz>       RF frequency in Hz (e.g. 599000000)\n"
		"  --plp <id>      PLP ID 0-63 (omit for all PLPs)\n"
		"  -a <adapter>    DVB adapter number (default 0)\n"
		"  -f <frontend>   Frontend number (default 0)\n"
		"  -r              Record full TS to stdout\n",
		prog);
}

int main(int argc, char **argv)
{
	unsigned int freq = 0, bw = 6000000;
	unsigned int stream_id = NO_STREAM_ID_FILTER;
	int adapter = 0, frontend = 0, record = 0;
	char fe_path[64];
	int fe_fd, dmx_fd = -1, dvr_fd = -1, i;

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
		} else if (strcmp(argv[i], "-r") == 0) {
			record = 1;
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
		fprintf(stderr, "Tuning %u Hz, PLP %u ...\n", freq, stream_id);
	else
		fprintf(stderr, "Tuning %u Hz, all PLPs ...\n", freq);

	/* Set up demux and DVR for recording */
	if (record) {
		char dmx_path[64], dvr_path[64];
		struct dmx_pes_filter_params filter;

		snprintf(dmx_path, sizeof(dmx_path),
			 "/dev/dvb/adapter%d/demux0", adapter);
		snprintf(dvr_path, sizeof(dvr_path),
			 "/dev/dvb/adapter%d/dvr0", adapter);

		dmx_fd = open(dmx_path, O_RDWR);
		if (dmx_fd < 0) {
			fprintf(stderr, "Cannot open %s: %s\n",
				dmx_path, strerror(errno));
			goto out;
		}

		memset(&filter, 0, sizeof(filter));
		filter.pid = 0x2000;
		filter.input = DMX_IN_FRONTEND;
		filter.output = DMX_OUT_TS_TAP;
		filter.pes_type = DMX_PES_OTHER;
		filter.flags = DMX_IMMEDIATE_START;

		if (ioctl(dmx_fd, DMX_SET_PES_FILTER, &filter) < 0) {
			perror("DMX_SET_PES_FILTER");
			goto out;
		}

		dvr_fd = open(dvr_path, O_RDONLY);
		if (dvr_fd < 0) {
			fprintf(stderr, "Cannot open %s: %s\n",
				dvr_path, strerror(errno));
			goto out;
		}
	}

	/* Main loop */
	if (record) {
		unsigned char buf[188 * 348];
		struct pollfd pfd = { .fd = dvr_fd, .events = POLLIN };

		while (running) {
			if (poll(&pfd, 1, 100) > 0) {
				ssize_t n = read(dvr_fd, buf, sizeof(buf));

				if (n > 0) {
					if (write(STDOUT_FILENO, buf, n) != n) {
						perror("write stdout");
						break;
					}
				} else if (n < 0 && errno != EINTR &&
					   errno != EAGAIN) {
					perror("read dvr");
					break;
				}
			}
		}
	} else {
		while (running) {
			enum fe_status status = 0;

			if (ioctl(fe_fd, FE_READ_STATUS, &status) < 0) {
				perror("FE_READ_STATUS");
				break;
			}

			if (status & FE_HAS_LOCK) {
				fprintf(stderr, "Locked.\n");
				usleep(500000);
			} else {
				fprintf(stderr, "Lock lost.\n");
				usleep(100000);
			}
		}
	}

	/* Cleanup */
	fprintf(stderr, "\nStopping.\n");

out:
	if (dvr_fd >= 0)
		close(dvr_fd);
	if (dmx_fd >= 0)
		close(dmx_fd);
	close(fe_fd);
	return 0;
}
