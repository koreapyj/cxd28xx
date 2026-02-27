/*
 * PC/SC IFD handler for IT930x smartcard reader
 *
 * Bridges pcscd to the kernel it930x.ko chardev interface.
 * Build: gcc -shared -fPIC -o libifd_it930x.so ifd_it930x.c $(pkg-config --cflags libpcsclite)
 *
 * Copyright (c) 2026 Yoonji Park <koreapyj@dcmys.kr>
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <ifdhandler.h>

#include "../it930x_smartcard.h"

#define MAX_SLOTS	16

static struct {
	int	fd;
	char	device[256];
	__u8	atr[IT930X_SC_MAX_ATR];
	int	atr_len;
} slots[MAX_SLOTS];

static int lun_to_idx(DWORD lun)
{
	int idx = (lun >> 16) & 0xFFFF;

	if (idx >= MAX_SLOTS)
		return -1;
	return idx;
}

RESPONSECODE IFDHCreateChannelByName(DWORD lun, LPSTR deviceName)
{
	int idx = lun_to_idx(lun);

	if (idx < 0)
		return IFD_COMMUNICATION_ERROR;

	slots[idx].fd = open(deviceName, O_RDWR);
	if (slots[idx].fd < 0)
		return IFD_COMMUNICATION_ERROR;

	strncpy(slots[idx].device, deviceName, sizeof(slots[idx].device) - 1);
	slots[idx].atr_len = 0;

	return IFD_SUCCESS;
}

RESPONSECODE IFDHCreateChannel(DWORD lun, DWORD channel)
{
	char name[64];

	snprintf(name, sizeof(name), "/dev/it930x_smartcard%lu",
		 (unsigned long)channel);
	return IFDHCreateChannelByName(lun, name);
}

RESPONSECODE IFDHCloseChannel(DWORD lun)
{
	int idx = lun_to_idx(lun);

	if (idx < 0)
		return IFD_COMMUNICATION_ERROR;

	if (slots[idx].fd >= 0) {
		close(slots[idx].fd);
		slots[idx].fd = -1;
	}

	return IFD_SUCCESS;
}

RESPONSECODE IFDHGetCapabilities(DWORD lun, DWORD tag, PDWORD length,
				 PUCHAR value)
{
	int idx = lun_to_idx(lun);

	if (idx < 0)
		return IFD_COMMUNICATION_ERROR;

	switch (tag) {
	case TAG_IFD_ATR:
		if (slots[idx].atr_len == 0) {
			*length = 0;
			return IFD_ICC_NOT_PRESENT;
		}
		if (*length < (DWORD)slots[idx].atr_len)
			return IFD_ERROR_INSUFFICIENT_BUFFER;
		memcpy(value, slots[idx].atr, slots[idx].atr_len);
		*length = slots[idx].atr_len;
		return IFD_SUCCESS;

	case TAG_IFD_SLOTS_NUMBER:
		if (*length < 1)
			return IFD_ERROR_INSUFFICIENT_BUFFER;
		*value = 1;
		*length = 1;
		return IFD_SUCCESS;

	case TAG_IFD_SIMULTANEOUS_ACCESS:
		if (*length < 1)
			return IFD_ERROR_INSUFFICIENT_BUFFER;
		*value = 1;
		*length = 1;
		return IFD_SUCCESS;

	default:
		return IFD_ERROR_TAG;
	}
}

RESPONSECODE IFDHSetCapabilities(DWORD lun, DWORD tag, DWORD length,
				 PUCHAR value)
{
	(void)lun;
	(void)tag;
	(void)length;
	(void)value;
	return IFD_SUCCESS;
}

RESPONSECODE IFDHSetProtocolParameters(DWORD lun, DWORD protocol,
				       UCHAR flags, UCHAR pts1,
				       UCHAR pts2, UCHAR pts3)
{
	(void)lun;
	(void)protocol;
	(void)flags;
	(void)pts1;
	(void)pts2;
	(void)pts3;
	return IFD_SUCCESS;
}

RESPONSECODE IFDHPowerICC(DWORD lun, DWORD action, PUCHAR atr,
			  PDWORD atrLength)
{
	int idx = lun_to_idx(lun);
	struct it930x_sc_atr atr_data;

	if (idx < 0)
		return IFD_COMMUNICATION_ERROR;

	switch (action) {
	case IFD_POWER_UP:
	case IFD_RESET:
		if (ioctl(slots[idx].fd, IT930X_SC_RESET, &atr_data) < 0)
			return IFD_COMMUNICATION_ERROR;
		memcpy(slots[idx].atr, atr_data.atr, atr_data.len);
		slots[idx].atr_len = atr_data.len;
		if (atr && atrLength) {
			memcpy(atr, atr_data.atr, atr_data.len);
			*atrLength = atr_data.len;
		}
		return IFD_SUCCESS;

	case IFD_POWER_DOWN:
		slots[idx].atr_len = 0;
		if (atrLength)
			*atrLength = 0;
		return IFD_SUCCESS;

	default:
		return IFD_NOT_SUPPORTED;
	}
}

RESPONSECODE IFDHTransmitToICC(DWORD lun, SCARD_IO_HEADER sendPci,
			       PUCHAR txBuffer, DWORD txLength,
			       PUCHAR rxBuffer, PDWORD rxLength,
			       PSCARD_IO_HEADER recvPci)
{
	int idx = lun_to_idx(lun);
	ssize_t n;

	(void)sendPci;

	if (idx < 0)
		return IFD_COMMUNICATION_ERROR;

	n = write(slots[idx].fd, txBuffer, txLength);
	if (n < 0 || (DWORD)n != txLength)
		return IFD_COMMUNICATION_ERROR;

	n = read(slots[idx].fd, rxBuffer, *rxLength);
	if (n < 0)
		return IFD_COMMUNICATION_ERROR;

	*rxLength = n;
	if (recvPci)
		recvPci->Protocol = 0;

	return IFD_SUCCESS;
}

RESPONSECODE IFDHICCPresence(DWORD lun)
{
	int idx = lun_to_idx(lun);
	int present = 0;

	if (idx < 0)
		return IFD_COMMUNICATION_ERROR;

	if (ioctl(slots[idx].fd, IT930X_SC_CARD_DETECT, &present) < 0)
		return IFD_COMMUNICATION_ERROR;

	return present ? IFD_ICC_PRESENT : IFD_ICC_NOT_PRESENT;
}

RESPONSECODE IFDHControl(DWORD lun, DWORD controlCode,
			 PUCHAR txBuffer, DWORD txLength,
			 PUCHAR rxBuffer, DWORD rxLength,
			 PDWORD bytesReturned)
{
	(void)lun;
	(void)controlCode;
	(void)txBuffer;
	(void)txLength;
	(void)rxBuffer;
	(void)rxLength;
	if (bytesReturned)
		*bytesReturned = 0;
	return IFD_ERROR_NOT_SUPPORTED;
}
