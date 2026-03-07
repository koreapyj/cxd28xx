/* SPDX-License-Identifier: GPL-2.0-or-later WITH Linux-syscall-note */
/*
 * IT930x smartcard reader ioctl interface
 *
 * Shared between kernel driver (it930x.ko) and user-space IFD handler.
 */
#ifndef _IT930X_SMARTCARD_H
#define _IT930X_SMARTCARD_H

#ifdef __KERNEL__
#include <linux/ioctl.h>
#include <linux/types.h>
#else
#include <sys/ioctl.h>
#include <stdint.h>
typedef uint8_t __u8;
#endif

#define IT930X_SC_MAX_ATR	32

struct it930x_sc_atr {
	__u8	atr[IT930X_SC_MAX_ATR];
	__u8	len;
};

#define IT930X_SC_IOC_MAGIC		'I'
#define IT930X_SC_CARD_DETECT		_IOR(IT930X_SC_IOC_MAGIC, 0, int)
#define IT930X_SC_RESET			_IOR(IT930X_SC_IOC_MAGIC, 1, struct it930x_sc_atr)
#define IT930X_SC_GET_ATR		_IOR(IT930X_SC_IOC_MAGIC, 2, struct it930x_sc_atr)
#define IT930X_SC_SET_BAUDRATE		_IOW(IT930X_SC_IOC_MAGIC, 3, int)

#endif /* _IT930X_SMARTCARD_H */
