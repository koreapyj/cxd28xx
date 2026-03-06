/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Compat shims for building on older kernels (e.g. Debian 12 / kernel 6.1).
 * DVB-S2X enums (FEC, modulation, rolloff) were added in kernel 6.2.
 * The i2c .probe_new → .probe rename happened in kernel 6.3.
 */
#ifndef _CXD28XX_COMPAT_H_
#define _CXD28XX_COMPAT_H_

#include <linux/version.h>

/* ATSC 3.0 delivery system (not in kernel) */
#ifndef SYS_ATSC3
#define SYS_ATSC3 35
#endif

#endif /* _CXD28XX_COMPAT_H_ */
