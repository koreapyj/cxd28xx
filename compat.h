/* SPDX-License-Identifier: GPL-2.0-or-later */
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

/* DVB-C2 delivery system (kernel 6.11+) */
#ifndef SYS_DVBC2
#define SYS_DVBC2 19
#endif

/* DVB-S2X FEC rates (kernel 6.2+) */
#ifndef FEC_2_5
#define FEC_2_5		12
#endif
#ifndef FEC_1_3
#define FEC_1_3		13
#endif
#ifndef FEC_1_4
#define FEC_1_4		14
#endif
#ifndef FEC_5_9
#define FEC_5_9		15
#endif
#ifndef FEC_7_9
#define FEC_7_9		16
#endif
#ifndef FEC_8_15
#define FEC_8_15	17
#endif
#ifndef FEC_11_15
#define FEC_11_15	18
#endif
#ifndef FEC_13_18
#define FEC_13_18	19
#endif
#ifndef FEC_9_20
#define FEC_9_20	20
#endif
#ifndef FEC_11_20
#define FEC_11_20	21
#endif
#ifndef FEC_23_36
#define FEC_23_36	22
#endif
#ifndef FEC_25_36
#define FEC_25_36	23
#endif
#ifndef FEC_13_45
#define FEC_13_45	24
#endif
#ifndef FEC_26_45
#define FEC_26_45	25
#endif
#ifndef FEC_28_45
#define FEC_28_45	26
#endif
#ifndef FEC_32_45
#define FEC_32_45	27
#endif
#ifndef FEC_77_90
#define FEC_77_90	28
#endif
#ifndef FEC_11_45
#define FEC_11_45	29
#endif
#ifndef FEC_4_15
#define FEC_4_15	30
#endif
#ifndef FEC_14_45
#define FEC_14_45	31
#endif
#ifndef FEC_7_15
#define FEC_7_15	32
#endif

/* DVB-S2X modulation (kernel 6.2+) */
#ifndef APSK_64
#define APSK_64		14
#endif

/* DVB-S2X rolloff (kernel 6.2+) */
#ifndef ROLLOFF_15
#define ROLLOFF_15	4
#endif
#ifndef ROLLOFF_10
#define ROLLOFF_10	5
#endif
#ifndef ROLLOFF_5
#define ROLLOFF_5	6
#endif

/* i2c driver struct changed across kernel versions:
 * < 6.1: .remove returns int, .probe takes two args
 * 6.1-6.2: .remove returns void, .probe_new for single-arg
 * >= 6.3: .remove returns void, .probe_new removed, .probe takes single-arg
 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 1, 0)
#define CXD28XX_I2C_PROBE .probe_new
#define CXD28XX_I2C_REMOVE_RETURN int
#define CXD28XX_I2C_REMOVE_EPILOGUE return 0;
#elif LINUX_VERSION_CODE < KERNEL_VERSION(6, 3, 0)
#define CXD28XX_I2C_PROBE .probe_new
#define CXD28XX_I2C_REMOVE_RETURN void
#define CXD28XX_I2C_REMOVE_EPILOGUE
#else
#define CXD28XX_I2C_PROBE .probe
#define CXD28XX_I2C_REMOVE_RETURN void
#define CXD28XX_I2C_REMOVE_EPILOGUE
#endif

#endif /* _CXD28XX_COMPAT_H_ */
