/*
 * Copyright(c) 2013-2015 Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */
/*
 * Intel Quark side-band driver
 *
 * Thread-safe sideband read/write routine.
 *
 * Author : Bryan O'Donoghue <bryan.odonoghue@linux.intel.com> 2012
 */

#ifndef __INTEL_QRK_SB_H__
#define __INTEL_QRK_SB_H__

#include <linux/types.h>

#define PCI_DEVICE_ID_X1000_HOST_BRIDGE			0x0958

/* Sideband ID
 * RMU = Remote Management Unit Port
 * MM = Memory Manager Port
 */
typedef enum {
	SB_ID_RMU = 0x04,
	SB_ID_MM = 0x05,
}qrk_sb_id;

int intel_qrk_sb_read_reg(qrk_sb_id id, u8 cmd, u32 reg, u32 *data, u8 lock);
int intel_qrk_sb_write_reg(qrk_sb_id id, u8 cmd, u32 reg, u32 data, u8 lock);
int intel_qrk_sb_runfn_lock(int (*fn)( void * arg ), void * arg);

#endif /* __INTEL_QRK_SB_H__ */
