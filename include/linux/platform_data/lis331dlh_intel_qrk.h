/*
 * Platform data for Intel Clanton Hill platform accelerometer driver
 *
 * Copyright(c) 2013 Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details
 *
 */

#ifndef __LINUX_PLATFORM_DATA_LIS331DLH_INTEL_QRK_H__
#define __LINUX_PLATFORM_DATA_LIS331DLH_INTEL_QRK_H__

#include <linux/iio/common/st_sensors.h>

/**
 * struct lis331dlh_intel_qrk_platform_data - Platform data for the ST Micro
 *                                            accelerometer driver
 * @irq2_pin: GPIO pin number for the threshold interrupt(INT2).
 **/
/**
* struct st_sensors_platform_data - default accel platform data
* @drdy_int_pin: default accel DRDY is available on INT1 pin.
*/

struct lis331dlh_intel_qrk_platform_data {
	int irq2_pin;
	struct st_sensors_platform_data *default_lis331dlh_pdata;

};



#endif /* LINUX_PLATFORM_DATA_LIS331DLH_INTEL_QRK_H_ */
