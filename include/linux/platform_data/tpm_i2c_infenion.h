/*
 * Platform data for tpm_i2c_infenion driver
 *
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

#ifndef __LINUX_PLATFORM_DATA_TPM_I2C_INFENION_H__
#define __LINUX_PLATFORM_DATA_TPM_I2C_INFENION_H__

#include <linux/kernel.h>
#include <linux/i2c.h>

struct tpm_i2c_infenion_platform_data {
	int gpio_irq;
	int gpio_reset;
	struct i2c_client *client;
	u8 *tpm_i2c_buffer[2]; /* 0 Request 1 Response */
	struct completion irq_detection;
	struct mutex lock;
};

#endif	/* __LINUX_PLATFORM_DATA_TPM_I2C_INFENION_H__ */
