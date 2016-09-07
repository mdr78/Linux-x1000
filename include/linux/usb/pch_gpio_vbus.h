/*
 * Copyright(c) 2015 Intel Corporation.
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

#ifndef PCH_GPIO_VBUS_H
#define PCH_GPIO_VBUS_H

#include <linux/platform_data/pch_udc.h>

/**
 * Get platform data structure filled in by BSP driver.
 */
struct pch_udc_platform_data *get_pch_udc_platform_data(void);

#endif /* PCH_GPIO_VBUS_H */
