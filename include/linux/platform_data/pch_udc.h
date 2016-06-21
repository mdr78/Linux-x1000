/*
 * Intel QUARK/EG20T OCH/LAPIS Semiconductor UDC driver
 *
 * Copyright(c) 2015 Intel Corporation
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

#ifndef LINUX_PLATFORM_DATA_PCH_UDC_H
#define LINUX_PLATFORM_DATA_PCH_UDC_H

/**
 * struct pch_udc_platform_data - Structure holding GPIO informaton
 *				  for detecting VBUS
 * @vbus_gpio_port: gpio port number
 */
struct pch_udc_platform_data {
	int	vbus_gpio_port;
};

#endif /* LINUX_PLATFORM_DATA_PCH_UDC_H */
