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

/*
 * Driver for when the BSP creates a pch-gpio-vbus platform device.
 * Used for passing the pdata from BSP layer to the PCI driver layer
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/usb/pch_gpio_vbus.h>

static struct pch_udc_platform_data *pdata;

struct pch_udc_platform_data *get_pch_udc_platform_data(void)
{
	return pdata;
}
EXPORT_SYMBOL_GPL(get_pch_udc_platform_data);

static int pch_udc_gpio_vbus_probe(struct platform_device *pdev)
{
	dev_info(&pdev->dev, "Probing driver...\n");
	pdata = pdev->dev.platform_data;
	if (!pdata)
		dev_warn(&pdev->dev, "pdata is NULL!\n");
	else
		dev_info(&pdev->dev, "pdata OK!\n");

	return 0;
}

static struct platform_driver pch_udc_gpio_vbus_driver = {
	.driver		= {
		.name	= "pch_gpio_vbus",
		.owner	= THIS_MODULE,
	},
	.probe		= pch_udc_gpio_vbus_probe,
	.remove		= NULL,
};
module_platform_driver(pch_udc_gpio_vbus_driver);

MODULE_AUTHOR("Intel Corporation");
MODULE_DESCRIPTION("USB pch-udc platform data");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:pch_gpio_vbus");
