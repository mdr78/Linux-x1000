/*
 * SC16IS7XX tty serial driver (I2C bus)
 *
 * Copyright (C) 2014 GridPoint
 * Author: Jon Ringle <jringle@gridpoint.com>
 * Copyright (C) 2015 Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/of_device.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/module.h>
#include <linux/i2c.h>

#include "sc16is7xx.h"

static int sc16is7xx_i2c_probe(struct i2c_client *i2c,
			       const struct i2c_device_id *id)
{
	unsigned long flags = 0;
	struct regmap *regmap;
	struct sc16is7xx_devtype *devtype;
	struct regmap_config regcfg;

	if (i2c->dev.of_node) {
		const struct of_device_id *of_id =
				of_match_device(sc16is7xx_dt_ids, &i2c->dev);
		if (of_id == NULL) {
			dev_err(&i2c->dev, "Error getting device id!\n");
			return -1;
		}
		devtype = (struct sc16is7xx_devtype *)of_id->data;
	} else {
		devtype = (struct sc16is7xx_devtype *)id->driver_data;
	}

	_dev_info(&i2c->dev, "device type: %s\n", devtype->name);
	flags = IRQF_TRIGGER_LOW;

	sc16is7xx_regmap_config_init(&regcfg, devtype->nr_uart);

	regmap = devm_regmap_init_i2c(i2c, &regcfg);
	if (IS_ERR(regmap)) {
		dev_err(&i2c->dev, "Error initialising the I2C regmap!\n");
		return -1;
	}

	return sc16is7xx_probe(&i2c->dev, devtype, regmap, i2c->irq, flags);
}

static int sc16is7xx_i2c_remove(struct i2c_client *client)
{
	return sc16is7xx_remove(&client->dev);
}

static const struct i2c_device_id sc16is7xx_i2c_id_table[] = {
	{ "sc16is74x",	(kernel_ulong_t)&sc16is74x_devtype, },
	{ "sc16is750",	(kernel_ulong_t)&sc16is750_devtype, },
	{ "sc16is752",	(kernel_ulong_t)&sc16is752_devtype, },
	{ "sc16is760",	(kernel_ulong_t)&sc16is760_devtype, },
	{ "sc16is762",	(kernel_ulong_t)&sc16is762_devtype, },
	{ }
};
MODULE_DEVICE_TABLE(i2c, sc16is7xx_i2c_id_table);

static struct i2c_driver sc16is7xx_i2c_uart_driver = {
	.driver = {
		.name		= SC16IS7XX_NAME,
		.owner		= THIS_MODULE,
		.of_match_table	= of_match_ptr(sc16is7xx_dt_ids),
	},
	.probe			= sc16is7xx_i2c_probe,
	.remove			= sc16is7xx_i2c_remove,
	.id_table		= sc16is7xx_i2c_id_table,
};
module_i2c_driver(sc16is7xx_i2c_uart_driver);

MODULE_ALIAS("i2c:sc16is7xx");

MODULE_AUTHOR("Intel Corporation");
MODULE_DESCRIPTION("SC16IS7XX tty serial driver over I2C bus");
MODULE_LICENSE("GPL v2");
