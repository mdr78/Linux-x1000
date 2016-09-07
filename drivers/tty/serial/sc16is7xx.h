/*
 * SC16IS7xx tty serial driver
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

#ifndef _SC16IS7XX_H_
#define _SC16IS7XX_H_

/* Misc definition used in core module (sc16is7xx.c) and in I2C/SPI bus
 * specific modules */
#define SC16IS7XX_NAME			"sc16is7xx"
#define SC16IS7XX_REG_SHIFT		2

struct sc16is7xx_devtype {
	char	name[10];
	int	nr_gpio;
	int	nr_uart;
};

static struct sc16is7xx_devtype sc16is74x_devtype = {
	.name		= "SC16IS74X",
	.nr_gpio	= 0,
	.nr_uart	= 1,
};

static struct sc16is7xx_devtype sc16is750_devtype = {
	.name		= "SC16IS750",
	.nr_gpio	= 8,
	.nr_uart	= 1,
};

static struct sc16is7xx_devtype sc16is752_devtype = {
	.name		= "SC16IS752",
	.nr_gpio	= 8,
	.nr_uart	= 2,
};

static struct sc16is7xx_devtype sc16is760_devtype = {
	.name		= "SC16IS760",
	.nr_gpio	= 8,
	.nr_uart	= 1,
};

static struct sc16is7xx_devtype sc16is762_devtype = {
	.name		= "SC16IS762",
	.nr_gpio	= 8,
	.nr_uart	= 2,
};

static const struct of_device_id __maybe_unused sc16is7xx_dt_ids[] = {
	{ .compatible = "nxp,sc16is740",	.data = &sc16is74x_devtype, },
	{ .compatible = "nxp,sc16is741",	.data = &sc16is74x_devtype, },
	{ .compatible = "nxp,sc16is750",	.data = &sc16is750_devtype, },
	{ .compatible = "nxp,sc16is752",	.data = &sc16is752_devtype, },
	{ .compatible = "nxp,sc16is760",	.data = &sc16is760_devtype, },
	{ .compatible = "nxp,sc16is762",	.data = &sc16is762_devtype, },
	{ }
};
MODULE_DEVICE_TABLE(of, sc16is7xx_dt_ids);

int sc16is7xx_probe(struct device *dev, struct sc16is7xx_devtype *devtype,
		struct regmap *regmap, int irq, unsigned long flags);

int sc16is7xx_remove(struct device *dev);

void sc16is7xx_regmap_config_init(struct regmap_config *regcfg, int nr_uart);

#endif /* _SC16IS7XX_H_ */
