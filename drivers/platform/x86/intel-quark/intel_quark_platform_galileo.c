/*
 * Copyright(c) 2013 Intel Corporation.
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
 * Intel Quark Legacy Platform Data accessor layer
 *
 * Simple Legacy SPI flash access layer
 *
 * Author : Bryan O'Donoghue <bryan.odonoghue@linux.intel.com> 2013
 */

#include <linux/dmi.h>
#include <linux/errno.h>
#include <linux/ioport.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/printk.h>

#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/platform_data/at24.h>
#include <linux/io.h>
#include <linux/mfd/cy8c9540a.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/physmap.h>
#include <linux/spi/pxa2xx_spi.h>
#include <linux/spi/spi.h>
#include <linux/spi/flash.h>
#include <linux/mfd/intel_qrk_gip_pdata.h>

#define DRIVER_NAME "Galileo"
#define GPIO_RESTRICT_NAME	"gpio-restrict-sc"
#define LPC_SCH_SPINAME		"spi-lpc-sch"

/* GPIO line used to detect the LSB of the Cypress i2c address */
#define GPIO_CYPRESS_A0			7
/* GPIO line Cypress interrupts are routed to (in S0 power state) */
#define GPIO_CYPRESS_INT_S0		13
/* GPIO line Cypress interrupts are routed to (in S3 power state) */
#define GPIO_CYPRESS_INT_S3		2

/* Cypress i2c address depending on A0 value */
#define CYPRESS_ADDR_A0_1		0x20
#define CYPRESS_ADDR_A0_0		0x21
#define EEPROM_ADDR_A0_1		0x50
#define EEPROM_ADDR_A0_0		0x51


/******************************************************************************
 *                   Cypress I/O Expander Platform Data
 ******************************************************************************/
static struct cy8c9540a_pdata cy8c9540a_platform_data = {
	.por_default		= {
		0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,	/* Output */
		0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, /* Int mask */
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* PWM */
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* Inversion */
		0xe0, 0xe0, 0xff, 0xf3, 0x00, 0xff, 0xff, 0xff, /* Direction */
		0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f,	/* P0 drive */
		0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f,	/* P1 drive */
		0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,	/* P2 drive */
		0xf3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0c,	/* P3 drive */
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff,	/* P4 drive */
		0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,	/* P5 drive */
		0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,	/* P6 drive */
		0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,	/* P7 drive */
		0x00, 0xff, 0x00,				/* PWM0 */
		0x00, 0xff, 0x00,				/* PWM1 */
		0x00, 0xff, 0x00,				/* PWM2 */
		0x00, 0xff, 0x00,				/* PWM3 */
		0x00, 0xff, 0x00,				/* PWM4 */
		0x00, 0xff, 0x00,				/* PWM5 */
		0x00, 0xff, 0x00,				/* PWM6 */
		0x00, 0xff, 0x00,				/* PWM7 */
		0x00, 0xff, 0x00,				/* PWM8 */
		0x00, 0xff, 0x00,				/* PWM9 */
		0x00, 0xff, 0x00,				/* PWM10 */
		0x00, 0xff, 0x00,				/* PWM11 */
		0x00, 0xff, 0x00,				/* PWM12 */
		0x00, 0xff, 0x00,				/* PWM13 */
		0x00, 0xff, 0x00,				/* PWM14 */
		0x00, 0xff, 0x00,				/* PWM15 */
		0xff,						/* PWM CLKdiv */
		0x02,						/* EEPROM en */
		0x00						/* CRC */
	},
	.pwm2gpio_mapping	= {
		CY8C9540A_PWM_UNUSED,
		3,
		CY8C9540A_PWM_UNUSED,
		2,
		9,
		1,
		8,
		0,
	},
	.gpio_base		= 16,
	.pwm_base		= 0,
	.irq_base		= 64,
};

/* Cypress expander requires i2c master to operate @100kHz 'standard mode' */
static struct intel_qrk_gip_pdata gip_pdata = {
	.i2c_std_mode = 1,
};
static struct intel_qrk_gip_pdata *galileo_gip_get_pdata(void)
{
	return &gip_pdata;
}

/******************************************************************************
 *             Analog Devices AD7298 SPI Device Platform Data
 ******************************************************************************/
#include "linux/platform_data/ad7298.h"

/* Maximum input voltage allowed for each ADC input, in milliVolts */
#define AD7298_MAX_EXT_VIN 5000

static const struct ad7298_platform_data ad7298_platform_data = {
	.ext_ref = false,
	.ext_vin_max = { AD7298_MAX_EXT_VIN, AD7298_MAX_EXT_VIN,
		AD7298_MAX_EXT_VIN, AD7298_MAX_EXT_VIN,
		AD7298_MAX_EXT_VIN, AD7298_MAX_EXT_VIN,
		AD7298_MAX_EXT_VIN, AD7298_MAX_EXT_VIN }
};

static struct at24_platform_data at24_platform_data = {
	.byte_len = (11 * 1024),
	.page_size = 1,
	.flags = AT24_FLAG_ADDR16,
};

/******************************************************************************
 *                        Intel Izmir i2c clients
 ******************************************************************************/
static struct i2c_board_info probed_i2c_cypress = {
	.platform_data = &cy8c9540a_platform_data,
};
static struct i2c_board_info probed_i2c_eeprom = {
	.platform_data = &at24_platform_data,
};
static struct i2c_adapter *i2c_adap;
static const unsigned short cypress_i2c_addr[] = {
	CYPRESS_ADDR_A0_1,
	CYPRESS_ADDR_A0_0,
	I2C_CLIENT_END
};

static const unsigned short eeprom_i2c_addr[] = {
	EEPROM_ADDR_A0_1,
	EEPROM_ADDR_A0_0,
	I2C_CLIENT_END
};

/******************************************************************************
 *                 Intel Quark SPI Controller Data
 ******************************************************************************/
static struct pxa2xx_spi_chip qrk_ffrd_spi_0_cs_0 = {
	.gpio_cs = 8,
};

static struct pxa2xx_spi_chip qrk_ffrd_spi_1_cs_0 = {
	.gpio_cs = 10,
};

#define LPC_SCH_SPI_BUS_ID 0x03

static struct spi_board_info spi_onboard_devs[] = {
	{
		.modalias = "ad7298",
		.max_speed_hz = 5000000,
		.platform_data = &ad7298_platform_data,
		.mode = SPI_MODE_2,
		.bus_num = 0,
		.chip_select = 0,
		.controller_data = &qrk_ffrd_spi_0_cs_0,
	},
	{
		.modalias = "spidev",
		.chip_select = 0,
		.controller_data = &qrk_ffrd_spi_1_cs_0,
		.max_speed_hz = 50000000,
		.bus_num = 1,
	},
};

static struct gpio reserved_gpios[] = {
	{
		GPIO_CYPRESS_A0,
		GPIOF_IN,
		"cy8c9540a-a0",
	},
	{
		GPIO_CYPRESS_INT_S0,
		GPIOF_IN,
		"cy8c9540a-int-s0",
	},
	{
		GPIO_CYPRESS_INT_S3,
		GPIOF_IN,
		"cy8c9540a-int-s3",
	},
};

static int eeprom_i2c_probe(struct i2c_adapter *adap, unsigned short addr)
{
	if (gpio_get_value(GPIO_CYPRESS_A0) && EEPROM_ADDR_A0_1 == addr)
		return 1;
	if (!gpio_get_value(GPIO_CYPRESS_A0) && EEPROM_ADDR_A0_0 == addr)
		return 1;
	return 0;
}
static int cypress_i2c_probe(struct i2c_adapter *adap, unsigned short addr)
{
	if (gpio_get_value(GPIO_CYPRESS_A0) && CYPRESS_ADDR_A0_1 == addr)
		return 1;
	if (!gpio_get_value(GPIO_CYPRESS_A0) && CYPRESS_ADDR_A0_0 == addr)
		return 1;
	return 0;
}

/**
 * intel_qrk_spi_add_onboard_devs
 *
 * @return 0 on success or standard errnos on failure
 *
 * Registers onboard SPI device(s) present on the Izmir platform
 */
static int intel_qrk_spi_add_onboard_devs(void)
{
	return spi_register_board_info(spi_onboard_devs,
			ARRAY_SIZE(spi_onboard_devs));
}

/**
 * intel_qrk_gpio_restrict_probe
 *
 * Register devices that depend on GPIOs.
 * Note this function makes extensive use of the probe deferral mechanism:
 * gpio_request() for a GPIO that is not yet available returns
 * -EPROBE_DEFER.
 */
static int intel_qrk_gpio_restrict_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct i2c_client *cypress = NULL, *eeprom = NULL;

	/* The sequence of calling spi, gpio & i2c was different and an issue
	 * was found that the system would lock-up later when running without
	 * APIC. Calling sequence is now changed to follow the Galileo Gen2. 
	 */
	i2c_adap = i2c_get_adapter(0);
	if (NULL == i2c_adap) {
		pr_info("%s: i2c adapter not ready yet. Deferring..\n",
			__func__);
		return -EPROBE_DEFER;
	}

	ret = gpio_request_array(reserved_gpios, ARRAY_SIZE(reserved_gpios));
	if (ret){
		pr_info("%s: failed to request reserved gpios\n",
			__func__);
		goto end;
	}

	strlcpy(probed_i2c_cypress.type, "cy8c9540a", I2C_NAME_SIZE);
	probed_i2c_cypress.irq = gpio_to_irq(GPIO_CYPRESS_INT_S3);
	cypress = i2c_new_probed_device(i2c_adap, &probed_i2c_cypress,
					cypress_i2c_addr, cypress_i2c_probe);
	strlcpy(probed_i2c_eeprom.type, "at24", I2C_NAME_SIZE);
	eeprom = i2c_new_probed_device(i2c_adap, &probed_i2c_eeprom,
					eeprom_i2c_addr, eeprom_i2c_probe);

	if (NULL == cypress || NULL == eeprom) {
		pr_err("%s: can't probe Cypress Expander\n", __func__);
		ret = -ENODEV;
		goto end;
	}

	ret = intel_qrk_spi_add_onboard_devs();

end:
	i2c_put_adapter(i2c_adap);

	return ret;
}

static struct platform_driver gpio_restrict_pdriver = {
	.driver		= {
		.name	= GPIO_RESTRICT_NAME,
		.owner	= THIS_MODULE,
	},
	.probe		= intel_qrk_gpio_restrict_probe,
};

static int intel_quark_platform_galileo_probe(struct platform_device *pdev)
{
	int ret = 0;

	/* Assign GIP driver handle for board-specific settings */
	intel_qrk_gip_get_pdata = galileo_gip_get_pdata;

	/* gpio */
	ret = platform_driver_register(&gpio_restrict_pdriver);
	if (ret)
		goto end;

#if 0
	/* legacy SPI - TBD */
	ret = platform_driver_register(&intel_qrk_plat_galileo_lpcspi_pdriver);
	if (ret)
		goto end;
#endif
end:
	return ret;
}

static int intel_quark_platform_galileo_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver quark_galileo_platform_driver = {
	.driver		= {
		.name	= DRIVER_NAME,
		.owner	= THIS_MODULE,
	},
	.probe		= intel_quark_platform_galileo_probe,
	.remove		= intel_quark_platform_galileo_remove,
};

module_platform_driver(quark_galileo_platform_driver);

MODULE_AUTHOR("Bryan O'Donoghue <bryan.odonoghue@intel.com>");
MODULE_DESCRIPTION("Intel Quark SPI Data API");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_ALIAS("platform:"DRIVER_NAME);
