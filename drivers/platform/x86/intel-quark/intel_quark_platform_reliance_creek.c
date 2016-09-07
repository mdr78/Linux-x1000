/*
 * Intel Quark board support platform driver
 *
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

#include <linux/errno.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/spi/pxa2xx_spi.h>
#include <linux/spi/spi.h>
#include <linux/i2c/pcf857x.h>
#include <linux/platform_data/tpm_i2c_infenion.h>

#define DRIVER_NAME		"RelianceCreek"
#define GPIO_RESTRICT_NAME_NC	"gpio-restrict-nc"
#define GPIO_RESTRICT_NAME_SC	"gpio-restrict-sc"

/* GPIO signal names from RelianceCreek board, prefixed with GPIO_ */
/* GPIO<7> */
#define GPIO_IRQ_SPI2UART_B_BUF		15
/* GPIO<1> */
#define GPIO_GPIO1_RS485_IRQ		9

/* GPIO line used to reset SLB9645TT */
/* GPIO<8> */
#define GPIO_TPM_RESET_B		0

/* GPIO line for the SLB9645TT interrupt */
/* GPIO<5> */
#define GPIO_TPM_DAVINT_B		13

/* GPIO line for quark_ffrd_spi_0_cs_0 */
/* GPIO_SUS<2> */
#define GPIO_SPI0_SS_B			4

/* GPIO line for quark_ffrd_spi_1_cs_0 */
/* GPIO<2> */
#define GPIO_SPI1_SS_B			10


static int nc_gpio_reg;
static int sc_gpio_reg;
static int gpio_reserved;

#define PCF8574_GPIO_BASE_OFFSET 16

static struct pcf857x_platform_data pcf8574_platform_data_exp3 = {
	.gpio_base = PCF8574_GPIO_BASE_OFFSET,
};

static struct tpm_i2c_infenion_platform_data slb9645tt_platform_data = {
	.gpio_reset = GPIO_TPM_RESET_B,
	.gpio_irq = GPIO_TPM_DAVINT_B,
};

/******************************************************************************
 *                        Reliance Creek i2c clients
 ******************************************************************************/
#define TMP75C_ADDR				0x48
#define EEPROM_ADDR				0x50
#define PCF8574_EXP3_ADDR			0x22
#define SLB9645TT_ADDR				0x20

static struct i2c_board_info probed_i2c_tmp75c;
static struct i2c_board_info probed_i2c_eeprom;
static struct i2c_board_info probed_i2c_pcf8574_exp3 = {
	.platform_data = &pcf8574_platform_data_exp3,
};
static struct i2c_board_info probed_slb9645tt = {
	.platform_data = &slb9645tt_platform_data,
};

static const unsigned short tmp75c_i2c_addr[] = {
	TMP75C_ADDR, I2C_CLIENT_END
};
static const unsigned short eeprom_i2c_addr[] = {
	EEPROM_ADDR, I2C_CLIENT_END
};
static const unsigned short pcf8574_exp3_i2c_addr[] = {
	PCF8574_EXP3_ADDR, I2C_CLIENT_END
};
static const unsigned short slb9645tt_i2c_addr[] = {
	SLB9645TT_ADDR, I2C_CLIENT_END
};

static int i2c_probe(struct i2c_adapter *adap, unsigned short addr)
{
	/* Always return success: the I2C clients are already known.  */
	return 1;
}

/******************************************************************************
 *             NXP SC16IS7XX SPI Device Platform Data
 ******************************************************************************/

/* Peripheral's crystal frequency */
static const unsigned long sc16is752_platform_data = 18432000;
static const unsigned long sc16is741_platform_data = 18432000;

/******************************************************************************
 *                 Intel Quark SPI Controller Data
 ******************************************************************************/
static struct pxa2xx_spi_chip quark_ffrd_spi_0_cs_0 = {
	.gpio_cs = GPIO_SPI0_SS_B,
};

static struct pxa2xx_spi_chip quark_ffrd_spi_1_cs_0 = {
	.gpio_cs = GPIO_SPI1_SS_B,
};

static struct spi_board_info spi_sc16is752_info = {
	.modalias = "sc16is752",
	.max_speed_hz = 4000000,
	.mode = SPI_MODE_0,
	.bus_num = 0,
	.chip_select = 0,
	.controller_data = &quark_ffrd_spi_0_cs_0,
	.platform_data = &sc16is752_platform_data,
};

static struct spi_board_info spi_sc16is741_info = {
	.modalias = "sc16is74x",
	.max_speed_hz = 4000000,
	.mode = SPI_MODE_0,
	.bus_num = 1,
	.chip_select = 0,
	.controller_data = &quark_ffrd_spi_1_cs_0,
	.platform_data = &sc16is741_platform_data,
};

/******************************************************************************
 *                           Reliance Creek GPIOs
 ******************************************************************************/

/*
 * Reserve the GPIOs used as interrupts to ensure no one else tries to claim
 * them. Also reserve the TPM reset gpio.
 */
static struct gpio reserved_gpios[] = {
	{
		GPIO_IRQ_SPI2UART_B_BUF,
		GPIOF_IN,
		"sc16is752-int",
	},
	{
		GPIO_GPIO1_RS485_IRQ,
		GPIOF_IN,
		"sc16is741-int",
	},
	{
		GPIO_TPM_RESET_B,
		GPIOF_OUT_INIT_HIGH,
		"slb96455tt-reset",
	},
	{
		GPIO_TPM_DAVINT_B,
		GPIOF_IN,
		"slb96455tt-int",
	},
};

static int slb9645tt_i2c_probe(struct i2c_adapter *adap, unsigned short addr)
{
	return gpio_get_value(GPIO_TPM_RESET_B);
}

/**
 * intel_quark_i2c_add_onboard_devs
 *
 * @return 0 or -EPROBE_DEFER
 *
 * Registers onboard I2c device(s) present on the Reliance Creek platforms
 *
 * If any of the i2c_new_probed_device() call fail, an error message will
 * be displayed but the probe will be allowed to continue in order to enable
 * as much board functionality as possible.
 */
static int intel_quark_i2c_add_onboard_devs(void)
{
	struct i2c_adapter *i2c_adap = NULL;
	struct i2c_client *client = NULL;

	i2c_adap = i2c_get_adapter(0);
	if (NULL == i2c_adap) {
		pr_info("%s: i2c adapter not ready yet. Deferring..\n",
			__func__);
		return -EPROBE_DEFER;
	}

	/*
	 * Register on-board I2C devices
	 */
	strlcpy(probed_i2c_tmp75c.type, "tmp75c", I2C_NAME_SIZE);
	client = i2c_new_probed_device(i2c_adap, &probed_i2c_tmp75c,
				       tmp75c_i2c_addr, i2c_probe);
	if (client == NULL)
		pr_err("%s: Failed to probe tmp75c I2C device\n", __func__);

	strlcpy(probed_i2c_eeprom.type, "24c64", I2C_NAME_SIZE);
	client = i2c_new_probed_device(i2c_adap, &probed_i2c_eeprom,
				       eeprom_i2c_addr, i2c_probe);
	if (client == NULL)
		pr_err("%s: Failed to probe 24c64 I2C device\n", __func__);

	strlcpy(probed_i2c_pcf8574_exp3.type, "pcf8574", I2C_NAME_SIZE);
	client = i2c_new_probed_device(i2c_adap, &probed_i2c_pcf8574_exp3,
				       pcf8574_exp3_i2c_addr, i2c_probe);
	if (client == NULL)
		pr_err("%s: Failed to probe pcf8574 I2C device\n", __func__);

	probed_slb9645tt.irq = gpio_to_irq(GPIO_TPM_DAVINT_B);
	strlcpy(probed_slb9645tt.type, "slb9645tt", I2C_NAME_SIZE);
	client = i2c_new_probed_device(i2c_adap, &probed_slb9645tt,
				       slb9645tt_i2c_addr,
				       slb9645tt_i2c_probe);
	if (client == NULL)
		pr_err("%s: Failed to probe slb9645tt I2C device\n", __func__);

	i2c_put_adapter(i2c_adap);

	return 0;
}

/**
 * intel_quark_spi_add_onboard_devs
 *
 * @return 0
 *
 * Registers onboard SPI device(s) present on the Reliance Creek platforms
 *
 * If any of the spi_register_board_info() call fail, an error message will
 * be displayed but the probe will be allowed to continue in order to enable
 * as much board functionality as possible.
 */
static int intel_quark_spi_add_onboard_devs(void)
{
	int ret = 0;
	/*
	 * Register on-board SPI devices
	 */
	spi_sc16is752_info.irq = gpio_to_irq(GPIO_IRQ_SPI2UART_B_BUF);
	ret = spi_register_board_info(&spi_sc16is752_info, 1);
	if (ret) {
		pr_err("%s: Failed to register sc16is752 SPI device\n",
		       __func__);
	}

	spi_sc16is741_info.irq = gpio_to_irq(GPIO_GPIO1_RS485_IRQ);
	ret = spi_register_board_info(&spi_sc16is741_info, 1);
	if (ret) {
		pr_err("%s: Failed to register sc16is741 SPI device\n",
		       __func__);
	}

	return 0;
}

/**
 * intel_quark_restrict_probe
 *
 * Register platform devices.
 *
 * Make GPIOs pertaining to Firmware inaccessible by requesting them.  The
 * GPIOs are never released nor accessed by this driver.
 *
 */
static int intel_quark_restrict_probe(void)
{
	int ret;

	/* The intel_quark_i2c_add_onboard_devs() function may return a defered
	 * probe error message.  In this, case gpio_reserved will be used to
	 * prevent the driver requesting the GPIO array a second time.
	 */
	if (!gpio_reserved) {
		/*
		 * Reserve GPIOs for I2C/SPI device interrupts (never released)
		 */
		ret = gpio_request_array(reserved_gpios,
					 ARRAY_SIZE(reserved_gpios));
		if (ret) {
			pr_err("%s: failed to reserved gpios\n",
			       __func__);
			return ret;
		}
		gpio_reserved = 1;
	}

	/* Register I2C devices */
	ret = intel_quark_i2c_add_onboard_devs();
	if (ret)
		return ret;

	/* Register SPI devices */
	ret = intel_quark_spi_add_onboard_devs();
	if (ret)
		return ret;

	return 0;
}

/**
 * intel_quark_gpio_restrict_probe_nc
 *
 * Ensure that modules gpio-sch and intel-qrk-gip are loaded before continuing
 */
static int intel_quark_gpio_restrict_probe_nc(struct platform_device *pdev)
{
	int ret;
	nc_gpio_reg = 1;

	if (nc_gpio_reg == 1 && sc_gpio_reg == 1) {
		ret = intel_quark_restrict_probe();
		if (ret)
			return ret;
	}
	return 0;
}

static int intel_quark_gpio_restrict_remove_nc(struct platform_device *pdev)
{
	nc_gpio_reg = 0;
	return 0;
}

/**
 * intel_quark_gpio_restrict_probe_sc
 *
 * Ensure that modules gpio-sch and intel-qrk-gip are loaded before continuing
 */
static int intel_quark_gpio_restrict_probe_sc(struct platform_device *pdev)
{
	int ret;
	sc_gpio_reg = 1;

	if (nc_gpio_reg == 1 && sc_gpio_reg == 1) {
		ret = intel_quark_restrict_probe();
		if (ret)
			return ret;
	}
	return 0;
}

static int intel_quark_gpio_restrict_remove_sc(struct platform_device *pdev)
{
	sc_gpio_reg = 0;
	return 0;
}

static struct platform_driver gpio_restrict_pdriver_nc = {
	.driver		= {
		.name	= GPIO_RESTRICT_NAME_NC,
		.owner	= THIS_MODULE,
	},
	.probe		= intel_quark_gpio_restrict_probe_nc,
	.remove		= intel_quark_gpio_restrict_remove_nc,
};

static struct platform_driver gpio_restrict_pdriver_sc = {
	.driver		= {
		.name	= GPIO_RESTRICT_NAME_SC,
		.owner	= THIS_MODULE,
	},
	.probe		= intel_quark_gpio_restrict_probe_sc,
	.remove		= intel_quark_gpio_restrict_remove_sc,
};

static int
intel_quark_platform_reliance_creek_probe(struct platform_device *pdev)
{
	int ret = 0;

	ret = platform_driver_register(&gpio_restrict_pdriver_nc);
	if (ret)
		return ret;
	ret = platform_driver_register(&gpio_restrict_pdriver_sc);
	if (ret)
		return ret;

	return 0;
}

static int
intel_quark_platform_reliance_creek_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_device_id intel_quark_platform_reliance_creek_ids[] = {
	{
		.name = "RelianceCreek",
		.driver_data = 0,
	},
	{
		.name = "RelianceCreekSPU",
		.driver_data = 1,
	},
};
MODULE_DEVICE_TABLE(platform, intel_quark_platform_reliance_creek_ids);

static struct platform_driver intel_quark_platform_reliance_creek_driver = {
	.driver		= {
		.name	= DRIVER_NAME,
		.owner	= THIS_MODULE,
	},
	.probe		= intel_quark_platform_reliance_creek_probe,
	.remove		= intel_quark_platform_reliance_creek_remove,
	.id_table	= intel_quark_platform_reliance_creek_ids,
};

module_platform_driver(intel_quark_platform_reliance_creek_driver);

MODULE_AUTHOR("Intel Corporation");
MODULE_DESCRIPTION("Reliance Creek BSP Data");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_ALIAS("platform:"DRIVER_NAME);
