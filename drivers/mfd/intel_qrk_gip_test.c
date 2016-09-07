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
 * Intel Quark GIP (GPIO/I2C) Test module
 *
 * Quark GIP + North-Cluster GPIO test module.
 */

#include <asm/tsc.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <linux/printk.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

#define DRIVER_NAME			"intel_qrk_gip_test"

/**************************** Exported to LISA *******************************/

/*
 * Internally-used ioctl code. At the moment it is not reserved by any mainline
 * driver.
 */
#define GIP_TEST_IOCTL_CODE			0xE0

/*
 * Integers for ioctl operation.
 */
#define IOCTL_QRK_GPIO_11			_IO(GIP_TEST_IOCTL_CODE, 0x00)
#define IOCTL_QRK_GPIO_11_CLEANUP		_IO(GIP_TEST_IOCTL_CODE, 0x01)
#define IOCTL_QRK_GPIO_12			_IO(GIP_TEST_IOCTL_CODE, 0x02)
#define IOCTL_QRK_GPIO_12_CLEANUP		_IO(GIP_TEST_IOCTL_CODE, 0x03)
#define IOCTL_QRK_GPIO_13			_IO(GIP_TEST_IOCTL_CODE, 0x04)
#define IOCTL_QRK_GPIO_13_CLEANUP		_IO(GIP_TEST_IOCTL_CODE, 0x05)
#define IOCTL_QRK_GPIO_14			_IO(GIP_TEST_IOCTL_CODE, 0x06)
#define IOCTL_QRK_GPIO_14_CLEANUP		_IO(GIP_TEST_IOCTL_CODE, 0x07)
#define IOCTL_QRK_GPIO_15			_IO(GIP_TEST_IOCTL_CODE, 0x08)
#define IOCTL_QRK_GPIO_15_CLEANUP		_IO(GIP_TEST_IOCTL_CODE, 0x09)
#define IOCTL_QRK_GPIO_16			_IO(GIP_TEST_IOCTL_CODE, 0x0A)
#define IOCTL_QRK_GPIO_16_CLEANUP		_IO(GIP_TEST_IOCTL_CODE, 0x0B)
#define IOCTL_QRK_GPIO_17			_IO(GIP_TEST_IOCTL_CODE, 0x0C)
#define IOCTL_QRK_GPIO_17_CLEANUP		_IO(GIP_TEST_IOCTL_CODE, 0x0D)
#define IOCTL_QRK_GPIO_19			_IO(GIP_TEST_IOCTL_CODE, 0x0E)
#define IOCTL_QRK_GPIO_19_CLEANUP		_IO(GIP_TEST_IOCTL_CODE, 0x0F)
#define IOCTL_QRK_GPIO_20			_IO(GIP_TEST_IOCTL_CODE, 0x10)
#define IOCTL_QRK_GPIO_20_CLEANUP		_IO(GIP_TEST_IOCTL_CODE, 0x11)
#define IOCTL_QRK_GPIO_21			_IO(GIP_TEST_IOCTL_CODE, 0x12)
#define IOCTL_QRK_GPIO_21_CLEANUP		_IO(GIP_TEST_IOCTL_CODE, 0x13)
#define IOCTL_QRK_GPIO_24			_IO(GIP_TEST_IOCTL_CODE, 0x14)
#define IOCTL_QRK_GPIO_26			_IO(GIP_TEST_IOCTL_CODE, 0x15)
#define IOCTL_QRK_GPIO_26_CLEANUP		_IO(GIP_TEST_IOCTL_CODE, 0x16)
#define IOCTL_QRK_GPIO_28			_IO(GIP_TEST_IOCTL_CODE, 0x1B)
#define IOCTL_QRK_GPIO_28_CLEANUP		_IO(GIP_TEST_IOCTL_CODE, 0x1C)
/* Exercise callbacks for S0/S3 power-state transitions and vice-versa */
#define IOCTL_QRK_GIP_SYSTEM_SUSPEND		_IO(GIP_TEST_IOCTL_CODE, 0x17)
#define IOCTL_QRK_GIP_SYSTEM_RESUME		_IO(GIP_TEST_IOCTL_CODE, 0x18)
#define IOCTL_QRK_GPIO_NMI_ENABLE		_IO(GIP_TEST_IOCTL_CODE, 0x19)
#define IOCTL_QRK_GPIO_NMI_DISABLE		_IO(GIP_TEST_IOCTL_CODE, 0x1A)

#define GPIO_INT_EDGE_POS_LABEL			"gpio-edge-pos"
#define GPIO_INT_EDGE_NEG_LABEL			"gpio-edge-neg"
#define GPIO_INT_LEVEL_HIGH_LABEL		"gpio-level-hi"
#define GPIO_INT_LEVEL_LOW_LABEL		"gpio-level-lo"
#define GPIO_INT_BASIC_LABEL			"gpio-edge-pos-basic"
#define GPIO_PM_TEST_IRQ_LABEL			"gpio_pm_test_irq"

/*
 * Board GPIO numbers.
 * Mapping between the North/South cluster GPIO and GPIOLIB IDs.
 */
#define SUT_GPIO_NC_0				0x00
#define SUT_GPIO_NC_1				0x01
#define SUT_GPIO_NC_2				0x02
#define SUT_GPIO_NC_7				0x07
#define SUT_GPIO_SC_0				0x08
#define SUT_GPIO_SC_1				0x09
#define SUT_GPIO_SC_6				0x0E
#define SUT_GPIO_SC_7				0x0F

/*
 * Bitbanged SPI bus numbers.
 */
#define GPIO_NC_BITBANG_SPI_BUS			0x0
#define GPIO_SC_BITBANG_SPI_BUS			0x1

/*****************************************************************************/

/**
 * struct intel_qrk_gip_dev
 *
 * Structure to represent module state/data/etc
 */
struct intel_qrk_gip_test_dev {
	unsigned int opened;
	struct platform_device *pldev;	/* Platform device */
	struct cdev cdev;
	struct mutex open_lock;
};

static struct intel_qrk_gip_test_dev gip_test_dev;
static struct class *gip_test_class;
static DEFINE_MUTEX(gip_test_mutex);
static int gip_test_major;

/*
 * Level-triggered interrupt variables
 */
/* Level-triggered GPIO workqueue */
static struct delayed_work work;
/* Level-triggered interrupt counter */
static unsigned int level_int_count;
/* By default, a level-triggered interrupt is a low-level triggered */
static int level_high_triggered = 0;

/*
 * Interrupt performance metrics variables and parameters
 */
/* How many captures */
#define INT_PERF_TEST_CAPTURES			10000
/* Timestamp for latency test interrupt handler */
static cycles_t perf_t1;
/* Captures to be returned to user space */
static cycles_t deltas[INT_PERF_TEST_CAPTURES];
/* Couldn't find the actual define for this */
#define UINT64_MAX		0xFFFFFFFFFFFFFFFFULL

/*
 * Threaded interrupt test variables
 */
static int gpio_irq = 0;
static struct timer_list timer;
static unsigned delay = (HZ/50);
static int timer_count = 0;


static irqreturn_t gpio_pm_test_handler(int irq, void *dev_id)
{
	/* Do nothing, just acknowledge the IRQ subsystem */
	return IRQ_HANDLED;
}

static irqreturn_t gpio_latency_handler(int irq, void *dev_id)
{
	/* t0 */
	perf_t1 = get_cycles();

	gpio_set_value(SUT_GPIO_SC_0, 0);

	return IRQ_HANDLED;
}

static irqreturn_t gpio_basic_handler(int irq, void *dev_id)
{
	/* Do nothing, just acknowledge the IRQ subsystem */
	return IRQ_HANDLED;
}

static irqreturn_t gpio_pos_edge_handler(int irq, void *dev_id)
{
	/* Do nothing, just acknowledge the IRQ subsystem */
	return IRQ_HANDLED;
}

static irqreturn_t gpio_neg_edge_handler(int irq, void *dev_id)
{
	/* Do nothing, just acknowledge the IRQ subsystem */
	return IRQ_HANDLED;
}

static irqreturn_t gpio_level_handler(int irq, void *dev_id)
{
	/* Untrigger the interrupt */
	gpio_set_value(SUT_GPIO_SC_7, level_high_triggered ? 0 : 1);

	level_int_count ++;
	if (level_int_count < 1000) {
		/* Next task due in a jiffy */
		schedule_delayed_work(&work, 1);
	} else if (level_int_count == 1000){
		/* OK */
	} else {
		/*
		 * We may get spurious interrupts. This because the TE requires
		 * some time to drive the actual value to the GPIO.
		 */
		pr_info("Spurious interrupt\n");
	}

	return IRQ_HANDLED;
}

static void gpio_level_drive(struct work_struct *work)
{
	/* TE to trigger the interrupt */
	gpio_set_value(SUT_GPIO_SC_7, level_high_triggered ? 1 : 0);
}

/**
 * gpio_sc_level_int
 *
 * Request level triggered IRQ for SUT_GPIO_SC_6 and register
 * SUT_GPIO_SC_7 as output GPIO.
 * If positive equals to 0, the IRQ is high-level triggered.
 * Otherwise, low-level triggered.
 * Mask the IRQ if requested.
 */
static int gpio_sc_level_int(int positive, int masking)
{
	int ret = 0;
	int irq = -1;

	unsigned long out_init_val =
		(positive ? GPIOF_OUT_INIT_LOW : GPIOF_OUT_INIT_HIGH);

	level_high_triggered = positive;

	/* Initialise workqueue task */
	INIT_DELAYED_WORK(&work, gpio_level_drive);

	if (!gpio_is_valid(SUT_GPIO_SC_6)) {
		pr_err("gpio%d is invalid\n", SUT_GPIO_SC_6);
		ret = -1;
		goto fail;
	}
	if (!gpio_is_valid(SUT_GPIO_SC_7)) {
		pr_err("gpio%d is invalid\n", SUT_GPIO_SC_7);
		ret = -1;
		goto fail;
	}

	ret = gpio_request_one(SUT_GPIO_SC_6, GPIOF_IN, "gpio_hi_level");
	if (ret) {
		pr_err("can't request gpio%d (error %d)\n", SUT_GPIO_SC_6, ret);
		goto fail;
	}
	ret = gpio_request_one(SUT_GPIO_SC_7, out_init_val, "gpio_output");
	if (ret) {
		pr_err("can't request gpio%d (error %d)\n", SUT_GPIO_SC_7, ret);
		goto fail_release_first_gpio;
	}

	irq = gpio_to_irq(SUT_GPIO_SC_6);
	if (irq < 0) {
		pr_err("can't map gpio%d to IRQ\n", SUT_GPIO_SC_6);
		goto fail_release_second_gpio;
	}

	if (0 != (ret = request_irq(irq, gpio_level_handler,
			positive ? IRQF_TRIGGER_HIGH : IRQF_TRIGGER_LOW,
			positive ? GPIO_INT_LEVEL_HIGH_LABEL : GPIO_INT_LEVEL_LOW_LABEL,
			NULL))) {
		pr_err("can't request IRQ for gpio%d\n", SUT_GPIO_SC_6);
		goto fail_release_second_gpio;
	}

	level_int_count = 0;

	pr_info("Registered output gpio%d and IRQ for gpio%d\n", SUT_GPIO_SC_7,
		SUT_GPIO_SC_6);

	if (masking) {
		disable_irq(gpio_to_irq(SUT_GPIO_SC_6));
		pr_info("Masked gpio%d IRQ\n", SUT_GPIO_SC_6);
	}

	/*
	 * Submit task to workqueue to drive the external Test Equipment.
	 * Note the task is delayed long enough to have Aarvark already set up.
	 * This because Aardvark has to ignore the initial glitches during the
	 * previous GPIO setup phase.
	 */
	schedule_delayed_work(&work, 20 * HZ);

	return 0;

fail_release_second_gpio:
	gpio_free(SUT_GPIO_SC_7);
fail_release_first_gpio:
	gpio_free(SUT_GPIO_SC_6);
fail:
	pr_err("%s() failed\n", __func__);

	return ret;
}

/**
 * gpio_sc_level_int_teardown
 *
 * Release resources reserved by gpio_sc_level_int().
 */
static int gpio_sc_level_int_teardown(void)
{
	int irq = -1;

	if (0 != cancel_delayed_work_sync(&work))
		pr_warn("delayed work was still pending\n");

	irq = gpio_to_irq(SUT_GPIO_SC_6);
	if (irq < 0) {
		pr_err("can't map gpio%d to IRQ\n", SUT_GPIO_SC_6);
	} else {
		free_irq(irq, NULL);
	}

	/* Make sure no handler is still running by this time */
	mdelay(20);

	gpio_free(SUT_GPIO_SC_7);
	gpio_free(SUT_GPIO_SC_6);

	return 0;
}

/*
 * gpio_sc_interrupt_perf
 *
 * Performs a basic GPIO interrupt latency test by timestamping delta between
 * interrupt driven and handled over a GPIO loopback.
 *
 * Returns to userspace the array of deltas obtained during each capture.
 * A total amount of INT_PERF_TEST_CAPTURES captures is performed.
 *
 */
static int gpio_sc_interrupt_perf(unsigned long user_memloc)
{
	int ret = 0;
	int irq = -1;
	int gpio_input = SUT_GPIO_SC_1;
	int gpio_output = SUT_GPIO_SC_0;
	unsigned int i = 0;
	cycles_t perf_t0 = 0;
	cycles_t delta = 0;

	/* Casting pointer to user-space location to write */
	cycles_t __user *user_ptr = (cycles_t __user *)user_memloc;

	/* Can we copy the captures array into user-space location? */
	if (!access_ok(VERIFY_WRITE, user_ptr, sizeof(deltas))) {
		pr_err("can't copy 0x%x bytes to user-space address 0x%p\n",
			sizeof(deltas),user_ptr);
		return -EFAULT;
	}

	/* Setup the GPIO */
	if (!gpio_is_valid(gpio_input)) {
		pr_err("gpio%d is invalid\n", gpio_input);
		ret = -1;
		goto fail;
	}
	if (!gpio_is_valid(gpio_output)) {
		pr_err("gpio%d is invalid\n", gpio_output);
		ret = -1;
		goto fail;
	}
	ret = gpio_request_one(gpio_input, GPIOF_IN, "gpio_intperf_in");
	if (ret) {
		pr_err("can't request gpio%d (error %d)\n", gpio_input, ret);
		goto fail;
	}
	ret = gpio_request_one(gpio_output, GPIOF_OUT_INIT_LOW, "gpio_intperf_out");
	if (ret) {
		pr_err("can't request gpio%d (error %d)\n", gpio_output, ret);
		goto fail_release_input_gpio;
	}

	/* Setup IRQ handler for input GPIO */
	irq = gpio_to_irq(gpio_input);
	if (irq < 0) {
		pr_err("can't map gpio%d to IRQ\n", gpio_input);
		goto fail_release_output_gpio;
	}
	if (0 != (ret = request_irq(irq, gpio_latency_handler,
			IRQF_TRIGGER_RISING, "gpio_latency_handler", NULL))) {
		pr_err("can't request IRQ for gpio%d\n", gpio_input);
		goto fail_release_output_gpio;
	}

	/* Perform test */
	for (i = 0; i < INT_PERF_TEST_CAPTURES; i ++) {
		/* t0 */
		perf_t0 = get_cycles();

		/* Trigger interrupt */
		gpio_set_value(gpio_output, 1);
		mdelay(2);

		/* Check for wrap-around and store delta */
		if(perf_t0 < perf_t1) {
			delta = perf_t1 - perf_t0;
		} else {
			delta = perf_t1 + (UINT64_MAX - perf_t0);
		}
		deltas[i] = delta;
	}

	/* Expose results to userspace */
	ret = copy_to_user(user_ptr, &deltas, sizeof(deltas));

	/* Release resources */

	free_irq(irq, NULL);

fail_release_output_gpio:
	gpio_free(gpio_output);
fail_release_input_gpio:
	gpio_free(gpio_input);
fail:
	if (0 != ret) {
		pr_err("%s() failed\n", __func__);
	}

	return ret;
}

/**
 * gpio_sc_pm_test_int
 *
 * Request rising edge-triggered IRQ for SUT_GPIO_SC_0
 */
static int gpio_sc_pm_test_int(void)
{
	int ret = 0;
	int irq = -1;
	int gpio_input = SUT_GPIO_SC_0;

	/* Setup the GPIO */
	if (!gpio_is_valid(gpio_input)) {
		pr_err("gpio%d is invalid\n", gpio_input);
		ret = -1;
		goto fail;
	}
	ret = gpio_request_one(gpio_input, GPIOF_IN, "gpio_pm_test_in");
	if (ret) {
		pr_err("can't request gpio%d (error %d)\n", gpio_input, ret);
		goto fail;
	}

	/* Setup IRQ handler for input GPIO */
	irq = gpio_to_irq(gpio_input);
	if (irq < 0) {
		pr_err("can't map gpio%d to IRQ\n", gpio_input);
		goto fail_release_input_gpio;
	}
	if (0 != (ret = request_irq(irq, gpio_pm_test_handler,
			IRQF_TRIGGER_RISING, GPIO_PM_TEST_IRQ_LABEL, NULL))) {
		pr_err("can't request IRQ for gpio%d\n", gpio_input);
		goto fail_release_input_gpio;
	}

	return 0;

fail_release_input_gpio:
	gpio_free(gpio_input);
fail:
	return ret;
}

/**
 * gpio_sc_pm_test_int
 *
 * Release resources reserved by gpio_sc_edge_int()
 */
static int gpio_sc_pm_test_int_teardown(void)
{
	int irq = -1;

	irq = gpio_to_irq(SUT_GPIO_SC_0);
	if (irq < 0) {
		pr_err("can't map gpio%d to IRQ\n", SUT_GPIO_SC_0);
	} else {
		free_irq(irq, NULL);
	}

	gpio_free(SUT_GPIO_SC_0);

	return 0;
}

/**
 * gpio_sc_edge_int
 *
 * Request IRQ for SUT_GPIO_SC_6 and SUT_GPIO_SC_7, respectively positive-edge
 * and negative edge-triggered.
 * Mask the IRQs if requested.
 */
static int gpio_sc_edge_int(int masking)
{
	int ret = 0;
	int irq_pos = -1, irq_neg = -1;

	if (!gpio_is_valid(SUT_GPIO_SC_6)) {
		pr_err("gpio%d is invalid\n", SUT_GPIO_SC_6);
		ret = -1;
		goto fail;
	}
	if (!gpio_is_valid(SUT_GPIO_SC_7)) {
		pr_err("gpio%d is invalid\n", SUT_GPIO_SC_7);
		ret = -1;
		goto fail;
	}

	ret = gpio_request_one(SUT_GPIO_SC_6, GPIOF_IN, "gpio_pos_edge");
	if (ret) {
		pr_err("can't request gpio%d (error %d)\n", SUT_GPIO_SC_6, ret);
		goto fail;
	}
	ret = gpio_request_one(SUT_GPIO_SC_7, GPIOF_IN, "gpio_neg_edge");
	if (ret) {
		pr_err("can't request gpio%d (error %d)\n", SUT_GPIO_SC_7, ret);
		goto fail_release_first_gpio;
	}

	irq_pos = gpio_to_irq(SUT_GPIO_SC_6);
	if (irq_pos < 0) {
		pr_err("can't map gpio%d to IRQ\n", SUT_GPIO_SC_6);
		goto fail_release_second_gpio;
	}
	irq_neg = gpio_to_irq(SUT_GPIO_SC_7);
	if (irq_neg < 0) {
		pr_err("can't map gpio%d to IRQ\n", SUT_GPIO_SC_7);
		goto fail_release_second_gpio;
	}

	if (0 != (ret = request_irq(irq_pos, gpio_pos_edge_handler,
			IRQF_TRIGGER_RISING, GPIO_INT_EDGE_POS_LABEL, NULL))) {
		pr_err("can't request IRQ for gpio%d\n", SUT_GPIO_SC_6);
		goto fail_release_second_gpio;
	}
	if (0 != (ret = request_irq(irq_neg, gpio_neg_edge_handler,
			IRQF_TRIGGER_FALLING, GPIO_INT_EDGE_NEG_LABEL, NULL))) {
		pr_err("can't request IRQ for gpio%d\n", SUT_GPIO_SC_7);
		goto fail_release_first_gpio_irq;
	}

	pr_info("Registered gpio%d and gpio%d IRQs\n", SUT_GPIO_SC_6,
		SUT_GPIO_SC_7);

	if (masking) {
		disable_irq(gpio_to_irq(SUT_GPIO_SC_6));
		disable_irq(gpio_to_irq(SUT_GPIO_SC_7));
		pr_info("Masked gpio%d and gpio%d IRQs\n", SUT_GPIO_SC_6,
			SUT_GPIO_SC_7);
	}

	return 0;

fail_release_first_gpio_irq:
	free_irq(irq_pos, NULL);
fail_release_second_gpio:
	gpio_free(SUT_GPIO_SC_7);
fail_release_first_gpio:
	gpio_free(SUT_GPIO_SC_6);
fail:
	pr_err("%s() failed\n", __func__);

	return ret;
}

/**
 * gpio_sc_edge_int_teardown
 *
 * Release resources reserved by gpio_sc_edge_int()
 */
static int gpio_sc_edge_int_teardown(void)
{
	int irq_pos = -1, irq_neg = -1;

	irq_neg = gpio_to_irq(SUT_GPIO_SC_7);
	if (irq_neg < 0) {
		pr_err("can't map gpio%d to IRQ\n", SUT_GPIO_SC_7);
	} else {
		free_irq(irq_neg, NULL);
	}
	irq_pos = gpio_to_irq(SUT_GPIO_SC_6);
	if (irq_pos < 0) {
		pr_err("can't map gpio%d to IRQ\n", SUT_GPIO_SC_6);
	} else {
		free_irq(irq_pos, NULL);
	}

	gpio_free(SUT_GPIO_SC_7);
	gpio_free(SUT_GPIO_SC_6);

	return 0;
}

/**
 * gpio_sc_basic_int
 *
 * Register rising-edge interrupt handler on SUT_GPIO_SC_1
 */
static int gpio_sc_basic_int(void)
{
	int ret = 0;
	int irq = -1;
	unsigned int gpio = SUT_GPIO_SC_1;

	if (!gpio_is_valid(gpio)) {
		pr_err("gpio%d is invalid\n", gpio);
		ret = -1;
		goto fail;
	}

	ret = gpio_request_one(gpio, GPIOF_IN, "gpio_pos_edge_basic");
	if (ret) {
		pr_err("can't request gpio%d (error %d)\n", gpio, ret);
		goto fail;
	}

	irq = gpio_to_irq(gpio);
	if (irq < 0) {
		pr_err("can't map gpio%d to IRQ\n", gpio);
		goto fail_release_gpio;
	}

	if (0 != (ret = request_irq(irq, gpio_basic_handler,
			IRQF_TRIGGER_RISING, GPIO_INT_BASIC_LABEL, NULL))) {
		pr_err("can't request IRQ for gpio%d\n", gpio);
		goto fail_release_gpio;
	}

	pr_info("Registered gpio%d IRQ\n", gpio);

	return 0;

fail_release_gpio:
	gpio_free(gpio);
fail:
	pr_err("%s() failed\n", __func__);

	return ret;
}

/**
 * gpio_sc_basic_int_teardown
 *
 * Release resources reserved by gpio_sc_basic_int()
 */
static int gpio_sc_basic_int_teardown(void)
{
	int irq = -1;
	unsigned int gpio = SUT_GPIO_SC_1;

	irq = gpio_to_irq(gpio);
	if (irq < 0) {
		pr_err("can't map gpio%d to IRQ\n", gpio);
	} else {
		free_irq(irq, NULL);
	}

	gpio_free(gpio);

	return 0;
}

/**
 * gpio_spidev_register
 *
 * Register a bitbanged SPI platform device and export a `spidev' to userspace.
 * For North Cluster and South Cluster.
 */
static int gpio_spidev_register(int north_cluster)
{
	/* Not needed anymore */
	return 0;
}

/**
 * gpio_spidev_unregister
 *
 * Release a bitbanged SPI platform device and its `spidev' interface.
 * For North Cluster and South Cluster.
 */
static int gpio_spidev_unregister(int north_cluster)
{
	/* Not needed anymore */
	return 0;
}

/**
 * gip_system_power_transition
 *
 * @param state: 0 if transition to S3, !0 if transition to S0
 * @return 0 success < 0 failure
 *
 * Exercise system-wide suspend/resume power management transitions.
 *
 */
static int gip_system_power_transition(int state)
{
	struct pci_dev *gip = pci_get_device(PCI_VENDOR_ID_INTEL, 0x0934, NULL);
	if (NULL == gip) {
		pr_err("can't find GIP PCI device\n");
		return -ENOENT;
	}

	if (0 == state) {
		gip->driver->driver.pm->suspend(&gip->dev);
	} else {
		gip->driver->driver.pm->resume(&gip->dev);
	}

	/* Decrement reference count of PCI device */
	if (NULL != pci_get_device(PCI_VENDOR_ID_INTEL, 0x0934, gip)) {
		pr_warn("found duplicate of GIP PCI device?!\n");
	}

	return 0;
}

/**
 * gpio_nmi_enable
 *
 * @param enable: 0 to disable, !0 to enable
 * @return 0 success < 0 failure
 *
 * Hack the legacy GPIO hardware to enable rising-edge triggered NMI on Core
 * Well gpio0.
 *
 */
static int gpio_nmi_enable(int enable)
{
	unsigned int base_u32 = 0x0;
	unsigned short base = 0x0;
	struct pci_dev *ilb = pci_get_device(PCI_VENDOR_ID_INTEL,
					     PCI_DEVICE_ID_INTEL_QUARK_X1000_ILB,
					     NULL);
	/* Assume interrupts are disabled by default by BIOS */
	unsigned char gpio = enable ? 0x01 : 0x00;

	if (NULL == ilb) {
		pr_err("can't find iLB device\n");
		return -ENOENT;
	}

	/* The GPIO base address is @ offset 0x44.  Sussed out from driver */
	pci_read_config_dword(ilb, 0x44, &base_u32);
	if (0x0 == base_u32) {
		pr_err("can't read iLB GPIO baseaddr\n");
		return -ENOENT;
	}
	base = (unsigned short)base_u32;

	/*
	 * Prepare for rising edge NMI triggering.  This assumes the pin
	 * is already set as input.
	 */
#define CGTPE		0x0C	/* Core Well trigger positive edge */
#define CGTS		0x1C	/* Core Well trigges status - W1C */
#define CGNMIEN		0x40	/* Core Well NMI enable */
	outb(0x01, base + CGTS);
	outb(gpio, base + CGTPE);
	outb(gpio, base + CGNMIEN);
#undef CGTPE
#undef CGTS
#undef CGNMIEN

	return 0;
}

/**
 * gpio_sc_debounce
 *
 * Enable GPIO debounce functionality for SC_GPIO_1 (edge and level triggered)
 *
 */
static int gpio_sc_debounce(int level)
{
	int ret = 0;
	int irq = -1;
	int gpio = SUT_GPIO_SC_0;

	if (!gpio_is_valid(gpio)) {
		pr_err("gpio%d is invalid\n", gpio);
		ret = -1;
		goto fail;
	}

	ret = gpio_request_one(gpio, GPIOF_IN,
		level ? "gpio_level_mask" : "gpio_edge_mask");
	if (ret) {
		pr_err("can't request gpio%d (error %d)\n", gpio, ret);
		goto fail;
	}

	irq = gpio_to_irq(gpio);
	if (irq < 0) {
		pr_err("can't map gpio%d to IRQ\n", gpio);
		goto fail_release_gpio;
	}

	/*
	 * Register IRQ. gpio_pos_edge_handler will do for both level and edge
	 * interrupts, as it's nooping.
	 */
	if (0 != (ret = request_irq(irq, gpio_pos_edge_handler,
			level ? IRQF_TRIGGER_HIGH : IRQF_TRIGGER_RISING,
			level ? GPIO_INT_LEVEL_HIGH_LABEL : GPIO_INT_EDGE_POS_LABEL,
			NULL))) {
		pr_err("can't request IRQ for gpio%d\n", gpio);
		goto fail_release_gpio;
	}

	/* Set debounce */
	if (0 != (ret = gpio_set_debounce(gpio, 1))) {
		pr_err("can't set debounce for gpio%d\n", gpio);
		goto fail_free_irq;
	}

	return 0;

fail_free_irq:
	free_irq(irq, NULL);
fail_release_gpio:
	gpio_free(gpio);
fail:
	pr_err("%s() failed\n", __func__);

	return ret;
}

/**
 * gpio_sc_debounce_teardown
 *
 * Undo gpio_sc_debounce
 *
 */
static int gpio_sc_debounce_teardown(int level)
{
	int irq = -1;
	unsigned int gpio = SUT_GPIO_SC_0;

	irq = gpio_to_irq(gpio);
	if (irq < 0) {
		pr_err("can't map gpio%d to IRQ\n", gpio);
	} else {
		free_irq(irq, NULL);
	}

	gpio_free(gpio);

	return 0;
}

/**
 * gpio_sc_threaded_interrupt_handler
 *
 * Threaded interrupt handler
 */
static irqreturn_t gpio_sc_threaded_interrupt_handler(int irq, void* dev_id)
{
    gpio_set_value(SUT_GPIO_SC_0, 1);

    return IRQ_HANDLED;
}

/**
 * gpio_sc_threaded_timer_handler
 *
 * Threaded timer handler
 */
static void gpio_sc_threaded_timer_handler(unsigned long ptr)
{
	if (timer_count < 1000) {
		mod_timer(&timer, jiffies + delay);
		gpio_set_value(SUT_GPIO_SC_0, 0);
		timer_count++;
	}
}

/**
 * gpio_sc_threaded
 *
 * Threaded interrupt test
 */
static int gpio_sc_threaded(void)
{
    int err = 0;

    err = gpio_request_one(SUT_GPIO_SC_0, GPIOF_OUT_INIT_HIGH, "gpio_out");
    if (err < 0) {
        pr_err("can't request gpio%d (error %d)\n", SUT_GPIO_SC_0, err);
        goto err_return;
    }

    err = gpio_request_one(SUT_GPIO_SC_1, GPIOF_IN, "gpio_high_level");
    if (err < 0) {
        pr_err("can't request gpio%d (error %d)\n", SUT_GPIO_SC_1, err);
        goto err_free_gpio_return;
    }

    gpio_irq = gpio_to_irq(SUT_GPIO_SC_1);
    if (gpio_irq < 0) {
        pr_err("failed to get IRQ for pin %d.\n", SUT_GPIO_SC_1);
        goto err_free_irq_return;
    }

    err = request_threaded_irq(
        gpio_irq,
        NULL,
        gpio_sc_threaded_interrupt_handler,
        IRQF_TRIGGER_LOW | IRQF_ONESHOT,
        "threaded_irq_test",
        NULL
        );
    if (err < 0) {
        pr_err("failed to enable IRQ %d for pin %d error code: %d.\n",
                gpio_irq, SUT_GPIO_SC_1, err);
        goto err_free_irq_return;
    }

    pr_info("beginning GPIO threaded IRQ test.\n");

    init_timer(&timer);
    timer.expires = jiffies + delay;
    timer.function = gpio_sc_threaded_timer_handler;
    add_timer(&timer);

    return 0;

err_free_irq_return:
    gpio_free(SUT_GPIO_SC_1);
err_free_gpio_return:
    gpio_free(SUT_GPIO_SC_0);
err_return:
    return err;
}

/**
 * gpio_sc_threaded_teardown
 *
 * Threaded interrupt test teardown
 */
static int gpio_sc_threaded_teardown(void)
{
    free_irq(gpio_irq, NULL);

    if (del_timer_sync(&timer) == 1)
		pr_err("Removed active timer.\n");

    gpio_free(SUT_GPIO_SC_1);
    gpio_free(SUT_GPIO_SC_0);

    pr_info("unloaded IRQ toggle test.\n");

    return 0;
}

/*
 * File ops
 */
static long gip_test_ioctl(struct file *file, unsigned int cmd,
		unsigned long arg)
{
	int ret = -EINVAL;

	switch (cmd) {
		case IOCTL_QRK_GPIO_11:
			/* Edge-triggered interrupts */
			ret = gpio_sc_edge_int(0);
			break;
		case IOCTL_QRK_GPIO_11_CLEANUP:
			/* Edge-triggered interrupts cleanup */
			ret = gpio_sc_edge_int_teardown();
			break;
		case IOCTL_QRK_GPIO_12:
			/* Edge-triggered interrupts (masking) */
			ret = gpio_sc_edge_int(1);
			break;
		case IOCTL_QRK_GPIO_12_CLEANUP:
			/* Edge-triggered interrupts (masking) cleanup */
			ret = gpio_sc_edge_int_teardown();
			break;
		case IOCTL_QRK_GPIO_13:
			/* GPIO debounce (edge) */
			ret = gpio_sc_debounce(0);
			break;
		case IOCTL_QRK_GPIO_13_CLEANUP:
			/* GPIO debounce cleanup (edge) */
			ret = gpio_sc_debounce_teardown(0);
			break;
		case IOCTL_QRK_GPIO_14:
			/* High-level triggered interrupts */
			ret = gpio_sc_level_int(1, 0);
			break;
		case IOCTL_QRK_GPIO_14_CLEANUP:
			/* High-level triggered interrupts cleanup */
			ret = gpio_sc_level_int_teardown();
			break;
		case IOCTL_QRK_GPIO_15:
			/* Low-level triggered interrupts */
			ret = gpio_sc_level_int(0, 0);
			break;
		case IOCTL_QRK_GPIO_15_CLEANUP:
			/*Low-level triggered interrupts cleanup */
			ret = gpio_sc_level_int_teardown();
			break;
		case IOCTL_QRK_GPIO_16:
			/* Level triggered interrupts (masking) */
			ret = gpio_sc_level_int(1, 1);
			break;
		case IOCTL_QRK_GPIO_16_CLEANUP:
			/* Level triggered interrupts (masking) cleanup */
			ret = gpio_sc_level_int_teardown();
			break;
		case IOCTL_QRK_GPIO_17:
			/* GPIO debounce (level) */
			ret = gpio_sc_debounce(1);
			break;
		case IOCTL_QRK_GPIO_17_CLEANUP:
			/* GPIO debounce cleanup (level) */
			ret = gpio_sc_debounce_teardown(1);
			break;
		case IOCTL_QRK_GPIO_19:
			/* Register IRQ for SC_GPIO0 (PM transitions test) */
			ret = gpio_sc_pm_test_int();
			break;
		case IOCTL_QRK_GPIO_19_CLEANUP:
			/* Free IRQ for SC_GPIO0 (PM transitions test) */
			ret = gpio_sc_pm_test_int_teardown();
			break;
		case IOCTL_QRK_GPIO_20:
			/* NC bitbanged SPI */
			ret = gpio_spidev_register(1);
			break;
		case IOCTL_QRK_GPIO_20_CLEANUP:
			/* NC bitbanged SPI cleanup */
			ret = gpio_spidev_unregister(1);
			break;
		case IOCTL_QRK_GPIO_21:
			/* SC bitbanged SPI */
			ret = gpio_spidev_register(0);
			break;
		case IOCTL_QRK_GPIO_21_CLEANUP:
			/* SC bitbanged SPI cleanup */
			ret = gpio_spidev_unregister(0);
			break;
		case IOCTL_QRK_GPIO_24:
			/*
			 * SC GPIO interrupt performance test.
			 * Note it's shared between QRK_GPIO_24 and QRK_GPIO_25
			 * plus it doesn't need any cleanup call.
			 */
			ret = gpio_sc_interrupt_perf(arg);
			break;
		case IOCTL_QRK_GPIO_26:
			/* Interrupt for basic loopback test */
			ret = gpio_sc_basic_int();
			break;
		case IOCTL_QRK_GPIO_26_CLEANUP:
			/* Interrupt for basic loopback test cleanup */
			ret = gpio_sc_basic_int_teardown();
			break;
		case IOCTL_QRK_GPIO_28:
			/* Threaded interrupt */
			ret = gpio_sc_threaded();
			break;
		case IOCTL_QRK_GPIO_28_CLEANUP:
			/* Threaded interrupt cleanup*/
			ret = gpio_sc_threaded_teardown();
			break;
		case IOCTL_QRK_GIP_SYSTEM_SUSPEND:
			ret = gip_system_power_transition(0);
			break;
		case IOCTL_QRK_GIP_SYSTEM_RESUME:
			ret = gip_system_power_transition(1);
			break;
		case IOCTL_QRK_GPIO_NMI_ENABLE:
			ret = gpio_nmi_enable(1);
			break;
		case IOCTL_QRK_GPIO_NMI_DISABLE:
			ret = gpio_nmi_enable(0);
			break;
		default:
			break;
	}

	return ret;
}

static int gip_test_open(struct inode *inode, struct file *file)
{
	mutex_lock(&gip_test_mutex);
	nonseekable_open(inode, file);

	if (mutex_lock_interruptible(&gip_test_dev.open_lock)) {
		mutex_unlock(&gip_test_mutex);
		return -ERESTARTSYS;
	}

	if (gip_test_dev.opened) {
		mutex_unlock(&gip_test_dev.open_lock);
		mutex_unlock(&gip_test_mutex);
		return -EINVAL;
	}

	gip_test_dev.opened++;
	mutex_unlock(&gip_test_dev.open_lock);
	mutex_unlock(&gip_test_mutex);
	return 0;
}

static int gip_test_release(struct inode *inode, struct file *file)
{
	mutex_lock(&gip_test_dev.open_lock);
	gip_test_dev.opened = 0;
	mutex_unlock(&gip_test_dev.open_lock);

	return 0;
}

static const struct file_operations gip_test_file_ops = {
	.open = gip_test_open,
	.release = gip_test_release,
	.unlocked_ioctl = gip_test_ioctl,
	.llseek = no_llseek,
};

/**
 * intel_qrk_gip_test_probe
 *
 * @param pdev: Platform device
 * @return 0 success < 0 failure
 *
 * Callback from platform sub-system to probe
 */
static int intel_qrk_gip_test_probe(struct platform_device * pdev)
{
	int retval = 0;
	unsigned int minor = 0;

	mutex_init(&gip_test_dev.open_lock);
	cdev_init(&gip_test_dev.cdev, &gip_test_file_ops);
	gip_test_dev.cdev.owner = THIS_MODULE;

	retval = cdev_add(&gip_test_dev.cdev, MKDEV(gip_test_major, minor), 1);
	if (retval) {
		printk(KERN_ERR "chardev registration failed\n");
		return -EINVAL;
	}
	if (IS_ERR(device_create(gip_test_class, NULL,
				 MKDEV(gip_test_major, minor), NULL,
				 "giptest%u", minor))){
		dev_err(&pdev->dev, "can't create device\n");
		return -EINVAL;
	}

	return 0;

}

static int intel_qrk_gip_test_remove(struct platform_device * pdev)
{
	unsigned int minor = MINOR(gip_test_dev.cdev.dev);

	device_destroy(gip_test_class, MKDEV(gip_test_major, minor));
	cdev_del(&gip_test_dev.cdev);

	class_destroy(gip_test_class);

	return 0;
}

/*
 * Platform structures useful for interface to PM subsystem
 */
static struct platform_driver intel_qrk_gip_test_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
	},
	.remove = intel_qrk_gip_test_remove,
};

/**
 * intel_qrk_gip_test_init
 *
 * Load module.
 */
static int __init intel_qrk_gip_test_init(void)
{
	int retval = 0;
	dev_t dev;

	gip_test_class = class_create(THIS_MODULE,"qrk_gip_test");
	if (IS_ERR(gip_test_class)) {
		retval = PTR_ERR(gip_test_class);
		printk(KERN_ERR "gip_test: can't register gip_test class\n");
		goto err;
	}

	retval = alloc_chrdev_region(&dev, 0, 1, "gip_test");
	if (retval) {
		printk(KERN_ERR "earam_test: can't register character device\n");
		goto err_class;
	}
	gip_test_major = MAJOR(dev);

	memset(&gip_test_dev, 0x00, sizeof(gip_test_dev));
	gip_test_dev.pldev = platform_create_bundle(
		&intel_qrk_gip_test_driver, intel_qrk_gip_test_probe, NULL, 0, NULL, 0);

	if(IS_ERR(gip_test_dev.pldev)){
		printk(KERN_ERR "platform_create_bundle fail!\n");
		retval = PTR_ERR(gip_test_dev.pldev);
		goto err_class;
	}

	return 0;

err_class:
	class_destroy(gip_test_class);
err:
	return retval;
}

static void __exit intel_qrk_gip_test_exit(void)
{
	platform_device_unregister(gip_test_dev.pldev);
	platform_driver_unregister(&intel_qrk_gip_test_driver);
}

module_init(intel_qrk_gip_test_init);
module_exit(intel_qrk_gip_test_exit);

MODULE_AUTHOR("Josef Ahmad <josef.ahmad@intel.com>");
MODULE_DESCRIPTION("Quark GIP test module");
MODULE_LICENSE("Dual BSD/GPL");
