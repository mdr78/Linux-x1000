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
 * Intel Quark GIP (GPIO/I2C) - GPIO-specific PCI and core driver
 *
 *  PCI glue logic and core driver for Quark GIP/GPIO.
 *  The GIP GPIO device is the DesignWare GPIO. This file defines the PCI glue
 *  for this driver and as well as the core logic for the device.
 *  Please note only a single instance of the GPIO device is supported.
 *  The default number of GPIO is 8, all interrupt-capable.
 */

#include <linux/errno.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <linux/mfd/intel_qrk_gip.h>

#ifdef CONFIG_INTEL_QRK_GPIO_UIO
#include <linux/uio_driver.h>
#endif

static void gpio_restrict_release(struct device *dev) {}
static struct platform_device gpio_restrict_pdev = {
	.name	= "gpio-restrict-sc",
	.dev.release = gpio_restrict_release,
};

#ifdef CONFIG_INTEL_QRK_GPIO_UIO
struct uio_info *info;
#endif

/* The base GPIO number under GPIOLIB framework */
#define INTEL_QRK_GIP_GPIO_BASE		8

/* The default number of South-Cluster GPIO on Quark. */
#define INTEL_QRK_GIP_NGPIO		8

/* The GPIO private data. */
static struct gpio_chip *gc;
static struct irq_chip_generic *igc;
static void __iomem *reg_base;
static spinlock_t lock;
static int irq_base;

/* Store GPIO context across system-wide suspend/resume transitions */
static struct gpio_saved_regs {
	u32 data;
	u32 dir;
	u32 int_en;
	u32 int_mask;
	u32 int_type;
	u32 int_pol;
	u32 int_deb;
} saved_regs;

/* PortA registers set. Note other ports are unused */
#define PORTA_DATA			0x00	/* Data */
#define PORTA_DIR			0x04	/* Direction */
#define PORTA_INT_EN			0x30	/* Interrupt enable */
#define PORTA_INT_MASK			0x34	/* Interrupt mask */
#define PORTA_INT_TYPE_LEVEL		0x38	/* Interrupt level*/
#define PORTA_INT_POLARITY		0x3c	/* Interrupt polarity */
#define PORTA_INT_STATUS		0x40	/* Interrupt status */
#define PORTA_INT_RAW_STATUS		0x44	/* Interrupt raw status */
#define PORTA_DEBOUNCE			0x48	/* Debounce enable */
#define PORTA_INT_EOI			0x4c	/* Clear interrupt */
#define PORTA_EXT			0x50	/* External */

/**
 * intel_qrk_gpio_get
 * @param chip: Pointer to GPIO chip registered by GPIOLIB
 * @param offset: the GPIO number within the GPIOLIB chip
 * @return 0 if GPIO is deasserted, 1 if GPIO is asserted
 *
 * Read back the value of a GPIO.
 */
static int intel_qrk_gpio_get(struct gpio_chip *chip, unsigned offset)
{
	void __iomem *reg_ext = reg_base + PORTA_EXT;
	u32 val_ext = ioread32(reg_ext);

	val_ext &= BIT(offset % 32);
	return (val_ext > 0);
}

/**
 * intel_qrk_gpio_set
 * @param chip: Pointer to GPIO chip registered by GPIOLIB
 * @param offset: the GPIO number within the GPIOLIB chip
 *
 * Set value of a GPIO.
 */
static void intel_qrk_gpio_set(struct gpio_chip *chip, unsigned offset,
				int value)
{
	void __iomem *reg_data = reg_base + PORTA_DATA;
	u32 val_data = 0;
	unsigned long flags = 0;

	spin_lock_irqsave(&lock, flags);

	val_data = ioread32(reg_data);
	if (value)
		iowrite32(val_data | BIT(offset % 32), reg_data);
	else
		iowrite32(val_data & ~BIT(offset % 32), reg_data);

	spin_unlock_irqrestore(&lock, flags);
}

/**
 * intel_qrk_gpio_direction_input
 * @param chip: Pointer to GPIO chip registered by GPIOLIB
 * @param offset: the GPIO number within the GPIOLIB chip
 * @return always 0 (success)
 *
 * Set direction of a GPIO as input.
 */
static int intel_qrk_gpio_direction_input(struct gpio_chip *chip,
						unsigned offset)
{
	u32 val_dir = 0;
	void __iomem *reg_dir = reg_base + PORTA_DIR;
	unsigned long flags = 0;

	spin_lock_irqsave(&lock, flags);

	val_dir = ioread32(reg_dir);
	iowrite32(val_dir & ~BIT(offset % 32), reg_dir);

	spin_unlock_irqrestore(&lock, flags);

	return 0;
}

/**
 * intel_qrk_gpio_direction_output
 * @param chip: Pointer to GPIO chip registered by GPIOLIB
 * @param offset: the GPIO number within the GPIOLIB chip
 * @param value: value to be driven to the GPIO
 * @return always 0 (success)
 *
 * Set the default value of a GPIO, and then set direction as output.
 */
static int intel_qrk_gpio_direction_output(struct gpio_chip *chip,
			unsigned offset, int value)
{
	u32 val_dir = 0;
	void __iomem *reg_dir = reg_base + PORTA_DIR;
	unsigned long flags = 0;

	/* Ensure glitch-free operation. */
	intel_qrk_gpio_set(chip, offset, value);

	spin_lock_irqsave(&lock, flags);

	val_dir = ioread32(reg_dir);
	iowrite32(val_dir | BIT(offset % 32), reg_dir);

	spin_unlock_irqrestore(&lock, flags);

	return 0;
}

/**
 * intel_qrk_gpio_set_debounce
 * @param chip: Pointer to GPIO chip registered by GPIOLIB
 * @param offset: the GPIO number within the GPIOLIB chip
 * @param debounce: 1 to enable, 0 to disable
 * @return always 0 (success)
 *
 * Enable/disable interrupt debounce logic for a GPIO.
 */
static int intel_qrk_gpio_set_debounce(struct gpio_chip *chip,
				 unsigned offset, unsigned debounce)
{
	u32 val_deb = 0;
	void __iomem *reg_deb = reg_base + PORTA_DEBOUNCE;
	unsigned long flags = 0;

	spin_lock_irqsave(&lock, flags);

	val_deb = ioread32(reg_deb);
	if (debounce)
		iowrite32(val_deb | BIT(offset % 32), reg_deb);
	else
		iowrite32(val_deb & ~BIT(offset % 32), reg_deb);

	spin_unlock_irqrestore(&lock, flags);

	return 0;
}

/**
 * intel_qrk_gpio_irq_type
 * @param irq_data: Pointer to information about the IRQ
 * @param type: set the triggering type of the interrupt
 * @return always 0 (success)
 *
 * Set interrupt triggering type for a GPIO.
 *
 * intel_qrk_gpio_irq_type() is called with irq_desc lock held.
 *
 */
static int intel_qrk_gpio_irq_type(struct irq_data *d, unsigned type)
{
	int ret = 0;
	unsigned long flags = 0;
	void __iomem *reg_level = reg_base + PORTA_INT_TYPE_LEVEL;
	void __iomem *reg_pol = reg_base + PORTA_INT_POLARITY;
	u32 val_level = 0;
	u32 val_pol = 0;
	u32 gpio = 0;

	if (NULL == d) {
		pr_err("%s(): null irq_data\n",  __func__);
		return -EFAULT;
	}

	gpio = d->irq - irq_base;

	spin_lock_irqsave(&lock, flags);

	val_level = ioread32(reg_level);
	val_pol = ioread32(reg_pol);

	switch (type) {
	case IRQ_TYPE_EDGE_RISING:
		iowrite32(val_level | BIT(gpio % 32), reg_level);
		iowrite32(val_pol | BIT(gpio % 32), reg_pol);
		break;
	case IRQ_TYPE_EDGE_FALLING:
		iowrite32(val_level | BIT(gpio % 32), reg_level);
		iowrite32(val_pol & ~BIT(gpio % 32), reg_pol);
		break;
	case IRQ_TYPE_LEVEL_HIGH:
		iowrite32(val_level & ~BIT(gpio % 32), reg_level);
		iowrite32(val_pol | BIT(gpio % 32), reg_pol);
		break;
	case IRQ_TYPE_LEVEL_LOW:
		iowrite32(val_level & ~BIT(gpio % 32), reg_level);
		iowrite32(val_pol & ~BIT(gpio % 32), reg_pol);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	if (type & (IRQ_TYPE_LEVEL_HIGH | IRQ_TYPE_LEVEL_LOW)) {
		__irq_set_handler_locked(d->irq, handle_level_irq);
	} else if (0 == ret && type & (IRQ_TYPE_EDGE_FALLING
		   | IRQ_TYPE_EDGE_RISING)) {
		__irq_set_handler_locked(d->irq, handle_edge_irq);
	}

	spin_unlock_irqrestore(&lock, flags);

	return ret;
}

/**
 * intel_qrk_gpio_irq_unmask
 * @param irq_data: Pointer to information about the IRQ
 *
 * Unmask interrupts for a GPIO.
 */
static void intel_qrk_gpio_irq_unmask(struct irq_data *d)
{
	unsigned long flags = 0;
	void __iomem *reg_mask = reg_base + PORTA_INT_MASK;
	u32 val_mask = 0;
	unsigned gpio = 0;

	if (NULL == d) {
		pr_err("%s(): null irq_data\n", __func__);
		return;
	}

	gpio = d->irq - irq_base;

	spin_lock_irqsave(&lock, flags);
	val_mask =  ioread32(reg_mask);
	iowrite32(val_mask & ~BIT(gpio % 32), reg_mask);
	spin_unlock_irqrestore(&lock, flags);
}

/**
 * intel_qrk_gpio_irq_mask
 * @param irq_data: Pointer to information about the IRQ
 *
 * Mask interrupts for a GPIO.
 */
static void intel_qrk_gpio_irq_mask(struct irq_data *d)
{
	unsigned long flags = 0;
	void __iomem *reg_mask = reg_base + PORTA_INT_MASK;
	u32 val_mask = 0;
	unsigned gpio = 0;

	if (NULL == d) {
		pr_err("%s(): null irq_data\n", __func__);
		return;
	}

	gpio = d->irq - irq_base;

	spin_lock_irqsave(&lock, flags);
	val_mask =  ioread32(reg_mask);
	iowrite32(val_mask | BIT(gpio % 32), reg_mask);
	spin_unlock_irqrestore(&lock, flags);
}

/**
 * intel_qrk_gpio_irq_enable
 * @param irq_data: Pointer to information about the IRQ
 *
 * Enable interrupts for a GPIO.
 */
static void intel_qrk_gpio_irq_enable(struct irq_data *d)
{
	unsigned long flags = 0;
	void __iomem *reg_inte = reg_base + PORTA_INT_EN;
	u32 val_inte = 0;
	unsigned gpio = 0;

	if (NULL == d) {
		pr_err("%s(): null irq_data\n", __func__);
		return;
	}

	gpio = d->irq - irq_base;

	spin_lock_irqsave(&lock, flags);
	val_inte =  ioread32(reg_inte);
	iowrite32(val_inte | BIT(gpio % 32), reg_inte);
	spin_unlock_irqrestore(&lock, flags);
}

/**
 * intel_qrk_gpio_irq_disable
 * @param irq_data: Pointer to information about the IRQ
 *
 * Disable interrupts for a GPIO.
 */
static void intel_qrk_gpio_irq_disable(struct irq_data *d)
{
	unsigned long flags = 0;
	void __iomem *reg_inte = reg_base + PORTA_INT_EN;
	u32 val_inte = 0;
	unsigned gpio = 0;

	if (NULL == d) {
		pr_err("%s(): null irq_data\n", __func__);
		return;
	}

	gpio = d->irq - irq_base;

	spin_lock_irqsave(&lock, flags);
	val_inte =  ioread32(reg_inte);
	iowrite32(val_inte & ~BIT(gpio % 32), reg_inte);
	spin_unlock_irqrestore(&lock, flags);
}

/**
 * intel_qrk_gpio_irq_ack
 * @param irq_data: Pointer to information about the IRQ
 *
 * Acknowledge the edge triggered interrupt by writing the appropriate bit to
 * the gpio_porta_eoi register. This function is called by the irq flow
 * handler.
 */
static void intel_qrk_gpio_irq_ack(struct irq_data *d)
{
	void __iomem *reg_eoi = reg_base + PORTA_INT_EOI;
	u32 line = d->irq - irq_base;

	iowrite32(BIT(line), reg_eoi);
}

/**
 * intel_qrk_gpio_to_irq
 * @param chip: Pointer to GPIO chip registered by GPIOLIB
 * @param offset: the GPIO number within the GPIOLIB chip
 * @return IRQ associated to GPIO
 *
 * Compute the IRQ number based on the GPIO.
 */
static int intel_qrk_gpio_to_irq(struct gpio_chip *chip, unsigned offset)
{
	return irq_base + offset;
}

/**
 * intel_qrk_gpio_isr
 * @param irq: IRQ number
 * @param dev_id: cookie used to tell what instance of the driver the interrupt
 *                belongs to
 * @return IRQ_HANDLED if interrupt served, IRQ_NONE if no interrupt pending
 *
 * Interrupt Service Routine for GPIO. Identify which GPIOs (if any) is pending
 * for interrupt to be served, acknowledge the interrupt and serve it.
 */
irqreturn_t intel_qrk_gpio_isr(int irq, void *dev_id)
{
	irqreturn_t ret = IRQ_NONE;
	u32 pending = 0, gpio = 0;
	void __iomem *reg_pending = reg_base + PORTA_INT_STATUS;

	/* Which pin (if any) triggered the interrupt */
	while ((pending = ioread32(reg_pending))) {

		/* Serve each asserted interrupt */
		do {
			gpio = __ffs(pending);
			generic_handle_irq(
				gpio_to_irq(INTEL_QRK_GIP_GPIO_BASE + gpio));
			pending &= ~BIT(gpio);
			ret = IRQ_HANDLED;
		} while(pending);
	}

	return ret;
}

/**
 * intel_qrk_gpio_save_state
 *
 * Save GPIO register state for system-wide suspend events and mask out
 * interrupts.
 */
void intel_qrk_gpio_save_state(void)
{
	unsigned long flags = 0;

	spin_lock_irqsave(&lock, flags);

	saved_regs.int_mask = ioread32(reg_base + PORTA_INT_MASK);
	saved_regs.int_en = ioread32(reg_base + PORTA_INT_EN);
	saved_regs.int_deb = ioread32(reg_base + PORTA_DEBOUNCE);
	saved_regs.int_pol = ioread32(reg_base + PORTA_INT_POLARITY);
	saved_regs.int_type = ioread32(reg_base + PORTA_INT_TYPE_LEVEL);
	saved_regs.dir = ioread32(reg_base + PORTA_DIR);
	saved_regs.data = ioread32(reg_base + PORTA_DATA);

	/* Mask out interrupts */
	iowrite32(0xffffffff, reg_base + PORTA_INT_MASK);

	spin_unlock_irqrestore(&lock, flags);
}

/**
 * intel_qrk_gpio_restore_state
 *
 * Restore GPIO register state for system-wide resume events and clear out
 * spurious interrupts.
 */
void intel_qrk_gpio_restore_state(void)
{
	unsigned long flags = 0;

	spin_lock_irqsave(&lock, flags);

	iowrite32(saved_regs.data, reg_base + PORTA_DATA);
	iowrite32(saved_regs.dir, reg_base + PORTA_DIR);
	iowrite32(saved_regs.int_type, reg_base + PORTA_INT_TYPE_LEVEL);
	iowrite32(saved_regs.int_pol, reg_base + PORTA_INT_POLARITY);
	iowrite32(saved_regs.int_deb, reg_base + PORTA_DEBOUNCE);
	iowrite32(saved_regs.int_en, reg_base + PORTA_INT_EN);
	iowrite32(saved_regs.int_mask, reg_base + PORTA_INT_MASK);

	/* Clear out spurious interrupts */
	iowrite32(0xffffffff, reg_base + PORTA_INT_EOI);

	spin_unlock_irqrestore(&lock, flags);
}

/**
 * intel_qrk_gpio_probe
 * @param pdev: Pointer to GIP PCI device
 * @return 0 success < 0 failure
 *
 * Perform GPIO-specific probing on behalf of the top-level GIP driver.
 * Initiate the GPIO device.
 */
int intel_qrk_gpio_probe(struct pci_dev *pdev)
{
	int retval = 0;
	resource_size_t start = 0, len = 0;

#ifdef CONFIG_INTEL_QRK_GPIO_UIO
	/* Get UIO memory */
	info = kzalloc(sizeof(struct uio_info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;
#endif

	/* Determine the address of the GPIO area */
	start = pci_resource_start(pdev, GIP_GPIO_BAR);
	len = pci_resource_len(pdev, GIP_GPIO_BAR);
	if (!start || len == 0) {
		dev_err(&pdev->dev, "bar%d not set\n", GIP_GPIO_BAR);
		retval = -ENODEV;
		goto exit;
	}

	reg_base = ioremap_nocache(start, len);
	if (NULL == reg_base) {
		dev_err(&pdev->dev, "I/O memory remapping failed\n");
		retval = -EFAULT;
		goto exit;
	}

	memset(&saved_regs, 0x0, sizeof(saved_regs));

	gc = kzalloc(sizeof(struct gpio_chip), GFP_KERNEL);
	if (!gc) {
		retval = -ENOMEM;
		goto err_iounmap;
	}

	gc->label = "intel_qrk_gip_gpio";
	gc->owner = THIS_MODULE;
	gc->direction_input = intel_qrk_gpio_direction_input;
	gc->direction_output = intel_qrk_gpio_direction_output;
	gc->get = intel_qrk_gpio_get;
	gc->set = intel_qrk_gpio_set;
	gc->set_debounce = intel_qrk_gpio_set_debounce;
	gc->to_irq = intel_qrk_gpio_to_irq;
	gc->base = INTEL_QRK_GIP_GPIO_BASE;
	gc->ngpio = INTEL_QRK_GIP_NGPIO;
	gc->can_sleep = 0;
	retval = gpiochip_add(gc);
	if (retval) {
		dev_err(&pdev->dev, "failure adding GPIO chip\n");
		goto err_free_gpiochip;
	}

	spin_lock_init(&lock);

	/*
	 * Allocate a range of IRQ descriptor for the available GPIO.
	 * IRQs are allocated dynamically.
	 */
	irq_base = irq_alloc_descs(-1, 0, gc->ngpio, NUMA_NO_NODE);
	if (irq_base < 0) {
		dev_err(&pdev->dev, "failure adding GPIO IRQ descriptors\n");
		goto err_remove_gpiochip;
	}

	retval = platform_device_register(&gpio_restrict_pdev);
	if (retval < 0){
		goto err_free_irq_descs;
	}

	igc = irq_alloc_generic_chip("intel_qrk_gip_gpio", 1, irq_base,
			reg_base, handle_simple_irq);
	if (NULL == igc) {
		retval = -ENOMEM;
		goto err_unregister_platform_device;
	}

#ifdef CONFIG_INTEL_QRK_GPIO_UIO
	/* UIO */
	info->mem[0].addr = start;
	info->mem[0].internal_addr = reg_base;
	info->mem[0].size = len;
	info->mem[0].memtype = UIO_MEM_PHYS;
	info->mem[0].name = "gpio_regs";
	info->name = "gpio uio";
	info->version = "0.0.1";

	if (uio_register_device(&pdev->dev, info))
		goto err_unregister_platform_device;

	pr_info("%s UIO addr 0x%08x internal_addr 0x%08x size %lu memtype %d\n",
		__func__, (unsigned int)info->mem[0].addr,
		(unsigned int)info->mem[0].internal_addr, info->mem[0].size,
		info->mem[0].memtype);
#endif

	igc->chip_types->chip.irq_mask = intel_qrk_gpio_irq_mask;
	igc->chip_types->chip.irq_unmask = intel_qrk_gpio_irq_unmask;
	igc->chip_types->chip.irq_set_type = intel_qrk_gpio_irq_type;
	igc->chip_types->chip.irq_enable = intel_qrk_gpio_irq_enable;
	igc->chip_types->chip.irq_disable = intel_qrk_gpio_irq_disable;
	igc->chip_types->chip.irq_ack = intel_qrk_gpio_irq_ack;

	irq_setup_generic_chip(igc, IRQ_MSK(gc->ngpio), IRQ_GC_INIT_MASK_CACHE,
			       IRQ_NOREQUEST | IRQ_NOPROBE, 0);

	return 0;

err_unregister_platform_device:
	platform_device_unregister(&gpio_restrict_pdev);
err_free_irq_descs:
	irq_free_descs(irq_base, gc->ngpio);
err_remove_gpiochip:
	if (0 != gpiochip_remove(gc))
		dev_err(&pdev->dev, "failed removing gpio_chip\n");
err_free_gpiochip:
	kfree(gc);
err_iounmap:
	iounmap(reg_base);
exit:
#ifdef CONFIG_INTEL_QRK_GPIO_UIO
	kfree(info);
#endif
	return retval;
}

/**
 * intel_qrk_gpio_remove
 * @param pdev: Pointer to GIP PCI device
 *
 * Perform GPIO-specific resource release on behalf of the top-level GIP
 * driver.
 */
void intel_qrk_gpio_remove(struct pci_dev *pdev)
{
	if (NULL == igc) {
		dev_err(&pdev->dev, "null pointer to irq_generic_chip\n");
		return;
	}
	if (NULL == gc) {
		dev_err(&pdev->dev, "null pointer to gpio_chip\n");
		return;
	}

	/* Tear down IRQ descriptors */
	irq_remove_generic_chip(igc, IRQ_MSK(gc->ngpio), 0,
				IRQ_NOREQUEST | IRQ_NOPROBE);
	kfree(igc);

	platform_device_unregister(&gpio_restrict_pdev);

	irq_free_descs(irq_base, gc->ngpio);

	/* Release GPIO chip */
	if (0 != gpiochip_remove(gc))
		dev_err(&pdev->dev, "failed removing gpio_chip\n");

#ifdef CONFIG_INTEL_QRK_GPIO_UIO
	uio_unregister_device(info);
	kfree(info);
#endif

	kfree(gc);
	iounmap(reg_base);

}
