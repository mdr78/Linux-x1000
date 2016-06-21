 /*
 * GPIO interface for Intel Poulsbo SCH
 *
 *  Copyright (c) 2010 CompuLab Ltd
 *  Copyright (c) 2014-2015 Intel Corporation
 *  Author: Denis Turischev <denis@compulab.co.il>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License 2 as published
 *  by the Free Software Foundation.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; see the file COPYING.  If not, write to
 *  the Free Software Foundation, 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/errno.h>
#include <linux/acpi.h>
#include <linux/platform_device.h>
#include <linux/pci_ids.h>
#include <linux/interrupt.h>
#include <linux/irq.h>

#include <linux/gpio.h>
#ifdef CONFIG_INTEL_QRK_GPIO_UIO
#include <linux/uio_driver.h>
#endif

#define GEN	0x00
#define GIO	0x04
#define GLV	0x08
#define GTPE	0x0C
#define GTNE	0x10
#define GGPE	0x14
#define GSMI	0x18
#define GTS	0x1C
#define RGTS	0x3C
#define CGNMIEN	0x40
#define RGNMIEN	0x44

#define RESUME_WELL_OFFSET	0x20

/* Maximum number of GPIOs supported by this driver */
#define MAX_GPIO	64

/* Cache register context */
struct sch_gpio_context {
	/* Core well generic registers */
	u32 cgen;
	u32 cgio;
	u32 cglvl;
	u32 cgsmi;
	u32 cgnmien;
	/* cache irq trigger setup */
	DECLARE_BITMAP(gtpe_irqs, MAX_GPIO);
	DECLARE_BITMAP(gtne_irqs, MAX_GPIO);
};

static void gpio_restrict_release(struct device *dev) {}
static struct platform_device gpio_restrict_pdev = {
	.name	= "gpio-restrict-nc",
	.dev.release = gpio_restrict_release,
};

struct sch_gpio {
	struct gpio_chip chip;
	struct sch_gpio_context context;
	spinlock_t lock;
	unsigned short iobase;
	unsigned short core_base;
	unsigned short resume_base;
	int irq;
	int irq_base;
	bool irq_support;
	DECLARE_BITMAP(wake_irqs, MAX_GPIO);
#ifdef CONFIG_INTEL_QRK_GPIO_UIO
	struct uio_info info;
#endif
};

#define to_sch_gpio(gc)		container_of(gc, struct sch_gpio, chip)
#define irq_to_gpio_number()	(d->irq - sch->irq_base)

static unsigned sch_gpio_offset(struct sch_gpio *sch, unsigned gpio,
				unsigned reg)
{
	unsigned base = 0;

	if (gpio >= sch->resume_base) {
		gpio -= sch->resume_base;
		base += RESUME_WELL_OFFSET;
	}

	return base + reg + gpio / 8;
}

static unsigned sch_gpio_bit(struct sch_gpio *sch, unsigned gpio)
{
	if (gpio >= sch->resume_base)
		gpio -= sch->resume_base;

	return gpio % 8;
}

static int sch_gpio_reg_get(struct gpio_chip *gc, unsigned gpio, unsigned reg)
{
	struct sch_gpio *sch = to_sch_gpio(gc);
	unsigned short offset, bit;
	u8 reg_val;

	offset = sch_gpio_offset(sch, gpio, reg);
	bit = sch_gpio_bit(sch, gpio);

	reg_val = !!(inb(sch->iobase + offset) & BIT(bit));

	return reg_val;
}

static void sch_gpio_reg_set(struct gpio_chip *gc, unsigned gpio, unsigned reg,
			     int val)
{
	struct sch_gpio *sch = to_sch_gpio(gc);
	unsigned short offset, bit;
	u8 reg_val;

	offset = sch_gpio_offset(sch, gpio, reg);
	bit = sch_gpio_bit(sch, gpio);

	reg_val = inb(sch->iobase + offset);

	if (val)
		outb(reg_val | BIT(bit), sch->iobase + offset);
	else
		outb((reg_val & ~BIT(bit)), sch->iobase + offset);
}

static int sch_gpio_direction_in(struct gpio_chip *gc, unsigned gpio_num)
{
	struct sch_gpio *sch = to_sch_gpio(gc);
	unsigned long flags;

	spin_lock_irqsave(&sch->lock, flags);
	sch_gpio_reg_set(gc, gpio_num, GIO, 1);
	spin_unlock_irqrestore(&sch->lock, flags);

	return 0;
}

static int sch_gpio_get(struct gpio_chip *gc, unsigned gpio_num)
{
	return sch_gpio_reg_get(gc, gpio_num, GLV);
}

static void sch_gpio_set(struct gpio_chip *gc, unsigned gpio_num, int val)
{
	struct sch_gpio *sch = to_sch_gpio(gc);
	unsigned long flags;

	spin_lock_irqsave(&sch->lock, flags);
	sch_gpio_reg_set(gc, gpio_num, GLV, val);
	spin_unlock_irqrestore(&sch->lock, flags);
}

static int sch_gpio_direction_out(struct gpio_chip *gc, unsigned gpio_num,
				  int val)
{
	struct sch_gpio *sch = to_sch_gpio(gc);
	unsigned long flags;

	spin_lock_irqsave(&sch->lock, flags);
	sch_gpio_reg_set(gc, gpio_num, GIO, 0);
	spin_unlock_irqrestore(&sch->lock, flags);

	/*
	 * according to the datasheet, writing to the level register has no
	 * effect when GPIO is programmed as input.
	 * Actually the the level register is read-only when configured as an
	 * input.
	 * Thus presetting the output level before switching to output is _NOT_
	 * possible.
	 * Hence we set the level after configuring the GPIO as output.
	 * But we cannot prevent a short low pulse if direction is set to high
	 * and an external pull-up is connected.
	 */
	sch_gpio_set(gc, gpio_num, val);

	return 0;
}

static int sch_gpio_to_irq(struct gpio_chip *gc, unsigned offset)
{
	struct sch_gpio *sch = to_sch_gpio(gc);

	return sch->irq_base + offset;
}

static struct gpio_chip sch_gpio_chip = {
	.label			= "sch_gpio",
	.owner			= THIS_MODULE,
	.direction_input	= sch_gpio_direction_in,
	.get			= sch_gpio_get,
	.direction_output	= sch_gpio_direction_out,
	.set			= sch_gpio_set,
	.to_irq			= sch_gpio_to_irq,
};

static void sch_gpio_irq_enable(struct irq_data *d)
{
	struct sch_gpio *sch = irq_data_get_irq_chip_data(d);
	struct sch_gpio_context *regs = &sch->context;
	u32 gpio_num;
	unsigned long flags;

	gpio_num = irq_to_gpio_number();

	spin_lock_irqsave(&sch->lock, flags);

	if (test_bit(gpio_num, regs->gtpe_irqs))
		sch_gpio_reg_set(&sch->chip, gpio_num, GTPE, 1);
	if (test_bit(gpio_num, regs->gtne_irqs))
		sch_gpio_reg_set(&sch->chip, gpio_num, GTNE, 1);
	sch_gpio_reg_set(&sch->chip, gpio_num, GGPE, 1);

	spin_unlock_irqrestore(&sch->lock, flags);
}

static void sch_gpio_irq_disable(struct irq_data *d)
{
	struct sch_gpio *sch = irq_data_get_irq_chip_data(d);
	u32 gpio_num;
	unsigned long flags;

	gpio_num = irq_to_gpio_number();

	if (!test_bit(gpio_num, sch->wake_irqs)) {
		spin_lock_irqsave(&sch->lock, flags);
		sch_gpio_reg_set(&sch->chip, gpio_num, GGPE, 0);
		sch_gpio_reg_set(&sch->chip, gpio_num, GTPE, 0);
		sch_gpio_reg_set(&sch->chip, gpio_num, GTNE, 0);
		sch_gpio_reg_set(&sch->chip, gpio_num, GTS, 1);
		spin_unlock_irqrestore(&sch->lock, flags);
	}
}

static void sch_gpio_irq_ack(struct irq_data *d)
{
	struct sch_gpio *sch = irq_data_get_irq_chip_data(d);
	u32 gpio_num;

	gpio_num = irq_to_gpio_number();
	sch_gpio_reg_set(&(sch->chip), gpio_num, GTS, 1);
}

static int sch_gpio_irq_type(struct irq_data *d, unsigned type)
{
	struct sch_gpio *sch = irq_data_get_irq_chip_data(d);
	struct sch_gpio_context *regs = &sch->context;
	unsigned long flags;
	u32 gpio_num;

	gpio_num = irq_to_gpio_number();

	spin_lock_irqsave(&sch->lock, flags);

	switch (type) {
	case IRQ_TYPE_EDGE_RISING:
		sch_gpio_reg_set(&sch->chip, gpio_num, GTPE, 1);
		sch_gpio_reg_set(&sch->chip, gpio_num, GTNE, 0);
		/* cache trigger setup */
		set_bit(gpio_num, regs->gtpe_irqs);
		clear_bit(gpio_num, regs->gtne_irqs);
		break;

	case IRQ_TYPE_EDGE_FALLING:
		sch_gpio_reg_set(&sch->chip, gpio_num, GTNE, 1);
		sch_gpio_reg_set(&sch->chip, gpio_num, GTPE, 0);
		/* cache trigger setup */
		set_bit(gpio_num, regs->gtne_irqs);
		clear_bit(gpio_num, regs->gtpe_irqs);
		break;

	case IRQ_TYPE_EDGE_BOTH:
		sch_gpio_reg_set(&sch->chip, gpio_num, GTPE, 1);
		sch_gpio_reg_set(&sch->chip, gpio_num, GTNE, 1);
		/* cache trigger setup */
		set_bit(gpio_num, regs->gtpe_irqs);
		set_bit(gpio_num, regs->gtne_irqs);
		break;

	case IRQ_TYPE_NONE:
		sch_gpio_reg_set(&sch->chip, gpio_num, GTPE, 0);
		sch_gpio_reg_set(&sch->chip, gpio_num, GTNE, 0);
		/* cache trigger setup */
		clear_bit(gpio_num, regs->gtpe_irqs);
		clear_bit(gpio_num, regs->gtne_irqs);
		break;

	default:
		spin_unlock_irqrestore(&sch->lock, flags);
		return -EINVAL;
	}

	spin_unlock_irqrestore(&sch->lock, flags);

	return 0;
}

/*
 * Enables/Disables power-management wake-on of an IRQ.
 * Inhibits disabling of the specified IRQ if on != 0.
 * Make sure you call it via irq_set_irq_wake() with on = 1 during suspend
 * and with on = 0 during resume.
 * Returns 0 if success, negative error code otherwhise
 */
int sch_gpio_resume_irq_set_wake(struct irq_data *d, unsigned int on)
{
	struct sch_gpio *sch = NULL;
	u32 gpio_num = 0;
	int ret = 0;

	if (NULL == d) {
		pr_err("%s: null irq_data\n", __func__);
		ret = -EFAULT;
		goto end;
	}

	sch = irq_data_get_irq_chip_data(d);
	gpio_num = irq_to_gpio_number();
	/* This function is only relavent on resume well GPIO */
	if (gpio_num < sch->resume_base) {
		dev_err(sch->chip.dev,
				"unable to change wakeup on core well GPIO%d\n",
				gpio_num);
		ret = -EINVAL;
		goto end;
	}

	if (on)
		set_bit(gpio_num, sch->wake_irqs);
	else
		clear_bit(gpio_num, sch->wake_irqs);

end:
	return ret;
}

static struct irq_chip sch_irq_chip = {
	.irq_enable	= sch_gpio_irq_enable,
	.irq_disable	= sch_gpio_irq_disable,
	.irq_ack	= sch_gpio_irq_ack,
	.irq_set_type	= sch_gpio_irq_type,
	.irq_set_wake	= sch_gpio_resume_irq_set_wake,
};

static void sch_gpio_irqs_init(struct sch_gpio *sch)
{
	unsigned int i;

	for (i = 0; i < sch->chip.ngpio; i++) {
		irq_set_chip_data(i + sch->irq_base, sch);
		irq_set_chip_and_handler_name(i + sch->irq_base, &sch_irq_chip,
					handle_edge_irq, "sch_gpio_irq_chip");
	}
}

static void sch_gpio_irq_disable_all(struct sch_gpio *sch)
{
	/* Disable core well interrupts */
	outl(0x00, sch->iobase + GTPE);
	outl(0x00, sch->iobase + GTNE);
	outl(0x00, sch->iobase + GGPE);
	outl(0x00, sch->iobase + GSMI);
	outl(0x00, sch->iobase + CGNMIEN);

	/* Disable resume well interrupts */
	outl(0x00, sch->iobase + GTPE + RESUME_WELL_OFFSET);
	outl(0x00, sch->iobase + GTNE + RESUME_WELL_OFFSET);
	outl(0x00, sch->iobase + GGPE + RESUME_WELL_OFFSET);
	outl(0x00, sch->iobase + GSMI + RESUME_WELL_OFFSET);
	outl(0x00, sch->iobase + RGNMIEN);

	/* Clear any pending interrupts */
	outl(0xFFFFFFFF, sch->iobase + GTS);
	outl(0xFFFFFFFF, sch->iobase + GTS + RESUME_WELL_OFFSET);
}

static inline irqreturn_t do_serve_irq(int reg_status, unsigned int irq_base)
{
	int ret = IRQ_NONE;
	u32 pending = 0, gpio = 0;
	/* Which pins (if any) triggered the interrupt */
	pending = inl(reg_status);

	/* Serve each asserted interrupt */
	/* Note that the interrupt is cleared as part of the irq_ack of
	 * the handle_edge_irq callback
	 */
	while (pending) {
		gpio = __ffs(pending);
		generic_handle_irq(irq_base + gpio);
		pending &= ~BIT(gpio);
		ret = IRQ_HANDLED;
	}

	return ret;
}

static irqreturn_t sch_gpio_irq_handler(int irq, void *dev_id)
{
	struct sch_gpio *sch = dev_id;
	int ret = IRQ_NONE;
	/* Both the core and resume banks are consolidated,
	 * but the physical registers are still separated. In order
	 * to access the status of resume bank irq status, we need
	 * recalculate the irq base for resume bank
	 */
	int resume_irq_base = sch->irq_base + sch->resume_base;

	ret |= do_serve_irq(sch->iobase + GTS, sch->irq_base);
	ret |= do_serve_irq(sch->iobase + RGTS, resume_irq_base);

	return ret;
}

static int sch_gpio_probe(struct platform_device *pdev)
{
	struct sch_gpio *sch;
	struct resource *res, *res_irq;
	int err = 0;
	int ret;

	sch = devm_kzalloc(&pdev->dev, sizeof(*sch), GFP_KERNEL);
	if (!sch)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_IO, 0);
	if (!res)
		return -EBUSY;

	if (!devm_request_region(&pdev->dev, res->start, resource_size(res),
				 pdev->name))
		return -EBUSY;

	res_irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (res_irq) {
		sch->irq = res_irq->start;
		sch->irq_support = true;
	} else {
		dev_warn(&pdev->dev,
			 "failed to obtain irq number for device\n");
		sch->irq_support = false;
	}

	spin_lock_init(&sch->lock);
	sch->iobase = res->start;
	sch->chip = sch_gpio_chip;
	sch->chip.label = dev_name(&pdev->dev);
	sch->chip.dev = &pdev->dev;

	switch (pdev->id) {
	case PCI_DEVICE_ID_INTEL_SCH_LPC:
		sch->core_base = 0;
		sch->resume_base = 10;
		sch->chip.ngpio = 14;

		/*
		 * GPIO[6:0] enabled by default
		 * GPIO7 is configured by the CMC as SLPIOVR
		 * Enable GPIO[9:8] core powered gpios explicitly
		 */
		sch_gpio_reg_set(&sch->chip, 8, GEN, 1);
		sch_gpio_reg_set(&sch->chip, 9, GEN, 1);
		/*
		 * SUS_GPIO[2:0] enabled by default
		 * Enable SUS_GPIO3 resume powered gpio explicitly
		 */
		sch_gpio_reg_set(&sch->chip, 13, GEN, 1);
		break;

	case PCI_DEVICE_ID_INTEL_ITC_LPC:
		sch->core_base = 0;
		sch->resume_base = 5;
		sch->chip.ngpio = 14;
		break;

	case PCI_DEVICE_ID_INTEL_CENTERTON_ILB:
		sch->core_base = 0;
		sch->resume_base = 21;
		sch->chip.ngpio = 30;
		break;

	case PCI_DEVICE_ID_INTEL_QUARK_X1000_ILB:
		sch->core_base = 0;
		sch->resume_base = 2;
		sch->chip.ngpio = 8;
		break;

	default:
		return -ENODEV;
	}

	gpiochip_add(&sch->chip);

	if (sch->irq_support) {
		sch->irq_base = irq_alloc_descs(-1, 0, sch->chip.ngpio,
						NUMA_NO_NODE);
		if (sch->irq_base < 0) {
			dev_err(&pdev->dev,
				"failed to allocate GPIO IRQ descs\n");
			goto err_sch_intr_chip;
		}

		/* disable interrupts */
		sch_gpio_irq_disable_all(sch);

		err = devm_request_irq(&pdev->dev, sch->irq,
				       sch_gpio_irq_handler, IRQF_SHARED,
				       KBUILD_MODNAME, sch);

		if (err) {
			dev_err(&pdev->dev,
				"failed to request IRQ\n");
			goto err_sch_request_irq;
		}

		sch_gpio_irqs_init(sch);
	}

	platform_set_drvdata(pdev, sch);

#ifdef CONFIG_INTEL_QRK_GPIO_UIO
	/* UIO */
	sch->info.port[0].name = "gpio_regs";
	sch->info.port[0].start = res->start;
	sch->info.port[0].size = resource_size(res);
	sch->info.port[0].porttype = UIO_PORT_X86;
	sch->info.name = "sch_gpio";
	sch->info.version = "0.0.1";

	if (uio_register_device(&pdev->dev, &sch->info))
		goto err_sch_uio_register;

	pr_info("%s UIO port addr 0x%04x size %lu porttype %d\n",
		__func__, (unsigned int)sch->info.port[0].start,
		sch->info.port[0].size, sch->info.port[0].porttype);
#endif

	err = platform_device_register(&gpio_restrict_pdev);
	if (err < 0)
		goto err_sch_gpio_device_register;

	return 0;

err_sch_gpio_device_register:
#ifdef CONFIG_INTEL_QRK_GPIO_UIO
	uio_unregister_device(&sch->info);

err_sch_uio_register:
#endif
err_sch_request_irq:
	irq_free_descs(sch->irq_base, sch->chip.ngpio);

err_sch_intr_chip:
	ret = gpiochip_remove(&sch->chip);

	return err;
}

static int sch_gpio_remove(struct platform_device *pdev)
{
	int ret;
	struct sch_gpio *sch = platform_get_drvdata(pdev);

#ifdef CONFIG_INTEL_QRK_GPIO_UIO
	uio_unregister_device(&sch->info);
#endif

	if (sch->irq_support)
		irq_free_descs(sch->irq_base, sch->chip.ngpio);

	platform_device_unregister(&gpio_restrict_pdev);

	ret = gpiochip_remove(&sch->chip);

	return ret;
}

/*
 * Disables IRQ line of Legacy GPIO chip so that its state is not controlled by
 * PM framework (disabled before calling suspend_noirq callback and
 * re-enabled after calling resume_noirq callback of devices).
 */
static int sch_gpio_suspend_sys(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct sch_gpio *sch = platform_get_drvdata(pdev);

	disable_irq(sch->irq);

	return 0;
}

/*
 * Saves the state of configuration registers for Core Well GPIOs.
 * Don't touch Core Well interrupt triggers and SCI/GPE because they are
 * handled by the irqchip subsystem.
 * Don't touch Suspend Well GPIO registers because they are alive and
 * functional in both S3 and S0 states.
 */
static int sch_gpio_suspend_sys_noirq(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct sch_gpio *sch = platform_get_drvdata(pdev);
	struct sch_gpio_context *regs = &sch->context;

	regs->cgen	= inl(sch->iobase + GEN);
	regs->cgio	= inl(sch->iobase + GIO);
	regs->cglvl	= inl(sch->iobase + GLV);
	regs->cgsmi	= inl(sch->iobase + GSMI);
	regs->cgnmien	= inl(sch->iobase + CGNMIEN);

	return 0;
}

/*
 * Restore the context saved by sch_gpio_suspend_sys_noirq().
 */
static int sch_gpio_resume_sys_noirq(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct sch_gpio *sch = platform_get_drvdata(pdev);
	struct sch_gpio_context *regs = &sch->context;

	outl(regs->cgio, sch->iobase + GIO);
	outl(regs->cglvl, sch->iobase + GLV);
	outl(regs->cgsmi, sch->iobase + GSMI);
	outl(regs->cgnmien, sch->iobase + CGNMIEN);
	outl(regs->cgen, sch->iobase + GEN);

	return 0;
}

/*
 * Re-enables the IRQ line of Legacy GPIO chip.
 * Done here instead of pm_resume_no_irq() PM handler in order to be sure that
 * all the system busses (I2C, SPI) are resumed when the IRQ is fired,
 * otherwise a SPI or I2C device might fail to handle its own interrupt because
 * the IRQ handler (bottom half) involves talking to the device.
 */
static int sch_gpio_resume_sys(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct sch_gpio *sch = platform_get_drvdata(pdev);

	enable_irq(sch->irq);

	return 0;
}

const struct dev_pm_ops sch_gpio_pm_ops = {
	.suspend	= sch_gpio_suspend_sys,
	.suspend_noirq	= sch_gpio_suspend_sys_noirq,
	.resume_noirq	= sch_gpio_resume_sys_noirq,
	.resume		= sch_gpio_resume_sys,
};

static struct platform_driver sch_gpio_driver = {
	.driver = {
		.name	= "sch_gpio",
		.pm	= &sch_gpio_pm_ops,
	},
	.probe		= sch_gpio_probe,
	.remove		= sch_gpio_remove,
};

module_platform_driver(sch_gpio_driver);

MODULE_AUTHOR("Denis Turischev <denis@compulab.co.il>");
MODULE_DESCRIPTION("GPIO interface for Intel Poulsbo SCH");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:sch_gpio");
