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
#include <linux/uio_driver.h>

#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/bitmap.h>
#include <linux/types.h>

static DEFINE_SPINLOCK(gpio_lock);

#define CGEN	(0x00)
#define CGIO	(0x04)
#define CGLV	(0x08)

#define CGTPE	(0x0C)
#define CGTNE	(0x10)
#define CGGPE	(0x14)
#define CGSMI	(0x18)
#define CGTS	(0x1C)

#define RGEN	(0x20)
#define RGIO	(0x24)
#define RGLV	(0x28)

#define RGTPE   (0x2C)
#define RGTNE   (0x30)
#define RGGPE   (0x34)
#define RGSMI   (0x38)
#define RGTS    (0x3C)

#define CGNMIEN (0x40)
#define RGNMIEN (0x44)

#define RESOURCE_IRQ	9

/* Maximum number of resume GPIOS supported by this driver */
#define MAX_GPIO_IRQS	9

static unsigned long gpio_ba;

static struct uio_info *info;

static int irq_num;

/* Cache register context */
struct sch_gpio_context {
	/* Core well generic registers */
	u32 cgen;
	u32 cgio;
	u32 cglvl;
	u32 cgsmi;
	u32 cgnmien;
	/* Core well interrupt trigger enable */
	u32 cgtpe;
	u32 cgtne;
	/* Resume well interrupt trigger enable */
	u32 rgtpe;
	u32 rgtne;
};

struct sch_gpio {
	int irq_base_core;
	struct sch_gpio_context context;
	int irq_base_resume;
	DECLARE_BITMAP(wake_irqs, MAX_GPIO_IRQS);
};

static struct sch_gpio *chip_ptr;

static void qrk_gpio_restrict_release(struct device *dev) {}
static struct platform_device qrk_gpio_restrict_pdev = {
	.name	= "qrk-gpio-restrict-nc",
	.dev.release = qrk_gpio_restrict_release,
};

static void sch_gpio_reg_clear_if_set(unsigned short reg,
					unsigned short gpio_num)
{
	u8 curr_dirs;
	unsigned short offset, bit;

	offset = reg + gpio_num / 8;
	bit = gpio_num % 8;

	curr_dirs = inb(gpio_ba + offset);

	if (curr_dirs & (1 << bit))
		outb(curr_dirs & ~(1 << bit), gpio_ba + offset);
}

static void sch_gpio_reg_set_if_clear(unsigned short reg,
					unsigned short gpio_num)
{
	u8 curr_dirs;
	unsigned short offset, bit;

	offset = reg + gpio_num / 8;
	bit = gpio_num % 8;

	curr_dirs = inb(gpio_ba + offset);

	if (!(curr_dirs & (1 << bit)))
		outb(curr_dirs | (1 << bit), gpio_ba + offset);
}

static void sch_gpio_reg_set(unsigned short reg, unsigned short gpio_num,
				int val)
{
	u8 curr_dirs;
	unsigned short offset, bit;

	offset = reg + gpio_num / 8;
	bit = gpio_num % 8;

	curr_dirs = inb(gpio_ba + offset);

	if (val)
		outb(curr_dirs | (1 << bit), gpio_ba + offset);
	else
		outb(curr_dirs & ~(1 << bit), gpio_ba + offset);
}

static unsigned short sch_gpio_reg_get(unsigned short reg,
					unsigned short gpio_num)
{
	u8 curr_dirs;
	unsigned short offset, bit;

	offset = reg + gpio_num / 8;
	bit = gpio_num % 8;

	curr_dirs = !!(inb(gpio_ba + offset) & (1 << bit));

	return curr_dirs;
}

static int sch_gpio_core_direction_in(struct gpio_chip *gc, unsigned  gpio_num)
{
	unsigned long flags = 0;
	spin_lock_irqsave(&gpio_lock, flags);
	sch_gpio_reg_set_if_clear(CGIO, gpio_num);
	spin_unlock_irqrestore(&gpio_lock, flags);

	return 0;
}

static int sch_gpio_core_get(struct gpio_chip *gc, unsigned gpio_num)
{
	int res;
	res = sch_gpio_reg_get(CGLV, gpio_num);

	return res;
}

static void sch_gpio_core_set(struct gpio_chip *gc, unsigned gpio_num, int val)
{
	unsigned long flags = 0;
	spin_lock_irqsave(&gpio_lock, flags);
	sch_gpio_reg_set(CGLV, gpio_num, val);
	spin_unlock_irqrestore(&gpio_lock, flags);

}

static int sch_gpio_core_direction_out(struct gpio_chip *gc,
					unsigned gpio_num, int val)
{
	unsigned long flags = 0;
	spin_lock_irqsave(&gpio_lock, flags);
	sch_gpio_reg_clear_if_set(CGIO, gpio_num);
	spin_unlock_irqrestore(&gpio_lock, flags);

	return 0;
}

static int sch_gpio_core_to_irq(struct gpio_chip *gc, unsigned offset)
{
	return chip_ptr->irq_base_core + offset;
}

static struct gpio_chip sch_gpio_core = {
	.label			= "sch_gpio_core",
	.owner			= THIS_MODULE,
	.direction_input	= sch_gpio_core_direction_in,
	.get			= sch_gpio_core_get,
	.direction_output	= sch_gpio_core_direction_out,
	.set			= sch_gpio_core_set,
	.to_irq			= sch_gpio_core_to_irq,
};

static void sch_gpio_core_irq_enable(struct irq_data *d)
{
	unsigned long flags = 0;
	u32 gpio_num = d->irq - chip_ptr->irq_base_core;
	struct sch_gpio_context *regs = &chip_ptr->context;

	spin_lock_irqsave(&gpio_lock, flags);

	if (regs->cgtpe & BIT(gpio_num))
		sch_gpio_reg_set_if_clear(CGTPE, gpio_num);
	if (regs->cgtne & BIT(gpio_num))
		sch_gpio_reg_set_if_clear(CGTNE, gpio_num);
	sch_gpio_reg_set_if_clear(CGGPE, gpio_num);

	spin_unlock_irqrestore(&gpio_lock, flags);
}

static void sch_gpio_core_irq_disable(struct irq_data *d)
{
	unsigned long flags = 0;
	u32 gpio_num = 0;

	gpio_num = d->irq - chip_ptr->irq_base_core;

	spin_lock_irqsave(&gpio_lock, flags);
	sch_gpio_reg_clear_if_set(CGGPE, gpio_num);
	sch_gpio_reg_clear_if_set(CGTPE, gpio_num);
	sch_gpio_reg_clear_if_set(CGTNE, gpio_num);
	outb(BIT(gpio_num), gpio_ba + CGTS);
	spin_unlock_irqrestore(&gpio_lock, flags);
}

static void sch_gpio_core_irq_ack(struct irq_data *d)
{
	u32 gpio_num = 0;

	gpio_num = d->irq - chip_ptr->irq_base_core;
	outb(BIT(gpio_num), gpio_ba + CGTS);
}

static int sch_gpio_core_irq_type(struct irq_data *d, unsigned type)
{
	int ret = 0;
	unsigned long flags = 0;
	u32 gpio_num = 0;
	struct sch_gpio_context *regs = &chip_ptr->context;

	if (NULL == d) {
		pr_err("%s(): null irq_data\n",  __func__);
		return -EFAULT;
	}

	gpio_num = d->irq - chip_ptr->irq_base_core;

	spin_lock_irqsave(&gpio_lock, flags);

	switch (type) {
	case IRQ_TYPE_EDGE_RISING:
		sch_gpio_reg_clear_if_set(CGTNE, gpio_num);
		sch_gpio_reg_set_if_clear(CGTPE, gpio_num);
		break;
	case IRQ_TYPE_EDGE_FALLING:
		sch_gpio_reg_clear_if_set(CGTPE, gpio_num);
		sch_gpio_reg_set_if_clear(CGTNE, gpio_num);
		break;
	case IRQ_TYPE_EDGE_BOTH:
		sch_gpio_reg_set_if_clear(CGTPE, gpio_num);
		sch_gpio_reg_set_if_clear(CGTNE, gpio_num);
		break;
	case IRQ_TYPE_NONE:
		sch_gpio_reg_clear_if_set(CGTPE, gpio_num);
		sch_gpio_reg_clear_if_set(CGTNE, gpio_num);
		break;
	default:
		ret = -EINVAL;
	break;
	}

	if (0 == ret) {
		/* cache trigger setup */
		regs->cgtpe &= ~BIT(gpio_num);
		regs->cgtne &= ~BIT(gpio_num);
		regs->cgtpe |= inl(gpio_ba + CGTPE) & BIT(gpio_num);
		regs->cgtne |= inl(gpio_ba + CGTNE) & BIT(gpio_num);
	}

	spin_unlock_irqrestore(&gpio_lock, flags);

	return ret;
}

static struct irq_chip sch_irq_core = {
	.irq_ack		= sch_gpio_core_irq_ack,
	.irq_set_type		= sch_gpio_core_irq_type,
	.irq_enable		= sch_gpio_core_irq_enable,
	.irq_disable		= sch_gpio_core_irq_disable,
};

static void sch_gpio_core_irqs_init(struct sch_gpio *chip, unsigned int num)
{
	int i;

	for (i = 0; i < num; i++) {
		irq_set_chip_data(i + chip->irq_base_core, chip);
		irq_set_chip_and_handler_name(i + chip->irq_base_core,
						&sch_irq_core,
						handle_edge_irq,
						"sch_gpio_irq_core");
	}
}

static void sch_gpio_core_irqs_deinit(struct sch_gpio *chip, unsigned int num)
{
	int i;

	for (i = 0; i < num; i++) {
		irq_set_chip_data(i + chip->irq_base_core, 0);
		irq_set_chip_and_handler_name(i + chip->irq_base_core,
						0, 0, 0);
	}
}

static void sch_gpio_core_irq_disable_all(struct sch_gpio *chip,
						unsigned int num)
{
	unsigned long flags = 0;
	u32 gpio_num = 0;

	spin_lock_irqsave(&gpio_lock, flags);

	for (gpio_num = 0; gpio_num < num; gpio_num++) {
		sch_gpio_reg_clear_if_set(CGTPE, gpio_num);
		sch_gpio_reg_clear_if_set(CGTNE, gpio_num);
		sch_gpio_reg_clear_if_set(CGGPE, gpio_num);
		sch_gpio_reg_clear_if_set(CGSMI, gpio_num);
		sch_gpio_reg_clear_if_set(CGNMIEN, gpio_num);
		/* clear any pending interrupt */
		sch_gpio_reg_set(CGTS, gpio_num, 1);
	}

	spin_unlock_irqrestore(&gpio_lock, flags);

}

static int sch_gpio_resume_direction_in(struct gpio_chip *gc,
					unsigned gpio_num)
{
	unsigned long flags = 0;
	spin_lock_irqsave(&gpio_lock, flags);
	sch_gpio_reg_set_if_clear(RGIO, gpio_num);
	spin_unlock_irqrestore(&gpio_lock, flags);
	return 0;
}

static int sch_gpio_resume_get(struct gpio_chip *gc, unsigned gpio_num)
{
	int res;
	res = sch_gpio_reg_get(RGLV, gpio_num);
	return res;
}

static void sch_gpio_resume_set(struct gpio_chip *gc, unsigned gpio_num,
					int val)
{
	unsigned long flags = 0;
	spin_lock_irqsave(&gpio_lock, flags);
	sch_gpio_reg_set(RGLV, gpio_num, val);
	spin_unlock_irqrestore(&gpio_lock, flags);

}

static int sch_gpio_resume_direction_out(struct gpio_chip *gc,
					unsigned gpio_num, int val)
{
	unsigned long flags = 0;
	spin_lock_irqsave(&gpio_lock, flags);
	sch_gpio_reg_clear_if_set(RGIO, gpio_num);
	spin_unlock_irqrestore(&gpio_lock, flags);
	return 0;
}

static int sch_gpio_resume_to_irq(struct gpio_chip *gc, unsigned offset)
{
	return chip_ptr->irq_base_resume + offset;
}

static struct gpio_chip sch_gpio_resume = {
	.label			= "sch_gpio_resume",
	.owner			= THIS_MODULE,
	.direction_input	= sch_gpio_resume_direction_in,
	.get			= sch_gpio_resume_get,
	.direction_output	= sch_gpio_resume_direction_out,
	.set			= sch_gpio_resume_set,
	.to_irq			= sch_gpio_resume_to_irq,
};

static void sch_gpio_resume_irq_enable(struct irq_data *d)
{
	unsigned long flags = 0;
	u32 gpio_num = d->irq - chip_ptr->irq_base_resume;
	struct sch_gpio_context *regs = &chip_ptr->context;

	spin_lock_irqsave(&gpio_lock, flags);

	if (regs->rgtpe & BIT(gpio_num))
		sch_gpio_reg_set_if_clear(RGTPE, gpio_num);
	if (regs->rgtne & BIT(gpio_num))
		sch_gpio_reg_set_if_clear(RGTNE, gpio_num);
	sch_gpio_reg_set_if_clear(RGGPE, gpio_num);

	spin_unlock_irqrestore(&gpio_lock, flags);
}

static void sch_gpio_resume_irq_disable(struct irq_data *d)
{
	unsigned long flags = 0;
	u32 gpio_num = 0;

	gpio_num = d->irq - chip_ptr->irq_base_resume;

	if (!test_bit(gpio_num, chip_ptr->wake_irqs)){
		spin_lock_irqsave(&gpio_lock, flags);
		sch_gpio_reg_clear_if_set(RGGPE, gpio_num);
		sch_gpio_reg_clear_if_set(RGTPE, gpio_num);
		sch_gpio_reg_clear_if_set(RGTNE, gpio_num);
		outb(BIT(gpio_num), gpio_ba + RGTS);
		spin_unlock_irqrestore(&gpio_lock, flags);
	}
}

static void sch_gpio_resume_irq_ack(struct irq_data *d)
{
	u32 gpio_num = 0;

	gpio_num = d->irq - chip_ptr->irq_base_resume;
	outb(BIT(gpio_num), gpio_ba + RGTS);
}

static int sch_gpio_resume_irq_type(struct irq_data *d, unsigned type)
{
	int ret = 0;
	unsigned long flags = 0;
	u32 gpio_num = 0;
	struct sch_gpio_context *regs = &chip_ptr->context;

	if (NULL == d) {
		pr_err("%s(): null irq_data\n",  __func__);
		return -EFAULT;
	}

	gpio_num = d->irq - chip_ptr->irq_base_resume;

	spin_lock_irqsave(&gpio_lock, flags);

	switch (type) {
	case IRQ_TYPE_EDGE_RISING:
		sch_gpio_reg_clear_if_set(RGTNE, gpio_num);
		sch_gpio_reg_set_if_clear(RGTPE, gpio_num);
		break;
	case IRQ_TYPE_EDGE_FALLING:
		sch_gpio_reg_clear_if_set(RGTPE, gpio_num);
		sch_gpio_reg_set_if_clear(RGTNE, gpio_num);
		break;
	case IRQ_TYPE_EDGE_BOTH:
		sch_gpio_reg_set_if_clear(RGTPE, gpio_num);
		sch_gpio_reg_set_if_clear(RGTNE, gpio_num);
		break;
	case IRQ_TYPE_NONE:
		sch_gpio_reg_clear_if_set(RGTPE, gpio_num);
		sch_gpio_reg_clear_if_set(RGTNE, gpio_num);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	if (0 == ret) {
		/* cache trigger setup */
		regs->rgtpe &= ~BIT(gpio_num);
		regs->rgtne &= ~BIT(gpio_num);
		regs->rgtpe |= inl(gpio_ba + RGTPE) & BIT(gpio_num);
		regs->rgtne |= inl(gpio_ba + RGTNE) & BIT(gpio_num);
	}

	spin_unlock_irqrestore(&gpio_lock, flags);

	return ret;
}

/*
 * Enables/Disables power-management wake-on of an IRQ.
 * Inhibits disabling of the specified IRQ if on != 0.
 * Make sure you call it via irq_set_irq_wake() with on = 1 during suspend and
 * with on = 0 during resume.
 * Returns 0 if success, negative error code otherwhise
 */
int sch_gpio_resume_irq_set_wake(struct irq_data *d, unsigned int on)
{
	u32 gpio_num = 0;
	int ret = 0;

	if (NULL == d) {
		pr_err("%s(): Null irq_data\n", __func__);
		ret = -EFAULT;
		goto end;
	}
	gpio_num = d->irq - chip_ptr->irq_base_resume;
	if (gpio_num >= MAX_GPIO_IRQS) {
		pr_err("%s(): gpio_num bigger(%d) than MAX_GPIO_IRQS(%d)-1\n",
				__func__, gpio_num, MAX_GPIO_IRQS);
		ret = -EINVAL;
		goto end;
	}
	if (on)
		set_bit(gpio_num, chip_ptr->wake_irqs);
	else
		clear_bit(gpio_num, chip_ptr->wake_irqs);

end:
	return ret;
}

static struct irq_chip sch_irq_resume = {
	.irq_ack		= sch_gpio_resume_irq_ack,
	.irq_set_type		= sch_gpio_resume_irq_type,
	.irq_enable		= sch_gpio_resume_irq_enable,
	.irq_disable		= sch_gpio_resume_irq_disable,
	.irq_set_wake		= sch_gpio_resume_irq_set_wake,
};

static void sch_gpio_resume_irqs_init(struct sch_gpio *chip, unsigned int num)
{
	int i;

	for (i = 0; i < num; i++) {
		irq_set_chip_data(i + chip->irq_base_resume, chip);
		irq_set_chip_and_handler_name(i + chip->irq_base_resume,
						&sch_irq_resume,
						handle_edge_irq,
						"sch_gpio_irq_resume");
	}
}

static void sch_gpio_resume_irqs_deinit(struct sch_gpio *chip, unsigned int num)
{
	int i;

	for (i = 0; i < num; i++) {
		irq_set_chip_data(i + chip->irq_base_core, 0);
		irq_set_chip_and_handler_name(i + chip->irq_base_core,
						0, 0, 0);
	}
}

static void sch_gpio_resume_irq_disable_all(struct sch_gpio *chip,
						unsigned int num)
{
	unsigned long flags = 0;
	u32 gpio_num = 0;

	spin_lock_irqsave(&gpio_lock, flags);

	for (gpio_num = 0; gpio_num < num; gpio_num++) {
		sch_gpio_reg_clear_if_set(RGTPE, gpio_num);
		sch_gpio_reg_clear_if_set(RGTNE, gpio_num);
		sch_gpio_reg_clear_if_set(RGGPE, gpio_num);
		sch_gpio_reg_clear_if_set(RGSMI, gpio_num);
		sch_gpio_reg_clear_if_set(RGNMIEN, gpio_num);
		/* clear any pending interrupt */
		sch_gpio_reg_set(RGTS, gpio_num, 1);
	}

	spin_unlock_irqrestore(&gpio_lock, flags);
}

static inline irqreturn_t do_serve_irq(int reg_status, unsigned int irq_base)
{
	int ret = IRQ_NONE;
	u32 pending = 0, gpio = 0;

	/* Which pin (if any) triggered the interrupt */
	while ((pending = inb(reg_status))) {
		/* Serve each asserted interrupt */
		do {
			gpio = __ffs(pending);
			generic_handle_irq(irq_base + gpio);
			pending &= ~BIT(gpio);
			ret = IRQ_HANDLED;
		} while (pending);
	}

	return ret;
}

static irqreturn_t sch_gpio_irq_handler(int irq, void *dev_id)
{
	int ret = IRQ_NONE;

	ret |= do_serve_irq(gpio_ba + CGTS, chip_ptr->irq_base_core);
	ret |= do_serve_irq(gpio_ba + RGTS, chip_ptr->irq_base_resume);

	return ret;
}

static int sch_gpio_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct sch_gpio *chip;
	int err, id;

	chip = kzalloc(sizeof(*chip), GFP_KERNEL);
	if (chip == NULL)
		return -ENOMEM;

	chip_ptr = chip;

	id = pdev->id;
	if (!id)
		return -ENODEV;

	/* Get UIO memory */
	info = kzalloc(sizeof(struct uio_info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_IO, 0);
	if (!res)
		return -EBUSY;

	if (!request_region(res->start, resource_size(res), pdev->name))
		return -EBUSY;

	gpio_ba = res->start;

	irq_num = RESOURCE_IRQ;

	switch (id) {
	case PCI_DEVICE_ID_INTEL_SCH_LPC:
		sch_gpio_core.base = 0;
		sch_gpio_core.ngpio = 10;

		sch_gpio_resume.base = 10;
		sch_gpio_resume.ngpio = 4;

		/*
		 * GPIO[6:0] enabled by default
		 * GPIO7 is configured by the CMC as SLPIOVR
		 * Enable GPIO[9:8] core powered gpios explicitly
		 */
		outb(0x3, gpio_ba + CGEN + 1);
		/*
		 * SUS_GPIO[2:0] enabled by default
		 * Enable SUS_GPIO3 resume powered gpio explicitly
		 */
		outb(0x8, gpio_ba + RGEN);
		break;

	case PCI_DEVICE_ID_INTEL_ITC_LPC:
		sch_gpio_core.base = 0;
		sch_gpio_core.ngpio = 5;

		sch_gpio_resume.base = 5;
		sch_gpio_resume.ngpio = 9;
		break;

	case PCI_DEVICE_ID_INTEL_CENTERTON_ILB:
		sch_gpio_core.base = 0;
		sch_gpio_core.ngpio = 21;

		sch_gpio_resume.base = 21;
		sch_gpio_resume.ngpio = 9;
		break;

	case PCI_DEVICE_ID_INTEL_QUARK_ILB:
		sch_gpio_core.base = 0;
		sch_gpio_core.ngpio = 2;

		sch_gpio_resume.base = 2;
		sch_gpio_resume.ngpio = 6;
		break;

	default:
		err = -ENODEV;
		goto err_sch_gpio_core;
	}

	sch_gpio_core.dev = &pdev->dev;
	sch_gpio_resume.dev = &pdev->dev;

	err = gpiochip_add(&sch_gpio_core);
	if (err < 0)
		goto err_sch_gpio_core;

	err = gpiochip_add(&sch_gpio_resume);
	if (err < 0)
		goto err_sch_gpio_resume;

	chip->irq_base_core = irq_alloc_descs(-1, 0,
						sch_gpio_core.ngpio,
						NUMA_NO_NODE);
	if (chip->irq_base_core < 0) {
		dev_err(&pdev->dev, "failure adding GPIO core IRQ descs\n");
		chip->irq_base_core = -1;
		goto err_sch_intr_core;
	}

	chip->irq_base_resume = irq_alloc_descs(-1, 0,
						sch_gpio_resume.ngpio,
						NUMA_NO_NODE);
	if (chip->irq_base_resume < 0) {
		dev_err(&pdev->dev, "failure adding GPIO resume IRQ descs\n");
		chip->irq_base_resume = -1;
		goto err_sch_intr_resume;
	}

	platform_set_drvdata(pdev, chip);

	err = platform_device_register(&qrk_gpio_restrict_pdev);
	if (err < 0)
		goto err_sch_gpio_device_register;

	/* disable interrupts */
	sch_gpio_core_irq_disable_all(chip, sch_gpio_core.ngpio);
	sch_gpio_resume_irq_disable_all(chip, sch_gpio_resume.ngpio);


	err = request_irq(irq_num, sch_gpio_irq_handler,
				IRQF_SHARED, KBUILD_MODNAME, chip);
	if (err != 0) {
			dev_err(&pdev->dev,
				"%s request_irq failed\n", __func__);
			goto err_sch_request_irq;
	}

	sch_gpio_core_irqs_init(chip, sch_gpio_core.ngpio);
	sch_gpio_resume_irqs_init(chip, sch_gpio_resume.ngpio);

	/* UIO */
	info->port[0].name = "gpio_regs";
	info->port[0].start = res->start;
	info->port[0].size = resource_size(res);
	info->port[0].porttype = UIO_PORT_X86;
	info->name = "sch_gpio";
	info->version = "0.0.1";

	if (uio_register_device(&pdev->dev, info))
		goto err_sch_uio_register;

	pr_info("%s UIO port addr 0x%04x size %lu porttype %d\n",
		__func__, (unsigned int)info->port[0].start,
		info->port[0].size, info->port[0].porttype);

	return 0;

err_sch_uio_register:
	free_irq(irq_num, chip);

err_sch_request_irq:
	platform_device_unregister(&qrk_gpio_restrict_pdev);

err_sch_gpio_device_register:
	irq_free_descs(chip->irq_base_resume, sch_gpio_resume.ngpio);

err_sch_intr_resume:
	irq_free_descs(chip->irq_base_core, sch_gpio_core.ngpio);

err_sch_intr_core:
	err = gpiochip_remove(&sch_gpio_resume);
	if (err)
		dev_err(&pdev->dev, "%s failed, %d\n",
		"resume gpiochip_remove()", err);

err_sch_gpio_resume:
	err = gpiochip_remove(&sch_gpio_core);
	if (err)
		dev_err(&pdev->dev, "%s failed, %d\n",
		"core gpiochip_remove()", err);

err_sch_gpio_core:
	release_region(res->start, resource_size(res));
	gpio_ba = 0;

	kfree(chip);
	chip_ptr = 0;

	if (info != NULL)
		kfree(info);

	return err;
}

static int sch_gpio_remove(struct platform_device *pdev)
{
	int err = 0;
	struct resource *res;

	struct sch_gpio *chip = platform_get_drvdata(pdev);

	if (gpio_ba) {

		if (info != NULL) {
			uio_unregister_device(info);
			kfree(info);
		}

		sch_gpio_resume_irqs_deinit(chip, sch_gpio_resume.ngpio);
		sch_gpio_core_irqs_deinit(chip, sch_gpio_core.ngpio);

		if (irq_num > 0)
			free_irq(irq_num, chip);

		platform_device_unregister(&qrk_gpio_restrict_pdev);

		irq_free_descs(chip->irq_base_resume,
				sch_gpio_resume.ngpio);

		irq_free_descs(chip->irq_base_core, sch_gpio_core.ngpio);

		err = gpiochip_remove(&sch_gpio_resume);
		if (err)
			dev_err(&pdev->dev, "%s failed, %d\n",
				"resume gpiochip_remove()", err);

		err  = gpiochip_remove(&sch_gpio_core);
		if (err)
			dev_err(&pdev->dev, "%s failed, %d\n",
				"core gpiochip_remove()", err);

		res = platform_get_resource(pdev, IORESOURCE_IO, 0);

		release_region(res->start, resource_size(res));
		gpio_ba = 0;
	}

	kfree(chip);

	chip_ptr = 0;

	return err;
}

/*
 * Disables IRQ line of Legacy GPIO chip so that its state is not controlled by
 * PM framework (disabled before calling suspend_noirq callback and re-enabled
 * after calling resume_noirq callback of devices).
 */
static int sch_gpio_suspend_sys(struct device *dev)
{
	disable_irq(irq_num);
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
	struct sch_gpio_context *regs = &chip_ptr->context;

	regs->cgen	= inl(gpio_ba + CGEN);
	regs->cgio	= inl(gpio_ba + CGIO);
	regs->cglvl	= inl(gpio_ba + CGLV);
	regs->cgsmi	= inl(gpio_ba + CGSMI);
	regs->cgnmien	= inl(gpio_ba + CGNMIEN);

	return 0;
}

/*
 * Restore the context saved by sch_gpio_suspend_sys_noirq().
 */
static int sch_gpio_resume_sys_noirq(struct device *dev)
{
	struct sch_gpio_context *regs = &chip_ptr->context;

	outl(regs->cgio,	gpio_ba + CGIO);
	outl(regs->cglvl,	gpio_ba + CGLV);
	outl(regs->cgsmi,	gpio_ba + CGSMI);
	outl(regs->cgnmien,	gpio_ba + CGNMIEN);
	outl(regs->cgen,	gpio_ba + CGEN);

	return 0;
}

/*
 * Re-enables the IRQ line of Legacy GPIO chip.
 * Done here instead of dpm_resume_no_irq() PM handler in order to be sure that
 * all the system busses (I2C, SPI) are resumed when the IRQ is fired, otherwise
 * a SPI or I2C device might fail to handle its own interrupt because the IRQ
 * handler (bottom half) involves talking to the device.
 */
static int sch_gpio_resume_sys(struct device *dev)
{
	enable_irq(irq_num);
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
		.owner	= THIS_MODULE,
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
