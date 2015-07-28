/*
 *  intel_mid_dma.c - Intel Langwell DMA Drivers
 *
 *  Copyright (C) 2008-12 Intel Corp
 *  Author: Vinod Koul <vinod.koul@intel.com>
 *  The driver design is based on dw_dmac driver
 *  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 *
 */
#define DEBUG
#include <linux/pci.h>
#include <linux/interrupt.h>
#include <linux/pm_runtime.h>
#include <linux/intel_mid_dma.h>
#include <linux/module.h>

#include "intel_mid_dma_regs.h"

#define INTEL_MID_DMAC1_ID		0x0814
#define INTEL_MID_DMAC2_ID		0x0813
#define INTEL_MID_GP_DMAC2_ID		0x0827
#define INTEL_MFLD_DMAC1_ID		0x0830

#define INFO(_max_chan, _ch_base, _block_size, _pimr_mask, _is_quark) \
	((kernel_ulong_t)&(struct intel_mid_dma_probe_info) {	\
		.max_chan = (_max_chan),			\
		.ch_base = (_ch_base),				\
		.block_size = (_block_size),			\
		.pimr_mask = (_pimr_mask),			\
		.is_quark = (_is_quark),				\
	})

/**
 * intel_mid_dma_probe -	PCI Probe
 * @pdev: Controller PCI device structure
 * @id: pci device id structure
 *
 * Initialize the PCI device, map BARs, query driver data.
 * Call setup_dma to complete contoller and chan initilzation
 */
static int intel_mid_dma_probe(struct pci_dev *pdev,
					const struct pci_device_id *id)
{
	struct middma_device *device;
	u32 base_addr, bar_size;
	struct intel_mid_dma_probe_info *info;
	int err;

	pr_debug("MDMA: probe for %x\n", pdev->device);
	info = (void *)id->driver_data;
	pr_debug("MDMA: CH %d, base %d, block len %d, Periphral mask %x\n",
				info->max_chan, info->ch_base,
				info->block_size, info->pimr_mask);

	err = pci_enable_device(pdev);
	if (err)
		goto err_enable_device;

	err = pci_request_regions(pdev, "intel_mid_dmac");
	if (err)
		goto err_request_regions;

	err = pci_set_dma_mask(pdev, DMA_BIT_MASK(32));
	if (err)
		goto err_set_dma_mask;

	err = pci_set_consistent_dma_mask(pdev, DMA_BIT_MASK(32));
	if (err)
		goto err_set_dma_mask;

	device = kzalloc(sizeof(*device), GFP_KERNEL);
	if (!device) {
		pr_err("ERR_MDMA:kzalloc failed probe\n");
		err = -ENOMEM;
		goto err_kzalloc;
	}
	device->pdev = pci_dev_get(pdev);

	base_addr = pci_resource_start(pdev, 0);
	bar_size  = pci_resource_len(pdev, 0);
	device->dma_base = ioremap_nocache(base_addr, DMA_REG_SIZE);
	if (!device->dma_base) {
		pr_err("ERR_MDMA:ioremap failed\n");
		err = -ENOMEM;
		goto err_ioremap;
	}
	pci_set_drvdata(pdev, device);
	pci_set_master(pdev);
	device->max_chan = info->max_chan;
	device->chan_base = info->ch_base;
	device->block_size = info->block_size;
	device->pimr_mask = info->pimr_mask;
	device->is_quark = info->is_quark;

	err = mid_setup_dma(pdev, device);
	if (err)
		goto err_dma;

	pm_runtime_put_noidle(&pdev->dev);
	pm_runtime_allow(&pdev->dev);
	return 0;

err_dma:
	iounmap(device->dma_base);
err_ioremap:
	pci_dev_put(pdev);
	kfree(device);
err_kzalloc:
err_set_dma_mask:
	pci_release_regions(pdev);
	pci_disable_device(pdev);
err_request_regions:
err_enable_device:
	pr_err("ERR_MDMA:Probe failed %d\n", err);
	return err;
}

/**
 * intel_mid_dma_remove -	PCI remove
 * @pdev: Controller PCI device structure
 *
 * Free up all resources and data
 * Call shutdown_dma to complete contoller and chan cleanup
 */
static void intel_mid_dma_remove(struct pci_dev *pdev)
{
	struct middma_device *device = pci_get_drvdata(pdev);

	pm_runtime_get_noresume(&pdev->dev);
	pm_runtime_forbid(&pdev->dev);
#if 0
	middma_shutdown(pdev, device);
#endif
	pci_dev_put(pdev);
	kfree(device);
	pci_release_regions(pdev);
	pci_disable_device(pdev);
}

/* Power Management */
/*
* dma_suspend - PCI suspend function
*
* @pci: PCI device structure
* @state: PM message
*
* This function is called by OS when a power event occurs
*/
static int dma_suspend(struct device *dev)
{
	struct pci_dev *pci = to_pci_dev(dev);
	int i;
	struct middma_device *device = pci_get_drvdata(pci);
	pr_debug("MDMA: dma_suspend called\n");

	for (i = 0; i < device->max_chan; i++) {
		if (device->ch[i].in_use)
			return -EAGAIN;
	}
#if 0
	dmac1_mask_periphral_intr(device);
#endif
	device->state = SUSPENDED;
	pci_save_state(pci);
	pci_disable_device(pci);
	pci_set_power_state(pci, PCI_D3hot);
	return 0;
}

/**
* dma_resume - PCI resume function
*
* @pci:	PCI device structure
*
* This function is called by OS when a power event occurs
*/
int dma_resume(struct device *dev)
{
	struct pci_dev *pci = to_pci_dev(dev);
	int ret;

	pr_debug("MDMA: dma_resume called\n");
	pci_set_power_state(pci, PCI_D0);
	pci_restore_state(pci);
	ret = pci_enable_device(pci);
	if (ret) {
		pr_err("MDMA: device can't be enabled for %x\n", pci->device);
		return ret;
	}

	return middma_resume(dev);
}

static int dma_runtime_suspend(struct device *dev)
{
	struct pci_dev *pci_dev = to_pci_dev(dev);
	struct middma_device *device = pci_get_drvdata(pci_dev);

	device->state = SUSPENDED;
	return 0;
}

static int dma_runtime_resume(struct device *dev)
{
	struct pci_dev *pci_dev = to_pci_dev(dev);
	struct middma_device *device = pci_get_drvdata(pci_dev);

	device->state = RUNNING;
	iowrite32(REG_BIT0, device->dma_base + DMA_CFG);
	return 0;
}

static int dma_runtime_idle(struct device *dev)
{
	struct pci_dev *pdev = to_pci_dev(dev);
	struct middma_device *device = pci_get_drvdata(pdev);
	int i;

	for (i = 0; i < device->max_chan; i++) {
		if (device->ch[i].in_use)
			return -EAGAIN;
	}

	return pm_schedule_suspend(dev, 0);
}

/******************************************************************************
* PCI stuff
*/
static struct pci_device_id intel_mid_dma_ids[] = {
	{ PCI_VDEVICE(INTEL, INTEL_MID_DMAC1_ID),	INFO(2, 6, 4095, 0x200020, 0)},
	{ PCI_VDEVICE(INTEL, INTEL_MID_DMAC2_ID),	INFO(2, 0, 2047, 0, 0)},
	{ PCI_VDEVICE(INTEL, INTEL_MID_GP_DMAC2_ID),	INFO(2, 0, 2047, 0, 0)},
	{ PCI_VDEVICE(INTEL, INTEL_MFLD_DMAC1_ID),	INFO(4, 0, 4095, 0x400040, 0)},
	{ 0, }
};
MODULE_DEVICE_TABLE(pci, intel_mid_dma_ids);

static const struct dev_pm_ops intel_mid_dma_pm = {
	.runtime_suspend = dma_runtime_suspend,
	.runtime_resume = dma_runtime_resume,
	.runtime_idle = dma_runtime_idle,
	.suspend = dma_suspend,
	.resume = dma_resume,
};

static struct pci_driver intel_mid_dma_pci_driver = {
	.name		=	"Intel MID DMA",
	.id_table	=	intel_mid_dma_ids,
	.probe		=	intel_mid_dma_probe,
	.remove		=	intel_mid_dma_remove,
#ifdef CONFIG_PM
	.driver = {
		.pm = &intel_mid_dma_pm,
	},
#endif
};

static int __init intel_mid_dma_init(void)
{
	pr_debug("INFO_MDMA: LNW DMA Driver Version %s\n",
			INTEL_MID_DMA_DRIVER_VERSION);
	return pci_register_driver(&intel_mid_dma_pci_driver);
}
fs_initcall(intel_mid_dma_init);

static void __exit intel_mid_dma_exit(void)
{
	pci_unregister_driver(&intel_mid_dma_pci_driver);
}
module_exit(intel_mid_dma_exit);

MODULE_AUTHOR("Vinod Koul <vinod.koul@intel.com>");
MODULE_DESCRIPTION("Intel (R) MID DMAC Driver");
MODULE_LICENSE("GPL v2");
MODULE_VERSION(INTEL_MID_DMA_DRIVER_VERSION);
