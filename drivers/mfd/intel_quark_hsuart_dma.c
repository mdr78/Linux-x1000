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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/irq.h>
#include <linux/pci.h>
#include <linux/mfd/core.h>
#include <linux/serial_8250.h>
#include <linux/dw_dmac.h>
#include <uapi/linux/serial_reg.h>
#include <uapi/linux/serial_core.h>
#include "../tty/serial/8250/8250.h"

static bool uart0_dma = true;
module_param(uart0_dma, bool, 0);
MODULE_PARM_DESC(uart0_dma, "Set UART0 to use DMA");

static bool uart1_dma = true;
module_param(uart1_dma, bool, 0);
MODULE_PARM_DESC(uart1_dma, "Set UART1 to use DMA");

/* MFD */
#define	QUARK_IORESOURCE_MEM			0
#define	QUARK_IORESOURCE_IRQ			1
#define	QUARK_MFD_DMA				0
#define	QUARK_MFD_UART				1

/* HSUART DMA Paired ID */
#define	INTEL_QUARK_UART0_PAIR_ID		0
#define	INTEL_QUARK_UART1_PAIR_ID		1

/* Serial */
#define	TX					0
#define	RX					1
#define	INTEL_QUARK_UART_MEM_BAR		0

/* DMA for Serial */
#define	INTEL_QUARK_UART0_RX_CH_ID		0
#define	INTEL_QUARK_UART0_TX_CH_ID		1
#define	INTEL_QUARK_UART1_RX_CH_ID		0
#define	INTEL_QUARK_UART1_TX_CH_ID		1

/* DMA */
#define	INTEL_QUARK_DMA_MEM_BAR			1

/* MISC */
	/* Intel Quark X1000 HSUART DMA Engine Internal Base Address */
#define	INTEL_X1000_DMA_MAPBASE			0xFFFFF000

/* PCI ID List */
#define	PCI_DEVICE_ID_INTEL_QUARK_X1000_UART	0x0936

static const struct pci_device_id intel_quark_hsuart_dma_ids[] = {
	{ PCI_VDEVICE(INTEL, PCI_DEVICE_ID_INTEL_QUARK_X1000_UART) },
	{ /* Sentinel */ },
};
/*
 * Multi-Functional Device Section
 */
static struct resource mfd_dma_res[] = {
	[QUARK_IORESOURCE_MEM] = {
		.start = 0x0,
		.end = 0x0,
		.flags = IORESOURCE_MEM,
	},
	[QUARK_IORESOURCE_IRQ] = {
		.start = 0x0,
		.end = 0x0,
		.flags = IORESOURCE_IRQ,
	},
};

static struct resource mfd_uart_res[] = {
	[QUARK_IORESOURCE_MEM] = {
		.start = 0x0,
		.end = 0x0,
		.flags = IORESOURCE_MEM,
	},
	[QUARK_IORESOURCE_IRQ] = {
		.start = 0x0,
		.end = 0x0,
		.flags = IORESOURCE_IRQ,
	},
};

static struct mfd_cell intel_quark_hsuart_dma_cells[] = {
	[QUARK_MFD_DMA] = {
		.name = "",
		.num_resources = ARRAY_SIZE(mfd_dma_res),
		.resources = mfd_dma_res,
		.ignore_resource_conflicts = true,
	},
	[QUARK_MFD_UART] = {
		.name = "",
		.num_resources = ARRAY_SIZE(mfd_uart_res),
		.resources = mfd_uart_res,
		.ignore_resource_conflicts = true,
	},
};

/*
 * Serial Sections
 */
enum serial8250_pdata_index {
	INTEL_QUARK_UART0 = 0,
	INTEL_QUARK_UART1,
};

static bool intel_quark_uart_dma_filter(struct dma_chan *chan, void *param);

struct serial8250_filter_param {
	int chan_id;
	int device_id;
	int slave_config_id;
	int slave_config_direction;
};

static struct serial8250_filter_param intel_quark_filter_param[][2] = {
	[INTEL_QUARK_UART0] = {
		[RX] = {
			.chan_id = INTEL_QUARK_UART0_RX_CH_ID,
			.device_id = INTEL_QUARK_UART0_PAIR_ID,
			.slave_config_id = INTEL_QUARK_UART0,
			.slave_config_direction = RX,
		},
		[TX] = {
			.chan_id = INTEL_QUARK_UART0_TX_CH_ID,
			.device_id = INTEL_QUARK_UART0_PAIR_ID,
			.slave_config_id = INTEL_QUARK_UART0,
			.slave_config_direction = TX,
		},
	},
	[INTEL_QUARK_UART1] = {
		[RX] = {
			.chan_id = INTEL_QUARK_UART1_RX_CH_ID,
			.device_id = INTEL_QUARK_UART1_PAIR_ID,
			.slave_config_id = INTEL_QUARK_UART1,
			.slave_config_direction = RX,
		},
		[TX] = {
			.chan_id = INTEL_QUARK_UART1_TX_CH_ID,
			.device_id = INTEL_QUARK_UART1_PAIR_ID,
			.slave_config_id = INTEL_QUARK_UART1,
			.slave_config_direction = TX,
		},
	},
};

static struct dw_dma_slave dma_dws[][2] = {
	[INTEL_QUARK_UART0] = {
		[RX] = {
			.cfg_hi = DWC_CFGH_SRC_PER(INTEL_QUARK_UART0_RX_CH_ID),
			.cfg_lo = (DWC_CFGL_HS_DST_POL | DWC_CFGL_HS_SRC_POL),
		},
		[TX] = {
			.cfg_hi = DWC_CFGH_DST_PER(INTEL_QUARK_UART0_TX_CH_ID),
			.cfg_lo = (DWC_CFGL_HS_DST_POL | DWC_CFGL_HS_SRC_POL),
		},
	},
	[INTEL_QUARK_UART1] = {
		[RX] = {
			.cfg_hi = DWC_CFGH_SRC_PER(INTEL_QUARK_UART1_RX_CH_ID),
			.cfg_lo = (DWC_CFGL_HS_DST_POL | DWC_CFGL_HS_SRC_POL),
		},
		[TX] = {
			.cfg_hi = DWC_CFGH_DST_PER(INTEL_QUARK_UART1_TX_CH_ID),
			.cfg_lo = (DWC_CFGL_HS_DST_POL | DWC_CFGL_HS_SRC_POL),
		},
	},
};

static struct uart_8250_dma serial8250_dma[] = {
	[INTEL_QUARK_UART0] = {
		.fn = &intel_quark_uart_dma_filter,
		.tx_chan_id = INTEL_QUARK_UART0_TX_CH_ID,
		.rx_chan_id = INTEL_QUARK_UART0_RX_CH_ID,
		.txconf.slave_id = INTEL_QUARK_UART0_TX_CH_ID,
		.rxconf.slave_id = INTEL_QUARK_UART0_RX_CH_ID,
		.txconf.dst_maxburst = 8,
		.rxconf.src_maxburst = 8,
		.rxconf.device_fc = false,
		.tx_param = &intel_quark_filter_param[INTEL_QUARK_UART0][TX],
		.rx_param = &intel_quark_filter_param[INTEL_QUARK_UART0][RX],
	},
	[INTEL_QUARK_UART1] = {
		.fn = &intel_quark_uart_dma_filter,
		.tx_chan_id = INTEL_QUARK_UART1_TX_CH_ID,
		.rx_chan_id = INTEL_QUARK_UART1_RX_CH_ID,
		.txconf.slave_id = INTEL_QUARK_UART1_TX_CH_ID,
		.rxconf.slave_id = INTEL_QUARK_UART1_RX_CH_ID,
		.txconf.dst_maxburst = 8,
		.rxconf.src_maxburst = 8,
		.rxconf.device_fc = false,
		.tx_param = &intel_quark_filter_param[INTEL_QUARK_UART1][TX],
		.rx_param = &intel_quark_filter_param[INTEL_QUARK_UART1][RX],
	},
};

static struct uart_8250_port serial8250_port[] = {
	[INTEL_QUARK_UART0] = {
		.port.fifosize = 16,
		.tx_loadsz = 8,
		.dma = &serial8250_dma[INTEL_QUARK_UART0],
	},
	[INTEL_QUARK_UART1] = {
		.port.fifosize = 16,
		.tx_loadsz = 8,
		.dma = &serial8250_dma[INTEL_QUARK_UART1],
	},
};

static struct plat_serial8250_port serial8250_pdata[] = {
	[INTEL_QUARK_UART0] = {
		.dma_mapbase = INTEL_X1000_DMA_MAPBASE,
		.uartclk = 44236800,
		.iotype = UPIO_MEM32,
		.regshift = 2,
		.private_data = &serial8250_port[INTEL_QUARK_UART0],
	},
	[INTEL_QUARK_UART1] = {
		.dma_mapbase = INTEL_X1000_DMA_MAPBASE,
		.uartclk = 44236800,
		.iotype = UPIO_MEM32,
		.regshift = 2,
		.private_data = &serial8250_port[INTEL_QUARK_UART1],
	},
};

static bool intel_quark_uart_dma_filter(struct dma_chan *chan, void *param)
{
	struct dw_dma_slave *dws;
	struct serial8250_filter_param *data;

	if (!param)
		return false;
	data = param;

	if (chan->device->dev_id != data->device_id)
		return false;
	if (chan->chan_id != data->chan_id)
		return false;

	dws = &dma_dws[data->slave_config_id][data->slave_config_direction];
	dws->dma_dev = chan->device->dev;
	chan->private = dws;

	switch (data->device_id) {
	case INTEL_QUARK_UART0_PAIR_ID:
		return uart0_dma;
	case INTEL_QUARK_UART1_PAIR_ID:
		return uart1_dma;
	}

	return true;
}

static int intel_quark_uart_device_probe(struct pci_dev *pdev,
					const struct pci_device_id *id)
{
	struct resource *res;
	void *pdata;
	struct mfd_cell *cell = &intel_quark_hsuart_dma_cells[QUARK_MFD_UART];
	int bar = 0, ret = 0;

	switch (id->device) {
	case PCI_DEVICE_ID_INTEL_QUARK_X1000_UART:
		cell->name = "dw-apb-uart";
		bar = INTEL_QUARK_UART_MEM_BAR;
		if (1 == PCI_FUNC(pdev->devfn)) {
			cell->id = INTEL_QUARK_UART0_PAIR_ID;
			pdata = &serial8250_pdata[INTEL_QUARK_UART0];
		} else if (5 == PCI_FUNC(pdev->devfn)) {
			cell->id = INTEL_QUARK_UART1_PAIR_ID;
			pdata = &serial8250_pdata[INTEL_QUARK_UART1];
		} else {
			goto uart_error;
		}
		cell->platform_data = pdata;
		cell->pdata_size = sizeof(struct plat_serial8250_port);
		break;
	default:
		goto uart_error;
	}

	res = &mfd_uart_res[QUARK_IORESOURCE_MEM];
	res->start = pci_resource_start(pdev, bar);
	res->end = pci_resource_end(pdev, bar);

	res = &mfd_uart_res[QUARK_IORESOURCE_IRQ];
	res->start = pdev->irq;
	res->end = pdev->irq;

	pci_set_master(pdev);

	ret = mfd_add_devices(&pdev->dev, 0, cell, 1, NULL, 0, NULL);
	if (ret) {
		pci_clear_master(pdev);
	}

	return ret;

uart_error:
	dev_warn(&pdev->dev, "no valid UART device.\n");
	ret = -ENODEV;
	return ret;
}

/*
 * DMA Sections
 */
enum dw_dma_pdata_index {
	INTEL_QUARK_DMA1 = 0,
	INTEL_QUARK_DMA2,
};

static struct dw_dma_platform_data dw_dma_pdata[] = {
	[INTEL_QUARK_DMA1] = {
		.nr_channels = 2,
		.is_private = true,
		.chan_allocation_order = CHAN_ALLOCATION_ASCENDING,
		.chan_priority = CHAN_PRIORITY_ASCENDING,
		.block_size = 4095U,
		.nr_masters = 1,
		.data_width = {2, 0, 0, 0},
		.nollp = {true, true,},
	},
	[INTEL_QUARK_DMA2] = {
		.nr_channels = 2,
		.is_private = true,
		.chan_allocation_order = CHAN_ALLOCATION_ASCENDING,
		.chan_priority = CHAN_PRIORITY_ASCENDING,
		.block_size = 4095U,
		.nr_masters = 1,
		.data_width = {2, 0, 0, 0},
		.nollp = {true, true, },
	},
};

static int intel_quark_dma_device_probe(struct pci_dev *pdev,
					const struct pci_device_id *id)
{
	struct resource *res;
	void *pdata;
	struct mfd_cell *cell = &intel_quark_hsuart_dma_cells[QUARK_MFD_DMA];
	int bar = 0, ret = 0;

	switch (id->device) {
	case PCI_DEVICE_ID_INTEL_QUARK_X1000_UART:
		cell->name = "dw_dmac";
		bar = INTEL_QUARK_DMA_MEM_BAR;
		if (1 == PCI_FUNC(pdev->devfn)) {
			cell->id = INTEL_QUARK_UART0_PAIR_ID;
			pdata = &dw_dma_pdata[INTEL_QUARK_DMA1];
		} else if (5 == PCI_FUNC(pdev->devfn)) {
			cell->id = INTEL_QUARK_UART1_PAIR_ID;
			pdata = &dw_dma_pdata[INTEL_QUARK_DMA2];
		} else {
			goto dma_error;
		}
		cell->platform_data = pdata;
		cell->pdata_size = sizeof(struct dw_dma_platform_data);
		break;
	default:
		goto dma_error;
	}

	res = &mfd_dma_res[QUARK_IORESOURCE_MEM];
	res->start = pci_resource_start(pdev, bar);
	res->end = pci_resource_end(pdev, bar);

	res = &mfd_dma_res[QUARK_IORESOURCE_IRQ];
	res->start = pdev->irq;
	res->end = pdev->irq;

	ret = mfd_add_devices(&pdev->dev, 0, cell, 1, NULL, 0, NULL);
	goto dma_done;

dma_error:
	dev_warn(&pdev->dev, "no valid DMA device.\n");
	ret = -ENODEV;
dma_done:
	return ret;
}

/*
 * Main Section
 */
static int intel_quark_hsuart_dma_probe(struct pci_dev *pdev,
					const struct pci_device_id *id)
{
	int ret = 0;

	dev_info(&pdev->dev, "found PCI serial controller(ID: %04x:%04x)\n",
		 pdev->vendor, pdev->device);

	ret = pci_enable_device(pdev);
	if (ret) {
		dev_warn(&pdev->dev, "Failed to enable PCI Device\n");
		goto probe_error;
	}
	/* Execute DMA device probe first. I/O can live with/without DMA */
	ret = intel_quark_dma_device_probe(pdev, id);
	if (ret)
		dev_warn(&pdev->dev, "Failed to initialize DMA device\n");

	ret = intel_quark_uart_device_probe(pdev, id);
	if (ret) {
		dev_warn(&pdev->dev, "Failed to initialize HSUART device\n");
		goto probe_disable_device;
	}

	return ret;

probe_disable_device:
	mfd_remove_devices(&pdev->dev);
	pci_disable_device(pdev);
probe_error:
	return ret;
}

static void intel_quark_hsuart_dma_remove(struct pci_dev *pdev)
{
	mfd_remove_devices(&pdev->dev);
	pci_disable_device(pdev);
}

#ifdef CONFIG_PM
/*
 * Quark SoC family does not support ACPI D3_hot, ie the HSUART is
 * powered off during ACPI S3 state.
 * UART and DMA context is restored by respectively the tty and dmaengine
 * subsystems.
 *
 * At the PCI level, we need to ensure that bus mastering is enabled, before
 * IRQs are switched on.
 */
static int intel_quark_hsuart_dma_suspend_noirq(struct device *dev)
{
	pci_clear_master(to_pci_dev(dev));
	return 0;
}
static int intel_quark_hsuart_dma_resume_noirq(struct device *dev)
{
	pci_set_master(to_pci_dev(dev));
	return 0;
}
#endif /* CONFIG_PM */

static const struct dev_pm_ops intel_quark_hsuart_dma_pm_ops = {
#ifdef CONFIG_PM
	.suspend_noirq = intel_quark_hsuart_dma_suspend_noirq,
	.resume_noirq = intel_quark_hsuart_dma_resume_noirq,
#endif
};

static struct pci_driver intel_quark_hsuart_dma_driver = {
	.name		= "intel_quark_hsuart_dma",
	.id_table	= intel_quark_hsuart_dma_ids,
	.probe		= intel_quark_hsuart_dma_probe,
	.remove		= intel_quark_hsuart_dma_remove,
	.driver = {
		.pm	= &intel_quark_hsuart_dma_pm_ops,
	},
};

module_pci_driver(intel_quark_hsuart_dma_driver);

MODULE_AUTHOR("Chew, Kean Ho <kean.ho.chew@intel.com>");
MODULE_DESCRIPTION("HSUART DMA MFD driver for Intel Quark Series");
MODULE_LICENSE("GPL");
