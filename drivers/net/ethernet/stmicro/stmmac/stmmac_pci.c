/*******************************************************************************
  This contains the functions to handle the pci driver.

  Copyright (C) 2011-2012  Vayavya Labs Pvt Ltd
  Copyright (C) 2013-2014  Intel Corporation

  This program is free software; you can redistribute it and/or modify it
  under the terms and conditions of the GNU General Public License,
  version 2, as published by the Free Software Foundation.

  This program is distributed in the hope it will be useful, but WITHOUT
  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
  more details.

  You should have received a copy of the GNU General Public License along with
  this program; if not, write to the Free Software Foundation, Inc.,
  51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.

  The full GNU General Public License is included in this distribution in
  the file called "COPYING".

  Author: Rayagond Kokatanur <rayagond@vayavyalabs.com>
  Author: Giuseppe Cavallaro <peppe.cavallaro@st.com>
*******************************************************************************/

#include <linux/dmi.h>
#include <linux/pci.h>
#include <linux/platform_data/quark.h>
#include "stmmac.h"

/* List of supported PCI device IDs */
#define STMMAC_VENDOR_ID 0x700
#define STMMAC_DEVICE_ID 0x1108
#define STMMAC_QUARK_ID 0x0937
#define MAX_INTERFACES	 0x02

#if defined(CONFIG_INTEL_QUARK_X1000_SOC)
static int enable_msi = 1;
#else
static int enable_msi; /* by default initialized to 0 */
#endif
module_param(enable_msi, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(enable_msi, "Enable PCI MSI mode");


struct stmmac_qrk_mac_data {
	int phy_addr;
	int bus_id;
	const char *name;
};

static struct stmmac_qrk_mac_data phy_data[] = {
	{
		.phy_addr	= -1,			/* not connected */
		.bus_id		= 1,
		.name		= "QuarkEmulation",
	},
	{
		.phy_addr	= 1,
		.bus_id		= 2,
		.name		= "QuarkEmulation",
	},
	{
		.phy_addr	= 3,
		.bus_id		= 1,
		.name		= "ClantonPeakSVP",
	},
	{
		.phy_addr	= 1,
		.bus_id		= 2,
		.name		= "ClantonPeakSVP",
	},
	{
		.phy_addr	= 1,
		.bus_id		= 1,
		.name		= "KipsBay",
	},
	{
		.phy_addr	= -1,			/* not connected */
		.bus_id		= 2,
		.name 		= "KipsBay",
	},
	{
		.phy_addr	= 1,
		.bus_id		= 1,
		.name		= "CrossHill",
	},
	{
		.phy_addr	= 1,
		.bus_id		= 2,
		.name		= "CrossHill",
	},
	{
		.phy_addr	= 1,
		.bus_id		= 1,
		.name		= "ClantonHill",
	},
	{
		.phy_addr	= 1,
		.bus_id		= 2,
		.name		= "ClantonHill",
	},
	{
		.phy_addr	= 1,
		.bus_id		= 1,
		.name		= "Galileo",
	},
	{
		.phy_addr	= -1,			/* not connected */
		.bus_id		= 2,
		.name		= "Galileo",
	},
	{
		.phy_addr	= 1,
		.bus_id		= 1,
		.name		= "GalileoGen2",
	},
	{
		.phy_addr	= -1,			/* not connected */
		.bus_id		= 2,
		.name		= "GalileoGen2",
	},
};


static int stmmac_find_phy_addr(int mdio_bus_id)
{
	int i = 0;
	const char *board_name = dmi_get_system_info(DMI_BOARD_NAME);
	if (board_name == NULL)
		return -1;

	for (; i < sizeof(phy_data)/sizeof(struct stmmac_qrk_mac_data); i++) {
		if ((!strcmp(phy_data[i].name, board_name)) &&
			phy_data[i].bus_id == mdio_bus_id)
			return phy_data[i].phy_addr;
	}

	return -1;
}

static int stmmac_default_data(struct plat_stmmacenet_data *plat_dat,
			       int mdio_bus_id, const struct pci_device_id *id)
{
	int phy_addr = 0;
	memset(plat_dat, 0, sizeof(struct plat_stmmacenet_data));

	plat_dat->mdio_bus_data = kzalloc(sizeof(struct stmmac_mdio_bus_data),
					GFP_KERNEL);
	if (plat_dat->mdio_bus_data == NULL)
		return -ENOMEM;

	plat_dat->dma_cfg = kzalloc(sizeof(struct stmmac_dma_cfg), GFP_KERNEL);
	if (plat_dat->dma_cfg == NULL)
		return -ENOMEM;

	if (id->device ==  STMMAC_QUARK_ID) {
		phy_addr = stmmac_find_phy_addr(mdio_bus_id);
		if (phy_addr == -1)
			return -ENODEV;

		plat_dat->interface = PHY_INTERFACE_MODE_RMII;
		/* clk_csr_i = 20-35MHz & MDC = clk_csr_i/16 */

		plat_dat->dma_cfg->pbl = 16;
		plat_dat->dma_cfg->fixed_burst = 1;
	} else {
		plat_dat->interface = PHY_INTERFACE_MODE_GMII;
		/* clk_csr_i = 20-35MHz & MDC = clk_csr_i/16 */

		plat_dat->dma_cfg->pbl = 32;
	}

	plat_dat->bus_id = mdio_bus_id;
	plat_dat->phy_addr = phy_addr;
	plat_dat->clk_csr = 2;
	plat_dat->has_gmac = 1;
	plat_dat->force_sf_dma_mode = 1;
	plat_dat->mdio_bus_data->phy_reset = NULL;
	plat_dat->mdio_bus_data->phy_mask = 0;
	plat_dat->dma_cfg->burst_len = DMA_AXI_BLEN_256;

	return 0;
}

/**
 * stmmac_pci_probe
 *
 * @pdev: pci device pointer
 * @id: pointer to table of device id/id's.
 *
 * Description: This probing function gets called for all PCI devices which
 * match the ID table and are not "owned" by other driver yet. This function
 * gets passed a "struct pci_dev *" for each device whose entry in the ID table
 * matches the device. The probe functions returns zero when the driver choose
 * to take "ownership" of the device or an error code(-ve no) otherwise.
 */
static int stmmac_pci_probe(struct pci_dev *pdev,
			    const struct pci_device_id *id)
{
	int ret = 0;
	void __iomem *addr = NULL;
	struct stmmac_priv *priv = NULL;
	struct plat_stmmacenet_data *plat_dat = NULL;
	int i;
	static int bus_id = 1;

	plat_dat = kmalloc(sizeof(struct plat_stmmacenet_data), GFP_KERNEL);
	if (plat_dat == NULL) {
		ret = -ENOMEM;
		goto err_out_map_failed;
	}

	/* return -ENODEV for non existing PHY, stop probing here  */
	ret = stmmac_default_data(plat_dat, bus_id, id);
	if (ret != 0)
		goto err_platdata;


	/* Enable pci device */
	ret = pci_enable_device(pdev);
	if (ret) {
		pr_err("%s : ERROR: failed to enable %s device\n", __func__,
		       pci_name(pdev));
		goto err_platdata;
	}
	if (pci_request_regions(pdev, STMMAC_RESOURCE_NAME)) {
		pr_err("%s: ERROR: failed to get PCI region\n", __func__);
		ret = -ENODEV;
		goto err_out_req_reg_failed;
	}

	/* Get the base address of device */
	for (i = 0; i <= 5; i++) {
		if (pci_resource_len(pdev, i) == 0)
			continue;
		addr = pci_iomap(pdev, i, 0);
		if (addr == NULL) {
			pr_err("%s: ERROR: cannot map register memory, aborting",
			       __func__);
			ret = -EIO;
			goto err_iomap_failed;
		}
		break;
	}
	if (NULL == addr) {
		pr_err("%s: couldn't find register memory\n", __func__);
		ret = -EIO;
		goto err_iomap_failed;
	}

	pci_set_master(pdev);
	if (enable_msi == 1) {
		ret = pci_enable_msi(pdev);
		if(ret)
			pr_info("stmmac MSI mode NOT enabled\n");
		else
			pr_info("stmmac MSI mode enabled\n");
	}

	pr_info("Vendor 0x%04x Device 0x%04x\n",
		id->vendor, id->device);

	priv = stmmac_dvr_probe(&(pdev->dev), plat_dat, addr);
	if (!priv) {
		pr_err("%s: main driver probe failed", __func__);
		goto err_out;
	}

	priv->dev->irq = pdev->irq;
	priv->wol_irq = pdev->irq;
	priv->irqmode_msi = enable_msi;
	priv->pdev = pdev;
	#ifdef CONFIG_INTEL_QUARK_X1000_SOC
	priv->lpi_irq = -ENXIO;
	#endif
	pci_set_drvdata(pdev, priv->dev);

	pr_debug("STMMAC platform driver registration completed");
	bus_id++;

	return 0;

err_out:
	pci_clear_master(pdev);
	iounmap(addr);
err_iomap_failed:
	pci_release_regions(pdev);
err_out_req_reg_failed:
	pci_disable_device(pdev);
err_platdata:
	if (plat_dat != NULL) {
		if (plat_dat->dma_cfg != NULL)
			kfree(plat_dat->dma_cfg);
		if (plat_dat->mdio_bus_data != NULL)
			kfree(plat_dat->mdio_bus_data);
		kfree(plat_dat);
	}
err_out_map_failed:
	bus_id++;
	return ret;
}

/**
 * stmmac_pci_remove
 *
 * @pdev: platform device pointer
 * Description: this function calls the main to free the net resources
 * and releases the PCI resources.
 */
static void stmmac_pci_remove(struct pci_dev *pdev)
{
	struct net_device *ndev = pci_get_drvdata(pdev);
	struct stmmac_priv *priv = netdev_priv(ndev);

	stmmac_dvr_remove(ndev);

	pci_set_drvdata(pdev, NULL);

	if (enable_msi == 1)
		if (pci_dev_msi_enabled(pdev))
			pci_disable_msi(pdev);

	if (priv->plat != NULL) {
		if (priv->plat->dma_cfg != NULL)
			kfree(priv->plat->dma_cfg);
		if (priv->plat->mdio_bus_data != NULL)
			kfree(priv->plat->mdio_bus_data);
		kfree(priv->plat);
	}

	pci_iounmap(pdev, priv->ioaddr);
	pci_release_regions(pdev);
	pci_disable_device(pdev);
}

#ifdef CONFIG_PM
static int stmmac_pci_suspend(struct pci_dev *pdev, pm_message_t state)
{
	struct net_device *ndev = pci_get_drvdata(pdev);
	int ret;

#ifdef CONFIG_INTEL_QUARK_X1000_SOC
	ret = stmmac_freeze(ndev);
#else
	ret = stmmac_suspend(ndev);
#endif
	pci_save_state(pdev);
	pci_set_power_state(pdev, pci_choose_state(pdev, state));

	return ret;
}

static int stmmac_pci_resume(struct pci_dev *pdev)
{
	struct net_device *ndev = pci_get_drvdata(pdev);

	pci_set_power_state(pdev, PCI_D0);
	pci_restore_state(pdev);

	return stmmac_restore(ndev);
}
#endif

static DEFINE_PCI_DEVICE_TABLE(stmmac_id_table) = {
	{PCI_DEVICE(STMMAC_VENDOR_ID, STMMAC_DEVICE_ID)},
	{PCI_DEVICE(PCI_VENDOR_ID_STMICRO, PCI_DEVICE_ID_STMICRO_MAC)},
	{PCI_DEVICE(PCI_VENDOR_ID_INTEL, STMMAC_QUARK_ID)},
	{}
};

MODULE_DEVICE_TABLE(pci, stmmac_id_table);

struct pci_driver stmmac_pci_driver = {
	.name = STMMAC_RESOURCE_NAME,
	.id_table = stmmac_id_table,
	.probe = stmmac_pci_probe,
	.remove = stmmac_pci_remove,
#ifdef CONFIG_PM
	.suspend = stmmac_pci_suspend,
	.resume = stmmac_pci_resume,
#endif
};

MODULE_DESCRIPTION("STMMAC 10/100/1000 Ethernet PCI driver");
MODULE_AUTHOR("Rayagond Kokatanur <rayagond.kokatanur@vayavyalabs.com>");
MODULE_AUTHOR("Giuseppe Cavallaro <peppe.cavallaro@st.com>");
MODULE_LICENSE("GPL");
