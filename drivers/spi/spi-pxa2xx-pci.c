/*
 * CE4100's SPI device is more or less the same one as found on PXA
 *
 */
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <linux/of_device.h>
#include <linux/module.h>
#include <linux/spi/pxa2xx_spi.h>
#include <linux/irq.h>
#include <linux/platform_data/quark.h>

/* defined here to avoid including arch/x86/pci/intel_media_proc_gen3.c */
#define CE3100_SOC_DEVICE_ID 0x2E50
#define CE4100_SOC_DEVICE_ID 0x0708
#define CE4200_SOC_DEVICE_ID 0x0709
#define CE5300_SOC_DEVICE_ID 0x0C40
#define CE2600_SOC_DEVICE_ID 0x0931

#ifdef CONFIG_INTEL_QUARK_X1000_SOC_FPGAEMU
#define CE4200_NUM_SPI_MASTER 1
#else
#define CE4200_NUM_SPI_MASTER 2
#endif

#define CE4X00_SPI_MAX_SPEED  1843200

#ifdef CONFIG_INTEL_QUARK_X1000_SOC
#ifdef CONFIG_INTEL_QUARK_X1000_SOC_FPGAEMU
#define CE5X00_SPI_MAX_SPEED  3500000
#else
#define CE5X00_SPI_MAX_SPEED  50000000
#endif
#else
#define CE5X00_SPI_MAX_SPEED  5000000
#endif

#define CE4200_NUM_CHIPSELECT 4

#define SPI_CE_DEBUG

static int interface;

#ifdef CONFIG_INTEL_QUARK_X1000_SOC
static int enable_msi = 1;
#else
static int enable_msi;
#endif
module_param(enable_msi, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(enable_msi, "Enable PCI MSI mode");

struct ce4100_info {
	struct ssp_device ssp;
	struct platform_device *spi_pdev;
};

static DEFINE_MUTEX(ssp_lock);
static LIST_HEAD(ssp_list);

struct ssp_device *pxa_ssp_request(int port, const char *label)
{
	struct ssp_device *ssp = NULL;

	mutex_lock(&ssp_lock);

	list_for_each_entry(ssp, &ssp_list, node) {
		if (ssp->port_id == port && ssp->use_count == 0) {
			ssp->use_count++;
			ssp->label = label;
			break;
		}
	}

	mutex_unlock(&ssp_lock);

	if (&ssp->node == &ssp_list)
		return NULL;

	return ssp;
}
EXPORT_SYMBOL_GPL(pxa_ssp_request);

void pxa_ssp_free(struct ssp_device *ssp)
{
	mutex_lock(&ssp_lock);
	if (ssp->use_count) {
		ssp->use_count--;
		ssp->label = NULL;
	} else
		dev_err(&ssp->pdev->dev, "device already free\n");
	mutex_unlock(&ssp_lock);
}
EXPORT_SYMBOL_GPL(pxa_ssp_free);

static int ce4100_spi_probe(struct pci_dev *dev,
		const struct pci_device_id *ent)
{
	int ret;
	resource_size_t phys_beg;
	resource_size_t phys_len;
	struct ce4100_info *spi_info = NULL;
	struct platform_device *pdev;
	struct pxa2xx_spi_master spi_pdata;
	struct ssp_device *ssp;
	unsigned int id;

	ret = pci_enable_device(dev);
	if (ret)
		return ret;

	phys_beg = pci_resource_start(dev, 0);
	phys_len = pci_resource_len(dev, 0);

	if (!request_mem_region(phys_beg, phys_len,
				"CE4100 SPI")) {
		dev_err(&dev->dev, "Can't request register space.\n");
		ret = -EBUSY;
		return ret;
	}

	pdev = platform_device_alloc("pxa2xx-spi", dev->devfn);
	if (!pdev) {
		ret = -ENOMEM;
		goto err_release_mem_region;
	}
	spi_info = kzalloc(sizeof(*spi_info), GFP_KERNEL);
	if (!spi_info) {
		ret = -ENOMEM;
		goto err_platform_device_put;
	}
	memset(&spi_pdata, 0, sizeof(spi_pdata));
	spi_pdata.num_chipselect = CE4200_NUM_CHIPSELECT;

	ret = platform_device_add_data(pdev, &spi_pdata, sizeof(spi_pdata));
	if (ret)
		goto err_free_spi_info;

	pdev->id = interface;
	pdev->dev.parent = &dev->dev;
#ifdef CONFIG_OF
	pdev->dev.of_node = dev->dev.of_node;
#endif
	ssp = &spi_info->ssp;
	ssp->pcidev = dev;
	ssp->phys_base = pci_resource_start(dev, 0);
	ssp->mmio_base = ioremap(phys_beg, phys_len);
	if (!ssp->mmio_base) {
		dev_err(&pdev->dev, "failed to ioremap() registers\n");
		ret = -EIO;
		goto err_ioremap;
	}
	pci_set_master(dev);
	if (enable_msi == 1) {
		ret = pci_enable_msi(dev);
		if (ret) {
			dev_dbg(&dev->dev, "failed to allocate MSI entry\n");
		}
	}

	ssp->irq = dev->irq;
	ssp->port_id = pdev->id;

#ifdef CONFIG_INTEL_QUARK_X1000_SOC
	id = CE5300_SOC_DEVICE_ID;
#else
	intelce_get_soc_info(&id, NULL);
#endif
	switch (id) {
	case CE5300_SOC_DEVICE_ID:
		ssp->type = CE5X00_SSP;
		break;
	case CE4200_SOC_DEVICE_ID:
	default:
		ssp->type = CE4100_SSP;
		break;
	}
	mutex_lock(&ssp_lock);
	list_add(&ssp->node, &ssp_list);
	mutex_unlock(&ssp_lock);

	pci_set_drvdata(dev, spi_info);

	spi_info->spi_pdev = pdev;
	ret = platform_device_add(pdev);
	if (ret)
		goto err_dev_add;

	interface++;

	return ret;

err_dev_add:
	pci_set_drvdata(dev, NULL);
	mutex_lock(&ssp_lock);
	list_del(&ssp->node);
	mutex_unlock(&ssp_lock);
	iounmap(ssp->mmio_base);
err_ioremap:

err_free_spi_info:
	kfree(spi_info);

err_platform_device_put:
	platform_device_put(pdev);

err_release_mem_region:
	release_mem_region(phys_beg, phys_len);
	return ret;
}

static void ce4100_spi_remove(struct pci_dev *dev)
{
	struct ce4100_info *spi_info;
	struct ssp_device *ssp;

	spi_info = pci_get_drvdata(dev);
	ssp = &spi_info->ssp;
	platform_device_unregister(spi_info->spi_pdev);

	iounmap(ssp->mmio_base);
	release_mem_region(pci_resource_start(dev, 0),
			pci_resource_len(dev, 0));

	mutex_lock(&ssp_lock);
	list_del(&ssp->node);
	mutex_unlock(&ssp_lock);

	if (enable_msi == 1) {
		if (pci_dev_msi_enabled(dev))
			pci_disable_msi(dev);
	}

	pci_set_drvdata(dev, NULL);
	pci_disable_device(dev);
	kfree(spi_info);
}


#ifdef CONFIG_PM
static int ce4XXX_spi_suspend(struct pci_dev *dev, pm_message_t state)
{
	pci_save_state(dev);
	pci_set_power_state(dev, pci_choose_state(dev, state));
	return 0;
}

static int ce4XXX_spi_resume(struct pci_dev *dev)
{
	pci_set_power_state(dev, PCI_D0);
	pci_restore_state(dev);

	return 0;
}
#endif

static DEFINE_PCI_DEVICE_TABLE(ce4100_spi_devices) = {
	{ PCI_DEVICE(PCI_VENDOR_ID_INTEL, 0x2e6a) },
	{ PCI_DEVICE(PCI_VENDOR_ID_INTEL, 0x0935) },
	{ },
};

MODULE_DEVICE_TABLE(pci, ce4100_spi_devices);

static struct pci_driver ce4100_spi_driver = {
	.name           = "ce4100_spi",
	.id_table       = ce4100_spi_devices,
	.probe          = ce4100_spi_probe,
#ifdef CONFIG_PM
	.suspend        = ce4XXX_spi_suspend,
	.resume         = ce4XXX_spi_resume,
#endif
	.remove         = ce4100_spi_remove,
};

module_pci_driver(ce4100_spi_driver);

MODULE_DESCRIPTION("CE4100 PCI-SPI glue code for PXA's driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Sebastian Andrzej Siewior <bigeasy@linutronix.de>");
