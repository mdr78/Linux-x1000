/*
 *  intel_mid_dma.h - Intel MID DMA Drivers
 *
 *  Copyright (C) 2008-10 Intel Corp
 *  Author: Vinod Koul <vinod.koul@intel.com>
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
#ifndef __INTEL_MID_DMA_H__
#define __INTEL_MID_DMA_H__

#include <linux/dmaengine.h>
#include <linux/interrupt.h>

#define DMA_PREP_CIRCULAR_LIST		(1 << 10)
#define MAX_CHAN 4

/*DMA mode configurations*/
enum intel_mid_dma_mode {
	LNW_DMA_PER_TO_MEM = 0, /*periphral to memory configuration*/
	LNW_DMA_MEM_TO_PER,	/*memory to periphral configuration*/
	LNW_DMA_MEM_TO_MEM,	/*mem to mem confg (testing only)*/
};

/*DMA handshaking*/
enum intel_mid_dma_hs_mode {
	LNW_DMA_HW_HS = 0,	/*HW Handshaking only*/
	LNW_DMA_SW_HS = 1,	/*SW Handshaking not recommended*/
};

/*Burst size configuration*/
enum intel_mid_dma_msize {
	LNW_DMA_MSIZE_1 = 0x0,
	LNW_DMA_MSIZE_4 = 0x1,
	LNW_DMA_MSIZE_8 = 0x2,
	LNW_DMA_MSIZE_16 = 0x3,
	LNW_DMA_MSIZE_32 = 0x4,
	LNW_DMA_MSIZE_64 = 0x5,
};

/**
 * struct intel_mid_dma_slave - DMA slave structure
 *
 * @dirn: DMA trf direction
 * @src_width: tx register width
 * @dst_width: rx register width
 * @hs_mode: HW/SW handshaking mode
 * @cfg_mode: DMA data transfer mode (per-per/mem-per/mem-mem)
 * @src_msize: Source DMA burst size
 * @dst_msize: Dst DMA burst size
 * @per_addr: Periphral address
 * @device_instance: DMA peripheral device instance, we can have multiple
 *		peripheral device connected to single DMAC
 */
struct intel_mid_dma_slave {
	enum intel_mid_dma_hs_mode	hs_mode;  /*handshaking*/
	enum intel_mid_dma_mode		cfg_mode; /*mode configuration*/
	unsigned int		device_instance; /*0, 1 for periphral instance*/
	struct dma_slave_config		dma_slave;
};

/**
 * struct intel_mid_dma_chan - internal mid representation of a DMA channel
 * @chan: dma_chan strcture represetation for mid chan
 * @ch_regs: MMIO register space pointer to channel register
 * @dma_base: MMIO register space DMA engine base pointer
 * @ch_id: DMA channel id
 * @lock: channel spinlock
 * @active_list: current active descriptors
 * @queue: current queued up descriptors
 * @free_list: current free descriptors
 * @slave: dma slave struture
 * @descs_allocated: total number of decsiptors allocated
 * @dma: dma device struture pointer
 * @busy: bool representing if ch is busy (active txn) or not
 * @in_use: bool representing if ch is in use or not
 * @raw_tfr: raw trf interrupt received
 * @raw_block: raw block interrupt received
 */
struct intel_mid_dma_chan {
	struct dma_chan		chan;
	void __iomem		*ch_regs;
	void __iomem		*dma_base;
	int			ch_id;
	spinlock_t		lock;
	struct list_head	active_list;
	struct list_head	queue;
	struct list_head	free_list;
	unsigned int		descs_allocated;
	struct middma_device	*dma;
	bool			busy;
	bool			in_use;
	u32			raw_tfr;
	u32			raw_block;
	struct intel_mid_dma_slave *mid_slave;
};

struct intel_mid_dma_desc {
	void __iomem			*block; /*ch ptr*/
	struct list_head		desc_node;
	struct dma_async_tx_descriptor	txd;
	size_t				len;
	dma_addr_t			sar;
	dma_addr_t			dar;
	u32				cfg_hi;
	u32				cfg_lo;
	u32				ctl_lo;
	u32				ctl_hi;
	struct pci_pool			*lli_pool;
	struct intel_mid_dma_lli	*lli;
	dma_addr_t			lli_phys;
	unsigned int			lli_length;
	unsigned int			current_lli;
	dma_addr_t			next;
	enum dma_transfer_direction		dirn;
	enum dma_status			status;
	enum dma_slave_buswidth		width; /*width of DMA txn*/
	enum intel_mid_dma_mode		cfg_mode; /*mode configuration*/

};


enum intel_mid_dma_state {
	RUNNING = 0,
	SUSPENDED,
};
/**
 * struct middma_device - internal representation of a DMA device
 * @pdev: PCI device
 * @dma_base: MMIO register space pointer of DMA
 * @dma_pool: for allocating DMA descriptors
 * @common: embedded struct dma_device
 * @tasklet: dma tasklet for processing interrupts
 * @ch: per channel data
 * @pci_id: DMA device PCI ID
 * @intr_mask: Interrupt mask to be used
 * @mask_reg: MMIO register for periphral mask
 * @chan_base: Base ch index (read from driver data)
 * @max_chan: max number of chs supported (from drv_data)
 * @block_size: Block size of DMA transfer supported (from drv_data)
 * @pimr_mask: MMIO register addr for periphral interrupt (from drv_data)
 * @state: dma PM device state
 */
struct middma_device {
	struct pci_dev		*pdev;
	void __iomem		*dma_base;
	struct pci_pool		*dma_pool;
	struct dma_device	common;
	struct tasklet_struct   tasklet;
	struct intel_mid_dma_chan ch[MAX_CHAN];
	unsigned int		pci_id;
	unsigned int		intr_mask;
	void __iomem		*mask_reg;
	int			chan_base;
	int			max_chan;
	int			block_size;
	bool			ispci_fn;
	unsigned int		pimr_mask;
	unsigned int		is_quark;
	enum intel_mid_dma_state state;
};

/**
 * struct intel_mid_dma_probe_info
 *
 * @max_chan: maximum channels to probe
 * @ch_base: offset from register base
 * @block_size: TBD
 * @pimr_mask: indicates if mask registers to be mapped
 * @is_quark: indicates if this is quark silicon
 */
struct intel_mid_dma_probe_info {
	u8 max_chan;
	u8 ch_base;
	u16 block_size;
	u32 pimr_mask;
	u8 is_quark;
};


/**
 * intel_mid_dma_interrupt -	DMA ISR
 * @irq: IRQ where interrupt occurred
 * @data: ISR cllback data (the controller structure)
 *
 * See if this is our interrupt if so then schedule the tasklet
 * otherwise ignore
 */
irqreturn_t intel_mid_dma_interrupt(int irq, void *data);

/**
 * mid_setup_dma -	Setup DMA controller
 * @pdev: Controller PCI device structure
 *
 * Called by remove
 * Unregister DMa controller, clear all structures and free interrupt
 */	
int mid_setup_dma(struct pci_dev *pdev, struct middma_device *dma);

/**
 * middma_shutdown -	Shutdown the DMA controller
 * @pdev: Controller PCI device structure
 *
 * Called by remove
 * Unregister DMa controller, clear all structures and free interrupt
 */	
void middma_shutdown(struct pci_dev *pdev, struct middma_device *device);

/**
 * intel_mid_dma_probe -	PCI Probe
 * @pdev: Controller PCI device structure
 * @id: pci device id structure
 *
 * Initialize the PCI device, map BARs, query driver data.
 * Call intel_setup_dma to complete contoller and chan initilzation
 */
int intel_qrk_dma_probe(struct pci_dev *pdev,
			struct middma_device *device);
/**
 * intel_mid_dma_remove -	PCI remove
 * @pdev: Controller PCI device structure
 *
 * Free up all resources and data
 * Call shutdown_dma to complete contoller and chan cleanup
 */
void intel_qrk_dma_remove(struct pci_dev *pdev, struct middma_device *device);

/* Power Management */
/*
* dma_suspend - PCI suspend function
*
* @pci: PCI device structure
* @state: PM message
*
* This function is called by OS when a power event occurs
*/
int intel_qrk_dma_suspend(struct middma_device *device);

/**
* intel_qrk_dma_resume - PCI resume function
*
* @pci:	PCI device structure
*
* This function is called by OS when a power event occurs
*/
int intel_qrk_dma_resume(struct middma_device *device);


#endif /*__INTEL_MID_DMA_H__*/
