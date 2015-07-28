/*******************************************************************************
  This is the driver for the GMAC on-chip Ethernet controller for ST SoCs.
  DWC Ether MAC 10/100/1000 Universal version 3.41a  has been used for
  developing this code.

  This only implements the mac core functions for this chip.

  Copyright (C) 2007-2009  STMicroelectronics Ltd
  Copyright (C) 2013 Intel Corporation

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

  Author: Giuseppe Cavallaro <peppe.cavallaro@st.com>
*******************************************************************************/

#include <linux/crc32.h>
#include <linux/slab.h>
#include <asm/io.h>
#include "dwmac1000.h"
#include "stmmac.h"

static void dwmac1000_core_init(void __iomem *ioaddr)
{
	u32 value = readl(ioaddr + GMAC_CONTROL);
	value |= GMAC_CORE_INIT;
	writel(value, ioaddr + GMAC_CONTROL);

	/* Mask GMAC interrupts */
	writel(GMAC_INT_MASK_DEFAULT, ioaddr + GMAC_INT_MASK);

#ifdef STMMAC_VLAN_TAG_USED
	/* Tag detection without filtering */
	writel(0x0, ioaddr + GMAC_VLAN_TAG);
#endif
}

static int dwmac1000_set_rx_ipc(void __iomem *ioaddr, bool on)
{
	u32 value = readl(ioaddr + GMAC_CONTROL);

	if (on == true)
		value |= GMAC_CONTROL_IPC;
	else
		value &= ~GMAC_CONTROL_IPC;

	writel(value, ioaddr + GMAC_CONTROL);

	value = readl(ioaddr + GMAC_CONTROL);

	return !!(value & GMAC_CONTROL_IPC);
}

static void dwmac1000_dump_regs(void __iomem *ioaddr)
{
	int i;
	pr_info("\tDWMAC1000 regs (base addr = 0x%p)\n", ioaddr);

	for (i = 0; i < 55; i++) {
		int offset = i * 4;
		pr_info("\tReg No. %d (offset 0x%x): 0x%08x\n", i,
			offset, readl(ioaddr + offset));
	}
}

static void dwmac1000_set_umac_addr(void __iomem *ioaddr, unsigned char *addr,
				unsigned int reg_n)
{
	stmmac_set_mac_addr(ioaddr, addr, GMAC_ADDR_HIGH(reg_n),
				GMAC_ADDR_LOW(reg_n));
}

static void dwmac1000_get_umac_addr(void __iomem *ioaddr, unsigned char *addr,
				unsigned int reg_n)
{
	stmmac_get_mac_addr(ioaddr, addr, GMAC_ADDR_HIGH(reg_n),
				GMAC_ADDR_LOW(reg_n));
}

static void dwmac1000_set_filter(struct net_device *dev, int id)
{
	void __iomem *ioaddr = (void __iomem *) dev->base_addr;
	struct stmmac_priv *priv = netdev_priv(dev);
	unsigned int value = 0;
	unsigned int perfect_addr_number;

	CHIP_DBG(KERN_INFO "%s: # mcasts %d, # unicast %d\n",
		 __func__, netdev_mc_count(dev), netdev_uc_count(dev));

	if (dev->flags & IFF_PROMISC)
		value = GMAC_FRAME_FILTER_PR;
	else if ((netdev_mc_count(dev) > HASH_TABLE_SIZE)
		   || (dev->flags & IFF_ALLMULTI)) {
		value = GMAC_FRAME_FILTER_PM;	/* pass all multi */
		writel(0xffffffff, ioaddr + GMAC_HASH_HIGH);
		writel(0xffffffff, ioaddr + GMAC_HASH_LOW);
	} else if (!netdev_mc_empty(dev)) {
		u32 mc_filter[2];
		struct netdev_hw_addr *ha;

		/* Hash filter for multicast */
		value = GMAC_FRAME_FILTER_HMC;

		memset(mc_filter, 0, sizeof(mc_filter));
		netdev_for_each_mc_addr(ha, dev) {
			/* The upper 6 bits of the calculated CRC are used to
			   index the contens of the hash table */
			int bit_nr =
			    bitrev32(~crc32_le(~0, ha->addr, 6)) >> 26;
			/* The most significant bit determines the register to
			 * use (H/L) while the other 5 bits determine the bit
			 * within the register. */
			mc_filter[bit_nr >> 5] |= 1 << (bit_nr & 31);
		}
		writel(mc_filter[0], ioaddr + GMAC_HASH_LOW);
		writel(mc_filter[1], ioaddr + GMAC_HASH_HIGH);
	}

	/* Extra 16 regs are available in cores newer than the 3.40. */
	if (id > DWMAC_CORE_3_40)
		perfect_addr_number = GMAC_MAX_PERFECT_ADDRESSES;
	else
		perfect_addr_number = GMAC_MAX_PERFECT_ADDRESSES / 2;

	/* Handle multiple unicast addresses (perfect filtering)*/
	if (netdev_uc_count(dev) > perfect_addr_number)
		/* Switch to promiscuous mode is more than 16 addrs
		   are required */
		value |= GMAC_FRAME_FILTER_PR;
	else {
		int reg = 1;
		struct netdev_hw_addr *ha;

		netdev_for_each_uc_addr(ha, dev) {
			dwmac1000_set_umac_addr(ioaddr, ha->addr, reg);
			reg++;
		}
	}

#ifdef FRAME_FILTER_DEBUG
	/* Enable Receive all mode (to debug filtering_fail errors) */
	value |= GMAC_FRAME_FILTER_RA;
#endif
	if (priv->active_vlans != 0)
		/* VLAN hash filtering is active on this interface */
		value |= GMAC_FRAME_FILTER_VTFE;

	writel(value, ioaddr + GMAC_FRAME_FILTER);

	CHIP_DBG(KERN_INFO "\tFrame Filter reg: 0x%08x\n\tHash regs: "
	    "HI 0x%08x, LO 0x%08x\n", readl(ioaddr + GMAC_FRAME_FILTER),
	    readl(ioaddr + GMAC_HASH_HIGH), readl(ioaddr + GMAC_HASH_LOW));
}

static void dwmac1000_flow_ctrl(void __iomem *ioaddr, unsigned int duplex,
			   unsigned int fc, unsigned int pause_time)
{
	unsigned int flow = 0;

	CHIP_DBG(KERN_DEBUG "GMAC Flow-Control:\n");
	if (fc & FLOW_RX) {
		CHIP_DBG(KERN_DEBUG "\tReceive Flow-Control ON\n");
		flow |= GMAC_FLOW_CTRL_RFE;
	}
	if (fc & FLOW_TX) {
		CHIP_DBG(KERN_DEBUG "\tTransmit Flow-Control ON\n");
		flow |= GMAC_FLOW_CTRL_TFE;
	}

	if (duplex) {
		CHIP_DBG(KERN_DEBUG "\tduplex mode: PAUSE %d\n", pause_time);
		flow |= (pause_time << GMAC_FLOW_CTRL_PT_SHIFT);
	}

	writel(flow, ioaddr + GMAC_FLOW_CTRL);
}

static void dwmac1000_pmt(void __iomem *ioaddr, unsigned long mode)
{
	unsigned int pmt = 0;

	if (mode & WAKE_MAGIC) {
		CHIP_DBG(KERN_DEBUG "GMAC: WOL Magic frame\n");
		pmt |= power_down | magic_pkt_en;
	}
	if (mode & WAKE_UCAST) {
		CHIP_DBG(KERN_DEBUG "GMAC: WOL on global unicast\n");
		pmt |= global_unicast;
	}

	writel(pmt, ioaddr + GMAC_PMT);
}


#ifndef CONFIG_STMMAC_PTP
#define stmmac_ptp_check_pps_event(x) {}
#endif

static int dwmac1000_irq_status(struct stmmac_priv *priv)
{
	u32 intr_status = readl(priv->ioaddr + GMAC_INT_STATUS);
	int status = 0;

	/* Not used events (e.g. MMC interrupts) are not handled. */
	if ((intr_status & mmc_tx_irq)) {
		CHIP_DBG(KERN_INFO "GMAC: MMC tx interrupt: 0x%08x\n",
		    readl(priv->ioaddr + GMAC_MMC_TX_INTR));
		status |= core_mmc_tx_irq;
	}
	if (unlikely(intr_status & mmc_rx_irq)) {
		CHIP_DBG(KERN_INFO "GMAC: MMC rx interrupt: 0x%08x\n",
		    readl(priv->ioaddr + GMAC_MMC_RX_INTR));
		status |= core_mmc_rx_irq;
	}
	if (unlikely(intr_status & mmc_rx_csum_offload_irq)) {
		CHIP_DBG(KERN_INFO "GMAC: MMC rx csum offload: 0x%08x\n",
		    readl(priv->ioaddr + GMAC_MMC_RX_CSUM_OFFLOAD));
		status |= core_mmc_rx_csum_offload_irq;
	}
	if (unlikely(intr_status & pmt_irq)) {
		CHIP_DBG(KERN_INFO "GMAC: received Magic frame\n");
		/* clear the PMT bits 5 and 6 by reading the PMT
		 * status register. */
		readl(priv->ioaddr + GMAC_PMT);
		status |= core_irq_receive_pmt_irq;
	}
	/* MAC trx/rx EEE LPI entry/exit interrupts */
	if (intr_status & lpiis_irq) {
		/* Clean LPI interrupt by reading the Reg 12 */
		u32 lpi_status = readl(priv->ioaddr + LPI_CTRL_STATUS);

		if (lpi_status & LPI_CTRL_STATUS_TLPIEN) {
			CHIP_DBG(KERN_INFO "GMAC TX entered in LPI\n");
			status |= core_irq_tx_path_in_lpi_mode;
		}
		if (lpi_status & LPI_CTRL_STATUS_TLPIEX) {
			CHIP_DBG(KERN_INFO "GMAC TX exit from LPI\n");
			status |= core_irq_tx_path_exit_lpi_mode;
		}
		if (lpi_status & LPI_CTRL_STATUS_RLPIEN) {
			CHIP_DBG(KERN_INFO "GMAC RX entered in LPI\n");
			status |= core_irq_rx_path_in_lpi_mode;
		}
		if (lpi_status & LPI_CTRL_STATUS_RLPIEX) {
			CHIP_DBG(KERN_INFO "GMAC RX exit from LPI\n");
			status |= core_irq_rx_path_exit_lpi_mode;
		}
	}
	if (unlikely(intr_status & time_stamp_irq))
		stmmac_ptp_check_pps_event(priv);

	return status;
}

static unsigned int dwmac1000_calc_vlan_4bit_crc(const char *vlan)
{
	int i = 0, j = 0, len = 0, bit = 0;
	unsigned int crc = 0xFFFFFFFF;
	unsigned int poly = 0x04C11DB7;
	unsigned char data;

	if (unlikely(vlan == NULL))
		return 0;

	for (i = 0; i < 2; i++) {
		data = vlan[i];

		if (i == 0)
			len = 8;
		else
			len = 4;

		for (bit = 0; bit < len; bit++) {
			j = ((crc >> 31) ^ data) & 0x1;
			crc <<= 1;

			if (j != 0)
				crc ^= poly;

			data >>= 1;
		}
	}
	return crc;
}

/* DesignWare Cores Ethernet MAC Universal Databook, 6.2.2.35
 * "The hash value of the destination address
 * is calculated in the following way:
 *  1. Calculate the 32-bit CRC for the VLAN tag or ID
 *  2. Perform bitwise reversal for the value obtained in 1.
 *  3. Take the upper four bits from the value obtained in 2."
 *
 * vlan here being either vlan tag or vlan ID,
 * since register understands both
 */
static inline unsigned int dwmac1000_get_hash_value(const char *vlan) {
	return (~dwmac1000_calc_vlan_4bit_crc(vlan)) >> 28;
}

static int dwmac1000_vlan_rx_add_vid(struct stmmac_priv *priv,
				unsigned short vid)
{
	u32 reg = 0;
	u32 bit_nr = 0;

	if (unlikely(priv == NULL || vid > GMAC_VLAN_HASH_MAXID))
		return -EINVAL;

	if (priv->active_vlans == 0) {

		/* Flip the VTFE bit in GMAC_FRAME_FILTER */
		reg = readl(priv->ioaddr + GMAC_FRAME_FILTER);
		reg |= GMAC_FRAME_FILTER_VTFE;
		writel(reg, priv->ioaddr + GMAC_FRAME_FILTER);

		/* Enable hash filtering - based on 12 bit vid */
		reg = readl(priv->ioaddr + GMAC_VLAN_TAG);
		reg |= GMAC_VLAN_TAG_VTHM | GMAC_VLAN_TAG_ETV | GMAC_VLAN_TAG_VLMASK;
		writel(reg, priv->ioaddr + GMAC_VLAN_TAG);
	}

	bit_nr = dwmac1000_get_hash_value((const char *)&vid);
	priv->active_vlans |= 1 << bit_nr;

	writel(priv->active_vlans, priv->ioaddr + GMAC_VLAN_HASH);

	return 0;
}

static int dwmac1000_vlan_rx_kill_vid(struct stmmac_priv *priv,
				unsigned short vid)
{
	u32 reg = 0;
	u32 bit_nr = 0;

	if (unlikely(priv == NULL || vid > GMAC_VLAN_HASH_MAXID))
		return -EINVAL;

	bit_nr = dwmac1000_get_hash_value((const char *)&vid);

	priv->active_vlans &= ~(1 << bit_nr);
	writel(priv->active_vlans, priv->ioaddr + GMAC_VLAN_HASH);

	if (priv->active_vlans == 0) {

		/* Disable hash filtering */
		reg = readl(priv->ioaddr + GMAC_VLAN_TAG);
		reg &= ~(GMAC_VLAN_TAG_VTHM | GMAC_VLAN_TAG_ETV | 0x00000001);
		writel(reg, priv->ioaddr + GMAC_VLAN_TAG);

		/* Flip the VTFE bit in GMAC_FRAME_FILTER */
		reg = readl(priv->ioaddr + GMAC_FRAME_FILTER);
		reg &= ~GMAC_FRAME_FILTER_VTFE;
		writel(reg, priv->ioaddr + GMAC_FRAME_FILTER);
	}

	return 0;
}

static void  dwmac1000_set_eee_mode(void __iomem *ioaddr)
{
	u32 value;

	/* Enable the link status receive on RGMII, SGMII ore SMII
	 * receive path and instruct the transmit to enter in LPI
	 * state. */
	value = readl(ioaddr + LPI_CTRL_STATUS);
	value |= LPI_CTRL_STATUS_LPIEN | LPI_CTRL_STATUS_LPITXA;
	writel(value, ioaddr + LPI_CTRL_STATUS);
}

static void  dwmac1000_reset_eee_mode(void __iomem *ioaddr)
{
	u32 value;

	value = readl(ioaddr + LPI_CTRL_STATUS);
	value &= ~(LPI_CTRL_STATUS_LPIEN | LPI_CTRL_STATUS_LPITXA);
	writel(value, ioaddr + LPI_CTRL_STATUS);
}

static void  dwmac1000_set_eee_pls(void __iomem *ioaddr, int link)
{
	u32 value;

	value = readl(ioaddr + LPI_CTRL_STATUS);

	if (link)
		value |= LPI_CTRL_STATUS_PLS;
	else
		value &= ~LPI_CTRL_STATUS_PLS;

	writel(value, ioaddr + LPI_CTRL_STATUS);
}

static void  dwmac1000_set_eee_timer(void __iomem *ioaddr, int ls, int tw)
{
	int value = ((tw & 0xffff)) | ((ls & 0x7ff) << 16);

	/* Program the timers in the LPI timer control register:
	 * LS: minimum time (ms) for which the link
	 *  status from PHY should be ok before transmitting
	 *  the LPI pattern.
	 * TW: minimum time (us) for which the core waits
	 *  after it has stopped transmitting the LPI pattern.
	 */
	writel(value, ioaddr + LPI_TIMER_CTRL);
}


static const struct stmmac_ops dwmac1000_ops = {
	.core_init = dwmac1000_core_init,
	.set_rx_ipc = dwmac1000_set_rx_ipc,
	.dump_regs = dwmac1000_dump_regs,
	.host_irq_status = dwmac1000_irq_status,
	.set_filter = dwmac1000_set_filter,
	.flow_ctrl = dwmac1000_flow_ctrl,
	.pmt = dwmac1000_pmt,
	.set_umac_addr = dwmac1000_set_umac_addr,
	.get_umac_addr = dwmac1000_get_umac_addr,
	.vlan_rx_add_vid = dwmac1000_vlan_rx_add_vid,
	.vlan_rx_kill_vid = dwmac1000_vlan_rx_kill_vid,
	.set_eee_mode =  dwmac1000_set_eee_mode,
	.reset_eee_mode =  dwmac1000_reset_eee_mode,
	.set_eee_timer =  dwmac1000_set_eee_timer,
	.set_eee_pls =  dwmac1000_set_eee_pls,
};

struct mac_device_info *dwmac1000_setup(void __iomem *ioaddr)
{
	struct mac_device_info *mac;
	u32 hwid = readl(ioaddr + GMAC_VERSION);

	mac = kzalloc(sizeof(const struct mac_device_info), GFP_KERNEL);
	if (!mac)
		return NULL;

	mac->mac = &dwmac1000_ops;
	mac->dma = &dwmac1000_dma_ops;

	mac->link.port = GMAC_CONTROL_PS;
	mac->link.duplex = GMAC_CONTROL_DM;
	mac->link.speed = GMAC_CONTROL_FES;
	mac->mii.addr = GMAC_MII_ADDR;
	mac->mii.data = GMAC_MII_DATA;
	mac->synopsys_uid = hwid;

	return mac;
}
