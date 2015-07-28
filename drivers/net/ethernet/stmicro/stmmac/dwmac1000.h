/*******************************************************************************
  Copyright (C) 2007-2009  STMicroelectronics Ltd

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
#ifndef __DWMAC1000_H__
#define __DWMAC1000_H__

#include <linux/phy.h>
#include "common.h"

#define GMAC_CONTROL		0x00000000	/* Configuration */
#define GMAC_FRAME_FILTER	0x00000004	/* Frame Filter */
#define GMAC_HASH_HIGH		0x00000008	/* Multicast Hash Table High */
#define GMAC_HASH_LOW		0x0000000c	/* Multicast Hash Table Low */
#define GMAC_MII_ADDR		0x00000010	/* MII Address */
#define GMAC_MII_DATA		0x00000014	/* MII Data */
#define GMAC_FLOW_CTRL		0x00000018	/* Flow Control */
#define GMAC_VLAN_TAG		0x0000001c	/* VLAN Tag */
#define GMAC_VERSION		0x00000020	/* GMAC CORE Version */
#define GMAC_WAKEUP_FILTER	0x00000028	/* Wake-up Frame Filter */

#define GMAC_INT_STATUS		0x00000038	/* interrupt status register */
enum dwmac1000_irq_status {
	lpiis_irq = 0x400,
	time_stamp_irq = 0x0200,
	mmc_rx_csum_offload_irq = 0x0080,
	mmc_tx_irq = 0x0040,
	mmc_rx_irq = 0x0020,
	mmc_irq = 0x0010,
	pmt_irq = 0x0008,
	pcs_ane_irq = 0x0004,
	pcs_link_irq = 0x0002,
	rgmii_irq = 0x0001,
};
#define GMAC_INT_MASK		0x0000003c	/* interrupt mask register */
#define GMAC_INT_MASK_LPIIM	0x00000200	/* LPI Interrupt Mask */
#define GMAC_INT_MASK_TSIM	0x00000100	/* Timestamp Interrupt Mask */
#define GMAC_INT_MASK_PMTIM	0x00000004	/* PMT Interrupt Mask */
#define GMAC_INT_MASK_PCSANCIM	0x00000002	/* PCS AN Completion */
#define GMAC_INT_MASK_PCSLCHGIM	0x00000001	/* PCS Link Status */
#define GMAC_INT_MASK_DEFAULT (GMAC_INT_MASK_PCSLCHGIM | GMAC_INT_MASK_PCSANCIM\
				| GMAC_INT_MASK_PMTIM | GMAC_INT_MASK_TSIM\
				| GMAC_INT_MASK_LPIIM)
/* PMT Control and Status */
#define GMAC_PMT		0x0000002c
enum power_event {
	pointer_reset = 0x80000000,
	global_unicast = 0x00000200,
	wake_up_rx_frame = 0x00000040,
	magic_frame = 0x00000020,
	wake_up_frame_en = 0x00000004,
	magic_pkt_en = 0x00000002,
	power_down = 0x00000001,
};

/* Energy Efficient Ethernet (EEE)
 *
 * LPI status, timer and control register offset
 */
#define LPI_CTRL_STATUS	0x0030
#define LPI_TIMER_CTRL	0x0034

/* LPI control and status defines */
#define LPI_CTRL_STATUS_LPITXA	0x00080000	/* Enable LPI TX Automate */
#define LPI_CTRL_STATUS_PLSEN	0x00040000	/* Enable PHY Link Status */
#define LPI_CTRL_STATUS_PLS	0x00020000	/* PHY Link Status */
#define LPI_CTRL_STATUS_LPIEN	0x00010000	/* LPI Enable */
#define LPI_CTRL_STATUS_RLPIST	0x00000200	/* Receive LPI state */
#define LPI_CTRL_STATUS_TLPIST	0x00000100	/* Transmit LPI state */
#define LPI_CTRL_STATUS_RLPIEX	0x00000008	/* Receive LPI Exit */
#define LPI_CTRL_STATUS_RLPIEN	0x00000004	/* Receive LPI Entry */
#define LPI_CTRL_STATUS_TLPIEX	0x00000002	/* Transmit LPI Exit */
#define LPI_CTRL_STATUS_TLPIEN	0x00000001	/* Transmit LPI Entry */

/* GMAC HW ADDR regs */
#define GMAC_ADDR_HIGH(reg)	(((reg > 15) ? 0x00000800 : 0x00000040) + \
				(reg * 8))
#define GMAC_ADDR_LOW(reg)	(((reg > 15) ? 0x00000804 : 0x00000044) + \
				(reg * 8))
#define GMAC_MAX_PERFECT_ADDRESSES	32

#define GMAC_AN_CTRL	0x000000c0	/* AN control */
#define GMAC_AN_STATUS	0x000000c4	/* AN status */
#define GMAC_ANE_ADV	0x000000c8	/* Auto-Neg. Advertisement */
#define GMAC_ANE_LINK	0x000000cc	/* Auto-Neg. link partener ability */
#define GMAC_ANE_EXP	0x000000d0	/* ANE expansion */
#define GMAC_TBI	0x000000d4	/* TBI extend status */
#define GMAC_GMII_STATUS 0x000000d8	/* S/R-GMII status */

/* GMAC Configuration defines */
#define GMAC_CONTROL_TC	0x01000000	/* Transmit Conf. in RGMII/SGMII */
#define GMAC_CONTROL_WD	0x00800000	/* Disable Watchdog on receive */
#define GMAC_CONTROL_JD	0x00400000	/* Jabber disable */
#define GMAC_CONTROL_BE	0x00200000	/* Frame Burst Enable */
#define GMAC_CONTROL_JE	0x00100000	/* Jumbo frame */
enum inter_frame_gap {
	GMAC_CONTROL_IFG_88 = 0x00040000,
	GMAC_CONTROL_IFG_80 = 0x00020000,
	GMAC_CONTROL_IFG_40 = 0x000e0000,
};
#define GMAC_CONTROL_DCRS	0x00010000 /* Disable carrier sense during tx */
#define GMAC_CONTROL_PS		0x00008000 /* Port Select 0:GMI 1:MII */
#define GMAC_CONTROL_FES	0x00004000 /* Speed 0:10 1:100 */
#define GMAC_CONTROL_DO		0x00002000 /* Disable Rx Own */
#define GMAC_CONTROL_LM		0x00001000 /* Loop-back mode */
#define GMAC_CONTROL_DM		0x00000800 /* Duplex Mode */
#define GMAC_CONTROL_IPC	0x00000400 /* Checksum Offload */
#define GMAC_CONTROL_DR		0x00000200 /* Disable Retry */
#define GMAC_CONTROL_LUD	0x00000100 /* Link up/down */
#define GMAC_CONTROL_ACS	0x00000080 /* Automatic Pad/FCS Stripping */
#define GMAC_CONTROL_DC		0x00000010 /* Deferral Check */
#define GMAC_CONTROL_TE		0x00000008 /* Transmitter Enable */
#define GMAC_CONTROL_RE		0x00000004 /* Receiver Enable */

#define GMAC_CORE_INIT (GMAC_CONTROL_JD | GMAC_CONTROL_PS | GMAC_CONTROL_ACS | \
			GMAC_CONTROL_JE | GMAC_CONTROL_BE)

/* GMAC Frame Filter defines */
#define GMAC_FRAME_FILTER_PR	0x00000001	/* Promiscuous Mode */
#define GMAC_FRAME_FILTER_HUC	0x00000002	/* Hash Unicast */
#define GMAC_FRAME_FILTER_HMC	0x00000004	/* Hash Multicast */
#define GMAC_FRAME_FILTER_DAIF	0x00000008	/* DA Inverse Filtering */
#define GMAC_FRAME_FILTER_PM	0x00000010	/* Pass all multicast */
#define GMAC_FRAME_FILTER_DBF	0x00000020	/* Disable Broadcast frames */
#define GMAC_FRAME_FILTER_SAIF	0x00000100	/* Inverse Filtering */
#define GMAC_FRAME_FILTER_SAF	0x00000200	/* Source Address Filter */
#define GMAC_FRAME_FILTER_HPF	0x00000400	/* Hash or perfect Filter */
#define GMAC_FRAME_FILTER_VTFE	0x00010000	/* VLAN Tag Filter Enable */
#define GMAC_FRAME_FILTER_RA	0x80000000	/* Receive all mode */
/* GMII ADDR  defines */
#define GMAC_MII_ADDR_WRITE	0x00000002	/* MII Write */
#define GMAC_MII_ADDR_BUSY	0x00000001	/* MII Busy */
/* GMAC FLOW CTRL defines */
#define GMAC_FLOW_CTRL_PT_MASK	0xffff0000	/* Pause Time Mask */
#define GMAC_FLOW_CTRL_PT_SHIFT	16
#define GMAC_FLOW_CTRL_RFE	0x00000004	/* Rx Flow Control Enable */
#define GMAC_FLOW_CTRL_TFE	0x00000002	/* Tx Flow Control Enable */
#define GMAC_FLOW_CTRL_FCB_BPA	0x00000001	/* Flow Control Busy ... */
/* GMAC VLAN TAG defines */
#define GMAC_VLAN_TAG_VTHM	0x00080000	/* Hash Table Match Enable */
#define GMAC_VLAN_TAG_ESVL	0x00040000	/* Enable S-VLAN */
#define GMAC_VLAN_TAG_VTIM	0x00020000	/* VLAN Tag inverse match */
#define GMAC_VLAN_TAG_ETV	0x00010000	/* Enable 12-bit tag comp */
#define GMAC_VLAN_TAG_VLMASK	0x0000FFFF	/* VLAN tag ID for Rx frames */
/*--- DMA BLOCK defines ---*/
/* DMA Bus Mode register defines */
#define DMA_BUS_MODE_SFT_RESET	0x00000001	/* Software Reset */
#define DMA_BUS_MODE_DA		0x00000002	/* Arbitration scheme */
#define DMA_BUS_MODE_ATDS	0X00000080	/* Alternate Descriptor Size */
#define DMA_BUS_MODE_DSL_MASK	0x0000007c	/* Descriptor Skip Length */
#define DMA_BUS_MODE_DSL_SHIFT	2	/*   (in DWORDS)      */
/* Programmable burst length (passed thorugh platform)*/
#define DMA_BUS_MODE_PBL_MASK	0x00003f00	/* Programmable Burst Len */
#define DMA_BUS_MODE_PBL_SHIFT	8

enum rx_tx_priority_ratio {
	double_ratio = 0x00004000,	/*2:1 */
	triple_ratio = 0x00008000,	/*3:1 */
	quadruple_ratio = 0x0000c000,	/*4:1 */
};

#define DMA_BUS_MODE_FB		0x00010000	/* Fixed burst */
#define DMA_BUS_MODE_MB		0x04000000	/* Mixed burst */
#define DMA_BUS_MODE_RPBL_MASK	0x003e0000	/* Rx-Programmable Burst Len */
#define DMA_BUS_MODE_RPBL_SHIFT	17
#define DMA_BUS_MODE_USP	0x00800000
#define DMA_BUS_MODE_PBL	0x01000000
#define DMA_BUS_MODE_AAL	0x02000000
#define DMA_BUS_MODE_RIX	0x80000000

/* DMA CRS Control and Status Register Mapping */
#define DMA_HOST_TX_DESC	  0x00001048	/* Current Host Tx descriptor */
#define DMA_HOST_RX_DESC	  0x0000104c	/* Current Host Rx descriptor */
/*  DMA Bus Mode register defines */
#define DMA_BUS_PR_RATIO_MASK	  0x0000c000	/* Rx/Tx priority ratio */
#define DMA_BUS_PR_RATIO_SHIFT	  14
#define DMA_BUS_FB	  	  0x00010000	/* Fixed Burst */

/* DMA operation mode defines (start/stop tx/rx are placed in common header)*/
#define DMA_CONTROL_DT		0x04000000 /* Disable Drop TCP/IP csum error */
#define DMA_CONTROL_RSF		0x02000000 /* Receive Store and Forward */
#define DMA_CONTROL_DFF		0x01000000 /* Disaable flushing */
/* Threshold for Activating the FC */
enum rfa {
	act_full_minus_1 = 0x00800000,
	act_full_minus_2 = 0x00800200,
	act_full_minus_3 = 0x00800400,
	act_full_minus_4 = 0x00800600,
};
/* Threshold for Deactivating the FC */
enum rfd {
	deac_full_minus_1 = 0x00400000,
	deac_full_minus_2 = 0x00400800,
	deac_full_minus_3 = 0x00401000,
	deac_full_minus_4 = 0x00401800,
};
#define DMA_CONTROL_TSF		0x00200000 /* Transmit  Store and Forward */

enum ttc_control {
	DMA_CONTROL_TTC_64 = 0x00000000,
	DMA_CONTROL_TTC_128 = 0x00004000,
	DMA_CONTROL_TTC_192 = 0x00008000,
	DMA_CONTROL_TTC_256 = 0x0000c000,
	DMA_CONTROL_TTC_40 = 0x00010000,
	DMA_CONTROL_TTC_32 = 0x00014000,
	DMA_CONTROL_TTC_24 = 0x00018000,
	DMA_CONTROL_TTC_16 = 0x0001c000,
};
#define DMA_CONTROL_TC_TX_MASK	0xfffe3fff

#define DMA_CONTROL_EFC		0x00000100
#define DMA_CONTROL_FEF		0x00000080
#define DMA_CONTROL_FUF		0x00000040

enum rtc_control {
	DMA_CONTROL_RTC_64 = 0x00000000,
	DMA_CONTROL_RTC_32 = 0x00000008,
	DMA_CONTROL_RTC_96 = 0x00000010,
	DMA_CONTROL_RTC_128 = 0x00000018,
};
#define DMA_CONTROL_TC_RX_MASK	0xffffffe7

#define DMA_CONTROL_OSF	0x00000004	/* Operate on second frame */

/* MMC registers offset */
#define GMAC_MMC_CTRL      0x100
#define GMAC_MMC_RX_INTR   0x104
#define GMAC_MMC_TX_INTR   0x108
#define GMAC_MMC_RX_CSUM_OFFLOAD   0x208

/* VLAN Hash register offset */
#define GMAC_VLAN_TAG_REP	0x584
#define GMAC_VLAN_HASH		0x588
#define GMAC_VLAN_HASH_MAXID	0x0F

/***************** 1588 regs *****************/
#define GMAC_TS_CTRL		0x700		/* Timestamp control reg */
#define GMAC_TS_CTRL_TSENA	0x00000001	/* Timestamp enable */
#define GMAC_TS_CTRL_TSCFUPDT	0x00000002	/* Timestamp fine/coarse */
#define GMAC_TS_CTRL_TSINT	0x00000004	/* Timestamp initialise */
#define GMAC_TS_CTRL_TSUPDT	0x00000008	/* Timestamp update */
#define GMAC_TS_CTRL_TSTRIG	0x00000010	/* Timestamp trigger en */
#define GMAC_TS_CTRL_TSADDREG	0x00000020	/* Timestamp addreg update */
#define GMAC_TS_CTRL_TSENALL	0x00000100	/* Timestamp RX enable all */
#define GMAC_TS_CTRL_TSCTRLSSR	0x00000200	/* Timestamp rollover ctr */
#define GMAC_TS_CTRL_TSVER2ENA	0x00000400	/* Timestamp PTP v2 en */
#define GMAC_TS_CTRL_TSIPENA	0x00000800	/* Timestamp PTP over eth */
#define GMAC_TS_CTRL_TSIPV6ENA	0x00001000	/* Timestamp over IPV6 */
#define GMAC_TS_CTRL_TSIPV4ENA	0x00002000	/* Timestamp over IPV4 */
#define GMAC_TS_CTRL_TSEVNTENA	0x00004000	/* Timestamp event only */
#define GMAC_TS_CTRL_TSMSTRENA	0x00008000	/* Timestamp master enable */
#define GMAC_TS_CTRL_SNTYPSEL0	0x00000000	/* Timestamp type 0 snapshot */
#define GMAC_TS_CTRL_SNTYPSEL1	0x00010000	/* Timestamp type 1 snapshot */
#define GMAC_TS_CTRL_SNTYPSEL2	0x00020000	/* Timestamp type 2 snapshot */
#define GMAC_TS_CTRL_SNTYPSEL3	0x00030000	/* Timestamp type 3 snapshot */
#define GMAC_TS_CTRL_TSENMACADR	0x00040000	/* Timestamp mac filter en */
#define GMAC_TS_CTRL_ATSFC	0x01000000	/* Timestamp aux fifo clear */
#define GMAC_TS_CTRL_ATSEN0	0x02000000	/* Timestamp aux0 snap en */
#define GMAC_TS_CTRL_ATSEN1	0x04000000	/* Timestamp aux1 snap en */
#define GMAC_TS_CTRL_ATSEN2	0x08000000	/* Timestamp aux2 snap en */
#define GMAC_TS_CTRL_ATSEN3	0x10000000	/* Timestamp aux3 enable */
#define GMAC_SS_INC		0x704		/* Sub-second increment reg */
#define GMAC_ST_SEC		0x708		/* System time seconds */
#define GMAC_ST_NSEC		0x70C		/* System time nseconds */
#define GMAC_ST_SECUP		0x710		/* System time sec-update */
#define GMAC_ST_NSECUP		0x714		/* System time nsec-update */
#define GMAC_TS_APPEND		0x718		/* Timestamp append */
#define GMAC_TT_SEC		0x71C		/* Target time seconds */
#define GMAC_TT_NSEC		0x720		/* Target time nseconds */
#define GMAC_ST_HWSEC		0x724		/* System time high word sec */
#define GMAC_ST_TS_STAT		0x728		/* Timestamp status */
#define GMAC_PPS_CTRL		0x72C		/* PPS signal output control */
#define GMAC_AUXTS_NSEC		0x730		/* Aux timestamp counter nsec */
#define GMAC_AUXTS_SEC		0x734		/* Aux timestamp counter sec */

extern const struct stmmac_dma_ops dwmac1000_dma_ops;
#endif /* __DWMAC1000_H__ */
