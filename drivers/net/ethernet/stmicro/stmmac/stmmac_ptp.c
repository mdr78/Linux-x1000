/*******************************************************************************

  STMMAC 1588-2005 hardware accel
  Copyright(c) 2012 Intel Corporation. This code is based on IXGBE_PTP

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

  Contact Information::w
  Intel Corporation, 5200 N.E. Elam Young Parkway, Hillsboro, OR 97124-6497

*******************************************************************************/
#include "stmmac.h"
#include "dwmac1000.h"
#include <linux/export.h>
#include <linux/net_tstamp.h>

/* PPSCMD0 */
#define STMMAC_PPSCMD_NONE		0x00	/* No command */
#define STMMAC_PPSCMD_START_SINGLE	0x01	/* Start single pulse */
#define STMMAC_PPSCMD_START_PULSE	0x02	/* Start pulse train */
#define STMMAC_PPSCMD_START_CANCEL	0x03	/* Cancel start */
#define STMMAC_PPSCMD_STOP_PULSE_ATTIME	0x04	/* Stop pulse train at time */
#define STMMAC_PPSCMD_STOP_PULSE_IMMED	0x05	/* Stop pulse train immediate */
#define STMMAC_PPSCMD_STOP_CANCEL	0x06	/* Stop cancel pulse train */

#define STMMAC_PTP_OVERFLOW_CHECK_ENABLED	(u32)(1)
#define STMMAC_PTP_PPS_ENABLED			(u32)(1 << 1)
#define STMMAC_PTP_HWTS_TX_EN			(u32)(1 << 2)
#define STMMAC_PTP_HWTS_RX_EN			(u32)(1 << 3)

#ifndef NSECS_PER_SEC
#define NSECS_PER_SEC 1000000000ULL
#endif

/*
 *  Structure of STMMAC timer registers
 *
 *           GMAC_TS_HWSEC    GMAC_ST_SEC     GMAC_ST_NSEC 
 *         +--------------+ +--------------+ +---+---+------+
 *  STMMAC |      16      | |      32      | |      32      |
 *         +--------------+ +--------------+ +---+---+------+
 *
 *  The counter for the STMMAC is 80 bits
 *   - HWSEC == overflow value for ST_SEC => 130 years to overflow (optional)
 *   - ST_SEC == seconds
 *   - ST_NSEC == nanoseconds
 */

/**
 * stmmac_ptp_read - read raw cycle counter (to be used by time counter)
 * @cc - the cyclecounter structure
 *
 * this function reads the cyclecounter registers and is called by the
 * cyclecounter structure used to construct a ns counter from the
 * arbitrary fixed point registers
 */
static cycle_t stmmac_ptp_read(const struct cyclecounter *ccnt)
{
	struct stmmac_priv *priv =
		container_of(ccnt, struct stmmac_priv, ccnt);
	cycle_t stamp = 0;

	stamp = (u64)readl(priv->ioaddr + GMAC_ST_NSEC);
	stamp |= (u64)readl(priv->ioaddr + GMAC_ST_SEC) << 32;

	return stamp;
}

/**
 * stmmac_ptp_adjfreq
 * @ptp - the ptp clock structure
 * @ppb - parts per billion adjustment from base
 *
 * adjust the frequency of the ptp cycle counter by the
 * indicated ppb from the base frequency.
 */
static int stmmac_ptp_adjfreq(struct ptp_clock_info *ptp, s32 ppb)
{
	return 0;
}

/**
 * stmmac_ptp_adjtime
 * @ptp - the ptp clock structure
 * @delta - offset to adjust the cycle counter by
 *
 * adjust the timer by resetting the timecounter structure.
 */
static int stmmac_ptp_adjtime(struct ptp_clock_info *ptp, s64 delta)
{
	struct stmmac_priv *priv =
		container_of(ptp, struct stmmac_priv, ptp_caps);
	unsigned long flags;
	u64 now;

	spin_lock_irqsave(&priv->tmreg_lock, flags);

	now = timecounter_read(&priv->tcnt);
	now += delta;

	/* reset the timecounter */
	timecounter_init(&priv->tcnt,
			 &priv->ccnt,
			 now);

	spin_unlock_irqrestore(&priv->tmreg_lock, flags);
	return 0;
}

/**
 * stmmac_ptp_gettime
 * @ptp - the ptp clock structure
 * @ts - timespec structure to hold the current time value
 *
 * read the timecounter and return the correct value on ns,
 * after converting it into a struct timespec.
 */
static int stmmac_ptp_gettime(struct ptp_clock_info *ptp, struct timespec *ts)
{
	struct stmmac_priv *priv =
		container_of(ptp, struct stmmac_priv, ptp_caps);
	u64 ns;
        u32 remainder;
        unsigned long flags;

        spin_lock_irqsave(&priv->tmreg_lock, flags);
        ns = timecounter_read(&priv->tcnt);
        spin_unlock_irqrestore(&priv->tmreg_lock, flags);

        ts->tv_sec = div_u64_rem(ns, 1000000000ULL, &remainder);
        ts->tv_nsec = remainder;

	return 0;
}

/**
 * stmmac_ptp_settime
 * @ptp - the ptp clock structure
 * @ts - the timespec containing the new time for the cycle counter
 *
 * reset the timecounter to use a new base value instead of the kernel
 * wall timer value.
 */
static int stmmac_ptp_settime(struct ptp_clock_info *ptp,
			     const struct timespec *ts)
{
	struct stmmac_priv *priv =
		container_of(ptp, struct stmmac_priv, ptp_caps);
	u64 ns;
	unsigned long flags;

	ns = ts->tv_sec * 1000000000ULL;
	ns += ts->tv_nsec;

	/* reset the timecounter */
	spin_lock_irqsave(&priv->tmreg_lock, flags);
	timecounter_init(&priv->tcnt, &priv->ccnt, ns);
	spin_unlock_irqrestore(&priv->tmreg_lock, flags);

	return 0;
}

/**
 * stmmac_ptp_enable
 * @ptp - the ptp clock structure
 * @rq - the requested feature to change
 * @on - whether to enable or disable the feature
 *
 * enable (or disable) ancillary features of the phc subsystem.
 * our driver only supports the PPS feature on the X540
 */
static int stmmac_ptp_enable(struct ptp_clock_info *ptp,
			    struct ptp_clock_request *rq, int on)
{
	struct stmmac_priv *priv =
		container_of(ptp, struct stmmac_priv, ptp_caps);
	uint32_t reg = 0;

	/**
	 * When enabling PPS functionality in STMMAC we need to unmask the
	 * interrupt mask reg and enable the TSTRIG bit in the timestamp control
	 * reg
	 */
	if (rq->type == PTP_CLK_REQ_PPS) {
		if (on){
			priv->hwts |= STMMAC_PTP_PPS_ENABLED;

			/* Enable TSTRIG */
			reg = readl(priv->ioaddr + GMAC_TS_CTRL);
			reg |= GMAC_TS_CTRL_TSTRIG;
			writel(reg, priv->ioaddr + GMAC_TS_CTRL);
			wmb();

			/* Unmask interrupt */
			reg = readl(priv->ioaddr + GMAC_INT_MASK);
			printk(KERN_INFO "%s[on] read interrupt mask 0x%08x\n", __func__, reg);
			reg &= ~GMAC_INT_MASK_TSIM;
			printk(KERN_INFO "%s[on] write interrupt mask 0x%08x\n", __func__, reg);
			writel(reg, priv->ioaddr + GMAC_INT_MASK);
			wmb();

		} else {
			/* Mask interrupt */
			reg = readl(priv->ioaddr + GMAC_INT_MASK);
			printk(KERN_INFO "%s[off] read interrupt mask 0x%08x\n", __func__, reg);
			reg |= GMAC_INT_MASK_TSIM;
			printk(KERN_INFO "%s[off] write interrupt mask 0x%08x\n", __func__, reg);
			writel(reg, priv->ioaddr + GMAC_INT_MASK);
			wmb();

			/* Disable TSTRIG */
			reg = readl(priv->ioaddr + GMAC_TS_CTRL);
			reg &= ~GMAC_TS_CTRL_TSTRIG;
			writel(reg, priv->ioaddr + GMAC_TS_CTRL);
			wmb();

			priv->hwts &=
				~STMMAC_PTP_PPS_ENABLED;
		}
		return 0;
	}

	return -ENOTSUPP;
}

/**
 * stmmac_ptp_check_pps_event
 * @priv - the private priv structure
 *
 * This function is called by the interrupt routine when checking for
 * interrupts. It will check and handle a pps event.
 */
void stmmac_ptp_check_pps_event(struct stmmac_priv *priv)
{
	struct ptp_clock_event event;
	event.type = PTP_CLOCK_PPS;

	/* Make sure ptp clock is valid, and PPS event enabled */
	if (!priv->ptp_clock ||
	    !(priv->hwts & STMMAC_PTP_PPS_ENABLED)){
		return;
	}

	ptp_clock_event(priv->ptp_clock, &event);
}

#if 0
/**
 * stmmac_ptp_overflow_check - delayed work to detect SYSTIME overflow
 * @work: structure containing information about this work task
 *
 * this work function is scheduled to continue reading the timecounter
 * in order to prevent missing when the system time registers wrap
 * around. This needs to be run approximately twice a minute when no
 * PTP activity is occurring.
 */
void stmmac_ptp_overflow_check(struct stmmac_priv *priv)
{
	unsigned long elapsed_jiffies = priv->last_overflow_check - jiffies;
	struct timespec ts;

	if ((priv->hwts & STMMAC_PTP_OVERFLOW_CHECK_ENABLED) &&
	    (elapsed_jiffies >= STMMAC_OVERFLOW_PERIOD)) {
		stmmac_ptp_gettime(&priv->ptp_caps, &ts);
		priv->last_overflow_check = jiffies;
	}
}
#endif

/**
 * stmmac_ptp_tx_hwtstamp - utility function which checks for TX time stamp
 * @q_vector: structure containing interrupt and ring information
 * @skb: particular skb to send timestamp with
 *
 * if the timestamp is valid, we convert it into the timecounter ns
 * value, then store that result into the shhwtstamps structure which
 * is passed up the network stack
 */
void stmmac_ptp_tx_hwtstamp(struct stmmac_priv *priv, struct dma_desc *pdma,
			   struct sk_buff *skb)
{
	/* Sanity check input */
	if (unlikely(priv == NULL || pdma == NULL || skb == NULL)){
		return;
	}

	if(skb_shinfo(skb)->tx_flags & SKBTX_HW_TSTAMP){
		struct skb_shared_hwtstamps shhwtstamps;
		u64 ns;
		u64 regval;
		unsigned long flags;

		regval = (u64)pdma->ts_lo;
		regval |= (u64)pdma->ts_hi << 32;

		spin_lock_irqsave(&priv->tmreg_lock, flags);
		ns = timecounter_cyc2time(&priv->tcnt, regval);
		spin_unlock_irqrestore(&priv->tmreg_lock, flags);

		memset(&shhwtstamps, 0, sizeof(shhwtstamps));
		shhwtstamps.hwtstamp = ns_to_ktime(ns);
		skb_tstamp_tx(skb, &shhwtstamps);
	}
}

/**
 * stmmac_ptp_rx_hwtstamp - utility function which checks for RX time stamp
 * @q_vector: structure containing interrupt and ring information
 * @skb: particular skb to send timestamp with
 *
 * if the timestamp is valid, we convert it into the timecounter ns
 * value, then store that result into the shhwtstamps structure which
 * is passed up the network stack
 */
void stmmac_ptp_rx_hwtstamp(struct stmmac_priv *priv, struct dma_desc *pdma,
			   struct sk_buff *skb)
{
	/* Sanity check input */
	if (unlikely(priv == NULL || pdma == NULL || skb == NULL)){
		return;
	}

	if(skb_shinfo(skb)->tx_flags & SKBTX_HW_TSTAMP){
		struct skb_shared_hwtstamps shhwtstamps;
		u64 ns;
		u64 regval;
		unsigned long flags;

		regval = (u64)pdma->ts_lo;
		regval |= (u64)pdma->ts_hi << 32;

		spin_lock_irqsave(&priv->tmreg_lock, flags);
		ns = timecounter_cyc2time(&priv->tcnt, regval);
		spin_unlock_irqrestore(&priv->tmreg_lock, flags);

		memset(&shhwtstamps, 0, sizeof(shhwtstamps));
		shhwtstamps.hwtstamp = ns_to_ktime(ns);
	}
}

/**
 * ixgbe_ptp_hwtstamp_ioctl - control hardware time stamping
 * @priv: pointer to priv struct
 * @ifreq: ioctl data
 * @cmd: particular ioctl requested
 *
 * Outgoing time stamping can be enabled and disabled. Play nice and
 * disable it when requested, although it shouldn't case any overhead
 * when no packet needs it. At most one packet in the queue may be
 * marked for time stamping, otherwise it would be impossible to tell
 * for sure to which packet the hardware time stamp belongs.
 *
 * Incoming time stamping has to be configured via the hardware
 * filters. Not all combinations are supported, in particular event
 * type has to be specified. Matching the kind of event packet is
 * not supported, with the exception of "all V2 events regardless of
 * level 2 or 4".
 */
int stmmac_ptp_hwtstamp_ioctl(struct stmmac_priv *priv,
			     struct ifreq *ifr, int cmd)
{
	struct hwtstamp_config config;
//	struct stmmac_priv *priv = netdev_priv(netdev);
	u32 tsctl = 0;

	if (copy_from_user(&config, ifr->ifr_data, sizeof(config)))
		return -EFAULT;

	/* reserved for future extensions */
	if (config.flags)
		return -EINVAL;

	/* Snapshot the reg - preserve Timestamp Interrupt Trigger Enable */
	tsctl = readl(priv->ioaddr + GMAC_TS_CTRL) & GMAC_TS_CTRL_TSTRIG;

	/* TX */
	switch (config.tx_type) {
	case HWTSTAMP_TX_OFF:
		priv->hwts &= ~STMMAC_PTP_HWTS_TX_EN;
		printk(KERN_INFO "%s set TX PTP en false\n", __func__);
		break;
	case HWTSTAMP_TX_ON:
		priv->hwts |= STMMAC_PTP_HWTS_TX_EN;
		printk(KERN_INFO "%s set TX PTP en true\n", __func__);
		break;
	default:
		return -ERANGE;
	}

	/* RX */
	priv->hwts |= STMMAC_PTP_HWTS_RX_EN;

	switch (config.rx_filter) {

	
	case HWTSTAMP_FILTER_PTP_V1_L4_EVENT:
		/* 
		 * V1_L4 UDP any event 
		 * SYNC, Follow_Up, Delay_Req, Delay_Resp,
		 * Pdelay_Req, Pdelay_Resp, Pdelay_Resp_Follow_Up
		 *
		 * SNAPTYPSEL=1, TSMSTRENA=x, TSEVNTENA=0, TSVER2ENA=0, IPV4=1 
		 */
		tsctl |= GMAC_TS_CTRL_SNTYPSEL1 | GMAC_TS_CTRL_TSIPV4ENA;
		printk(KERN_INFO "%s HWTSTAMP_FILTER_PTP_V1_L4_EVENT \n", __func__);
		break;

	case HWTSTAMP_FILTER_PTP_V1_L4_DELAY_REQ:
		/*
		 * V1_L4 Master
		 * Delay_Req
		 *
		 * SNAPTYPSEL=0, TSMSTRENA=1, TSEVNTENA=1, TSVER2ENA=0, IPV4=1
		 */
		tsctl |= GMAC_TS_CTRL_SNTYPSEL0 | GMAC_TS_CTRL_TSMSTRENA;
		tsctl |= GMAC_TS_CTRL_TSEVNTENA | GMAC_TS_CTRL_TSIPV4ENA;
		printk(KERN_INFO "%s HWTSTAMP_FILTER_PTP_V1_L4_DELAY_REQ \n", __func__);
		break;

	case HWTSTAMP_FILTER_PTP_V1_L4_SYNC:
		/*
		 * V1_L4 Slave
		 * Sync
		 *
 		 * SNAPTYPSEL=0, TSMSTRENA=0, TSEVNTENA=1, TSVER2ENA=0, IPV4=1
		 */ 
		tsctl |= GMAC_TS_CTRL_SNTYPSEL0 | GMAC_TS_CTRL_TSEVNTENA;
		tsctl |= GMAC_TS_CTRL_TSIPV4ENA;
		printk(KERN_INFO "%s HWTSTAMP_FILTER_PTP_V1_L4_SYNC \n", __func__);
		break;

	case HWTSTAMP_FILTER_PTP_V2_L4_EVENT:
		/*
		 * V2_L4 UDP any event 
		 * SYNC, Follow_Up, Delay_Req, Delay_Resp,
		 * Pdelay_Req, Pdelay_Resp, Pdelay_Resp_Follow_Up
		 *
		 * SNAPTYPSEL=1, TSMSTRENA=x, TSEVNTENA=0, TSVER2ENA=1, IPV4=1 
		 */
		tsctl |= GMAC_TS_CTRL_SNTYPSEL1 | GMAC_TS_CTRL_TSVER2ENA | GMAC_TS_CTRL_TSIPV4ENA;
		printk(KERN_INFO "%s HWTSTAMP_FILTER_PTP_V2_L4_EVENT \n", __func__);
		break;

	case HWTSTAMP_FILTER_PTP_V2_L4_DELAY_REQ:
		/*
		 * V2_L4 Master
		 * Delay_Req
		 *
		 * SNAPTYPSEL=0, TSMSTRENA=1, TSEVNTENA=1, TSVER2ENA=1, IPV4=1
		 */
		tsctl |= GMAC_TS_CTRL_SNTYPSEL0 | GMAC_TS_CTRL_TSMSTRENA;
		tsctl |= GMAC_TS_CTRL_TSEVNTENA | GMAC_TS_CTRL_TSVER2ENA;
		tsctl |= GMAC_TS_CTRL_TSIPV4ENA;

		printk(KERN_INFO "%s HWTSTAMP_FILTER_PTP_V2_L4_DELAY_REQ \n", __func__);
		break;

	case HWTSTAMP_FILTER_PTP_V2_L4_SYNC:
		/*
		 * V2_L4 Slave
		 * Sync
		 *
 		 * SNAPTYPSEL=0, TSMSTRENA=0, TSEVNTENA=1, TSVER2ENA=1, IPV4=1
		 */ 
		tsctl |= GMAC_TS_CTRL_SNTYPSEL0 | GMAC_TS_CTRL_TSVER2ENA | GMAC_TS_CTRL_TSEVNTENA;
		tsctl |= GMAC_TS_CTRL_TSIPV4ENA;
		printk(KERN_INFO "%s HWTSTAMP_FILTER_PTP_V2_L4_SYNC \n", __func__);
		break;

	case HWTSTAMP_FILTER_PTP_V2_L2_EVENT:
		/*
		 * V2_L2 Ethernet any event
		 * SYNC, Follow_Up, Delay_Req, Delay_Resp,
		 * Pdelay_Req, Pdelay_Resp, Pdelay_Resp_Follow_Up
		 *
		 * SNAPTYPSEL=1, TSMSTRENA=x, TSEVNTENA=0, TSVER2ENA=1,TSIPENA=1
		 * TSIPENA=1
		 */
		tsctl |= GMAC_TS_CTRL_SNTYPSEL1 | GMAC_TS_CTRL_TSVER2ENA | GMAC_TS_CTRL_TSIPENA;
		printk(KERN_INFO "%s HWTSTAMP_FILTER_PTP_V2_L2_EVENT \n", __func__);
		break;

	case HWTSTAMP_FILTER_PTP_V2_L2_DELAY_REQ:
		/*
		 * V2_L2 Master
		 * Delay_Req
		 *
		 * SNAPTYPSEL=0, TSMSTRENA=1, TSEVNTENA=1, TSVER2ENA=1,TSIPENA=1
		 */
		tsctl |= GMAC_TS_CTRL_SNTYPSEL0 | GMAC_TS_CTRL_TSMSTRENA;
		tsctl |= GMAC_TS_CTRL_TSEVNTENA | GMAC_TS_CTRL_TSVER2ENA;
		tsctl |= GMAC_TS_CTRL_TSIPENA;
		printk(KERN_INFO "%s HWTSTAMP_FILTER_PTP_V2_L2_DELAY_REQ \n", __func__);
		break;
	
	case HWTSTAMP_FILTER_PTP_V2_L2_SYNC:
		/*
		 * V2_L2 Slave
		 * Sync
		 *
 		 * SNAPTYPSEL=0, TSMSTRENA=0, TSEVNTENA=1, TSVER2ENA=1,
 		 * TSIPENA=1 
		 */
		tsctl |= GMAC_TS_CTRL_SNTYPSEL0 | GMAC_TS_CTRL_TSVER2ENA;
		tsctl |= GMAC_TS_CTRL_TSEVNTENA | GMAC_TS_CTRL_TSIPENA;
		printk(KERN_INFO "%s HWTSTAMP_FILTER_PTP_V2_L2_SYNC \n", __func__);
		break;
	
	case HWTSTAMP_FILTER_PTP_V2_EVENT:
		/*
		 * V2_L2 Ethernet any event
		 * SYNC, Follow_Up, Delay_Req, Delay_Resp,
		 * Pdelay_Req, Pdelay_Resp, Pdelay_Resp_Follow_Up
		 *
		 * SNAPTYPSEL=1, TSMSTRENA=x, TSEVNTENA=0, TSVER2ENA=1
		 * TSIPENA=1, TSIPV4ENA=1
		 */
		tsctl |= GMAC_TS_CTRL_SNTYPSEL1 | GMAC_TS_CTRL_TSVER2ENA;
		tsctl |= GMAC_TS_CTRL_TSIPENA | GMAC_TS_CTRL_TSIPV4ENA;
		printk(KERN_INFO "%s HWTSTAMP_FILTER_PTP_V2_EVENT \n", __func__);
		break;

	case HWTSTAMP_FILTER_PTP_V2_DELAY_REQ:
		/*
		 * V2_L2_L4 Master 
		 * Delay_Req
		 *
		 * SNAPTYPSEL=0, TSMSTRENA=1, TSEVNTENA=1,
		 * TSVER2ENA=1,TSIPENA=1, TSIPV4ENA=1
		 */
		tsctl |= GMAC_TS_CTRL_SNTYPSEL0 | GMAC_TS_CTRL_TSMSTRENA;
		tsctl |= GMAC_TS_CTRL_TSEVNTENA | GMAC_TS_CTRL_TSVER2ENA;
		tsctl |= GMAC_TS_CTRL_TSIPENA | GMAC_TS_CTRL_TSIPV4ENA;
		printk(KERN_INFO "%s HWTSTAMP_FILTER_PTP_V2_DELAY_REQ \n", __func__);
		break;

	case HWTSTAMP_FILTER_PTP_V2_SYNC:
		/* 
		 * V2_L2_L4 Slave
		 * Sync
		 *
 		 * SNAPTYPSEL=0, TSMSTRENA=0, TSEVNTENA=1, TSVER2ENA=1,
 		 * TSIPENA=1, TSIPV4ENA=1 
		 */
		tsctl |= GMAC_TS_CTRL_SNTYPSEL0 | GMAC_TS_CTRL_TSVER2ENA;
		tsctl |= GMAC_TS_CTRL_TSEVNTENA | GMAC_TS_CTRL_TSIPENA;
		tsctl |= GMAC_TS_CTRL_TSIPV4ENA;
		printk(KERN_INFO "%s HWTSTAMP_FILTER_PTP_V2_SYNC \n", __func__);
		break;

	case HWTSTAMP_FILTER_ALL:
		/*
		 * V2_L2_L4 Ethernet any event
		 *
		 * SYNC, Follow_Up, Delay_Req, Delay_Resp,
		 * Pdelay_Req, Pdelay_Resp, Pdelay_Resp_Follow_Up
		 *
		 * GMAC_TS_CTRL_TSENALL
		 */
		tsctl |= GMAC_TS_CTRL_TSENALL;
		printk(KERN_INFO "%s HWTSTAMP_FILTER_ALL \n", __func__);
		break;

	case HWTSTAMP_FILTER_NONE:
		printk(KERN_INFO "%s HWTSTAMP_FILTER_NONE  \n", __func__);
		priv->hwts &= ~STMMAC_PTP_HWTS_RX_EN;
		break;

	default:
		printk(KERN_INFO "%s error ioctl rx type %d \n", __func__, config.rx_filter);
		/* bad/unknown parameter */
		return -ERANGE;
	}

	/* Set the TS CTRL reg */
	tsctl |= GMAC_TS_CTRL_TSENA;
	writel(tsctl, priv->ioaddr + GMAC_TS_CTRL);
	wmb();

	printk(KERN_INFO "%s sync ts_ctl @ value 0x%08x\n", __func__, tsctl);

	return copy_to_user(ifr->ifr_data, &config, sizeof(config)) ?
		-EFAULT : 0;
}


#define DEFAULT_PTP_CLK 50

/**
 * stmmac_ptp_init_timestamp - initialise the PTP clock
 * @priv - pointer to the priv structure
 *
 * This function initialises the PTP clock consistent with the method spelled
 * out in the Snopsys documentation
 */
static void stmmac_ptp_init_timestamp(struct stmmac_priv *priv)
{
	unsigned long tsctl = GMAC_TS_CTRL_TSENA, flags = 0;
	uint8_t ssinc = CONFIG_STMMAC_PTP_CLK_MHZ;

	/* Enable TS */	
	writel(tsctl, priv->ioaddr + GMAC_TS_CTRL);
	wmb();

	/* Write SSINC - number of nano seconds to increment on each clock */
	if(ssinc == 0){
		ssinc = DEFAULT_PTP_CLK;
	}
	ssinc = 1000/ssinc;
	writel(ssinc, priv->ioaddr + GMAC_SS_INC);
	wmb();

	printk(KERN_INFO "%s setting PTP_CLK to 0x%02x\n", __func__, ssinc);

	/* Reset system time registers to zero */
	writel(0x00000000, priv->ioaddr + GMAC_ST_SEC);
	writel(0x00000000, priv->ioaddr + GMAC_ST_NSEC);
	wmb();

	/* Set TSINT to latch values in ST_SEC and ST_NSEC */
	tsctl |= GMAC_TS_CTRL_TSINT;
	writel(tsctl, priv->ioaddr + GMAC_TS_CTRL);
	wmb();
	printk(KERN_INFO "%s tsctl == 0x%08lx (TSINIT | TSENA)\n", __func__, tsctl);

	spin_lock_irqsave(&priv->tmreg_lock, flags);

	/* Init timecounter */
	memset(&priv->ccnt, 0, sizeof(priv->ccnt));
	priv->ccnt.read = stmmac_ptp_read;
	priv->ccnt.mask = CLOCKSOURCE_MASK(64);
	priv->ccnt.mult = 1;
	priv->ccnt.shift = 0;

	/* reset the ns time counter */
	timecounter_init(&priv->tcnt, &priv->ccnt,
			 ktime_to_ns(ktime_get_real()));

	spin_unlock_irqrestore(&priv->tmreg_lock, flags);
}

/**
 * stmmac_ptp_init
 * @priv - the stmmac private priv structure
 *
 * This function performs the required steps for enabling ptp
 * support. If ptp support has already been loaded it simply calls the
 * cyclecounter init routine and exits.
 */
void stmmac_ptp_init(struct net_device *ndev, struct device * pdev)
{
	struct stmmac_priv *priv = netdev_priv(ndev);

	/* Ensure the timestamp interrupt is masked */
	writel(GMAC_INT_MASK_TSIM, priv->ioaddr + GMAC_INT_MASK);

	/* Fill out PTP callback contents */
	snprintf(priv->ptp_caps.name, 16, "%pm", ndev->dev_addr);
	priv->ptp_caps.owner = THIS_MODULE;
	priv->ptp_caps.max_adj = 0;	/* Cannot be adjusted */
	priv->ptp_caps.n_alarm = 0;
	priv->ptp_caps.n_ext_ts = 0;
	priv->ptp_caps.n_per_out = 0;
	priv->ptp_caps.pps = 0;
	priv->ptp_caps.adjfreq = stmmac_ptp_adjfreq;
	priv->ptp_caps.adjtime = stmmac_ptp_adjtime;
	priv->ptp_caps.gettime = stmmac_ptp_gettime;
	priv->ptp_caps.settime = stmmac_ptp_settime;
	priv->ptp_caps.enable = stmmac_ptp_enable;

	spin_lock_init(&priv->tmreg_lock);

	stmmac_ptp_init_timestamp(priv);

	/* Init to default state */
//	priv->hwts = STMMAC_PTP_OVERFLOW_CHECK_ENABLED;

	priv->ptp_clock = ptp_clock_register(&priv->ptp_caps, pdev);
	if (IS_ERR(priv->ptp_clock)) {
		priv->ptp_clock = NULL;
		printk(KERN_ERR "%s ptp_clock_reg failed!\n", __func__);
	} else {
		printk(KERN_INFO "%s ptp_clock_reg success!\n", __func__);
	}

	return;
}

/**
 * stmmac_ptp_remove - disable ptp device and stop the overflow check
 * @priv: pointer to priv struct
 *
 * this function stops the ptp support, and cancels the delayed work.
 */
void stmmac_ptp_remove(struct stmmac_priv *priv)
{
	/* Ensure the timestamp interrupt is masked */
	writel(GMAC_INT_MASK_TSIM, priv->ioaddr + GMAC_INT_MASK);

	/* stop the overflow check task */
//	priv->hwts &= ~STMMAC_PTP_OVERFLOW_CHECK_ENABLED;

	if (priv->ptp_clock != NULL) {
		ptp_clock_unregister(priv->ptp_clock);
		priv->ptp_clock = NULL;
		printk(KERN_INFO "%s removed ptp_clock\n", __func__);
	}
}

