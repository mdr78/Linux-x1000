/*
 *Copyright (C) 2011 LAPIS Semiconductor Co., Ltd.
 *Copyright (C) 2014 Intel Corporation.
 *
 *This program is free software; you can redistribute it and/or modify
 *it under the terms of the GNU General Public License as published by
 *the Free Software Foundation; version 2 of the License.
 *
 *This program is distributed in the hope that it will be useful,
 *but WITHOUT ANY WARRANTY; without even the implied warranty of
 *MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *GNU General Public License for more details.
 *
 *You should have received a copy of the GNU General Public License
 *along with this program; if not, write to the Free Software
 *Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307, USA.
 */
#if defined(CONFIG_SERIAL_QUARK_UART_CONSOLE) && defined(CONFIG_MAGIC_SYSRQ)
#define SUPPORT_SYSRQ
#endif
#if defined(CONFIG_INTEL_QUARK_X1000_SOC)
#include <asm/qrk.h>
#endif
#include <linux/kernel.h>
#include <linux/serial_reg.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/console.h>
#include <linux/serial_core.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/dmi.h>
#include <linux/nmi.h>
#include <linux/delay.h>
#include <linux/intel_mid_dma.h>
#include <linux/debugfs.h>
#include <linux/dmaengine.h>
#include <linux/spinlock.h>
#include <linux/wait.h>
#include <linux/workqueue.h>

enum {
	QUARK_UART_HANDLED_RX_INT_SHIFT,
	QUARK_UART_HANDLED_TX_INT_SHIFT,
	QUARK_UART_HANDLED_RX_ERR_INT_SHIFT,
	QUARK_UART_HANDLED_RX_TRG_INT_SHIFT,
	QUARK_UART_HANDLED_MS_INT_SHIFT,
	QUARK_UART_HANDLED_LS_INT_SHIFT,
};

enum {
	QUARK_UART_8LINE,
	QUARK_UART_2LINE,
};

#define INFO(_max_chan, _ch_base, _block_size, _pimr_mask, _is_quark) \
	((kernel_ulong_t)&(struct intel_mid_dma_probe_info) {	\
		.max_chan = (_max_chan),			\
		.ch_base = (_ch_base),				\
		.block_size = (_block_size),			\
		.pimr_mask = (_pimr_mask),			\
		.is_quark = (_is_quark),			\
	})

#define QUARK_UART_DRIVER_DEVICE	"ttyQRK"
#define QUARK_UART_FIFO_LEN		16
#define QUARK_UART_MAXBURST		0x20
#define QUARK_FIFO_FASTER_LEN		(QUARK_UART_MAXBURST/2)
/* Set the max number of UART port
 * Intel QUARK X1000 : 2 port
*/
#define QUARK_UART_NR 2

/* Set the AHB address for DMA transfers */
#define QUARK_UART_AHB_REG_BASE		0xFFFFF000

#define QUARK_UART_HANDLED_RX_INT	(1<<((QUARK_UART_HANDLED_RX_INT_SHIFT)<<1))
#define QUARK_UART_HANDLED_TX_INT	(1<<((QUARK_UART_HANDLED_TX_INT_SHIFT)<<1))
#define QUARK_UART_HANDLED_RX_ERR_INT	(1<<((\
					QUARK_UART_HANDLED_RX_ERR_INT_SHIFT)<<1))
#define QUARK_UART_HANDLED_RX_TRG_INT	(1<<((\
					QUARK_UART_HANDLED_RX_TRG_INT_SHIFT)<<1))
#define QUARK_UART_HANDLED_MS_INT	(1<<((QUARK_UART_HANDLED_MS_INT_SHIFT)<<1))

#define QUARK_UART_HANDLED_LS_INT	(1<<((QUARK_UART_HANDLED_LS_INT_SHIFT)<<1))

#define QUARK_UART_RBR			0x00
#define QUARK_UART_THR			0x00

#define QUARK_UART_IER_MASK		(QUARK_UART_IER_ERBFI|QUARK_UART_IER_ETBEI|\
					QUARK_UART_IER_ELSI|QUARK_UART_IER_EDSSI)
#define QUARK_UART_IER_ERBFI		0x00000001
#define QUARK_UART_IER_ETBEI		0x00000002
#define QUARK_UART_IER_ELSI		0x00000004
#define QUARK_UART_IER_EDSSI		0x00000008

#define QUARK_UART_IIR_IP		0x00000001
#define QUARK_UART_IIR_IID		0x00000006
#define QUARK_UART_IIR_MSI		0x00000000
#define QUARK_UART_IIR_TRI		0x00000002
#define QUARK_UART_IIR_RRI		0x00000004
#define QUARK_UART_IIR_REI		0x00000006
#define QUARK_UART_IIR_TOI		0x00000008
#define QUARK_UART_IIR_FIFO256		0x00000020
#define QUARK_UART_IIR_FIFO64		QUARK_UART_IIR_FIFO256
#define QUARK_UART_IIR_FE		0x000000C0

#define QUARK_UART_FCR_FIFOE		0x00000001
#define QUARK_UART_FCR_RFR		0x00000002
#define QUARK_UART_FCR_TFR		0x00000004
#define QUARK_UART_FCR_DMS		0x00000008
#define QUARK_UART_FCR_FIFO256		0x00000020
#define QUARK_UART_FCR_RFTL		0x000000C0

#define QUARK_UART_FCR_RFTL1		0x00000000
#define QUARK_UART_FCR_RFTL64		0x00000040
#define QUARK_UART_FCR_RFTL128		0x00000080
#define QUARK_UART_FCR_RFTL224		0x000000C0
#define QUARK_UART_FCR_RFTL16		QUARK_UART_FCR_RFTL64
#define QUARK_UART_FCR_RFTL32		QUARK_UART_FCR_RFTL128
#define QUARK_UART_FCR_RFTL56		QUARK_UART_FCR_RFTL224
#define QUARK_UART_FCR_RFTL4		QUARK_UART_FCR_RFTL64
#define QUARK_UART_FCR_RFTL8		QUARK_UART_FCR_RFTL128
#define QUARK_UART_FCR_RFTL14		QUARK_UART_FCR_RFTL224
#define QUARK_UART_FCR_RFTL_SHIFT	6

#define QUARK_UART_LCR_WLS		0x00000003
#define QUARK_UART_LCR_STB		0x00000004
#define QUARK_UART_LCR_PEN		0x00000008
#define QUARK_UART_LCR_EPS		0x00000010
#define QUARK_UART_LCR_SP		0x00000020
#define QUARK_UART_LCR_SB		0x00000040
#define QUARK_UART_LCR_DLAB		0x00000080
#define QUARK_UART_LCR_NP		0x00000000
#define QUARK_UART_LCR_OP		QUARK_UART_LCR_PEN
#define QUARK_UART_LCR_EP		(QUARK_UART_LCR_PEN | QUARK_UART_LCR_EPS)
#define QUARK_UART_LCR_1P		(QUARK_UART_LCR_PEN | QUARK_UART_LCR_SP)
#define QUARK_UART_LCR_0P		(QUARK_UART_LCR_PEN | QUARK_UART_LCR_EPS |\
					QUARK_UART_LCR_SP)

#define QUARK_UART_LCR_5BIT		0x00000000
#define QUARK_UART_LCR_6BIT		0x00000001
#define QUARK_UART_LCR_7BIT		0x00000002
#define QUARK_UART_LCR_8BIT		0x00000003

#define QUARK_UART_MCR_DTR		0x00000001
#define QUARK_UART_MCR_RTS		0x00000002
#define QUARK_UART_MCR_OUT		0x0000000C
#define QUARK_UART_MCR_LOOP		0x00000010
#define QUARK_UART_MCR_AFE		0x00000020

#define QUARK_UART_LSR_DR		0x00000001
#define QUARK_UART_LSR_ERR		(1<<7)

#define QUARK_UART_MSR_DCTS		0x00000001
#define QUARK_UART_MSR_DDSR		0x00000002
#define QUARK_UART_MSR_TERI		0x00000004
#define QUARK_UART_MSR_DDCD		0x00000008
#define QUARK_UART_MSR_CTS		0x00000010
#define QUARK_UART_MSR_DSR		0x00000020
#define QUARK_UART_MSR_RI		0x00000040
#define QUARK_UART_MSR_DCD		0x00000080
#define QUARK_UART_MSR_DELTA		(QUARK_UART_MSR_DCTS | QUARK_UART_MSR_DDSR |\
					QUARK_UART_MSR_TERI | QUARK_UART_MSR_DDCD)

#define QUARK_UART_DLL			0x00
#define QUARK_UART_DLM			0x01

#define QUARK_UART_BRCSR		0x0E

#define QUARK_UART_IID_RLS		(QUARK_UART_IIR_REI)
#define QUARK_UART_IID_RDR		(QUARK_UART_IIR_RRI)
#define QUARK_UART_IID_RDR_TO		(QUARK_UART_IIR_RRI | QUARK_UART_IIR_TOI)
#define QUARK_UART_IID_THRE		(QUARK_UART_IIR_TRI)
#define QUARK_UART_IID_MS		(QUARK_UART_IIR_MSI)

#define QUARK_UART_HAL_PARITY_NONE	(QUARK_UART_LCR_NP)
#define QUARK_UART_HAL_PARITY_ODD		(QUARK_UART_LCR_OP)
#define QUARK_UART_HAL_PARITY_EVEN	(QUARK_UART_LCR_EP)
#define QUARK_UART_HAL_PARITY_FIX1	(QUARK_UART_LCR_1P)
#define QUARK_UART_HAL_PARITY_FIX0	(QUARK_UART_LCR_0P)
#define QUARK_UART_HAL_5BIT		(QUARK_UART_LCR_5BIT)
#define QUARK_UART_HAL_6BIT		(QUARK_UART_LCR_6BIT)
#define QUARK_UART_HAL_7BIT		(QUARK_UART_LCR_7BIT)
#define QUARK_UART_HAL_8BIT		(QUARK_UART_LCR_8BIT)
#define QUARK_UART_HAL_STB1		0
#define QUARK_UART_HAL_STB2		(QUARK_UART_LCR_STB)

#define QUARK_UART_HAL_CLR_TX_FIFO	(QUARK_UART_FCR_TFR)
#define QUARK_UART_HAL_CLR_RX_FIFO	(QUARK_UART_FCR_RFR)
#define QUARK_UART_HAL_CLR_ALL_FIFO	(QUARK_UART_HAL_CLR_TX_FIFO | \
					QUARK_UART_HAL_CLR_RX_FIFO)

#define QUARK_UART_HAL_DMA_MODE0	0
#define QUARK_UART_HAL_FIFO_DIS		0
#define QUARK_UART_HAL_FIFO16		(QUARK_UART_FCR_FIFOE)
#define QUARK_UART_HAL_FIFO256		(QUARK_UART_FCR_FIFOE | \
					QUARK_UART_FCR_FIFO256)
#define QUARK_UART_HAL_FIFO64		(QUARK_UART_HAL_FIFO256)
#define QUARK_UART_HAL_TRIGGER1		(QUARK_UART_FCR_RFTL1)
#define QUARK_UART_HAL_TRIGGER64	(QUARK_UART_FCR_RFTL64)
#define QUARK_UART_HAL_TRIGGER128	(QUARK_UART_FCR_RFTL128)
#define QUARK_UART_HAL_TRIGGER224	(QUARK_UART_FCR_RFTL224)
#define QUARK_UART_HAL_TRIGGER16	(QUARK_UART_FCR_RFTL16)
#define QUARK_UART_HAL_TRIGGER32	(QUARK_UART_FCR_RFTL32)
#define QUARK_UART_HAL_TRIGGER56	(QUARK_UART_FCR_RFTL56)
#define QUARK_UART_HAL_TRIGGER4		(QUARK_UART_FCR_RFTL4)
#define QUARK_UART_HAL_TRIGGER8		(QUARK_UART_FCR_RFTL8)
#define QUARK_UART_HAL_TRIGGER14	(QUARK_UART_FCR_RFTL14)
#define QUARK_UART_HAL_TRIGGER_L	(QUARK_UART_FCR_RFTL64)
#define QUARK_UART_HAL_TRIGGER_M	(QUARK_UART_FCR_RFTL128)
#define QUARK_UART_HAL_TRIGGER_H	(QUARK_UART_FCR_RFTL224)

#define QUARK_UART_HAL_RX_INT		(QUARK_UART_IER_ERBFI)
#define QUARK_UART_HAL_TX_INT		(QUARK_UART_IER_ETBEI)
#define QUARK_UART_HAL_RX_ERR_INT	(QUARK_UART_IER_ELSI)
#define QUARK_UART_HAL_MS_INT		(QUARK_UART_IER_EDSSI)
#define QUARK_UART_HAL_ALL_INT		(QUARK_UART_IER_MASK)

#define QUARK_UART_HAL_DTR		(QUARK_UART_MCR_DTR)
#define QUARK_UART_HAL_RTS		(QUARK_UART_MCR_RTS)
#define QUARK_UART_HAL_OUT		(QUARK_UART_MCR_OUT)
#define QUARK_UART_HAL_LOOP		(QUARK_UART_MCR_LOOP)
#define QUARK_UART_HAL_AFE		(QUARK_UART_MCR_AFE)

#define PCI_VENDOR_ID_ROHM		0x10DB

#define BOTH_EMPTY (UART_LSR_TEMT | UART_LSR_THRE)

#define DEFAULT_UARTCLK			44236800	/* 2.76 MHz * 16 */

/**
 * struct inel_qrk_uart_buffer
 *
 * Descriptor for a UART bufer
 */
struct quark_uart_buffer {
	dma_addr_t	dma_addr;
	unsigned char	*buf;
	u32		offs;
	int		size;
};

struct x1000_port {
	struct uart_port port;
	int port_type;
	void __iomem *membase;
	resource_size_t mapbase;
	struct pci_dev *pdev;
	int fifo_size;
	unsigned int uartclk;
	int start_tx;
	int start_rx;
	int tx_empty;
	int trigger;
	int trigger_level;
	unsigned int dmsr;
	unsigned int fcr;
	unsigned int mcr;
	unsigned int ier;
	bool use_dma;
	struct dma_async_tx_descriptor	*desc_tx;
	struct dma_async_tx_descriptor	*desc_rx;
	struct dma_chan			*tx_chan;
	struct dma_chan			*rx_chan;
	struct middma_device		mid_dma;
	struct quark_uart_buffer	txbuf;
	struct quark_uart_buffer	rxbuf;
	struct intel_mid_dma_slave	dmas_rx;
	struct intel_mid_dma_slave	dmas_tx;
	struct scatterlist		*sg_tx_p;
	int				nent;
	struct scatterlist		sg_rx;
	int				tx_dma_use;
	void				*rx_buf_virt;
	dma_addr_t			rx_buf_dma;
	struct dentry	*debugfs;
	struct work_struct		work;
	int dma_tx_in_flight;
	/* protect the x1000_port private structure and io access to membase */
	spinlock_t lock;
	wait_queue_head_t 		w_queue;

};

static struct uart_driver quark_uart_driver;

/**
 * struct quark_uart_driver_data - private data structure for UART-DMA
 * @port_type:			The number of DMA channel
 * @line_no:			UART port line number (0, 1, 2...)
 */
struct quark_uart_driver_data {
	int port_type;
	int line_no;
};

/**
 * intel_qrk_dma_chan_filter
 *
 * Simple descriptor disjunct function
 */
static bool intel_qrk_dma_chan_filter(struct dma_chan * chan, void *param)
{
	return 1;
}


/**
 * serial_in
 *
 * @param up: pointer to uart descriptor
 * @param offset: register offset
 *
 * Reads a register @ offset
 */
static inline unsigned int serial_in(struct x1000_port *up, int offset)
{
	int soffset = offset << 2;

	return  (unsigned int)readb(up->membase + soffset);
}

/**
 * serial_out
 *
 * @param up: pointer to uart descriptor
 * @param offset: register offset
 *
 * Writes a register @ offset
 */
static inline void serial_out(struct x1000_port *up, int offset, int value)
{
	unsigned char val = value & 0xff;
	int soffset = offset << 2;

	writeb(val, up->membase + soffset);
}

#ifdef CONFIG_SERIAL_QUARK_UART_CONSOLE
static struct x1000_port *quark_uart_ports[QUARK_UART_NR];
#endif
static unsigned int default_baud = 115200;
static bool use_dma = true;
static const int trigger_level_256[4] = { 1, 64, 128, 224 };
static const int trigger_level_64[4] = { 1, 16, 32, 56 };
static const int trigger_level_16[4] = { 1, 4, 8, 14 };
static const int trigger_level_1[4] = { 1, 1, 1, 1 };

#ifdef CONFIG_DEBUG_FS

#define QUARK_REGS_BUFSIZE	1024

static ssize_t port_show_regs(struct file *file, char __user *user_buf,
				size_t count, loff_t *ppos)
{
	struct x1000_port *priv = file->private_data;
	char *buf;
	u32 len = 0;
	ssize_t ret;
	unsigned char lcr;

	buf = kzalloc(QUARK_REGS_BUFSIZE, GFP_KERNEL);
	if (!buf)
		return 0;

	len += snprintf(buf + len, QUARK_REGS_BUFSIZE - len,
			"QUARK X1000 port[%d] regs:\n", priv->port.line);

	len += snprintf(buf + len, QUARK_REGS_BUFSIZE - len,
			"=================================\n");
	len += snprintf(buf + len, QUARK_REGS_BUFSIZE - len,
			"IER: \t0x%02x\n", serial_in(priv, UART_IER));
	len += snprintf(buf + len, QUARK_REGS_BUFSIZE - len,
			"IIR: \t0x%02x\n", serial_in(priv, UART_IIR));
	len += snprintf(buf + len, QUARK_REGS_BUFSIZE - len,
			"LCR: \t0x%02x\n", serial_in(priv, UART_LCR));
	len += snprintf(buf + len, QUARK_REGS_BUFSIZE - len,
			"MCR: \t0x%02x\n", serial_in(priv, UART_MCR));
	len += snprintf(buf + len, QUARK_REGS_BUFSIZE - len,
			"LSR: \t0x%02x\n", serial_in(priv, UART_LSR));
	len += snprintf(buf + len, QUARK_REGS_BUFSIZE - len,
			"MSR: \t0x%02x\n", serial_in(priv, UART_MSR));
	len += snprintf(buf + len, QUARK_REGS_BUFSIZE - len,
			"BRCSR: \t0x%02x\n",
			serial_in(priv, QUARK_UART_BRCSR));

	lcr = serial_in(priv, UART_LCR);
	serial_out(priv, UART_LCR, QUARK_UART_LCR_DLAB);
	len += snprintf(buf + len, QUARK_REGS_BUFSIZE - len,
			"DLL: \t0x%02x\n", serial_in(priv, UART_DLL));
	len += snprintf(buf + len, QUARK_REGS_BUFSIZE - len,
			"DLM: \t0x%02x\n", serial_in(priv, UART_DLM));
	serial_out(priv, UART_LCR, lcr);

	if (len > QUARK_REGS_BUFSIZE)
		len = QUARK_REGS_BUFSIZE;

	ret =  simple_read_from_buffer(user_buf, count, ppos, buf, len);
	kfree(buf);
	return ret;
}


static const struct file_operations port_regs_ops = {
	.owner		= THIS_MODULE,
	.open		= simple_open,
	.read		= port_show_regs,
	.llseek		= default_llseek,
};
#endif	/* CONFIG_DEBUG_FS */

/* Return UART clock, checking for board specific clocks. */
static unsigned int quark_uart_get_uartclk(void)
{
	return DEFAULT_UARTCLK;
}

static void quark_uart_hal_enable_interrupt(struct x1000_port *priv,
					  unsigned int flag)
{
	u8 ier = serial_in(priv, UART_IER);
	ier |= flag & QUARK_UART_IER_MASK;
	priv->ier = ier;
	serial_out(priv, UART_IER, ier);
}

static void quark_uart_hal_disable_interrupt(struct x1000_port *priv,
					   unsigned int flag)
{
	u8 ier = serial_in(priv, UART_IER);
	ier &= ~(flag & QUARK_UART_IER_MASK);
	priv->ier = ier;
	serial_out(priv, UART_IER, ier);
}

static int quark_uart_hal_set_line(struct x1000_port *priv, unsigned int baud,
				 unsigned int parity, unsigned int bits,
				 unsigned int stb)
{
	unsigned int dll, dlm, lcr;
	int div;

	div = DIV_ROUND_CLOSEST(priv->uartclk / 16, baud);
	if (div < 0 || USHRT_MAX <= div) {
		dev_err(priv->port.dev, "Invalid Baud(div=0x%x)\n", div);
		return -EINVAL;
	}

	dll = (unsigned int)div & 0x00FFU;
	dlm = ((unsigned int)div >> 8) & 0x00FFU;

	if (parity & ~(QUARK_UART_LCR_PEN | QUARK_UART_LCR_EPS | QUARK_UART_LCR_SP)) {
		dev_err(priv->port.dev, "Invalid parity(0x%x)\n", parity);
		return -EINVAL;
	}

	if (bits & ~QUARK_UART_LCR_WLS) {
		dev_err(priv->port.dev, "Invalid bits(0x%x)\n", bits);
		return -EINVAL;
	}

	if (stb & ~QUARK_UART_LCR_STB) {
		dev_err(priv->port.dev, "Invalid STB(0x%x)\n", stb);
		return -EINVAL;
	}

	lcr = parity;
	lcr |= bits;
	lcr |= stb;

	serial_out(priv, UART_LCR, QUARK_UART_LCR_DLAB);
	serial_out(priv, QUARK_UART_DLL, dll);
	serial_out(priv, QUARK_UART_DLM, dlm);
	serial_out(priv, UART_LCR, lcr);

	return 0;
}

static int quark_uart_hal_fifo_reset(struct x1000_port *priv,
				    unsigned int flag)
{
	if (flag & ~(QUARK_UART_FCR_TFR | QUARK_UART_FCR_RFR)) {
		dev_err(priv->port.dev, "%s:Invalid flag(0x%x)\n",
			__func__, flag);
		return -EINVAL;
	}

	serial_out(priv, UART_FCR, QUARK_UART_FCR_FIFOE | priv->fcr);
	serial_out(priv,
		 UART_FCR, QUARK_UART_FCR_FIFOE | priv->fcr | flag);
	serial_out(priv, UART_FCR, priv->fcr);

	return 0;
}

static int quark_uart_hal_set_fifo(struct x1000_port *priv,
				 unsigned int dmamode,
				 unsigned int fifo_size, unsigned int trigger)
{
	u8 fcr;

	if (dmamode & ~QUARK_UART_FCR_DMS) {
		dev_err(priv->port.dev, "%s:Invalid DMA Mode(0x%x)\n",
			__func__, dmamode);
		return -EINVAL;
	}

	if (fifo_size & ~(QUARK_UART_FCR_FIFOE | QUARK_UART_FCR_FIFO256)) {
		dev_err(priv->port.dev, "%s:Invalid FIFO SIZE(0x%x)\n",
			__func__, fifo_size);
		return -EINVAL;
	}

	if (trigger & ~QUARK_UART_FCR_RFTL) {
		dev_err(priv->port.dev, "%s:Invalid TRIGGER(0x%x)\n",
			__func__, trigger);
		return -EINVAL;
	}

	switch (priv->fifo_size) {
	case 256:
		priv->trigger_level =
		    trigger_level_256[trigger >> QUARK_UART_FCR_RFTL_SHIFT];
		break;
	case 64:
		priv->trigger_level =
		    trigger_level_64[trigger >> QUARK_UART_FCR_RFTL_SHIFT];
		break;
	case 16:
		priv->trigger_level =
		    trigger_level_16[trigger >> QUARK_UART_FCR_RFTL_SHIFT];
		break;
	default:
		priv->trigger_level =
		    trigger_level_1[trigger >> QUARK_UART_FCR_RFTL_SHIFT];
		break;
	}
	fcr =
	    dmamode | fifo_size | trigger | QUARK_UART_FCR_RFR | QUARK_UART_FCR_TFR;

	serial_out(priv, UART_FCR, QUARK_UART_FCR_FIFOE);
	serial_out(priv,
		 UART_FCR, QUARK_UART_FCR_FIFOE | QUARK_UART_FCR_RFR | QUARK_UART_FCR_TFR);
	serial_out(priv, UART_FCR, fcr);
	priv->fcr = fcr;

	return 0;
}

static u8 quark_uart_hal_get_modem(struct x1000_port *priv)
{
	unsigned int msr = serial_in(priv, UART_MSR);
	priv->dmsr = msr & QUARK_UART_MSR_DELTA;
	return (u8)msr;
}

static void quark_uart_hal_write(struct x1000_port *priv,
			      const unsigned char *buf, int tx_size)
{
	int i;
	unsigned int thr;

	for (i = 0; i < tx_size;) {
		thr = buf[i++];
		serial_out(priv, QUARK_UART_THR, thr);
	}
}

static int quark_uart_hal_read(struct x1000_port *priv, unsigned char *buf,
			     int rx_size)
{
	int i;
	u8 rbr, lsr;
	struct uart_port *port = &priv->port;

	lsr = serial_in(priv, UART_LSR);
	for (i = 0, lsr = serial_in(priv, UART_LSR);
	     i < rx_size && lsr & (UART_LSR_DR | UART_LSR_BI);
	     lsr = serial_in(priv, UART_LSR)) {
		rbr = serial_in(priv, QUARK_UART_RBR);

		if (lsr & UART_LSR_BI) {
			port->icount.brk++;
			if (uart_handle_break(port))
				continue;
		}
#ifdef SUPPORT_SYSRQ
		if (port->sysrq) {
			if (uart_handle_sysrq_char(port, rbr))
				continue;
		}
#endif

		buf[i++] = rbr;
	}
	return i;
}

static unsigned char quark_uart_hal_get_iid(struct x1000_port *priv)
{
	return serial_in(priv, UART_IIR) &\
		      (QUARK_UART_IIR_IID | QUARK_UART_IIR_TOI | QUARK_UART_IIR_IP);
}

static u8 quark_uart_hal_get_line_status(struct x1000_port *priv)
{
	return serial_in(priv, UART_LSR);
}

static void quark_uart_hal_set_break(struct x1000_port *priv, int on)
{
	unsigned int lcr;

	lcr = serial_in(priv, UART_LCR);
	if (on)
		lcr |= QUARK_UART_LCR_SB;
	else
		lcr &= ~QUARK_UART_LCR_SB;

	serial_out(priv, UART_LCR, lcr);
}

static int push_rx(struct x1000_port *priv, const unsigned char *buf,
		   int size)
{
	struct uart_port *port = &priv->port;
	struct tty_struct *tty = tty_port_tty_get(&port->state->port);

	tty_insert_flip_string(tty, buf, size);
	tty_flip_buffer_push(tty);

	return 0;
}

static int pop_tx_x(struct x1000_port *priv, unsigned char *buf)
{
	int ret = 0;
	struct uart_port *port = &priv->port;

	if (port->x_char) {
		dev_dbg(priv->port.dev, "%s:X character send %02x (%lu)\n",
			__func__, port->x_char, jiffies);
		buf[0] = port->x_char;
		port->x_char = 0;
		ret = 1;
	}

	return ret;
}

static int dma_push_rx(struct x1000_port *priv, int size)
{
	int room;
	struct uart_port *port = &priv->port;
	struct tty_struct *tty = tty_port_tty_get(&port->state->port);

	room = tty_buffer_request_room(tty, size);

	if (room < size)
		dev_warn(port->dev, "Rx overrun: dropping %u bytes\n",
			 size - room);
	if (!room)
		return 0;

	tty_insert_flip_string(tty, sg_virt(&priv->sg_rx), size);

	port->icount.rx += room;

	return room;
}

static void quark_dma_rx_complete(void *arg)
{
	struct x1000_port *priv = arg;
	struct uart_port *port = &priv->port;
	struct tty_struct *tty = tty_port_tty_get(&port->state->port);
	int count;

	dma_sync_sg_for_cpu(port->dev, &priv->sg_rx, 1, DMA_FROM_DEVICE);
	count = dma_push_rx(priv, priv->trigger_level);
	if (count)
		tty_flip_buffer_push(tty);
	async_tx_ack(priv->desc_rx);
	quark_uart_hal_enable_interrupt(priv, QUARK_UART_HAL_RX_INT |
					    QUARK_UART_HAL_RX_ERR_INT);
}

static void quark_dma_tx_complete(void *arg)
{
	struct x1000_port *priv = arg;
	struct uart_port *port = &priv->port;
	struct circ_buf *xmit = &port->state->xmit;
	struct scatterlist *sg = priv->sg_tx_p;
	int i;
	unsigned long flags;

	spin_lock_irqsave(&priv->lock, flags);

	for (i = 0; i < priv->nent; i++, sg++) {
		xmit->tail += sg_dma_len(sg);
		port->icount.tx += sg_dma_len(sg);
	}
	/* Make sure that xmit->head and xmit->tail are equal
	   to zero at the end of a transaction */
	if(priv->tx_dma_use == 0)
		xmit->head = xmit->tail = 0;
	else
		xmit->tail &= UART_XMIT_SIZE - 1;
	async_tx_ack(priv->desc_tx);
	dma_unmap_sg(port->dev, sg, priv->nent, DMA_TO_DEVICE);
	priv->tx_dma_use = 0;
	priv->nent = 0;
	priv->tx_empty = 1;
	kfree(priv->sg_tx_p);

	quark_uart_hal_enable_interrupt(priv, QUARK_UART_HAL_TX_INT);
	priv->dma_tx_in_flight = 0;
	wake_up(&priv->w_queue);

	spin_unlock_irqrestore(&priv->lock, flags);
}

static int pop_tx(struct x1000_port *priv, int size)
{
	int count = 0;
	struct uart_port *port = &priv->port;
	struct circ_buf *xmit = &port->state->xmit;

	if (uart_tx_stopped(port) || uart_circ_empty(xmit) || count >= size)
		goto pop_tx_end;

	do {
		int cnt_to_end =
		    CIRC_CNT_TO_END(xmit->head, xmit->tail, UART_XMIT_SIZE);
		int sz = min(size - count, cnt_to_end);
		quark_uart_hal_write(priv, &xmit->buf[xmit->tail], sz);
		xmit->tail = (xmit->tail + sz) & (UART_XMIT_SIZE - 1);
		count += sz;
	} while (!uart_circ_empty(xmit) && count < size);

pop_tx_end:
	dev_dbg(priv->port.dev, "%d characters. Remained %d characters.(%lu)\n",
		 count, size - count, jiffies);

	return count;
}

static int handle_rx_to(struct x1000_port *priv)
{
	struct quark_uart_buffer *buf;
	int rx_size;
	int ret;
	if (!priv->start_rx) {
		quark_uart_hal_disable_interrupt(priv, QUARK_UART_HAL_RX_INT |
						     QUARK_UART_HAL_RX_ERR_INT);
		return 0;
	}
	buf = &priv->rxbuf;
	do {
		rx_size = quark_uart_hal_read(priv, buf->buf, buf->size);
		ret = push_rx(priv, buf->buf, rx_size);
		if (ret)
			return 0;
	} while (rx_size == buf->size);

	return QUARK_UART_HANDLED_RX_INT;
}

static int handle_rx(struct x1000_port *priv)
{
	return handle_rx_to(priv);
}

static int dma_handle_rx(struct x1000_port *priv)
{
	struct uart_port *port = &priv->port;
	struct dma_async_tx_descriptor *desc;
	struct scatterlist *sg;

	priv = container_of(port, struct x1000_port, port);
	sg = &priv->sg_rx;

	sg_init_table(&priv->sg_rx, 1); /* Initialize SG table */

	sg_dma_len(sg) = priv->trigger_level;

	sg_set_page(&priv->sg_rx, virt_to_page(priv->rx_buf_virt),
		     sg_dma_len(sg), (unsigned long)priv->rx_buf_virt &
		     ~PAGE_MASK);

	sg_dma_address(sg) = priv->rx_buf_dma;

	/* Configure RX */
	priv->dmas_rx.dma_slave.dst_addr = sg_dma_address(sg);
	priv->dmas_rx.dma_slave.dst_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
	priv->dmas_rx.dma_slave.dst_maxburst = LNW_DMA_MSIZE_1;

	priv->dmas_rx.dma_slave.src_addr = QUARK_UART_AHB_REG_BASE + QUARK_UART_THR;	/* Wants an AHB address */
	priv->dmas_rx.dma_slave.src_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
	priv->dmas_rx.dma_slave.src_maxburst = LNW_DMA_MSIZE_4; 
	priv->dmas_rx.dma_slave.direction = DMA_DEV_TO_MEM;
	priv->dmas_rx.dma_slave.device_fc = false;

	dmaengine_slave_config(priv->rx_chan, &priv->dmas_rx.dma_slave);
	desc = dmaengine_prep_slave_sg(priv->rx_chan,
					sg, 1, DMA_DEV_TO_MEM,
					DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
	if (!desc)
		return 0;

	priv->desc_rx = desc;
	desc->callback = quark_dma_rx_complete;
	desc->callback_param = priv;
	desc->tx_submit(desc);
	dma_async_issue_pending(priv->rx_chan);

	return QUARK_UART_HANDLED_RX_INT;
}

static unsigned int handle_tx(struct x1000_port *priv)
{
	struct uart_port *port = &priv->port;
	struct circ_buf *xmit = &port->state->xmit;
	int fifo_size;
	int tx_size;
	int size;
	int tx_empty;

	if (!priv->start_tx) {
		dev_dbg(priv->port.dev, "%s:Tx isn't started. (%lu)\n",
			__func__, jiffies);
		quark_uart_hal_disable_interrupt(priv, QUARK_UART_HAL_TX_INT);
		wake_up(&priv->w_queue);
		priv->tx_empty = 1;
		return 0;
	}

	fifo_size = max(priv->fifo_size, 1);
	tx_empty = 1;
	if (pop_tx_x(priv, xmit->buf)) {
		quark_uart_hal_write(priv, xmit->buf, 1);
		port->icount.tx++;
		tx_empty = 0;
		fifo_size--;
	}
	size = min(xmit->head - xmit->tail, fifo_size);
	if (size < 0)
		size = fifo_size;

	tx_size = pop_tx(priv, size);
	if (tx_size > 0) {
		port->icount.tx += tx_size;
		tx_empty = 0;
	}

	priv->tx_empty = tx_empty;
	if (tx_empty) {
		quark_uart_hal_disable_interrupt(priv, QUARK_UART_HAL_TX_INT);
		wake_up(&priv->w_queue);
		if (port->state->port.tty != NULL)
			uart_write_wakeup(port);
	}

	return QUARK_UART_HANDLED_TX_INT;
}


static unsigned int dma_handle_tx(struct x1000_port *priv)
{
	struct uart_port *port = &priv->port;
	struct circ_buf *xmit = &port->state->xmit;
	struct scatterlist *sg;
	int nent;
	int fifo_size;
	int tx_empty;
	int num;
	int i;
	int bytes;
	int size;
	int rem;
	int ret;

	if (!priv->start_tx) {
		dev_dbg(priv->port.dev, "%s:Tx isn't started. (%lu)\n",
			__func__, jiffies);
		quark_uart_hal_disable_interrupt(priv, QUARK_UART_HAL_TX_INT);
		wake_up(&priv->w_queue);
		priv->tx_empty = 1;
		return 0;
	}

	if (priv->tx_dma_use) {
		dev_dbg(priv->port.dev, "%s:Tx is not completed. (%lu)\n",
			__func__, jiffies);
		quark_uart_hal_disable_interrupt(priv, QUARK_UART_HAL_TX_INT);
		wake_up(&priv->w_queue);
		priv->tx_empty = 1;
		return 0;
	}

	fifo_size = QUARK_UART_MAXBURST;
	tx_empty = 1;
	if (pop_tx_x(priv, xmit->buf)) {
		quark_uart_hal_write(priv, xmit->buf, 1);
		port->icount.tx++;
		tx_empty = 0;
		fifo_size--;
	}

	bytes = min((int)CIRC_CNT(xmit->head, xmit->tail,
			     UART_XMIT_SIZE), CIRC_CNT_TO_END(xmit->head,
			     xmit->tail, UART_XMIT_SIZE));

	if (!bytes) {
		dev_dbg(priv->port.dev, "%s 0 bytes return\n", __func__);
		quark_uart_hal_disable_interrupt(priv, QUARK_UART_HAL_TX_INT);
		wake_up(&priv->w_queue);
		if (port->state->port.tty != NULL)
			uart_write_wakeup(port);
		return 0;
	}

	/* DMA block doesn't do !dword aligned */
	if (bytes % 0x04){
		if (bytes < 4){
			ret = handle_tx(priv);
			/* DMA transactions want to start DWORD aligned */
			xmit->head = xmit->tail = 0;
			return ret;
		}
		bytes &= 0xFFFFFFFC;
	}

	/* For small payloads its just faster to write the FIFO directly */
	if (bytes < QUARK_FIFO_FASTER_LEN){
		ret = handle_tx(priv);
		/* DMA transactions want to start DWORD aligned */
		xmit->head = xmit->tail = 0;
		return ret;
	}

	if (bytes > fifo_size) {
		num = bytes / fifo_size;
		size = fifo_size;
		rem = bytes % fifo_size;
	} else {
		num = 1;
		size = bytes;
		rem = bytes;
	}

	dev_dbg(priv->port.dev, "%s num=%d size=%d rem=%d\n",
		__func__, num, size, rem);

	priv->tx_dma_use = 1;

	priv->sg_tx_p = kzalloc(sizeof(struct scatterlist)*num, GFP_ATOMIC);
	if (!priv->sg_tx_p) {
		dev_err(priv->port.dev, "%s:kzalloc Failed\n", __func__);
		return 0;
	}

	sg_init_table(priv->sg_tx_p, num); /* Initialize SG table */
	sg = priv->sg_tx_p;

	for (i = 0; i < num; i++, sg++) {
		BUG_ON((int)xmit->buf & ~PAGE_MASK);
		if (i == (num - 1))
			sg_set_page(sg, virt_to_page(xmit->buf),
				    rem, fifo_size * i);
		else
			sg_set_page(sg, virt_to_page(xmit->buf),
				    size, fifo_size * i);
	}

	sg = priv->sg_tx_p;
	nent = dma_map_sg(port->dev, sg, num, DMA_TO_DEVICE);
	if (!nent) {
		dev_err(priv->port.dev, "%s:dma_map_sg Failed\n", __func__);
		return 0;
	}
	priv->nent = nent;

	for (i = 0; i < nent; i++, sg++) {
		sg->offset = (xmit->tail & (UART_XMIT_SIZE - 1)) +
			      fifo_size * i;
		sg_dma_address(sg) = (sg_dma_address(sg) &
				    ~(UART_XMIT_SIZE - 1)) + sg->offset;

		if (i == (nent - 1))
			sg_dma_len(sg) = bytes;
		else
			sg_dma_len(sg) = size;
		bytes -= size;
	}

	priv->dma_tx_in_flight = 1;
	quark_uart_hal_disable_interrupt(priv, QUARK_UART_HAL_TX_INT);
	wake_up(&priv->w_queue);
	schedule_work(&priv->work);

	return QUARK_UART_HANDLED_TX_INT;
}

static void quark_uart_err_ir(struct x1000_port *priv, unsigned int lsr)
{
	struct uart_port *port = &priv->port;
	struct tty_struct *tty = tty_port_tty_get(&port->state->port);
	char   *error_msg[5] = {};
	int    i = 0;

	if (lsr & QUARK_UART_LSR_ERR)
		error_msg[i++] = "Error data in FIFO\n";

	if (lsr & UART_LSR_FE) {
		port->icount.frame++;
		error_msg[i++] = "  Framing Error\n";
	}

	if (lsr & UART_LSR_PE) {
		port->icount.parity++;
		error_msg[i++] = "  Parity Error\n";
	}

	if (lsr & UART_LSR_OE) {
		port->icount.overrun++;
		error_msg[i++] = "  Overrun Error\n";
	}

	if (tty == NULL) {
		for (i = 0; error_msg[i] != NULL; i++)
			dev_err(&priv->pdev->dev, error_msg[i]);
	} else {
		tty_kref_put(tty);
	}
}

#if defined(CONFIG_INTEL_QUARK_X1000_SOC)
	#define mask_pvm(x) qrk_pci_pvm_mask(x)
	#define unmask_pvm(x) qrk_pci_pvm_unmask(x)
#else
	#define mask_pvm(x)
	#define unmask_pvm(x)
#endif

static void quark_uart_work(struct work_struct *work)
{
	struct x1000_port *priv = container_of(work, struct x1000_port,work);
	struct dma_async_tx_descriptor *desc;

	if (priv == NULL) {
		pr_err("ERR_X1000: tasklet Null param\n");
		return;
	}
	/* Configure TX */
	priv->dmas_tx.dma_slave.src_addr = sg_dma_address(priv->sg_tx_p);
	priv->dmas_tx.dma_slave.src_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
	priv->dmas_tx.dma_slave.src_maxburst = LNW_DMA_MSIZE_1;
	priv->dmas_tx.dma_slave.dst_addr = QUARK_UART_AHB_REG_BASE + QUARK_UART_THR;	/* Wants an AHB address */
	priv->dmas_tx.dma_slave.dst_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
	priv->dmas_tx.dma_slave.dst_maxburst = LNW_DMA_MSIZE_4;
	priv->dmas_tx.dma_slave.direction = DMA_MEM_TO_DEV;
	priv->dmas_tx.dma_slave.device_fc = false;

	dmaengine_slave_config(priv->tx_chan, &priv->dmas_tx.dma_slave);
	desc = dmaengine_prep_slave_sg(priv->tx_chan,
					priv->sg_tx_p, priv->nent, DMA_MEM_TO_DEV,
					DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
	if (!desc) {
		dev_err(priv->port.dev, "%s:device_prep_slave_sg Failed\n",
			__func__);
		return;
	}
	dma_sync_sg_for_device(priv->port.dev, priv->sg_tx_p, priv->nent, DMA_TO_DEVICE);
	priv->desc_tx = desc;
	desc->callback = quark_dma_tx_complete;
	desc->callback_param = priv;
	desc->tx_submit(desc);
	dma_async_issue_pending(priv->tx_chan);
}

static irqreturn_t quark_uart_interrupt(int irq, void *dev_id)
{
	struct x1000_port *priv = dev_id;
	unsigned int handled;
	u8 lsr;
	int ret = 0;
	unsigned char iid;
	unsigned long flags;
	int next = 1;
	u8 msr;

	spin_lock_irqsave(&priv->lock, flags);
	handled = 0;
	while (next) {
		iid = quark_uart_hal_get_iid(priv);
		if (iid & QUARK_UART_IIR_IP) /* No Interrupt */
			break;
		switch (iid) {
		case QUARK_UART_IID_RLS:	/* Receiver Line Status */
			lsr = quark_uart_hal_get_line_status(priv);
			if (lsr & (QUARK_UART_LSR_ERR | UART_LSR_FE |
						UART_LSR_PE | UART_LSR_OE)) {
				quark_uart_err_ir(priv, lsr);
				ret = QUARK_UART_HANDLED_RX_ERR_INT;
			} else {
				ret = QUARK_UART_HANDLED_LS_INT;
			}
			break;
		case QUARK_UART_IID_RDR:	/* Received Data Ready */

			(void)dma_handle_rx;   /*Not worth setup time*/
			ret = handle_rx(priv);
			break;
		case QUARK_UART_IID_RDR_TO:	/* Received Data Ready
						   (FIFO Timeout) */
			ret = handle_rx_to(priv);
			break;
		case QUARK_UART_IID_THRE:	/* Transmitter Holding Register
						   Empty */
			if (priv->use_dma)
				ret = dma_handle_tx(priv);
			else
				ret = handle_tx(priv);
			break;
		case QUARK_UART_IID_MS:	/* Modem Status */
			msr = quark_uart_hal_get_modem(priv);
			next = 0; /* MS ir prioirty is the lowest. So, MS ir
				     means final interrupt */
			if ((msr & UART_MSR_ANY_DELTA) == 0)
				break;
			ret |= QUARK_UART_HANDLED_MS_INT;
			break;
		default:	/* Never junp to this label */
			dev_err(priv->port.dev, "%s:iid=%02x (%lu)\n", __func__,
				iid, jiffies);
			ret = -1;
			next = 0;
			break;
		}
		handled |= (unsigned int)ret;
	}

	spin_unlock_irqrestore(&priv->lock, flags);

	return IRQ_RETVAL(handled);
}

/* This function tests whether the transmitter fifo and shifter for the port
						described by 'port' is empty. */
static unsigned int quark_uart_tx_empty(struct uart_port *port)
{
	struct x1000_port *priv;

	priv = container_of(port, struct x1000_port, port);
	if (priv->tx_empty)
		return TIOCSER_TEMT;
	else
		return 0;
}

/* Returns the current state of modem control inputs. */
static unsigned int quark_uart_get_mctrl(struct uart_port *port)
{
	struct x1000_port *priv;
	u8 modem;
	unsigned int ret = 0;

	priv = container_of(port, struct x1000_port, port);
	modem = quark_uart_hal_get_modem(priv);

	if (modem & UART_MSR_DCD)
		ret |= TIOCM_CAR;

	if (modem & UART_MSR_RI)
		ret |= TIOCM_RNG;

	if (modem & UART_MSR_DSR)
		ret |= TIOCM_DSR;

	if (modem & UART_MSR_CTS)
		ret |= TIOCM_CTS;

	return ret;
}

static void quark_uart_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	u32 mcr = 0;
	struct x1000_port *priv = container_of(port, struct x1000_port, port);

	if (mctrl & TIOCM_DTR)
		mcr |= UART_MCR_DTR;
	if (mctrl & TIOCM_RTS)
		mcr |= UART_MCR_RTS;
	if (mctrl & TIOCM_LOOP)
		mcr |= UART_MCR_LOOP;

	if (priv->mcr & UART_MCR_AFE)
		mcr |= UART_MCR_AFE;

	if (mctrl)
		serial_out(priv, UART_MCR, mcr);
}

static void quark_uart_stop_tx(struct uart_port *port)
{
	struct x1000_port *priv;
	unsigned long flags;
	priv = container_of(port, struct x1000_port, port);

	spin_lock_irqsave(&priv->lock, flags);
	priv->start_tx = 0;
	priv->tx_dma_use = 0;

	spin_unlock_irqrestore(&priv->lock, flags);
}

static void quark_uart_start_tx(struct uart_port *port)
{
	struct x1000_port *priv;

	unsigned long flags;
	priv = container_of(port, struct x1000_port, port);

	spin_lock_irqsave(&priv->lock, flags);
	if (priv->use_dma) {
		if (priv->tx_dma_use) {
			dev_dbg(priv->port.dev, "%s : Tx DMA is NOT empty.\n",
				__func__);
			goto done;
		}
	}

	priv->start_tx = 1;
	if( priv->dma_tx_in_flight == 0)
		quark_uart_hal_enable_interrupt(priv, QUARK_UART_HAL_TX_INT);

done:
	spin_unlock_irqrestore(&priv->lock, flags);

}

static void quark_uart_stop_rx(struct uart_port *port)
{
	struct x1000_port *priv;
	priv = container_of(port, struct x1000_port, port);

	wait_event(priv->w_queue,!(priv->ier & QUARK_UART_HAL_TX_INT) && !priv->dma_tx_in_flight);
	priv->start_rx = 0;
	quark_uart_hal_disable_interrupt(priv, QUARK_UART_HAL_RX_INT |
					     QUARK_UART_HAL_RX_ERR_INT);
}

/* Enable the modem status interrupts. */
static void quark_uart_enable_ms(struct uart_port *port)
{
	struct x1000_port *priv;
	priv = container_of(port, struct x1000_port, port);
	quark_uart_hal_enable_interrupt(priv, QUARK_UART_HAL_MS_INT);
}

/* Control the transmission of a break signal. */
static void quark_uart_break_ctl(struct uart_port *port, int ctl)
{
	struct x1000_port *priv;
	unsigned long flags;

	priv = container_of(port, struct x1000_port, port);
	spin_lock_irqsave(&priv->lock, flags);
	quark_uart_hal_set_break(priv, ctl);
	spin_unlock_irqrestore(&priv->lock, flags);
}

/* Grab any interrupt resources and initialise any low level driver state. */
static int quark_uart_startup(struct uart_port *port)
{
	struct x1000_port *priv;
	int ret;
	int fifo_size;
	int trigger_level;

	priv = container_of(port, struct x1000_port, port);
	priv->tx_empty = 1;

	if (port->uartclk)
		priv->uartclk = port->uartclk;
	else
		port->uartclk = priv->uartclk;

	quark_uart_hal_disable_interrupt(priv, QUARK_UART_HAL_ALL_INT);
	ret = quark_uart_hal_set_line(priv, default_baud,
			      QUARK_UART_HAL_PARITY_NONE, QUARK_UART_HAL_8BIT,
			      QUARK_UART_HAL_STB1);

	if (ret)
		return ret;

	switch (priv->fifo_size) {
	case 256:
		fifo_size = QUARK_UART_HAL_FIFO256;
		break;
	case 64:
		fifo_size = QUARK_UART_HAL_FIFO64;
		break;
	case 16:
		fifo_size = QUARK_UART_HAL_FIFO16;
		break;
	case 1:
	default:
		fifo_size = QUARK_UART_HAL_FIFO_DIS;
		break;
	}

	switch (priv->trigger) {
	case QUARK_UART_HAL_TRIGGER1:
		trigger_level = 1;
		break;
	case QUARK_UART_HAL_TRIGGER_L:
		trigger_level = priv->fifo_size / 4;
		break;
	case QUARK_UART_HAL_TRIGGER_M:
		trigger_level = priv->fifo_size / 2;
		break;
	case QUARK_UART_HAL_TRIGGER_H:
	default:
		trigger_level = priv->fifo_size - (priv->fifo_size / 8);
		break;
	}

	priv->trigger_level = trigger_level;
	ret = quark_uart_hal_set_fifo(priv, QUARK_UART_HAL_DMA_MODE0,
				    fifo_size, priv->trigger);
	if (ret < 0)
		return ret;
	priv->start_rx = 1;
	quark_uart_hal_enable_interrupt(priv, QUARK_UART_HAL_RX_INT |
					    QUARK_UART_HAL_RX_ERR_INT);
	uart_update_timeout(port, CS8, default_baud);
	return 0;
}

static void quark_uart_shutdown(struct uart_port *port)
{
	struct x1000_port *priv;
	int ret;

	priv = container_of(port, struct x1000_port, port);

	wait_event(priv->w_queue, !(priv->dma_tx_in_flight));

	quark_uart_hal_disable_interrupt(priv, QUARK_UART_HAL_ALL_INT);
	quark_uart_hal_fifo_reset(priv, QUARK_UART_HAL_CLR_ALL_FIFO);
	ret = quark_uart_hal_set_fifo(priv, QUARK_UART_HAL_DMA_MODE0,
			      QUARK_UART_HAL_FIFO_DIS, QUARK_UART_HAL_TRIGGER1);
	if (ret)
		dev_err(priv->port.dev,
			"quark_uart_hal_set_fifo Failed(ret=%d)\n", ret);
}

/* Change the port parameters, including word length, parity, stop
 *bits.  Update read_status_mask and ignore_status_mask to indicate
 *the types of events we are interested in receiving.  */
static void quark_uart_set_termios(struct uart_port *port,
				 struct ktermios *termios, struct ktermios *old)
{
	int rtn;
	unsigned int baud, parity, bits, stb;
	struct x1000_port *priv;
	unsigned long flags;

	priv = container_of(port, struct x1000_port, port);
	switch (termios->c_cflag & CSIZE) {
	case CS5:
		bits = QUARK_UART_HAL_5BIT;
		break;
	case CS6:
		bits = QUARK_UART_HAL_6BIT;
		break;
	case CS7:
		bits = QUARK_UART_HAL_7BIT;
		break;
	default:		/* CS8 */
		bits = QUARK_UART_HAL_8BIT;
		break;
	}
	if (termios->c_cflag & CSTOPB)
		stb = QUARK_UART_HAL_STB2;
	else
		stb = QUARK_UART_HAL_STB1;

	if (termios->c_cflag & PARENB) {
		if (termios->c_cflag & PARODD)
			parity = QUARK_UART_HAL_PARITY_ODD;
		else
			parity = QUARK_UART_HAL_PARITY_EVEN;

	} else
		parity = QUARK_UART_HAL_PARITY_NONE;

	/* Only UART0 has auto hardware flow function */
	if ((termios->c_cflag & CRTSCTS) && (priv->fifo_size == 256))
		priv->mcr |= UART_MCR_AFE;
	else
		priv->mcr &= ~UART_MCR_AFE;

	termios->c_cflag &= ~CMSPAR; /* Mark/Space parity is not supported */

	baud = uart_get_baud_rate(port, termios, old, 0, port->uartclk / 16);
	spin_lock_irqsave(&priv->lock, flags);
	spin_lock(&port->lock);

	uart_update_timeout(port, termios->c_cflag, baud);
	rtn = quark_uart_hal_set_line(priv, baud, parity, bits, stb);
	if (rtn)
		goto out;

	quark_uart_set_mctrl(&priv->port, priv->port.mctrl);
	/* Don't rewrite B0 */
	if (tty_termios_baud_rate(termios))
		tty_termios_encode_baud_rate(termios, baud, baud);

out:
	spin_unlock(&port->lock);
	spin_unlock_irqrestore(&priv->lock, flags);
}

static const char *quark_uart_type(struct uart_port *port)
{
	return KBUILD_MODNAME;
}

static void quark_uart_release_port(struct uart_port *port)
{
	struct x1000_port *priv;

	priv = container_of(port, struct x1000_port, port);
	pci_iounmap(priv->pdev, priv->membase);
	pci_release_regions(priv->pdev);
}

static int quark_uart_request_port(struct uart_port *port)
{
	return 0;
}

static void quark_uart_config_port(struct uart_port *port, int type)
{
	struct x1000_port *priv;

	priv = container_of(port, struct x1000_port, port);
	if (type & UART_CONFIG_TYPE) {
		port->type = priv->port_type;
		quark_uart_request_port(port);
	}
}

static int quark_uart_verify_port(struct uart_port *port,
				struct serial_struct *serinfo)
{
	struct x1000_port *priv;
	priv = container_of(port, struct x1000_port, port);

	if (serinfo->flags & UPF_LOW_LATENCY) {
		dev_info(priv->port.dev,
			"QUARK UART : Use PIO Mode (without DMA)\n");
		priv->use_dma = false;
		serinfo->flags &= ~UPF_LOW_LATENCY;
	} else {
		dev_info(priv->port.dev, "QUARK UART : Use DMA Mode\n");
		#if 0
		if (!priv->use_dma)
			quark_request_dma(port);
		#endif
		priv->use_dma = true;
	}

	return 0;
}

#if defined(CONFIG_CONSOLE_POLL) || defined(CONFIG_SERIAL_QUARK_UART_CONSOLE)
/*
 *	Wait for transmitter & holding register to empty
 */
static void wait_for_xmitr(struct x1000_port *up, int bits)
{
	unsigned int status, tmout = 10000;

	/* Wait up to 10ms for the character(s) to be sent. */
	for (;;) {
		status = serial_in(up, UART_LSR);

		if ((status & bits) == bits)
			break;
		if (--tmout == 0)
			break;
		udelay(1);
	}

	/* Wait up to 1s for flow control if necessary */
	if (up->port.flags & UPF_CONS_FLOW) {
		unsigned int tmout;
		for (tmout = 1000000; tmout; tmout--) {
			unsigned int msr = serial_in(up, UART_MSR);
			if (msr & UART_MSR_CTS)
				break;
			udelay(1);
			touch_nmi_watchdog();
		}
	}
}
#endif /* CONFIG_CONSOLE_POLL || CONFIG_SERIAL_QUARK_UART_CONSOLE */

#ifdef CONFIG_CONSOLE_POLL
/*
 * Console polling routines for communicate via uart while
 * in an interrupt or debug context.
 */
static int quark_uart_get_poll_char(struct uart_port *port)
{
	struct x1000_port *priv =
		container_of(port, struct x1000_port, port);
	u8 lsr = serial_in(priv, UART_LSR);

	if (!(lsr & UART_LSR_DR))
		return NO_POLL_CHAR;

	return serial_in(priv, QUARK_UART_RBR);
}


static void quark_uart_put_poll_char(struct uart_port *port,
			 unsigned char c)
{
	unsigned int ier;
	struct x1000_port *priv =
		container_of(port, struct x1000_port, port);

	/*
	 * First save the IER then disable the interrupts
	 */
	ier = serial_in(priv, UART_IER);
	quark_uart_hal_disable_interrupt(priv, QUARK_UART_HAL_ALL_INT);

	wait_for_xmitr(priv, UART_LSR_THRE);
	/*
	 * Send the character out.
	 * If a LF, also do CR...
	 */
	serial_out(priv, QUARK_UART_THR, c);
	if (c == 10) {
		wait_for_xmitr(priv, UART_LSR_THRE);
		serial_out(priv, QUARK_UART_THR, 13);
	}

	/*
	 * Finally, wait for transmitter to become empty
	 * and restore the IER
	 */
	wait_for_xmitr(priv, BOTH_EMPTY);
	serial_out(priv, UART_IER, ier);
}
#endif /* CONFIG_CONSOLE_POLL */

static struct uart_ops quark_uart_ops = {
	.tx_empty = quark_uart_tx_empty,
	.set_mctrl = quark_uart_set_mctrl,
	.get_mctrl = quark_uart_get_mctrl,
	.stop_tx = quark_uart_stop_tx,
	.start_tx = quark_uart_start_tx,
	.stop_rx = quark_uart_stop_rx,
	.enable_ms = quark_uart_enable_ms,
	.break_ctl = quark_uart_break_ctl,
	.startup = quark_uart_startup,
	.shutdown = quark_uart_shutdown,
	.set_termios = quark_uart_set_termios,
/*	.pm		= quark_uart_pm,	Not supported yet */
/*	.set_wake	= quark_uart_set_wake,	Not supported yet */
	.type = quark_uart_type,
	.release_port = quark_uart_release_port,
	.request_port = quark_uart_request_port,
	.config_port = quark_uart_config_port,
	.verify_port = quark_uart_verify_port,
#ifdef CONFIG_CONSOLE_POLL
	.poll_get_char = quark_uart_get_poll_char,
	.poll_put_char = quark_uart_put_poll_char,
#endif
};

#ifdef CONFIG_SERIAL_QUARK_UART_CONSOLE

static void quark_console_putchar(struct uart_port *port, int ch)
{
	struct x1000_port *priv =
		container_of(port, struct x1000_port, port);

	wait_for_xmitr(priv, UART_LSR_THRE);
	serial_out(priv, QUARK_UART_THR, ch);
}

/*
 *	Print a string to the serial port trying not to disturb
 *	any possible real use of the port...
 *
 *	The console_lock must be held when we get here.
 */
static void
quark_console_write(struct console *co, const char *s, unsigned int count)
{
	struct x1000_port *priv;
	unsigned long flags;
	int priv_locked = 1;
	int port_locked = 1;
	u8 ier;

	priv = quark_uart_ports[co->index];

	touch_nmi_watchdog();

	local_irq_save(flags);
	if (priv->port.sysrq) {
		/* call to uart_handle_sysrq_char already took the priv lock */
		priv_locked = 0;
		/* serialquark_uart_handle_port() already took the port lock */
		port_locked = 0;
	} else if (oops_in_progress) {
		priv_locked = spin_trylock(&priv->lock);
		port_locked = spin_trylock(&priv->port.lock);
	} else {
		spin_lock(&priv->lock);
		spin_lock(&priv->port.lock);
	}

	/*
	 *	First save the IER then disable the interrupts
	 */
	ier = serial_in(priv, UART_IER);

	quark_uart_hal_disable_interrupt(priv, QUARK_UART_HAL_ALL_INT);

	uart_console_write(&priv->port, s, count, quark_console_putchar);

	/*
	 *	Finally, wait for transmitter to become empty
	 *	and restore the IER
	 */
	wait_for_xmitr(priv, BOTH_EMPTY);
	serial_out(priv, UART_IER, ier);

	if (port_locked)
		spin_unlock(&priv->port.lock);
	if (priv_locked)
		spin_unlock(&priv->lock);
	local_irq_restore(flags);
}

static int __init quark_console_setup(struct console *co, char *options)
{
	struct uart_port *port;
	int baud = default_baud;
	int bits = 8;
	int parity = 'n';
	int flow = 'n';

	/*
	 * Check whether an invalid uart number has been specified, and
	 * if so, search for the first available port that does have
	 * console support.
	 */
	if (co->index >= QUARK_UART_NR)
		co->index = 0;
	port = &quark_uart_ports[co->index]->port;

	if (!port || !port->membase)
		return -ENODEV;

	port->uartclk = quark_uart_get_uartclk();

	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);

	return uart_set_options(port, co, baud, parity, bits, flow);
}

static struct console quark_console = {
	.name		= QUARK_UART_DRIVER_DEVICE,
	.write		= quark_console_write,
	.device		= uart_console_device,
	.setup		= quark_console_setup,
	.flags		= CON_PRINTBUFFER | CON_ANYTIME,
	.index		= -1,
	.data		= &quark_uart_driver,
};

#define QUARK_CONSOLE	(&quark_console)
#else
#define QUARK_CONSOLE	NULL
#endif	/* CONFIG_SERIAL_QUARK_UART_CONSOLE */

static struct uart_driver quark_uart_driver = {
	.owner = THIS_MODULE,
	.driver_name = KBUILD_MODNAME,
	.dev_name = QUARK_UART_DRIVER_DEVICE,
	.nr = QUARK_UART_NR,
	.cons = QUARK_CONSOLE,
};

static int line_no; /* eek! TODO */

static struct x1000_port *quark_uart_init_port(struct pci_dev *pdev,
					     const struct pci_device_id *id)
{
	struct x1000_port *priv;
	int ret, len;
	dma_cap_mask_t mask;
	unsigned char *rxbuf;
	char name[32];	/* for debugfs file name */
	struct intel_mid_dma_probe_info * info = NULL;

	dev_dbg(&pdev->dev,"QUARK UART-DMA (ID: %04x:%04x) pdev->irq %d\n",
		pdev->vendor, pdev->device, pdev->irq);

	info = (void*)id->driver_data;
	dev_info(&pdev->dev,"QUARK UART-DMA : CH %d base %d block len %d per mask %x\n",
		info->max_chan, info->ch_base, info->block_size, info->pimr_mask);

	priv = kzalloc(sizeof(struct x1000_port), GFP_KERNEL);
	if (priv == NULL)
		goto init_port_alloc_err;

	rxbuf = (unsigned char *)__get_free_page(GFP_KERNEL);
	if (!rxbuf)
		goto init_port_free_txbuf;

	priv->mid_dma.pdev = pci_dev_get(pdev);
	pci_set_master(pdev);

	spin_lock_init(&priv->lock);

	/* UART regs */
	priv->mapbase = pci_resource_start(pdev, 0);
	len = pci_resource_len(pdev, 0);
	priv->membase = ioremap_nocache(priv->mapbase, len);
	if(priv->membase == NULL){
		ret = -ENODEV;
		goto init_port_free_txbuf;
	}

	/* DMA driver */
	priv->mid_dma.max_chan = info->max_chan;		/* Max channels */
	priv->mid_dma.chan_base = info->ch_base;		/* Index start */
	priv->mid_dma.block_size = info->block_size;		/* MAX DMA block */
	priv->mid_dma.pimr_mask = info->pimr_mask;		/* Per int regs bool */
	priv->mid_dma.is_quark = info->is_quark;

	ret = intel_qrk_dma_probe(pdev, &priv->mid_dma);
	if(ret != 0){
		dev_err(&pdev->dev, "Unable to init DMA sub-system\n");
		goto init_port_free_uart_iomem;
	}

	/* Request DMA channels TODO: move to startup() once debugged on hw */
	dma_cap_zero(mask);
	dma_cap_set(DMA_SLAVE, mask);

	priv->rx_chan = dma_request_channel(mask, intel_qrk_dma_chan_filter, &priv->dmas_rx);
	if(priv->rx_chan == NULL){
		dev_err(&pdev->dev, "Unable to hook DMA RX channel\n");
		goto init_port_free_dma_iomem;
	};
	priv->rx_buf_virt = dma_alloc_coherent(&pdev->dev, QUARK_UART_FIFO_LEN,
				    &priv->rx_buf_dma, GFP_KERNEL);
	if (priv->rx_buf_virt == NULL){
		dev_err(&pdev->dev, "Unable to allocate %d bytes for DMA\n",
			QUARK_UART_FIFO_LEN);
		goto init_port_free_dma_iomem;
	}

	priv->dmas_rx.hs_mode = LNW_DMA_HW_HS;
	priv->dmas_rx.cfg_mode = LNW_DMA_PER_TO_MEM;
	/* Configure RX */

	priv->tx_chan = dma_request_channel(mask, intel_qrk_dma_chan_filter, &priv->dmas_tx);
	if(priv->tx_chan == NULL){
		dev_err(&pdev->dev, "Unable to hook DMA RX channel\n");
		goto init_port_free_dma_iomem;
	};
	priv->dmas_tx.hs_mode = LNW_DMA_HW_HS;
	priv->dmas_tx.cfg_mode = LNW_DMA_MEM_TO_PER;

	dev_info(&pdev->dev, "using %s for DMA RX %s for DMA TX DMA %s\n",
			dev_name(&priv->rx_chan->dev->device),
			dev_name(&priv->tx_chan->dev->device), use_dma ?
			"enabled" : "disabled");

	/* Setup UART port descriptor */
	priv->pdev = pdev;
	priv->tx_empty = 1;
	priv->rxbuf.buf = rxbuf;
	priv->rxbuf.size = PAGE_SIZE;
	priv->fifo_size = QUARK_UART_FIFO_LEN;
	priv->uartclk = quark_uart_get_uartclk();
	priv->port_type = PORT_MAX_8250 + 1;	/* BOD what does this do ? TBD*/
	priv->port.dev = &pdev->dev;
	priv->port.membase = priv->membase;
	priv->port.mapbase = priv->mapbase;
	priv->port.irq = pdev->irq;
	priv->port.iotype = UPIO_MEM;
	priv->port.ops = &quark_uart_ops;
	priv->port.flags = UPF_BOOT_AUTOCONF;
	priv->port.fifosize = QUARK_UART_FIFO_LEN;
	priv->port.line = line_no; 
	priv->trigger = QUARK_UART_HAL_TRIGGER_L;
	priv->use_dma = use_dma;
	use_dma = 0;

	spin_lock_init(&priv->port.lock);
	pci_set_drvdata(pdev, priv);
	priv->trigger_level = 1;
	priv->fcr = 0;

	ret = request_irq(pdev->irq, quark_uart_interrupt, IRQF_SHARED,
			KBUILD_MODNAME, priv);
	if (ret < 0){
		dev_err(&pdev->dev, "Unable to request irq %d err %d\n",
			pdev->irq, ret);
		goto init_port_hal_free;
	}

#ifdef CONFIG_SERIAL_QUARK_UART_CONSOLE
	quark_uart_ports[line_no++] = priv;
#endif
	ret = uart_add_one_port(&quark_uart_driver, &priv->port);

	if (ret < 0){
		dev_err(&pdev->dev, "uart_add_one_port fail %d\n", ret);
		goto init_port_hal_free;
	}

#ifdef CONFIG_DEBUG_FS
	snprintf(name, sizeof(name), "uart%d_regs", pdev->dev.id);
	priv->debugfs = debugfs_create_file(name, S_IFREG | S_IRUGO,
				NULL, priv, &port_regs_ops);
#endif
	INIT_WORK(&priv->work, quark_uart_work);
	init_waitqueue_head(&priv->w_queue);
	return priv;

init_port_hal_free:
	free_page((unsigned long)rxbuf);
init_port_free_dma_iomem:
init_port_free_uart_iomem:
	if (sg_dma_address(&priv->sg_rx))
		dma_free_coherent(priv->port.dev, priv->port.fifosize,
			  sg_virt(&priv->sg_rx),
			  sg_dma_address(&priv->sg_rx));

	iounmap(priv->membase);
init_port_free_txbuf:
	kfree(priv);
init_port_alloc_err:

	return NULL;
}

static void quark_uart_exit_port(struct pci_dev *pdev, struct x1000_port *priv)
{

#ifdef CONFIG_DEBUG_FS
	if (priv->debugfs)
		debugfs_remove(priv->debugfs);
#endif
	/* Shutdown DMA */
	intel_qrk_dma_remove(pdev, &priv->mid_dma);

	/* TODO: move to remove() when h/w proved out */
	if (priv->tx_chan) {
		dma_release_channel(priv->tx_chan);
		priv->tx_chan = NULL;
	}
	if (priv->rx_chan) {
		dma_release_channel(priv->rx_chan);
		priv->rx_chan = NULL;
	}

	if (sg_dma_address(&priv->sg_rx))
		dma_free_coherent(priv->port.dev, priv->port.fifosize,
				  sg_virt(&priv->sg_rx),
				  sg_dma_address(&priv->sg_rx));

	free_irq(priv->port.irq, priv);
	uart_remove_one_port(&quark_uart_driver, &priv->port);
	pci_set_drvdata(priv->pdev, NULL);
	free_page((unsigned long)priv->rxbuf.buf);
}

static void quark_uart_pci_remove(struct pci_dev *pdev)
{
	struct x1000_port *priv = pci_get_drvdata(pdev);

	pci_disable_msi(pdev);

#ifdef CONFIG_SERIAL_QUARK_UART_CONSOLE
	quark_uart_ports[priv->port.line] = NULL;
#endif
	quark_uart_exit_port(pdev, priv);
	pci_disable_device(pdev);
	kfree(priv);
	return;
}
#ifdef CONFIG_PM
static int quark_uart_pci_suspend(struct pci_dev *pdev, pm_message_t state)
{
	struct x1000_port *priv = pci_get_drvdata(pdev);

	/* Suspend DMA regs */
	intel_qrk_dma_suspend(&priv->mid_dma);

	/* Suspend UART regs */
	uart_suspend_port(&quark_uart_driver, &priv->port);

	pci_save_state(pdev);
	pci_set_power_state(pdev, pci_choose_state(pdev, state));
	return 0;
}

static int quark_uart_pci_resume(struct pci_dev *pdev)
{
	struct x1000_port *priv = pci_get_drvdata(pdev);
	int ret;

	pci_set_power_state(pdev, PCI_D0);
	pci_restore_state(pdev);

	ret = pci_enable_device(pdev);
	if (ret) {
		dev_err(&pdev->dev,
		"%s-pci_enable_device failed(ret=%d) ", __func__, ret);
		return ret;
	}

	/* Resume UART regs */
	uart_resume_port(&quark_uart_driver, &priv->port);

	/* Resume DMA regs */
	intel_qrk_dma_resume(&priv->mid_dma);

	return 0;
}
#else
#define quark_uart_pci_suspend NULL
#define quark_uart_pci_resume NULL
#endif

struct pci_device_id quark_uart_pci_ids[] = {
	/* channels = 2, offset = 0, block size = FIFO_LEN, pimr = 0 */
        { PCI_VDEVICE(INTEL, 0x0936), INFO(2, 0, QUARK_UART_FIFO_LEN, 0, 1)},
        { 0 }
};

static int quark_uart_pci_probe(struct pci_dev *pdev,
					const struct pci_device_id *id)
{
	int ret;
	struct x1000_port *priv;

	ret = pci_enable_device(pdev);
	if (ret < 0)
		goto probe_error;

	priv = quark_uart_init_port(pdev, id);
	if (!priv) {
		ret = -EBUSY;
		goto probe_disable_device;
	}
	pci_set_drvdata(pdev, priv);

	return ret;

probe_disable_device:
	pci_disable_msi(pdev);
	pci_disable_device(pdev);
probe_error:
	return ret;
}

static struct pci_driver quark_uart_pci_driver = {
	.name = "quark_uart",
	.id_table = quark_uart_pci_ids,
	.probe = quark_uart_pci_probe,
	.remove = quark_uart_pci_remove,
	.suspend = quark_uart_pci_suspend,
	.resume = quark_uart_pci_resume,
};

static int __init quark_uart_module_init(void)
{
	int ret;

	/* register as UART driver */
	ret = uart_register_driver(&quark_uart_driver);
	if (ret < 0)
		return ret;

	/* register as PCI driver */
	ret = pci_register_driver(&quark_uart_pci_driver);
	if (ret < 0)
		uart_unregister_driver(&quark_uart_driver);

	return ret;
}
module_init(quark_uart_module_init);

static void __exit quark_uart_module_exit(void)
{
	pci_unregister_driver(&quark_uart_pci_driver);
	uart_unregister_driver(&quark_uart_driver);
}
module_exit(quark_uart_module_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Intel QUARK X1000 UART PCI Driver");
module_param(default_baud, uint, S_IRUGO);
MODULE_PARM_DESC(default_baud,
                 "Default BAUD for initial driver state and console (default 115200)");
module_param(use_dma, bool, S_IRUGO);
MODULE_PARM_DESC(use_dma,
                 "Use DMA (default true)");
