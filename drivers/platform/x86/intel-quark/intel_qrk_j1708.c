/*
 * Copyright(c) 2013 Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details
 */

/*
 * J1708 receiver driver for Intel Quark platform
 *
 * - J1708 particular definition:
 *	The J1708 standard is a bus protocol where multiple nodes have access to
 *	the same bus. A particular feature of the J1708 protocol is related to
 *	bus access timing. The principle timing constraints are based around max
 *	time between characters (must be < 2 bits) and the min time between
 *	messages (always >= 12 bits). The line is considered idle once 10 bit
 *	time with no transmission has elapsed. J1708 requires 9600 BAUD, so 1
 *	bit time equals to about 0.104 ms.
 *
 * - J1708 solution overview
 *	(1) Extension of Intel Quark UART driver to support time-stamping(tsc)
 *	in interrupt handler. Interrupt trigger level sets at 1 character. We
 *	assume the tsc of interrupt handler representing the arrival time of
 *	each characters.
 *	(2) J1708 receive and buffer of the incoming char/tsc in
 *	j1708_char_ring_buffer.
 *	(3) J1708 periodicaly(every 4 ms, implemented with a timer) query
 *	buffered char/tsc and detect J1708 message boundary by comparing the tsc
 *	gap of two consective characters to the min message gap of 22 bit time
 *	(2.2 ms, equals to sum of the minimal message gap of 12 bit time + 1
 *	charater's transmission of 10 bit time).
 *	(4) After detecting one J1708 message, checks its checksum, message
 *	length and inter-char violation etc., push correct message into
 *	j1708_msg_ring_buffer for reader to read.
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/serial_core.h>
#include <linux/math64.h>
#include <asm/desc.h>
#include <linux/device.h>
#include <linux/intel_qrk_j1708.h>

#define MSECS_PER_SEC 1000UL

/* by default two j1708 devices installed in system: e.g. /dev/j1708_0[1] */
static unsigned int dev_num = 2;
module_param(dev_num, uint, S_IRUGO);

extern int bind_uart_port_to_j1708(struct uart_port **port, unsigned int idx,
				   unsigned int j1708_idx, void *push_func);
extern int unbind_uart_port_to_j1708(struct uart_port *port);

/* max in-char gap (12-bit time) in cpu cycle */
static u64 max_char_gap_cycle;
/* min inter-msg gap (22-bit time) in cpu cycle */
static u64 min_msg_gap_cycle;

static struct class *j1708_class;

struct j1708_device {
	unsigned int uart_idx;
	struct uart_port *up;

	int max_msg_size;

	enum j1708_dev_state dev_state;
	int ref_cnt;
	dev_t dev_id;
	struct cdev cdev;
	struct device *dev;

	struct mutex mutex;
	wait_queue_head_t read_wait;

	struct j1708_stats stats;

	/* data path: char_rb -> mid_char_rb -> msg_rb */
	struct j1708_msg_ring_buffer *msg_rb;
	struct j1708_char_ring_buffer *char_rb;
	struct j1708_mid_char_ring_buffer *mid_char_rb;
	u64 start_tsc;
	/* using 4 ms timeout to check message periodically */
	struct timer_list watchdog_timer;

	/* for debug */
	int mid_char_rb_overrun;	/* intermediate char ringbuf overrun */
	int rx_char_rb_overrun;		/* rx char ring buf overrun cnt */
};

static struct j1708_device *j1708_dev;

static struct j1708_device *lookup_j1708_dev(const dev_t dev_id,
					     unsigned int *idx)
{
	unsigned int i = 0;

	BUG_ON(idx == NULL);

	for (i = 0; i < dev_num; i++) {
		if (j1708_dev[i].dev_id == dev_id) {
			*idx = i;
			return &j1708_dev[i];
		}
	}

	return NULL;
}

static bool j1708_chksum_passed(u8 mid, u8 *buf, int buf_len)
{
	int sum = 0;
	int i;

	for (i = 0; i < buf_len; i++)
		sum += buf[i];
	sum += mid;
	sum &= 0xff;

	return (sum == 0) ? true : false;
}

static bool char_timeout_detected(u64 pre_tsc, u64 post_tsc)
{
	u64 cycles = 0;

	BUG_ON(post_tsc < pre_tsc);
	cycles = post_tsc - pre_tsc;

	return (cycles > max_char_gap_cycle) ? true : false;
}

static bool message_gap_detected(u64 pre_tsc, u64 post_tsc)
{
	u64 cycles = 0;

	BUG_ON(post_tsc < pre_tsc);
	cycles = post_tsc - pre_tsc;

	return (cycles > min_msg_gap_cycle) ? true : false;
}

/*
 * migrate char/tsc from char_rb to mid_char_rb
 */
static void pop_from_char_buffer(struct j1708_device *pdev, u64 *pop_tsc)
{
	unsigned long flags;
	u64 tsc = 0;
	int i = 0;

	spin_lock_irqsave(&pdev->char_rb->lock, flags);

	rdtscll(tsc);
	*pop_tsc = tsc;

	/* if char_rb->cnt == 0, represents line idle */
	for (i = 0; i < pdev->char_rb->cnt; i++) {
		pdev->mid_char_rb->data[pdev->mid_char_rb->head] =
		    pdev->char_rb->data[pdev->char_rb->tail];
		pdev->mid_char_rb->tsc[pdev->mid_char_rb->head] =
		    pdev->char_rb->tsc[pdev->char_rb->tail];

		pdev->char_rb->tail = (pdev->char_rb->tail + 1) & CBUF_MASK;
		pdev->mid_char_rb->head =
		    (pdev->mid_char_rb->head + 1) & MID_CBUF_MASK;

		if (pdev->mid_char_rb->cnt == MAX_J1708_MID_CHAR_BUF_SIZE) {
			pdev->mid_char_rb->tail =
			    (pdev->mid_char_rb->tail + 1) & MID_CBUF_MASK;
			pdev->mid_char_rb_overrun++;
		} else {
			pdev->mid_char_rb->cnt++;
		}
	}

	pdev->char_rb->cnt = 0;
	BUG_ON(pdev->char_rb->head != pdev->char_rb->tail);

	spin_unlock_irqrestore(&pdev->char_rb->lock, flags);
}

static void process_one_message(struct j1708_device *pdev, u32 msg_len,
				u32 pos_lo, u32 pos_hi, bool cgap_exceed)
{
	struct j1708_msg_entry msg;
	int i;

	BUG_ON(pdev == NULL);

	/* msg_len including mid, data and chksum */
	/* check max message length including chksum, the msg_len should >= 2 */
	if (msg_len > pdev->max_msg_size || msg_len < 2) {
		/* discard this message */
		pdev->stats.j1708_error_cnt++;
		return;
	}

	BUG_ON(((pos_lo + msg_len - 1) & MID_CBUF_MASK) != pos_hi);

	msg.length = msg_len - 1;
	msg.mid = pdev->mid_char_rb->data[pos_lo];
	msg.char_gap_exceed = cgap_exceed;

	for (i = 0; i < msg_len; i++) {
		/* include MID + chksum */
		msg.tsc[i] =
		    pdev->mid_char_rb->tsc[(pos_lo + i) & MID_CBUF_MASK];
	}

	for (i = 0; i < msg_len - 1; i++) {
		/* exclude MID */
		msg.data[i] = pdev->mid_char_rb->data[(pos_lo + i + 1) &
						      MID_CBUF_MASK];
	}

	if (!j1708_chksum_passed(msg.mid, msg.data, (msg_len - 1))) {
		/* discard this message */
		pdev->stats.j1708_error_cnt++;
		return;
	}

	if (cgap_exceed) {
		/* NOT abandon this message */
		pdev->stats.j1708_warn_cnt++;
	}

	spin_lock(&pdev->msg_rb->lock);
	pdev->msg_rb->msg[pdev->msg_rb->head] = msg;
	pdev->msg_rb->head = ((pdev->msg_rb->head + 1) & MRB_MASK);
	if (pdev->msg_rb->cnt == MAX_J1708_MSG_ENTRY_NUM) {
		/* message ring buffer overflow */
		pdev->msg_rb->tail = (pdev->msg_rb->tail + 1) & MRB_MASK;

		pdev->stats.overrun_msg_cnt++;
	} else {
		pdev->msg_rb->cnt++;
	}

	pdev->stats.total_msg_cnt++;
	pdev->stats.buf_msg_cnt = pdev->msg_rb->cnt;

	spin_unlock(&pdev->msg_rb->lock);

	if (waitqueue_active(&pdev->read_wait))
		wake_up_interruptible(&pdev->read_wait);
}

static void watchdog_timeout_handler(unsigned long data)
{
	u32 msg_hi_pos;		/* last char position of a message */
	u32 msg_lo_pos;		/* 1st char position of a message */
	u32 cur_pos, next_pos;
	u64 cur_tsc, next_tsc;
	u32 msg_len;
	bool char_gap_exceed;

	int cnt;
	u64 pop_tsc;

	struct j1708_device *pdev = (struct j1708_device *)data;

	pop_from_char_buffer(pdev, &pop_tsc);
	cnt = pdev->mid_char_rb->cnt;

	BUG_ON(pdev->dev_state != J1708_STATE_START_BUS_UNSYNC &&
	       pdev->dev_state != J1708_STATE_START_BUS_SYNC);

	if (cnt == 0) {
		/*
		 * if line idle time(from device starting) exceeds the min msg
		 * gap, update status to bus_synced
		 */
		if ((pdev->dev_state == J1708_STATE_START_BUS_UNSYNC) &&
		    message_gap_detected(pdev->start_tsc, pop_tsc)) {
			pdev->dev_state = J1708_STATE_START_BUS_SYNC;
		}
		goto restart_timer;
	}

	/*
	 * if the interval(from device starting to the 1st char) > min_msg_gap,
	 * update status to bus_synced
	 */
	if ((pdev->dev_state == J1708_STATE_START_BUS_UNSYNC) &&
	    message_gap_detected(pdev->start_tsc,
				 pdev->mid_char_rb->
					tsc[pdev->mid_char_rb->tail])) {
		pdev->dev_state = J1708_STATE_START_BUS_SYNC;
	}

	/* detect and extract j1708 messages from mid_char_rb */
	msg_lo_pos = pdev->mid_char_rb->tail;
	cur_pos = pdev->mid_char_rb->tail;
	msg_len = 0;
	char_gap_exceed = false;
	while (cnt) {
		msg_len++;
		cur_tsc = pdev->mid_char_rb->tsc[cur_pos];
		next_pos = (cur_pos + 1) & MID_CBUF_MASK;
		next_tsc =
		    (cnt == 1) ? pop_tsc : pdev->mid_char_rb->tsc[next_pos];

		if (message_gap_detected(cur_tsc, next_tsc)) {
			/*
			 * last char position: it is correct that
			 * hi_pos < lo_pos since this is a ring
			 */
			msg_hi_pos = cur_pos;

			if (pdev->dev_state == J1708_STATE_START_BUS_UNSYNC) {
				/* discard 1st possibly incompleted message*/
				pdev->dev_state = J1708_STATE_START_BUS_SYNC;
			} else {
				process_one_message(pdev, msg_len, msg_lo_pos,
						    msg_hi_pos,
						    char_gap_exceed);
			}

			/* remove message's char/tsc from mid_char_rb */
			pdev->mid_char_rb->cnt -= msg_len;
			pdev->mid_char_rb->tail =
			    (pdev->mid_char_rb->tail + msg_len) & MID_CBUF_MASK;

			if (!pdev->mid_char_rb->cnt) {
				BUG_ON(pdev->mid_char_rb->tail !=
				       pdev->mid_char_rb->head);
			}

			/* reset for next message */
			msg_lo_pos = pdev->mid_char_rb->tail;
			char_gap_exceed = false;
			msg_len = 0;
		} else {
			if (char_timeout_detected(cur_tsc, next_tsc))
				char_gap_exceed = true;
		}

		cnt--;
		cur_pos = (cur_pos + 1) & MID_CBUF_MASK;
	}

 restart_timer:
	mod_timer(&pdev->watchdog_timer,
		  jiffies + msecs_to_jiffies(TIMEOUT_IN_MS));
}

static void j1708_char_buffer_push(unsigned int j1708_idx, unsigned char ch,
				   u64 tsc, bool uart_err)
{
	struct j1708_device *pdev;
	unsigned long flags;

	BUG_ON(j1708_idx >= dev_num);
	pdev = &j1708_dev[j1708_idx];
	if (uart_err) {
		pdev->stats.uart_error_cnt++;
		return;
	}

	spin_lock_irqsave(&pdev->char_rb->lock, flags);
	pdev->char_rb->data[pdev->char_rb->head] = ch;
	pdev->char_rb->tsc[pdev->char_rb->head] = tsc;
	pdev->char_rb->head = (pdev->char_rb->head + 1) & CBUF_MASK;

	if (pdev->char_rb->cnt == MAX_J1708_CHAR_BUF_SIZE) {
		pdev->char_rb->tail = (pdev->char_rb->tail + 1) & CBUF_MASK;
		pdev->rx_char_rb_overrun++;
	} else {
		pdev->char_rb->cnt++;
	}
	spin_unlock_irqrestore(&pdev->char_rb->lock, flags);
}

static int j1708_open(struct inode *inode, struct file *filp)
{
	unsigned int j1708_idx = 0;
	int ret = 0;
	dev_t dev_id = inode->i_rdev;
	struct j1708_device *pdev;

	pdev = lookup_j1708_dev(dev_id, &j1708_idx);
	if (!pdev)
		return -EINVAL;

	if (filp->f_flags & (~(O_NONBLOCK | O_RDONLY)))
		return -EINVAL;

	mutex_lock(&pdev->mutex);
	if (pdev->ref_cnt == 0)
		pdev->dev_state = J1708_STATE_OPEN;
	pdev->ref_cnt++;
	mutex_unlock(&pdev->mutex);

	return ret;
}

static int j1708_release(struct inode *inode, struct file *filp)
{
	unsigned int j1708_idx = 0;
	int ret = 0;
	dev_t dev_id = inode->i_rdev;
	struct j1708_device *pdev;
	pdev = lookup_j1708_dev(dev_id, &j1708_idx);

	if (!pdev)
		return -EINVAL;

	mutex_lock(&pdev->mutex);

	if (pdev->ref_cnt == 1) {
		switch (pdev->dev_state) {
		case J1708_STATE_OPEN:
		case J1708_STATE_CLOSE:
		case J1708_STATE_UART_UNBIND:
			break;
		case J1708_STATE_START_BUS_UNSYNC:
		case J1708_STATE_START_BUS_SYNC:
			pdev->up->ops->shutdown(pdev->up);

			ret = unbind_uart_port_to_j1708(pdev->up);
			del_timer_sync(&pdev->watchdog_timer);

			dev_dbg(pdev->dev,
				"j1708_ioctl_stop: rx_char_rb_overrun[%d] "
				"mid_char_rb_overrun[%d]\n",
				pdev->rx_char_rb_overrun,
				pdev->mid_char_rb_overrun);
			break;
		case J1708_STATE_STOP:
		case J1708_STATE_UART_BIND:
			ret = unbind_uart_port_to_j1708(pdev->up);
			break;
		default:
			break;
		}
		pdev->dev_state = J1708_STATE_CLOSE;
	}

	if (pdev->ref_cnt > 0)
		pdev->ref_cnt--;
	mutex_unlock(&pdev->mutex);

	return ret;
}

static long j1708_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	unsigned int j1708_idx = 0;
	int ret = 0;
	struct inode *inode = filp->f_path.dentry->d_inode;
	dev_t dev_id = inode->i_rdev;
	struct j1708_device *pdev;

	pdev = lookup_j1708_dev(dev_id, &j1708_idx);
	if (!pdev)
		return -EINVAL;
	BUG_ON(j1708_idx >= dev_num);

	mutex_lock(&pdev->mutex);

	switch (cmd) {
	case J1708_IOCTL_BIND_UART:
		if (arg != PORT0 && arg != PORT1) {
			ret = -EINVAL;
			goto error;
		}

		pdev->uart_idx = (unsigned int) arg;

		if ((pdev->dev_state == J1708_STATE_UART_UNBIND) ||
		    (pdev->dev_state == J1708_STATE_OPEN)) {
			ret = bind_uart_port_to_j1708(&pdev->up,
						      pdev->uart_idx,
						      j1708_idx,
						      j1708_char_buffer_push);
			if (!ret)
				pdev->dev_state = J1708_STATE_UART_BIND;
		} else {
			ret = -EINVAL;
		}
		break;

	case J1708_IOCTL_UNBIND_UART:
		if ((pdev->dev_state == J1708_STATE_STOP) ||
		    (pdev->dev_state == J1708_STATE_UART_BIND)) {
			ret = unbind_uart_port_to_j1708(pdev->up);
			if (!ret) {
				pdev->dev_state = J1708_STATE_UART_UNBIND;
				pdev->up = NULL;
			}
		} else {
			ret = -EINVAL;
		}
		break;

	case J1708_IOCTL_START:
		if ((pdev->dev_state == J1708_STATE_UART_BIND) ||
		    (pdev->dev_state == J1708_STATE_STOP)) {
			memset(&pdev->stats, 0, sizeof(struct j1708_stats));

			pdev->char_rb->cnt = 0;
			pdev->char_rb->tail = 0;
			pdev->char_rb->head = 0;

			pdev->mid_char_rb->cnt = 0;
			pdev->mid_char_rb->tail = 0;
			pdev->mid_char_rb->head = 0;

			pdev->msg_rb->cnt = 0;
			pdev->msg_rb->tail = 0;
			pdev->msg_rb->head = 0;

			pdev->rx_char_rb_overrun = 0;
			pdev->mid_char_rb_overrun = 0;

			rdtscll(pdev->start_tsc);
			ret = pdev->up->ops->startup(pdev->up);
			if (!ret) {
				pdev->dev_state = J1708_STATE_START_BUS_UNSYNC;
				mod_timer(&pdev->watchdog_timer,
					  jiffies +
					  msecs_to_jiffies(TIMEOUT_IN_MS));
			}
		} else {
			ret = -EINVAL;
		}
		break;

	case J1708_IOCTL_STOP:
		if ((pdev->dev_state == J1708_STATE_START_BUS_UNSYNC) ||
		    (pdev->dev_state == J1708_STATE_START_BUS_SYNC)) {
			pdev->up->ops->shutdown(pdev->up);
			pdev->dev_state = J1708_STATE_STOP;
			del_timer_sync(&pdev->watchdog_timer);

			dev_dbg(pdev->dev,
				"j1708_ioctl_stop: rx_char_rb_overrun[%d] "
				"mid_char_rb_overrun[%d]\n",
				pdev->rx_char_rb_overrun,
				pdev->mid_char_rb_overrun);
		} else {
			ret = -EINVAL;
		}
		break;

	case J1708_IOCTL_RESET:
		if ((pdev->dev_state == J1708_STATE_START_BUS_UNSYNC) ||
		    (pdev->dev_state == J1708_STATE_START_BUS_SYNC)) {
			pdev->up->ops->shutdown(pdev->up);

			memset(&pdev->stats, 0, sizeof(struct j1708_stats));

			pdev->max_msg_size = MAX_J1708_LENGTH;

			pdev->char_rb->cnt = 0;
			pdev->char_rb->tail = 0;
			pdev->char_rb->head = 0;

			pdev->mid_char_rb->cnt = 0;
			pdev->mid_char_rb->tail = 0;
			pdev->mid_char_rb->head = 0;

			pdev->msg_rb->cnt = 0;
			pdev->msg_rb->tail = 0;
			pdev->msg_rb->head = 0;

			dev_dbg(pdev->dev,
				"j1708_ioctl_stop: rx_char_rb_overrun[%d] "
				"mid_char_rb_overrun[%d]\n",
				pdev->rx_char_rb_overrun,
				pdev->mid_char_rb_overrun);

			pdev->rx_char_rb_overrun = 0;
			pdev->mid_char_rb_overrun = 0;

			rdtscll(pdev->start_tsc);
			ret = pdev->up->ops->startup(pdev->up);
			if (!ret) {
				pdev->dev_state = J1708_STATE_START_BUS_UNSYNC;
				mod_timer(&pdev->watchdog_timer,
					  jiffies +
					  msecs_to_jiffies(TIMEOUT_IN_MS));
			}
		} else {
			ret = -EINVAL;
		}

		break;

	case J1708_IOCTL_SET_MSG_LENGTH:
		if (arg < 2 || arg > MAX_CONFIG_J1708_LENGTH)
			ret = -EINVAL;
		pdev->max_msg_size = arg;
		break;

	case J1708_IOCTL_GET_MSG_LENGTH:
		put_user(pdev->max_msg_size, (unsigned long *)arg);
		break;

	case J1708_IOCTL_GET_STAT:
		pdev->stats.j1708_state = pdev->dev_state;

		if (copy_to_user((struct j1708_stats *)arg, &pdev->stats,
				 sizeof(struct j1708_stats)))
			ret = -EFAULT;
		break;

	case J1708_IOCTL_SET_NONBLOCK:
		if (arg)
			filp->f_flags |= O_NONBLOCK;
		else
			filp->f_flags &= ~O_NONBLOCK;
		break;

	case J1708_IOCTL_GET_NONBLOCK:
		put_user((filp->f_flags & O_NONBLOCK), (unsigned long *)arg);
		break;

	default:
		ret = -EINVAL;
	}

 error:
	mutex_unlock(&pdev->mutex);

	return ret;
}

static ssize_t j1708_read_one_message(struct file *filp, char __user *buf,
				      size_t len, loff_t *off)
{
	unsigned int j1708_idx = 0;
	struct j1708_msg_entry *pmsg;
	struct j1708_device *pdev;
	ssize_t retval = 0;
	unsigned long flags = 0;

	struct inode *inode = filp->f_path.dentry->d_inode;
	dev_t dev_id = inode->i_rdev;
	pdev = lookup_j1708_dev(dev_id, &j1708_idx);
	if (!pdev)
		return -EINVAL;

	if (len != sizeof(struct j1708_msg_entry))
		return -EINVAL;

	if (filp->f_flags & O_NONBLOCK) {
		if (!spin_trylock_irq(&pdev->msg_rb->lock))
			return -EAGAIN;
	} else {
		spin_lock_irqsave(&pdev->msg_rb->lock, flags);
	}

	while (pdev->msg_rb->cnt == 0) {
		spin_unlock_irqrestore(&pdev->msg_rb->lock, flags);

		if (filp->f_flags & O_NONBLOCK)
			return -EAGAIN;

		if (wait_event_interruptible(pdev->read_wait,
					     (pdev->msg_rb->cnt > 0)))
			return -ERESTARTSYS;

		spin_lock_irqsave(&pdev->msg_rb->lock, flags);
	}

	pmsg = &pdev->msg_rb->msg[pdev->msg_rb->tail];
	pdev->msg_rb->tail = (pdev->msg_rb->tail + 1) &
			     (MAX_J1708_MSG_ENTRY_NUM - 1);
	pdev->msg_rb->cnt--;

	pdev->stats.buf_msg_cnt = pdev->msg_rb->cnt;

	if (copy_to_user(buf, pmsg, sizeof(struct j1708_msg_entry)))
		retval = -EFAULT;
	else
		retval = sizeof(struct j1708_msg_entry);

	spin_unlock_irqrestore(&pdev->msg_rb->lock, flags);

	return retval;
}

static const struct file_operations j1708_fops = {
	.owner = THIS_MODULE,
	.open = j1708_open,
	.release = j1708_release,
	.read = j1708_read_one_message,
	.write = NULL,
	.unlocked_ioctl = j1708_ioctl
};

static int one_j1708_device_init(struct j1708_device *pdev,
				 const unsigned int idx)
{
	int ret;
	struct device *dev_ret;

	pdev->msg_rb =
	    kzalloc(sizeof(struct j1708_msg_ring_buffer), GFP_KERNEL);
	if (!pdev->msg_rb) {
		ret = -ENOMEM;
		goto tsc_buf_error;
	}

	pdev->msg_rb->msg =
	    kzalloc(sizeof(struct j1708_msg_entry) * MAX_J1708_MSG_ENTRY_NUM,
		    GFP_KERNEL);
	if (!pdev->msg_rb->msg) {
		ret = -ENOMEM;
		goto mrb_error;
	}

	pdev->char_rb = kzalloc(sizeof(struct j1708_char_ring_buffer),
				GFP_KERNEL);
	if (!pdev->char_rb) {
		ret = -ENOMEM;
		goto cbuf_error;
	}

	pdev->mid_char_rb =
	    kzalloc(sizeof(struct j1708_mid_char_ring_buffer), GFP_KERNEL);
	if (!pdev->mid_char_rb) {
		ret = -ENOMEM;
		goto mid_cbuf_error;
	}

	ret = alloc_chrdev_region(&pdev->dev_id, 0, 1, "j1708");
	if (ret < 0)
		goto chrdev_region_error;

	dev_ret = device_create(j1708_class, NULL, pdev->dev_id, NULL,
				"j1708_" "%d", idx);
	if (IS_ERR(dev_ret)) {
		pr_err("Failed to create j1708 device: %ld\n",
		       PTR_ERR(dev_ret));
		ret = PTR_ERR(dev_ret);
		goto device_error;
	}
	pdev->dev = dev_ret;

	cdev_init(&pdev->cdev, &j1708_fops);
	ret = cdev_add(&pdev->cdev, pdev->dev_id, 1);
	if (ret != 0) {
		pr_err("Failed to add cdev for j1708: %d\n", ret);
		goto cdev_error;
	}

	spin_lock_init(&pdev->msg_rb->lock);
	spin_lock_init(&pdev->char_rb->lock);

	pdev->max_msg_size = MAX_J1708_LENGTH;
	memset(&pdev->stats, 0, sizeof(struct j1708_stats));

	mutex_init(&pdev->mutex);
	init_waitqueue_head(&pdev->read_wait);

	init_timer(&pdev->watchdog_timer);
	pdev->watchdog_timer.function = watchdog_timeout_handler;
	pdev->watchdog_timer.data = (unsigned long) pdev;

	dev_info(dev_ret, "Intel Quark J1708 node %u init\n", idx);

	return 0;

 cdev_error:
	device_destroy(j1708_class, pdev->dev_id);
 device_error:
	unregister_chrdev_region(pdev->dev_id, 1);
 chrdev_region_error:
	kfree(pdev->mid_char_rb);
 mid_cbuf_error:
	kfree(pdev->char_rb);
 cbuf_error:
	kfree(pdev->msg_rb->msg);
 mrb_error:
	kfree(pdev->msg_rb);
 tsc_buf_error:
	return ret;
}

static int __init quark_j1708_module_init(void)
{
	unsigned int i = 0;
	int ret = 0;
	s32 remainder;
	u64 cpu_hz = 0;

	if (dev_num > 8)
		return -EINVAL;

	j1708_dev = kzalloc(sizeof(struct j1708_device) * dev_num, GFP_KERNEL);
	if (!j1708_dev)
		return -ENOMEM;

	j1708_class = class_create(THIS_MODULE, "j1708");
	if (IS_ERR(j1708_class)) {
		pr_err("Failed to create j1708 device class: %ld\n",
		       PTR_ERR(j1708_class));
		ret = PTR_ERR(j1708_class);
		goto class_error;
	}

	for (i = 0; i < dev_num; i++) {
		ret = one_j1708_device_init(&j1708_dev[i], i);
		if (ret != 0)
			goto init_one_j1708_error;
	}

	BUG_ON(!cpu_khz);
	cpu_hz = cpu_khz * MSECS_PER_SEC;
	max_char_gap_cycle = div_u64_rem(MAX_J1708_CHAR_GAP_BIT_TIME * cpu_hz,
					 J1708_BAUD, &remainder) + 1;

	min_msg_gap_cycle = div_u64_rem(MIN_J1708_MSG_GAP_BIT_TIME * cpu_hz,
					J1708_BAUD, &remainder) + 1;

	dev_dbg(j1708_dev[0].dev,
		"J1708: system-cpu_hz[%lld] max_char_gap_cycle[%lld] "
		"min_msg_gap_cycle [%lld]\n",
		cpu_hz, max_char_gap_cycle, min_msg_gap_cycle);

	return 0;

 init_one_j1708_error:
	class_destroy(j1708_class);
 class_error:
	kfree(j1708_dev);

	return ret;
}

static void one_j1708_device_release(struct j1708_device *pdev)
{
	cdev_del(&(pdev->cdev));
	device_destroy(j1708_class, pdev->dev_id);
	unregister_chrdev_region(pdev->dev_id, 1);

	kfree(pdev->char_rb);
	kfree(pdev->mid_char_rb);
	kfree(pdev->msg_rb->msg);
	kfree(pdev->msg_rb);
}

static void __exit quark_j1708_module_exit(void)
{
	unsigned int i = 0;
	for (i = 0; i < dev_num; i++)
		one_j1708_device_release(&j1708_dev[i]);

	kfree(j1708_dev);

	class_destroy(j1708_class);
}

module_init(quark_j1708_module_init);
module_exit(quark_j1708_module_exit);

MODULE_DESCRIPTION("Intel Quark Platform J1708 Driver");
MODULE_AUTHOR("Wei Lin<wei.w.lin@intel.com>, Xin Zhang<xin.x.zhang@intel.com>");
MODULE_LICENSE("Dual BSD/GPL");
