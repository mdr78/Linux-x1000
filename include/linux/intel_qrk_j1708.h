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
 * Intel Quark J1708 driver
 */

#ifndef __INTEL_QRK_J1708_H__
#define __INTEL_QRK_J1708_H__

#include <linux/types.h>
#include <linux/ioctl.h>
#include <linux/spinlock.h>

#define MAX_J1708_LENGTH	21
#define MAX_CONFIG_J1708_LENGTH 64

#define PORT0 0
#define PORT1 1

/* keep as 2^xx */
#define MAX_J1708_MSG_ENTRY_NUM 2048
#define MRB_MASK (MAX_J1708_MSG_ENTRY_NUM - 1)

#define MAX_J1708_CHAR_BUF_SIZE 512
#define CBUF_MASK (MAX_J1708_CHAR_BUF_SIZE - 1)

#define MAX_J1708_MID_CHAR_BUF_SIZE 1024
#define MID_CBUF_MASK (MAX_J1708_MID_CHAR_BUF_SIZE - 1)

#define J1708_BAUD 9600
#define ONE_BIT_TIME_NS 104167

/* 12 bits =  2 bits max character gap + 10 bits message data access */
#define MAX_J1708_CHAR_GAP_BIT_TIME  12

/* 22 bits =  12 bits minimal message gap + 10 bits message data access */
#define MIN_J1708_MSG_GAP_BIT_TIME  22
#define MIN_J1708_MSG_GAP_NS (MIN_J1708_MSG_GAP_BIT_TIME * ONE_BIT_TIME_NS)

/* for watchdog_timer timeout set as 4 ms */
#define TIMEOUT_IN_MS 4

#define J1708_IOCTL_MAGIC 'Z'

#define	J1708_IOCTL_BIND_UART		_IO(J1708_IOCTL_MAGIC, 1)
#define	J1708_IOCTL_UNBIND_UART		_IO(J1708_IOCTL_MAGIC, 2)
#define	J1708_IOCTL_START		_IO(J1708_IOCTL_MAGIC, 3)
#define	J1708_IOCTL_STOP		_IO(J1708_IOCTL_MAGIC, 4)
#define	J1708_IOCTL_RESET		_IO(J1708_IOCTL_MAGIC, 5)
#define	J1708_IOCTL_SET_MSG_LENGTH	_IOW(J1708_IOCTL_MAGIC, 6, int)
#define	J1708_IOCTL_GET_MSG_LENGTH	_IOR(J1708_IOCTL_MAGIC, 7, int)
#define	J1708_IOCTL_GET_STAT		_IOR(J1708_IOCTL_MAGIC, 8, int)
#define	J1708_IOCTL_SET_NONBLOCK	_IOW(J1708_IOCTL_MAGIC, 9, int)
#define	J1708_IOCTL_GET_NONBLOCK	_IOR(J1708_IOCTL_MAGIC, 10, int)

enum j1708_dev_state {
	J1708_STATE_OPEN               = 0x0,
	J1708_STATE_UART_BIND          = 0x1,
	J1708_STATE_UART_UNBIND        = 0x2,
	J1708_STATE_START_BUS_UNSYNC   = 0x3,
	J1708_STATE_START_BUS_SYNC     = 0x4,
	J1708_STATE_STOP               = 0x5,
	J1708_STATE_CLOSE              = 0x6,
};


/*
 * J1708 RX message entry
 */
struct j1708_msg_entry {
	u32 length;				/* Msg length including MID */
	u8  mid;				/* Message Identification */
	u8  data[MAX_CONFIG_J1708_LENGTH];	/* message data with checksum */
	u64 tsc[MAX_CONFIG_J1708_LENGTH];	/* timestamp */
	u8  char_gap_exceed;			/* exceed 2 bit time char gap */
} __packed;

struct j1708_stats {
	u32 uart_error_cnt;
	u32 j1708_error_cnt;           /* excceed msg length + chksum error */
	u32 j1708_warn_cnt;            /* break the max char gap */
	u32 buf_msg_cnt;               /* unread message left in ring buffer */
	u64 total_msg_cnt;             /* total messages received from uart */
	u32 debug;                     /* TBD */
	u32 overrun_msg_cnt;           /* msgs discarded due to overflow */
	enum j1708_dev_state j1708_state; /* J1708 device state */
} __packed;

/*
 * j1708 message ring buffer
 */
struct j1708_msg_ring_buffer {
	struct j1708_msg_entry *msg;
	u32 head;
	u32 tail;
	int cnt;
	spinlock_t lock;
};

/*
 * ring buffer for characters/tsc from UART I/O
 */
struct j1708_char_ring_buffer {
	u8  data[MAX_J1708_CHAR_BUF_SIZE];
	u64 tsc[MAX_J1708_CHAR_BUF_SIZE];

	u32 head;
	u32 tail;
	int cnt;
	spinlock_t lock;
};

/*
 * intermidiate ring buffer for char/tsc message boundary detection
 */
struct j1708_mid_char_ring_buffer {
	u8  data[MAX_J1708_MID_CHAR_BUF_SIZE];
	u64 tsc[MAX_J1708_MID_CHAR_BUF_SIZE];

	u32 head;
	u32 tail;
	int cnt;
};

#endif /* __INTEL_QRK_J1708_H__ */
