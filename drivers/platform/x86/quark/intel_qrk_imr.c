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
/*
 * Intel Quark IMR driver
 *
 * IMR stand for Isolated Memory Region, supported by Quark SoC.
 *
 * A total number of 8 IMRs have implemented by Quark SoC,
 * Some IMRs might be already occupied by BIOS or Linux during
 * booting time.
 *
 * A user can cat /sys/devices/platform/intel-qrk-imr/status for current IMR
 * status
 *
 * To allocate an IMR addresses must be alinged to 1kB
 *
 * The IMR alloc API will locate the next available IMR slot set up
 * with input memory region, then apply the input access right masks
 *
 * The IMR can be freed with the pre-allocated memory addresses if its not
 * locked.
 */

#include <asm-generic/uaccess.h>
#include <linux/io.h>
#include <linux/intel_qrk_sb.h>
#include <linux/kallsyms.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/printk.h>
#include <linux/sysfs.h>

#include "intel_qrk_imr.h"

#define DRIVER_NAME	"intel-qrk-imr"

#define IMR_EN		0x40000000
#define IMR_READ_MASK	0x1

/* IMR HW register address structre */
struct qrk_imr_reg_t {
	u32  imr_xl;   /* high address register */
	u32  imr_xh;   /* low address register */
	u32  imr_rm;   /* read mask register */
	u32  imr_wm;   /* write mask register */
} qrk_imr_reg_t;

/**
 * struct qrk_imr_addr_t
 *
 * IMR memory address structure
 */
struct qrk_imr_addr_t {
	u32 addr_low;		/* low boundary memroy address */
	u32 addr_high_base;	/* high base boundary memory address */
	u32 read_mask;		/* read access right mask */
	u32 write_mask;		/* write access right mask */
} qrk_imr_addr_t;

/**
 * struct qrk_imr_pack
 *
 * local IMR pack structure
 */
struct qrk_imr_pack {
	bool occupied;       /* IMR occupied */
	bool locked;         /* IMR lock */
	struct qrk_imr_reg_t reg;   /* predefined imr register address */
	struct qrk_imr_addr_t addr; /* IMR address region structure */
	unsigned char info[MAX_INFO_SIZE]; /* IMR info */
} qrk_imr_pack;


/* Predefined HW register address */
static struct qrk_imr_reg_t imr_reg_value[] = {
	{ IMR0L, IMR0H, IMR0RM, IMR0WM },
	{ IMR1L, IMR1H, IMR1RM, IMR1WM },
	{ IMR2L, IMR2H, IMR2RM, IMR2WM },
	{ IMR3L, IMR3H, IMR3RM, IMR3WM },
	{ IMR4L, IMR4H, IMR4RM, IMR4WM },
	{ IMR5L, IMR5H, IMR5RM, IMR5WM },
	{ IMR6L, IMR6H, IMR6RM, IMR6WM },
	{ IMR7L, IMR7H, IMR7RM, IMR7WM }
};

static struct platform_device *pdev;

/**
 * module parameter
 *
 * If imr_enable is true:
 * 1. tear down the unlocked IMRs set up by firmware and bootloader
 * 2. set up kernel 'runtime' IMR
 * 3. perform final IMR locking according to imr_lock parameter.
 *
 * If imr_enable is false:
 * 1. same as above
 * 2. do not set up any new IMR
 * 3. do not lock any IMR (regardless of imr_lock parameter).
 *
 * Note: regardless of imr_enable value, the reporting sysfs interface is
 * always available.
 */
static int imr_enable = 1;
module_param(imr_enable, int, S_IRUGO|S_IWUSR);
MODULE_PARM_DESC(imr_enable, "set up IMRs (default=on)");

/**
 * module parameter
 *
 * If imr_lock is true, lock all the IMRs.
 */
static int imr_lock = 1;
module_param(imr_lock, int, S_IRUGO|S_IWUSR);
MODULE_PARM_DESC(imr_lock, "lock all IMRs (default=on)");

/* local IMR data structure */
struct qrk_imr_pack local_imr[IMR_NUM];

static unsigned short host_id;

/**
 * intel_qrk_imr_read_reg
 *
 * @param reg: register address
 * @return nothing
 *
 * return register value from input address.
 */
static uint32_t intel_qrk_imr_read_reg(uint8_t reg)
{
	uint32_t temp = 0;

	/* enable sidband spinlock for thread safe read */
	if (intel_qrk_sb_read_reg(SB_ID_MM, CFG_READ_OPCODE, reg, &temp, 1))
		return -EINVAL;

	return temp;
}

 /**
  *  intel_qrk_imr_is_occupied
  *
  *  @param i: IMR number
  *  @return true if IMR is enabled, false otherwise.
  *
  *  Check if the IMR is occupied.  Must be called by
  *  intel_clm_imr_latch_data
  *  as the caller sets up local_imr[] array first.
  */
static bool intel_qrk_imr_is_occupied(unsigned int i)
{
	if (PCI_DEVICE_ID_X1000_HOST_BRIDGE == host_id) {
		/* Occupied if not all the system agents are allowed.  */
		if (local_imr[i].addr.read_mask != IMR_READ_ENABLE_ALL ||
		    local_imr[i].addr.write_mask != IMR_WRITE_ENABLE_ALL) {
			return true;
		}
	} else {
		if (local_imr[i].addr.addr_low & IMR_EN) {
			return true;
		}
	}

	return false;
}

/**
 * intel_qrk_imr_latch_data
 *
 * @return nothing
 *
 * Populate IMR data structure from HW.
 */
static void intel_qrk_imr_latch_data(void)
{
	int i = 0;

	for (i = 0; i < IMR_NUM; i++) {
		local_imr[i].addr.addr_low =
			intel_qrk_imr_read_reg(imr_reg_value[i].imr_xl);
		local_imr[i].addr.addr_high_base =
			intel_qrk_imr_read_reg(imr_reg_value[i].imr_xh);
		local_imr[i].addr.read_mask =
			intel_qrk_imr_read_reg(imr_reg_value[i].imr_rm);
		local_imr[i].addr.write_mask =
			intel_qrk_imr_read_reg(imr_reg_value[i].imr_wm);

		local_imr[i].locked = false;
		if (local_imr[i].addr.addr_low & IMR_LOCK_BIT) {
			local_imr[i].locked = true;
		}

		local_imr[i].occupied = false;
		if (true == intel_qrk_imr_is_occupied(i)) {
			local_imr[i].occupied = true;
		} else {
			memcpy(local_imr[i].info, "NOT USED", MAX_INFO_SIZE);
		}
	}
}
/**
 * phyaddr_to_imraddr
 *
 * @param addr: physical memory address to be formatted into IMR addr register
 * @return 0 if success, error code otherwise
 *
 * 1. verify input memory address alignment
 * 2. shift and mask to match the format required by HW
 */
static int phyaddr_to_imraddr(uint32_t *addr)
{
	uint32_t val = *addr;

	if (val & (IMR_PAGE_SIZE - 1)) {
		pr_err("%s: addr 0x%x not IMR-page aligned\n", __func__, val);
		return -EINVAL;
	}

	*addr = (val >> IMR_ADDR_SHIFT) & IMR_ADDR_MASK;
	return 0;
}

/**
 * intel_qrk_imr_find_free_entry
 *
 * @return the next free imr slot
 */
static int intel_qrk_imr_find_free_entry(void)
{
	int i = 0;

	intel_qrk_imr_latch_data();

	for (i = 0; i < IMR_NUM; i++) {
		if ((!local_imr[i].occupied) && (!local_imr[i].locked))
			return i;
	}

	pr_err("%s: No more free IMR available.\n", __func__);
	return -ENOMEM;
}


/**
 * sb_write_chk
 *
 * @param id: Sideband identifier
 * @param cmd: Command to send to destination identifier
 * @param reg: Target register (std+extended) w/r to qrk_sb_id
 * @param data: Data to write to target register
 * @return nothing
 *
 * Set SB register and read back to verify register has been updated.
 */
static void sb_write_chk(qrk_sb_id id, u8 cmd, u32 reg, u32 data)
{
	u32 data_verify = 0;

	intel_qrk_sb_write_reg(id, cmd, reg, data, 1);
	intel_qrk_sb_read_reg(id, cmd, reg, &data_verify, 1);
	WARN_ON(data != data_verify);
}

/**
 * imr_add_entry
 *
 * @param id: imr slot id
 * @param hi: hi memory address
 * @param lo: lo memory address
 * @param read: read access mask
 * @param write: write access mask
 * @return nothing
 *
 */
static void imr_add_entry(int id, uint32_t hi, uint32_t lo,
				 uint32_t read, uint32_t write)
{
	u32 val = 0;

	if (PCI_DEVICE_ID_X1000_HOST_BRIDGE != host_id) {
		intel_qrk_sb_read_reg(SB_ID_MM, CFG_READ_OPCODE,
				imr_reg_value[id].imr_xl, &val, 1);
		val &= ~IMR_EN;
		sb_write_chk(SB_ID_MM, CFG_WRITE_OPCODE,
				imr_reg_value[id].imr_xl, val);
	}

	sb_write_chk(SB_ID_MM, CFG_WRITE_OPCODE, imr_reg_value[id].imr_rm,
			read);
	sb_write_chk(SB_ID_MM, CFG_WRITE_OPCODE, imr_reg_value[id].imr_wm,
			write);
	sb_write_chk(SB_ID_MM, CFG_WRITE_OPCODE, imr_reg_value[id].imr_xh,
			hi);
	sb_write_chk(SB_ID_MM, CFG_WRITE_OPCODE, imr_reg_value[id].imr_xl,
			lo);
}

/**
 * intel_qrk_imr_add_entry
 *
 * @param id: imr slot id
 * @param hi: hi memory address
 * @param lo: lo memory address
 * @param read: read access mask
 * @param write: write access mask
 *
 * Setup an IMR entry
 */
static void intel_qrk_imr_add_entry(int id, uint32_t hi,
		uint32_t lo, uint32_t read, uint32_t write, bool lock)
{
	imr_add_entry(id, hi, lo, read, write);

	if (lock) {
		lo |= IMR_LOCK_BIT;
		sb_write_chk(SB_ID_MM, CFG_WRITE_OPCODE,
			imr_reg_value[id].imr_xl, lo);
	}
}

/**
 * get_phy_addr
 * @return phy address value
 *
 * convert register format to physical address format.
 */
static uint32_t get_phy_addr(uint32_t reg_value)
{
	reg_value = ((reg_value & IMR_ADDR_MASK) << IMR_ADDR_SHIFT);
	return reg_value;
}

/**
 * intel_qrk_imr_teardown
 *
 * Tear down (unlocked) IMRs.
 */
static void intel_qrk_imr_teardown(void)
{
	int i = 0;
	int ret = 0;

	intel_qrk_imr_latch_data();

	for (i = 0; i < IMR_NUM; i++) {
		if ((local_imr[i].addr.addr_low & IMR_LOCK_BIT)) {
			continue;
		}
		ret = intel_qrk_remove_imr_entry(i);
		WARN(ret, "can't tear down IMR%d (%d)\n", i, ret);
	}
}

/**
 * imr_rm_entry
 *
 * @param id: imr slot id
 * @return nothing
 *
 */
static void imr_rm_entry(int id)
{
	u32 val = 0;

	if (PCI_DEVICE_ID_X1000_HOST_BRIDGE != host_id) {
		intel_qrk_sb_read_reg(SB_ID_MM, CFG_READ_OPCODE,
				imr_reg_value[id].imr_xl, &val, 1);
		val &= ~IMR_EN;
		sb_write_chk(SB_ID_MM, CFG_WRITE_OPCODE,
				imr_reg_value[id].imr_xl, val);
	}

	/* set default values to disabled imr slot */
	sb_write_chk(SB_ID_MM, CFG_WRITE_OPCODE,
			imr_reg_value[id].imr_rm, IMR_READ_ENABLE_ALL);
	sb_write_chk(SB_ID_MM, CFG_WRITE_OPCODE,
			imr_reg_value[id].imr_wm, IMR_WRITE_ENABLE_ALL);
	sb_write_chk(SB_ID_MM, CFG_WRITE_OPCODE,
			imr_reg_value[id].imr_xl, IMR_BASE_ADDR);
	sb_write_chk(SB_ID_MM, CFG_WRITE_OPCODE,
			imr_reg_value[id].imr_xh, IMR_BASE_ADDR);
}

/**
 * intel_qrk_remove_imr_entry
 *
 * @param id: imr slot id
 * @return zero if sucess
 *
 * remove imr slot based on input id
 */
int intel_qrk_remove_imr_entry(int id)
{

	intel_qrk_imr_latch_data();

	if (id >= IMR_NUM || local_imr[id].locked)
		return -EINVAL;

	imr_rm_entry(id);

	return 0;
}

/**
 * intel_qrk_imr_alloc
 *
 * @param high_base: base address (1kB IMR page) of high boundary
 * @param low: low boundary of memorry address
 * @param read: IMR read mask value
 * @param write: IMR write mask value
 * @param info: IMR usage info
 * @param lock: lock flag
 * @return id: -errcode if error, allocated IMR ID otherwise.
 *
 * setup the next available IMR with customized read and write masks
 */
int intel_qrk_imr_alloc(uint32_t high_base, uint32_t low, uint32_t read,
			uint32_t write, unsigned char *info, bool lock)
{
	int ret = 0;
	int id = 0;

	if (info == NULL)
		return -EINVAL;

	if ((low & IMR_LOCK_BIT) || (read == 0 || write == 0)) {
		pr_err("%s: Invalid acces mode\n", __func__);
		return -EINVAL;
	}

	/* Validate physical address and format it to IMR address */
	ret = phyaddr_to_imraddr(&high_base);
	if (ret) {
		return ret;
	}
	ret = phyaddr_to_imraddr(&low);
	if (ret) {
		return ret;
	}

	/* Find a free entry */
	ret = intel_qrk_imr_find_free_entry();
	if (ret < 0) {
		return ret;
	}
	id = ret;

	/* Add entry - locking as necessary */
	intel_qrk_imr_add_entry(id, high_base, low,
				(read & IMR_READ_ENABLE_ALL),
				write, lock);

	/* Name the new entry */
	memcpy(local_imr[id].info, info, MAX_INFO_SIZE);

	pr_info("IMR alloc id=%d phys=[0x%08x-0x%08x] %s\n", id,
		get_phy_addr(low), get_phy_addr(high_base) + IMR_PAGE_SIZE,
		lock ? "locked" : "unlocked");

	return id;
}

/**
 * intel_qrk_imr_init_data
 *
 * @return nothing
 * initialize local_imr data structure
 */
static void intel_qrk_imr_init_data(void)
{
	int i = 0;
	char *res_str = "System Reserved Region";

	memset(local_imr, 0, sizeof(struct qrk_imr_pack)*IMR_NUM);

	for (i = 0; i < IMR_NUM; i++) {
		local_imr[i].reg = imr_reg_value[i];
		memcpy(local_imr[i].info, res_str, MAX_INFO_SIZE);
	}

	intel_qrk_imr_latch_data();
}

/**
 * intel_qrk_imr_lockall
 *
 * lock up all un-locked IMRs
 */
static void intel_qrk_imr_lockall(void)
{
	int i = 0;
	uint32_t temp_addr;

	intel_qrk_imr_latch_data();

	/* Cycle through IMRs locking whichever are unlocked */
	for (i = 0; i < IMR_NUM; i++) {

		temp_addr = local_imr[i].addr.addr_low;
		if (!(temp_addr & IMR_LOCK_BIT)) {
			pr_debug("%s: locking IMR %d\n", __func__, i);
			temp_addr |= IMR_LOCK_BIT;
			sb_write_chk(SB_ID_MM, CFG_WRITE_OPCODE,
					local_imr[i].reg.imr_xl, temp_addr);
		}
	}
}

/**
 * intel_qrk_imr_stat_show
 *
 * @param dev: pointer to device
 * @param attr: attribute pointer
 * @param buf: output buffer
 * @return number of bytes successfully read
 *
 * Populates IMR state via /sys/devices/platform/intel-qrk-imr/status
 */
static int intel_qrk_imr_stat_show(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{
	int len = 0;
	int i = 0;
	int imr_size_kb, size, count = PAGE_SIZE;
	uint32_t hi_base_phy_addr, lo_phy_addr;

	intel_qrk_imr_latch_data();

	for (i = 0; i < IMR_NUM; i++) {

		/* read back the actual  input physical memory address */
		hi_base_phy_addr =
			get_phy_addr(local_imr[i].addr.addr_high_base);
		lo_phy_addr = get_phy_addr(local_imr[i].addr.addr_low);

		/* the IMR high addr reg stores the high base address, ie it
		 * covers the 1kB above it as well.
		 */
		imr_size_kb =
			((hi_base_phy_addr - lo_phy_addr) / IMR_PAGE_SIZE) + 1;

		size = snprintf(buf+len, count,
				"imr - id           : %d\n"
				"info               : %s\n"
				"occupied           : %s\n"
				"locked             : %s\n"
				"size               : %d kB\n"
				"hi base addr (phy) : 0x%08x\n"
				"lo addr (phy)      : 0x%08x\n"
				"hi base addr (virt): 0x%08x\n"
				"lo addr (virt)     : 0x%08x\n"
				"read mask          : 0x%08x\n"
				"write mask         : 0x%08x\n\n",
				i,
				local_imr[i].info,
				local_imr[i].occupied ? "yes" : "no",
				local_imr[i].locked ? "yes" : "no",
				imr_size_kb,
				hi_base_phy_addr,
				lo_phy_addr,
				(uint32_t)phys_to_virt(hi_base_phy_addr),
				(uint32_t)phys_to_virt(lo_phy_addr),
				local_imr[i].addr.read_mask,
				local_imr[i].addr.write_mask);
		len += size;
		count -= size;
	}
	return len;
}

/**
 * intel_qrk_imr_runt_kerndata_setup
 *
 * @return IMR ID if success, -errcode if failure
 *
 * Setup imr for kernel text, read only data section
 *
 * The read only data (rodata) section placed between text and initialized data
 * section by kernel.
 */
static int intel_qrk_imr_runt_kerndata_setup(void)
{
	uint32_t hi;
	uint32_t lo;
	int ret;

	hi = (uint32_t)virt_to_phys(
		(void *) kallsyms_lookup_name("__init_begin"));
	lo = (uint32_t)virt_to_phys(
		(void *) kallsyms_lookup_name("_text"));

	ret = intel_qrk_imr_alloc((hi - IMR_PAGE_SIZE), lo,
				IMR_DEFAULT_READ, IMR_DEFAULT_WRITE,
				"KERNEL RUNTIME DATA", 1);

	return ret;
}

static struct device_attribute dev_attr_stats = {
	.attr = {
		.name = "stat",
		.mode = 0444, },
	.show = intel_qrk_imr_stat_show,
};

static struct attribute *platform_attributes[] = {
	&dev_attr_stats.attr,
	NULL,
};

static struct attribute_group imr_attrib_group = {
	.attrs = platform_attributes
};

/**
 * intel_qrk_imr_init
 *
 * @param dev_id: Host Bridge's PCI device ID
 * @return 0 success < 0 failue
 *
 * module entry point
 */
int intel_qrk_imr_init(unsigned short dev_id)
{
	int ret;

	host_id = dev_id;

	pdev = platform_device_alloc(DRIVER_NAME, -1);
	if (!pdev)
		return -ENOMEM;

	ret = platform_device_add(pdev);
	if (ret)
		goto fail_platform_add;

	/* initialise local imr data structure */
	intel_qrk_imr_init_data();
	ret = sysfs_create_group(&pdev->dev.kobj, &imr_attrib_group);
	if (ret)
		goto fail_create_sysfs;

	if (imr_enable) {
		/* Setup a locked kernel 'runtime' IMR */
		ret = intel_qrk_imr_runt_kerndata_setup();
		WARN(ret < 0, "can't set up runtime IMR (%d)\n", ret);
	}

	/* Remove any other (unlocked) IMR */
	intel_qrk_imr_teardown();

	if (imr_enable && imr_lock) {
		intel_qrk_imr_lockall();
	}

	return 0;

fail_create_sysfs:
	platform_device_del(pdev);
fail_platform_add:
	platform_device_put(pdev);
	return ret;
}

MODULE_DESCRIPTION("Intel Quark SOC IMR driver");
MODULE_AUTHOR("Intel Corporation");
MODULE_LICENSE("Dual BSD/GPL");

