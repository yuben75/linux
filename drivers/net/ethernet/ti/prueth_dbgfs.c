// SPDX-License-Identifier: GPL-2.0
/* PRU Ethernet Driver debugfs files
 *
 * Copyright (C) 2018 Texas Instruments Incorporated - http://www.ti.com
 *
 */

#include <linux/debugfs.h>
#include <linux/pruss.h>
#include <linux/etherdevice.h>
#include "icss_switch.h"
#include "prueth.h"
#include "prueth_node_tbl.h"
#include "hsr_prp_firmware.h"

#if IS_ENABLED(CONFIG_DEBUG_FS)

/* prueth_vlan_filter_show - Formats and prints vlan_filter entries
 */
static int
prueth_vlan_filter_show(struct seq_file *sfp, void *data)
{
	struct prueth_emac *emac = (struct prueth_emac *)sfp->private;
	struct prueth *prueth = emac->prueth;
	void __iomem *ram;
	u8 val, mask;
	int i, j;
	u32 vlan_ctrl_byte = prueth->fw_offsets->vlan_ctrl_byte;
	u32 vlan_filter_tbl = prueth->fw_offsets->vlan_filter_tbl;

	if (PRUETH_IS_EMAC(prueth))
		ram = prueth->mem[emac->dram].va;
	else
		ram = prueth->mem[PRUETH_MEM_SHARED_RAM].va;

	val = readb(ram + vlan_ctrl_byte);
	seq_printf(sfp, "VLAN Filter : %s",
		   val & BIT(VLAN_FLTR_CTRL_SHIFT) ?
			 "enabled\n" : "disabled\n");
	if (val & BIT(VLAN_FLTR_CTRL_SHIFT)) {
		seq_printf(sfp, "VLAN Filter untagged : %s",
			   val & BIT(VLAN_FLTR_UNTAG_HOST_RCV_CTRL_SHIFT) ?
			   "not allowed to Host\n" : "allowed to Host\n");
		seq_printf(sfp, "VLAN Filter priority tagged: %s",
			   val & BIT(VLAN_FLTR_PRIOTAG_HOST_RCV_CTRL_SHIFT) ?
			   "not allowed to Host\n" : "allowed to Host\n");
	}
	if (val) {
		for (i = 0; i < VLAN_FLTR_TBL_SIZE; i++) {
			val = readb(ram + vlan_filter_tbl + i);
			if (!(i % 8))
				seq_printf(sfp, "\n%5d: ", i * 8);

			for (j = 0; j < 8; j++) {
				mask = BIT(j);
				if (mask & val)
					seq_printf(sfp, "%1x", 1);
				else
					seq_printf(sfp, "%1x", 0);
			}
		}
	}
	seq_puts(sfp, "\n");

	return 0;
}

/* prueth_vlan_filter_open - Open the vlan_filter file
 *
 * Description:
 * This routine opens a debugfs file vlan_filter
 */
static int
prueth_vlan_filter_open(struct inode *inode, struct file *filp)
{
	return single_open(filp, prueth_vlan_filter_show,
			   inode->i_private);
}

static const struct file_operations prueth_vlan_filter_fops = {
	.owner  = THIS_MODULE,
	.open   = prueth_vlan_filter_open,
	.read   = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

/* prueth_mc_filter_show - Formats and prints mc_filter entries
 */
static int
prueth_mc_filter_show(struct seq_file *sfp, void *data)
{
	struct prueth_emac *emac = (struct prueth_emac *)sfp->private;
	struct prueth *prueth = emac->prueth;
	void __iomem *ram;
	u8 val;
	int i;
	u32 mc_ctrl_byte = prueth->fw_offsets->mc_ctrl_byte;
	u32 mc_filter_mask = prueth->fw_offsets->mc_filter_mask;
	u32 mc_filter_tbl = prueth->fw_offsets->mc_filter_tbl;

	if (PRUETH_IS_EMAC(prueth))
		ram = prueth->mem[emac->dram].va;
	else
		ram = prueth->mem[PRUETH_MEM_DRAM1].va;

	val = readb(ram + mc_ctrl_byte);

	seq_printf(sfp, "MC Filter : %s", val ? "enabled\n" : "disabled\n");
	seq_puts(sfp, "MC Mask : ");
	for (i = 0; i < 6; i++) {
		val = readb(ram + mc_filter_mask + i);
		if (i == 5)
			seq_printf(sfp, "%x", val);
		else
			seq_printf(sfp, "%x:", val);
	}
	seq_puts(sfp, "\n");

	val = readb(ram + mc_ctrl_byte);
	seq_puts(sfp, "MC Filter table below 1 - Allowed, 0 - Dropped\n");

	if (val) {
		for (i = 0; i < MULTICAST_TABLE_SIZE; i++) {
			val = readb(ram + mc_filter_tbl + i);
			if (!(i % 16))
				seq_printf(sfp, "\n%3x: ", i);
			seq_printf(sfp, "%d ", val);
		}
	}
	seq_puts(sfp, "\n");

	return 0;
}

/* prueth_mc_filter_open - Open the mc_filter file
 *
 * Description:
 * This routine opens a debugfs file mc_filter
 */
static int
prueth_mc_filter_open(struct inode *inode, struct file *filp)
{
	return single_open(filp, prueth_mc_filter_show,
			   inode->i_private);
}

static const struct file_operations prueth_mc_filter_fops = {
	.owner	= THIS_MODULE,
	.open	= prueth_mc_filter_open,
	.read	= seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

/* prueth_dualemac_debugfs_term - Tear down debugfs intrastructure for dual emac
 *
 * Description:
 * When Debufs is configured this routine removes debugfs file system
 * elements that are specific to dual emac
 */
void
prueth_dualemac_debugfs_term(struct prueth_emac *emac)
{
	debugfs_remove(emac->vlan_filter_file);
	debugfs_remove(emac->mc_filter_file);
	emac->vlan_filter_file = NULL;
	emac->mc_filter_file = NULL;
}

/* prueth_dualemac_debugfs_init - create  debugfs file for dual emac
 *
 * Description:
 * When debugfs is configured this routine creates dual emac debugfs files
 */
int prueth_dualemac_debugfs_init(struct prueth_emac *emac)
{
	int rc = -1;
	struct dentry *de;

	if (emac->root_dir && !emac->mc_filter_file &&
	    !emac->vlan_filter_file) {
		de = debugfs_create_file("mc_filter", S_IFREG | 0444,
					 emac->root_dir, emac,
					 &prueth_mc_filter_fops);
		if (!de) {
			netdev_err(emac->ndev,
				   "Cannot create mc_filter file\n");
			return rc;
		}
		emac->mc_filter_file = de;

		de = debugfs_create_file("vlan_filter", S_IFREG | 0444,
					 emac->root_dir, emac,
					 &prueth_vlan_filter_fops);
		if (!de) {
			netdev_err(emac->ndev,
				   "Cannot create vlan_filter file\n");
			goto error;
		}

		emac->vlan_filter_file = de;
	}
	return 0;
error:
	prueth_dualemac_debugfs_term(emac);
	return rc;
}

/* prueth_debugfs_term - Remove debugfs files
 *
 * Description:
 * Routine to clean up the debugfs files
 */
void
prueth_debugfs_term(struct prueth_emac *emac)
{
	debugfs_remove_recursive(emac->root_dir);
	emac->vlan_filter_file = NULL;
	emac->mc_filter_file = NULL;
}

/* prueth_debugfs_init - create  debugfs files to display debug info
 *
 * Description:
 * This routine creates various debugfs files to show debug information
 * about the driver.
 */
int prueth_debugfs_init(struct prueth_emac *emac)
{
	int rc = -1;
	struct dentry *de;
	char name[32];

	memset(name, 0, sizeof(name));
	sprintf(name, "prueth-");
	strncat(name, emac->ndev->name, sizeof(name) - 1);
	de = debugfs_create_dir(name, NULL);

	if (!de) {
		netdev_err(emac->ndev,
			   "Cannot create debugfs dir name %s\n",
			   name);
		return rc;
	}
	emac->root_dir = de;

	rc = prueth_dualemac_debugfs_init(emac);
	if (rc)
		goto error;

	return 0;
error:
	prueth_debugfs_term(emac);
	return rc;
}

#endif
