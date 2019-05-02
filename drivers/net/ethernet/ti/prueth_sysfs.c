// SPDX-License-Identifier: GPL-2.0
/* PRU Ethernet Driver sysfs file
 *
 * Copyright (C) 2018 Texas Instruments Incorporated - http://www.ti.com
 *
 */
#include <linux/netdevice.h>
#include <linux/sysfs.h>
#include "prueth.h"

#define nsp_credit_to_emac(attr) \
	container_of(attr, struct prueth_emac, nsp_credit_attr)

static ssize_t nsp_credit_store(struct device *dev,
				struct device_attribute *attr,
				const char *buffer, size_t count)
{
	struct prueth_emac *emac = nsp_credit_to_emac(attr);
	u32 val;

	if (kstrtou32(buffer, 0, &val))
		return -EINVAL;

	if (val)
		emac->nsp_credit =
			(val << PRUETH_NSP_CREDIT_SHIFT) | PRUETH_NSP_ENABLE;
	else
		emac->nsp_credit = PRUETH_NSP_DISABLE;

	return count;
}

static ssize_t nsp_credit_show(struct device *dev,
			       struct device_attribute *attr,
			       char *buffer)
{
	struct prueth_emac *emac = nsp_credit_to_emac(attr);

	return snprintf(buffer, PAGE_SIZE, "%u\n",
			emac->nsp_credit >> PRUETH_NSP_CREDIT_SHIFT);
}
DEVICE_ATTR_RW(nsp_credit);

int prueth_sysfs_init(struct prueth_emac *emac)
{
	int ret;

	emac->nsp_credit_attr = dev_attr_nsp_credit;
	sysfs_attr_init(&emac->nsp_credit_attr.attr);
	ret = device_create_file(&emac->ndev->dev, &emac->nsp_credit_attr);
	if (ret < 0)
		return ret;

	return 0;
}

void prueth_remove_sysfs_entries(struct prueth_emac *emac)
{
	device_remove_file(&emac->ndev->dev, &emac->nsp_credit_attr);
}
