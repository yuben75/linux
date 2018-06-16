/*
 * PRU Ethernet Driver sysfs file
 *
 * Copyright (C) 2018 Texas Instruments Incorporated - http://www.ti.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/netdevice.h>
#include <linux/sysfs.h>
#include "prueth.h"

static struct attribute nsp_credit = {
	.name = "nsp_credit",
	.mode = 0666,
};

static struct attribute *nsp_credit_attrs[] = {
	&nsp_credit,
	NULL,
};

static ssize_t nsp_credit_store(struct kobject *kobj,
				struct attribute *attr,
				const char *buffer, size_t size)
{
	struct prueth_emac *emac = container_of(kobj, struct prueth_emac, kobj);
	u32 val;

	if (!PRUETH_HAS_RED(emac->prueth))
		return -EINVAL;

	if (kstrtou32(buffer, 0, &val))
		return -EINVAL;

	if (val)
		emac->nsp_credit =
			(val << PRUETH_NSP_CREDIT_SHIFT) | PRUETH_NSP_ENABLE;
	else
		emac->nsp_credit = PRUETH_NSP_DISABLE;
	return size;
}

static ssize_t nsp_credit_show(struct kobject *kobj,
			       struct attribute *attr, char *buffer)
{
	struct prueth_emac *emac = container_of(kobj, struct prueth_emac, kobj);

	return snprintf(buffer, PAGE_SIZE, "%u\n",
			emac->nsp_credit >> PRUETH_NSP_CREDIT_SHIFT);
}

static const struct sysfs_ops nsp_credit_sysfs_ops = {
	.show = nsp_credit_show,
	.store = nsp_credit_store,
};

static struct kobj_type nsp_credit_kobj_type = {
	.sysfs_ops = &nsp_credit_sysfs_ops,
	.default_attrs = nsp_credit_attrs,
};

int prueth_sysfs_init(struct prueth_emac *emac)
{
	struct device *dev = emac->prueth->dev;

	return kobject_init_and_add(&emac->kobj,
				    &nsp_credit_kobj_type,
				    &dev->kobj, netdev_name(emac->ndev));
}

void prueth_remove_sysfs_entries(struct prueth_emac *emac)
{
	kobject_put(&emac->kobj);
}
