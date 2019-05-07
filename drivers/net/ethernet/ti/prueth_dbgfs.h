/* SPDX-License-Identifier: GPL-2.0 */
/* PRU Ethernet Driver
 *
 * Copyright (C) 2018 Texas Instruments Incorporated - http://www.ti.com
 *
 */
#if IS_ENABLED(CONFIG_DEBUG_FS)
int prueth_dualemac_debugfs_init(struct prueth_emac *emac);
void prueth_dualemac_debugfs_term(struct prueth_emac *emac);
int prueth_debugfs_init(struct prueth_emac *emac);
void prueth_debugfs_term(struct prueth_emac *emac);

#else

int prueth_dualemac_debugfs_init(struct prueth_emac *emac)
{
	return 0;
}

void prueth_dualemac_debugfs_term(struct prueth_emac *emac)
{}

static inline int prueth_debugfs_init(struct prueth_emac *emac)
{
	return 0;
}

static inline void prueth_debugfs_term(struct prueth_emac *emac)
{}
#endif
