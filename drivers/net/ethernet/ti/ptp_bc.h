/* SPDX-License-Identifier: GPL-2.0 */
/* Texas Instruments Ethernet Switch Driver
 *
 * Copyright (C) 2017 Texas Instruments
 *
 */
#ifndef _TI_PTP_BC_H_
#define _TI_PTP_BC_H_

/* PTP_BC_CLOCK_TYPE is used to identify the clock for multiplexer
 * output selection.
 * The PTP clock should pass its clock type during clock registration
 */
#define PTP_BC_CLOCK_TYPE_GMAC		0
#define PTP_BC_CLOCK_TYPE_PRUICSS1	1
#define PTP_BC_CLOCK_TYPE_PRUICSS2	2

#if IS_ENABLED(CONFIG_TI_PTP_BC)
int ptp_bc_clock_register(int clktype);
void ptp_bc_clock_unregister(int clkid);
bool ptp_bc_clock_sync_enable(int clkid, int enable);
#else
static int ptp_bc_clock_register(int clktype)
{
	return -1;
}

static void ptp_bc_clock_unregister(int clkid)
{
}

static bool ptp_bc_clock_sync_enable(int clkid, int enable)
{
	return true;
}
#endif
#endif
