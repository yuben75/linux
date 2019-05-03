/* SPDX-License-Identifier: GPL-2.0 */

/* PRU ICSS Ethernet driver
 *
 * Copyright (C) 2015-2018 Texas Instruments Incorporated - http://www.ti.com
 */

#ifndef __NET_TI_PRUETH_H
#define __NET_TI_PRUETH_H

#include <linux/kobject.h>
#include <linux/hrtimer.h>
#include <linux/kthread.h>
#include <linux/pruss.h>
#include <linux/if_ether.h>
#ifdef CONFIG_PREEMPT_RT_FULL
#include <linux/swork.h>
#endif
#include "icss_switch.h"

#define PRUETH_NUMQUEUES	5

/* ECAP registers */
#define ECAP_TSCTR                      0
#define ECAP_CAP1                       8
#define ECAP_CAP2                       0xC
#define ECAP_ECCTL1                     0x28
#define ECAP_ECCTL2                     0x2A
#define ECAP_ECEINT                     0x2C
#define ECAP_ECCLR                      0x30

#define ECAP_ECCTL2_TSCTRSTOP_MASK      0x10
#define ECAP_ECCTL2_CAP_APWM_MASK       0x200
#define ECAP_ECCLR_INT_MASK             1
#define ECAP_ECCLR_CEVT1_MASK           2
#define ECAP_ECCLR_CEVT2_MASK           4
#define ECAP_ECCLR_CEVT3_MASK           8
#define ECAP_ECCLR_CEVT4_MASK           0x10
#define ECAP_ECCLR_CNTOVF_MASK          0x20
#define ECAP_ECCLR_PRDEQ_MASK           0x40
#define ECAP_ECCLR_CMPEQ_MASK           0x80
#define ECAP_ECEINT_PRDEQ_MASK          0x40

#define ECAP_ECCTL2_INIT_VAL           (ECAP_ECCTL2_TSCTRSTOP_MASK | \
					ECAP_ECCTL2_CAP_APWM_MASK)
#define ECAP_CAP2_MAX_COUNT            0xFFFFFFFF
#define ECAP_ECCLR_CLR_VAL             0xFF

/* in usec */
#define DEFAULT_RX_TIMEOUT_USEC       123
/* Duration of 3 frames of 1528 bytes each. If we go beyond this,
 * receive buffer overflow may happen assuming 4 MTU buffer. So
 * set this as the limit
 */
#define MAX_RX_TIMEOUT_USEC            (123 * 3)
/* ECAP has 200Mhz clock. So each tick is 5 nsec. i.e 1000/200 */
#define ECAP_TICK_NSEC                  5

/**
 * struct prueth_queue_desc - Queue descriptor
 * @rd_ptr:	Read pointer, points to a buffer descriptor in Shared PRU RAM.
 * @wr_ptr:	Write pointer, points to a buffer descriptor in Shared PRU RAM.
 * @busy_s:	Slave queue busy flag, set by slave(us) to request access from
 *		master(PRU).
 * @status:	Bit field status register, Bits:
 *			0: Master queue busy flag.
 *			1: Packet has been placed in collision queue.
 *			2: Packet has been discarded due to overflow.
 * @max_fill_level:	Maximum queue usage seen.
 * @overflow_cnt:	Count of queue overflows.
 *
 * Each port has up to 4 queues with variable length. The queue is processed
 * as ring buffer with read and write pointers. Both pointers are address
 * pointers and increment by 4 for each buffer descriptor position. Queue has
 * a length defined in constants and a status.
 */
struct prueth_queue_desc {
	u16 rd_ptr;
	u16 wr_ptr;
	u8 busy_s;
	u8 status;
	u8 max_fill_level;
	u8 overflow_cnt;
} __packed;

/**
 * struct prueth_queue - Information about a queue in memory
 * @buffer_offset: buffer offset in OCMC RAM
 * @queue_desc_offset: queue descriptor offset in Shared RAM
 * @buffer_desc_offset: buffer descriptors offset in Shared RAM
 * @buffer_desc_end: end address of buffer descriptors in Shared RAM
 */
struct prueth_queue_info {
	u16 buffer_offset;
	u16 queue_desc_offset;
	u16 buffer_desc_offset;
	u16 buffer_desc_end;
} __packed;

/**
 * struct prueth_packet_info - Info about a packet in buffer
 * @shadow: this packet is stored in the collision queue
 * @port: port packet is on
 * @length: length of packet
 * @broadcast: this packet is a broadcast packet
 * @error: this packet has an error
 */
struct prueth_packet_info {
	bool shadow;
	unsigned int port;
	unsigned int length;
	bool broadcast;
	bool error;
};

/**
 * struct port_statistics - Statistics structure for capturing statistics
 *			    on PRUs
 * @tx_bcast: Number of broadcast packets sent
 * @tx_mcast:Number of multicast packets sent
 * @tx_ucast:Number of unicast packets sent
 *
 * @tx_octets:Number of undersized frames rcvd
 *
 * @rx_bcast:Number of broadcast packets rcvd
 * @rx_mcast:Number of multicast packets rcvd
 * @rx_ucast:Number of unicast packets rcvd
 *
 * @rx_octets:Number of Rx packets
 *
 * @tx64byte:Number of 64 byte packets sent
 * @tx65_127byte:Number of 65-127 byte packets sent
 * @tx128_255byte:Number of 128-255 byte packets sent
 * @tx256_511byte:Number of 256-511 byte packets sent
 * @tx512_1023byte:Number of 512-1023 byte packets sent
 * @tx1024byte:Number of 1024 and larger size packets sent
 *
 * @rx64byte:Number of 64 byte packets rcvd
 * @rx65_127byte:Number of 65-127 byte packets rcvd
 * @rx128_255byte:Number of 128-255 byte packets rcvd
 * @rx256_511byte:Number of 256-511 byte packets rcvd
 * @rx512_1023byte:Number of 512-1023 byte packets rcvd
 * @rx1024byte:Number of 1024 and larger size packets rcvd
 *
 * @late_coll:Number of late collisions(Half Duplex)
 * @single_coll:Number of single collisions (Half Duplex)
 * @multi_coll:Number of multiple collisions (Half Duplex)
 * @excess_coll:Number of excess collisions(Half Duplex)
 *
 * @rx_misalignment_frames:Number of non multiple of 8 byte frames rcvd
 * @stormprev_counter:Number of packets dropped because of Storm Prevention
 * @mac_rxerror:Number of MAC receive errors
 * @sfd_error:Number of invalid SFD
 * @def_tx:Number of transmissions deferred
 * @mac_txerror:Number of MAC transmit errors
 * @rx_oversized_frames:Number of oversized frames rcvd
 * @rx_undersized_frames:Number of undersized frames rcvd
 * @rx_crc_frames:Number of CRC error frames rcvd
 * @dropped_packets:Number of packets dropped due to link down on opposite port
 *
 * @tx_hwq_overflow:Hardware Tx Queue (on PRU) over flow count
 * @tx_hwq_underflow:Hardware Tx Queue (on PRU) under flow count
 *
 * @u32 cs_error: Number of carrier sense errors
 * @sqe_test_error: Number of MAC receive errors
 *
 * The fields here are aligned here so that it's consistent
 * with the memory layout in PRU DRAM, this is to facilitate easy
 * memcpy. Don't change the order of the fields.
 */
struct port_statistics {
	u32 tx_bcast;
	u32 tx_mcast;
	u32 tx_ucast;

	u32 tx_octets;

	u32 rx_bcast;
	u32 rx_mcast;
	u32 rx_ucast;

	u32 rx_octets;

	u32 tx64byte;
	u32 tx65_127byte;
	u32 tx128_255byte;
	u32 tx256_511byte;
	u32 tx512_1023byte;
	u32 tx1024byte;

	u32 rx64byte;
	u32 rx65_127byte;
	u32 rx128_255byte;
	u32 rx256_511byte;
	u32 rx512_1023byte;
	u32 rx1024byte;

	u32 late_coll;
	u32 single_coll;
	u32 multi_coll;
	u32 excess_coll;

	u32 rx_misalignment_frames;
	u32 stormprev_counter;
	u32 mac_rxerror;
	u32 sfd_error;
	u32 def_tx;
	u32 mac_txerror;
	u32 rx_oversized_frames;
	u32 rx_undersized_frames;
	u32 rx_crc_frames;
	u32 dropped_packets;

	u32 tx_hwq_overflow;
	u32 tx_hwq_underflow;

	u32 cs_error;
	u32 sqe_test_error;
} __packed;

enum pruss_device {
	PRUSS_AM57XX = 0,
	PRUSS_AM4376,
	PRUSS_AM3359,
	PRUSS_K2G
};

#define PRUSS0 0
#define PRUSS1 1
#define PRUSS2 2

#define MS_TO_NS(msec)		((msec) * 1000 * 1000)
/* NSP (Network Storm Prevention) timer re-uses NT timer */
#define PRUETH_DEFAULT_NSP_TIMER_MS	100
#define PRUETH_DEFAULT_NSP_TIMER_COUNT	\
		(PRUETH_DEFAULT_NSP_TIMER_MS / 10)
#define PRUETH_NSP_CREDIT_SHIFT       8
#define PRUETH_NSP_ENABLE             1
#define PRUETH_NSP_DISABLE            0
#define PRUETH_NSP_EN_MASK            0xff

/* In switch mode there are 3 real ports i.e. 3 mac addrs.
 * however Linux sees only the host side port. The other 2 ports
 * are the switch ports.
 * In emac mode there are 2 real ports i.e. 2 mac addrs.
 * Linux sees both the ports.
 */
enum prueth_port {
	PRUETH_PORT_HOST = 0,	/* host side port */
	PRUETH_PORT_MII0,	/* physical port MII 0 */
	PRUETH_PORT_MII1,	/* physical port MII 1 */
};

enum prueth_mac {
	PRUETH_MAC0 = 0,
	PRUETH_MAC1,
	PRUETH_NUM_MACS,
};

/* In both switch & emac modes there are 3 port queues
 * EMAC mode:
 *	RX packets for both MII0 & MII1 ports come on
 *	QUEUE_HOST.
 *	TX packets for MII0 go on QUEUE_MII0, TX packets
 *	for MII1 go on QUEUE_MII1.
 * Switch mode:
 *	Host port RX packets come on QUEUE_HOST
 *	TX packets might have to go on MII0 or MII1 or both.
 *	MII0 TX queue is QUEUE_MII0 and MII1 TX queue is
 *	QUEUE_MII1.
 */
enum prueth_port_queue_id {
	PRUETH_PORT_QUEUE_HOST = 0,
	PRUETH_PORT_QUEUE_MII0,
	PRUETH_PORT_QUEUE_MII1,
	PRUETH_PORT_QUEUE_MAX,
};

/* Each port queue has 4 queues and 1 collision queue */
enum prueth_queue_id {
	PRUETH_QUEUE1 = 0,
	PRUETH_QUEUE2,
	PRUETH_QUEUE3,
	PRUETH_QUEUE4,
	PRUETH_COLQUEUE,	/* collision queue */
};

/* PRUeth memory range identifiers */
enum prueth_mem {
	PRUETH_MEM_DRAM0 = 0,
	PRUETH_MEM_DRAM1,
	PRUETH_MEM_SHARED_RAM,
	PRUETH_MEM_ECAP,
	PRUETH_MEM_OCMC,
	PRUETH_MEM_MAX,
};

enum fw_revision {
	FW_REV_V1_0 = 0,
	FW_REV_V2_1
};

/* Firmware offsets/size information */
struct prueth_fw_offsets {
	u32 vlan_ctrl_byte;
	u32 vlan_filter_tbl;
	u32 mc_ctrl_byte;
	u32 mc_filter_mask;
	u32 mc_filter_tbl;
};

/**
 * struct prueth_private_data - PRU Ethernet private data
 * @fw_names: firmware names to be used for PRUSS ethernet usecases
 */
struct prueth_private_data {
	enum pruss_device driver_data;
	const char *fw_names[PRUSS_NUM_PRUS];
	enum fw_revision fw_rev;
};

/* data for each emac port */
struct prueth_emac {
	struct prueth *prueth;
	struct net_device *ndev;
	u8 mac_addr[6];
	struct napi_struct napi;
	u32 msg_enable;

	int link;
	int speed;
	int duplex;

	const char *phy_id;
	struct device_node *phy_node;
	int phy_if;
	struct phy_device *phydev;
	struct rproc *pru;

	enum prueth_port port_id;
	enum prueth_port_queue_id tx_port_queue;

	enum prueth_queue_id rx_queue_start;
	enum prueth_queue_id rx_queue_end;

	enum prueth_mem dram;

	int rx_irq;
	int tx_irq;

	struct prueth_queue_desc __iomem *rx_queue_descs;
	struct prueth_queue_desc __iomem *tx_queue_descs;

	struct port_statistics stats; /* stats holder when i/f is down */

	spinlock_t lock;	/* serialize access */
	spinlock_t addr_lock;
	unsigned int nsp_timer_count;
	unsigned int nsp_credit;
	unsigned char mc_mac_mask[ETH_ALEN];
	struct kobject kobj;
#ifdef CONFIG_DEBUG_FS
	struct dentry *root_dir;
	struct dentry *vlan_filter_file;
#endif
#ifdef CONFIG_SYSFS
	struct device_attribute nsp_credit_attr;
#endif

	u32 rx_int_pacing_offset;
	unsigned int rx_pacing_timeout;
};

/**
 * struct prueth - PRUeth structure
 * @dev: device
 * @pruss: pruss handle
 * @pru0: rproc instance to PRU0
 * @pru1: rproc instance to PRU1
 * @mem: PRUSS memory resources we need to access
 * @sram_pool: OCMC ram pool for buffers
 * @mii_rt: regmap to mii_rt block
 * @iep: regmap to IEP block
 *
 * @eth_node: node for each emac node
 * @emac: emac data for three ports, one host and two physical
 * @registered_netdevs: net device for each registered emac
 * @fw_data: firmware names to be used with PRU remoteprocs
 * @pruss_id: PRUSS instance id
 */
struct prueth {
	struct device *dev;
	struct pruss *pruss;
	struct rproc *pru0, *pru1;
	struct pruss_mem_region mem[PRUETH_MEM_MAX];
	struct gen_pool *sram_pool;
	struct regmap *mii_rt;
	struct regmap *iep;

	struct device_node *eth_node[PRUETH_NUM_MACS];
	struct prueth_emac *emac[PRUETH_NUM_MACS];
	struct net_device *registered_netdevs[PRUETH_NUM_MACS];
	int pruss_id;
	const struct prueth_private_data *fw_data;
	struct prueth_fw_offsets *fw_offsets;
	unsigned int emac_configured;
	unsigned int tbl_check_period;
	struct hrtimer tbl_check_timer;
};

#ifdef CONFIG_SYSFS
int prueth_sysfs_init(struct prueth_emac *emac);
void prueth_remove_sysfs_entries(struct prueth_emac *emac);
#else
static inline int prueth_sysfs_init(struct prueth_emac *emac)
{
	return 0;
}

static inline void prueth_remove_sysfs_entries(struct prueth_emac *emac)
{}
#endif
#endif /* __NET_TI_PRUETH_H */
