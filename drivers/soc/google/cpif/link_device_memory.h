/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2010 Samsung Electronics.
 *
 */

#ifndef __MODEM_LINK_DEVICE_MEMORY_H__
#define __MODEM_LINK_DEVICE_MEMORY_H__

#include <linux/cpumask.h>
#include <linux/freezer.h>
#include <linux/kthread.h>
#include <linux/sched/rt.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/time.h>
#include <linux/timer.h>
#include <linux/ktime.h>
#include <linux/hrtimer.h>
#include <linux/notifier.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/netdevice.h>
#include <linux/vmalloc.h>
#include <linux/mm.h>
#include <asm/cacheflush.h>

#include <soc/google/mcu_ipc.h>
#include "modem_prj.h"
#include "include/circ_queue.h"
#include "include/sbd.h"
#include "include/sipc5.h"
#include "include/legacy.h"
#include "link_rx_pktproc.h"
#if IS_ENABLED(CONFIG_CP_PKTPROC_UL)
#include "link_tx_pktproc.h"
#endif
#include "boot_device_spi.h"
#include "cpif_tp_monitor.h"

/*============================================================================*/
enum mem_iface_type {
	MEM_EXT_DPRAM = 0x0001,	/* External DPRAM */
	MEM_AP_IDPRAM = 0x0002,	/* DPRAM in AP    */
	MEM_CP_IDPRAM = 0x0004,	/* DPRAM in CP    */
	MEM_PLD_DPRAM = 0x0008,	/* PLD or FPGA    */
	MEM_SYS_SHMEM = 0x0100,	/* Shared-memory (SHMEM) on a system bus   */
	MEM_C2C_SHMEM = 0x0200,	/* SHMEM with C2C (Chip-to-chip) interface */
	MEM_LLI_SHMEM = 0x0400,	/* SHMEM with MIPI-LLI interface           */
};

#define MEM_DPRAM_TYPE_MASK	0x00FF
#define MEM_SHMEM_TYPE_MASK	0xFF00

/*============================================================================*/
#define MASK_INT_VALID		0x0080
#define MASK_TX_FLOWCTL_SUSPEND	0x0010
#define MASK_TX_FLOWCTL_RESUME	0x0000

#define MASK_CMD_VALID		0x0040
#define MASK_CMD_FIELD		0x003F

#define MASK_REQ_ACK_FMT	0x0020
#define MASK_REQ_ACK_RAW	0x0010
#define MASK_RES_ACK_FMT	0x0008
#define MASK_RES_ACK_RAW	0x0004
#define MASK_SEND_FMT		0x0002
#define MASK_SEND_RAW		0x0001
#define MASK_SEND_DATA		0x0001

#define CMD_INIT_START		0x0001
#define CMD_INIT_END		0x0002
#define CMD_REQ_ACTIVE		0x0003
#define CMD_RES_ACTIVE		0x0004
#define CMD_REQ_TIME_SYNC	0x0005
#define CMD_KERNEL_PANIC	0x0006
#define CMD_CRASH_RESET		0x0007
#define CMD_PHONE_START		0x0008
#define CMD_CRASH_EXIT		0x0009
#define CMD_CP_DEEP_SLEEP	0x000A
#define CMD_NV_REBUILDING	0x000B
#define CMD_EMER_DOWN		0x000C
#define CMD_PIF_INIT_DONE	0x000D
#define CMD_SILENT_NV_REBUILD	0x000E
#define CMD_NORMAL_POWER_OFF	0x000F

/*============================================================================*/
#define MAX_SKB_TXQ_DEPTH		1024
#define TX_PERIOD_MS			1	/* 1 ms */
#define MAX_TX_BUSY_COUNT		1024
#define BUSY_COUNT_MASK			0xF

#define RES_ACK_WAIT_TIMEOUT		10	/* 10 ms */

enum tx_flowctrl_mask_bit {
	TXQ_STOP_MASK = 1,
	TX_SUSPEND_MASK,
};

#define SHM_FLOWCTL_BIT			BIT(2)

/*============================================================================*/
#define FORCE_CRASH_ACK_TIMEOUT		(5 * HZ)

/*============================================================================*/
#define SHMEM_SRINFO_DATA_STR	64

#define SHMEM_BOOTLOG_BASE		0xC00
#define SHMEM_BOOTLOG_BUFF		0x1FF
#define SHMEM_BOOTLOG_OFFSET		0x4

/*============================================================================*/
struct __packed mem_snapshot {
	/* Timestamp */
	struct timespec64 ts;

	/* Direction (TX or RX) */
	enum direction dir;

	/* The status of memory interface at the time */
	unsigned int magic;
	unsigned int access;

	unsigned int head[IPC_MAP_MAX][MAX_DIR];
	unsigned int tail[IPC_MAP_MAX][MAX_DIR];

	u16 int2ap;
	u16 int2cp;
};

struct mst_buff {
	/* These two members must be first. */
	struct mst_buff *next;
	struct mst_buff *prev;

	struct mem_snapshot snapshot;
};

struct mst_buff_head {
	/* These two members must be first. */
	struct mst_buff *next;
	struct mst_buff	*prev;

	u32 qlen;
	spinlock_t lock;
};

/*============================================================================*/
enum mem_ipc_mode {
	MEM_LEGACY_IPC,
	MEM_SBD_IPC,
};

#define FREQ_MAX_LV (40)

struct freq_table {
	int num_of_table;
	u32 use_dfs_max_freq;
	u32 cal_id_mif;
	u32 freq[FREQ_MAX_LV];
};

struct ctrl_msg {
	u32 type;
	union {
		u32 sr_num;
		u32 __iomem *addr;
	};
};

struct mem_link_device {
	/**
	 * COMMON and MANDATORY to all link devices
	 */
	struct link_device link_dev;

	/**
	 * Attributes
	 */
	unsigned long attrs;		/* Set of link_attr_bit flags	*/

	/**
	 * Flags
	 */
	bool dpram_magic;		/* DPRAM-style magic code	*/

	/**
	 * {physical address, size, virtual address} for BOOT region
	 */
	phys_addr_t boot_start;
	size_t boot_size;
	u8 __iomem *boot_base;
	u32 boot_img_offset;		/* From IPC base */
	u32 boot_img_size;

	/**
	 * {physical address, size, virtual address} for IPC region
	 */
	phys_addr_t start;
	size_t size;
	u8 __iomem *base;		/* virtual address of ipc mem start */
#if IS_ENABLED(CONFIG_MODEM_IF_LEGACY_QOS)
	u8 __iomem *hiprio_base;	/* virtual address of priority queue start */
#endif

	/**
	 * vss region for dump
	 */
	u8 __iomem *vss_base;

	/**
	 * acpm region for dump
	 */
	u8 __iomem *acpm_base;
	int acpm_size;

	/* Boot link device */
	struct legacy_link_device legacy_link_dev;

	/* sbd link device */
	struct sbd_link_device sbd_link_dev;

	/**
	 * GPIO#, MBOX#, IRQ# for IPC
	 */
	unsigned int int_ap2cp_msg;		/* INTR# for IPC TX */
	unsigned int irq_cp2ap_msg;		/* IRQ# for IPC RX  */

	unsigned int sbi_cp2ap_wakelock_mask;
	unsigned int sbi_cp2ap_wakelock_pos;
	unsigned int irq_cp2ap_wakelock;	/* IRQ# for wakelock */

	unsigned int sbi_cp_status_mask;
	unsigned int sbi_cp_status_pos;
	unsigned int irq_cp2ap_status;		/* IRQ# for TX FLOWCTL */

	unsigned int total_freq_table_count;

	struct freq_table mif_table;
	struct freq_table cp_cpu_table;
	struct freq_table cp_table;
	struct freq_table cp_em_table;
	struct freq_table cp_mcw_table;

	unsigned int sbi_cp_rat_mode_mask;	/* MBOX# for pcie */
	unsigned int sbi_cp_rat_mode_pos;	/* MBOX# for pcie */
	unsigned int irq_cp2ap_rat_mode;	/* IRQ# for pcie */

	unsigned int tx_flowctrl_cmd;

	struct wakeup_source *ws;

#if IS_ENABLED(CONFIG_CP_PKTPROC_CLAT)
	unsigned int int_ap2cp_clatinfo_send;
	unsigned int irq_cp2ap_clatinfo_ack;

	struct mutex clatinfo_lock;
#endif

	/**
	 * Member variables for TX & RX
	 */
	struct mst_buff_head msb_rxq;
	struct mst_buff_head msb_log;

	struct hrtimer tx_timer;
	struct hrtimer sbd_tx_timer;
	struct hrtimer sbd_print_timer;
#if IS_ENABLED(CONFIG_CP_PKTPROC_UL)
	struct hrtimer pktproc_tx_timer;
#endif
	struct work_struct page_reclaim_work;

	/**
	 * Member variables for CP booting and crash dump
	 */
	struct delayed_work bootdump_rx_dwork;
	atomic_t init_end_cnt;
	atomic_t init_end_busy;
	int last_init_end_cnt;

	/**
	 * Mandatory methods for the common memory-type interface framework
	 */
	void (*send_ap2cp_irq)(struct mem_link_device *mld, u16 mask);

	/**
	 * Optional methods for some kind of memory-type interface media
	 */
	u16 (*recv_cp2ap_irq)(struct mem_link_device *mld);
	u16 (*read_ap2cp_irq)(struct mem_link_device *mld);
	u16 (*recv_cp2ap_status)(struct mem_link_device *mld);
	void (*finalize_cp_start)(struct mem_link_device *mld);
	void (*unmap_region)(void *rgn);
	void (*debug_info)(void);
	void (*cmd_handler)(struct mem_link_device *mld, u16 cmd);

	unsigned int tx_period_ns;
	unsigned int force_use_memcpy;
	unsigned int memcpy_packet_count;
	unsigned int zeromemcpy_packet_count;

	atomic_t forced_cp_crash;
	struct timer_list crash_ack_timer;

	spinlock_t state_lock;
	/* protects boot_base nc region */
	struct mutex vmap_lock;
	enum link_state state;

	struct net_device dummy_net;
	struct napi_struct mld_napi;
	unsigned int rx_int_enable;
	unsigned int rx_int_count;
	unsigned int rx_poll_count;
	unsigned long long rx_int_disabled_time;

	/* Location for arguments in shared memory */
	u32 __iomem *ap_version;
	u32 __iomem *cp_version;
	u32 __iomem *cmsg_offset;	/* address where cmsg offset is written */
	u32 __iomem *srinfo_offset;
	u32 __iomem *clk_table_offset;
	u32 __iomem *buff_desc_offset;
	u32 __iomem *capability_offset;

	u32 __iomem *ap_capability_offset[AP_CP_CAP_PARTS];
	u32 __iomem *cp_capability_offset[AP_CP_CAP_PARTS];

	/* Location for control messages in shared memory */
	struct ctrl_msg ap2cp_msg;
	struct ctrl_msg cp2ap_msg;
	struct ctrl_msg ap2cp_united_status;
	struct ctrl_msg cp2ap_united_status;
#if IS_ENABLED(CONFIG_CP_PKTPROC_CLAT)
	struct ctrl_msg ap2cp_clatinfo_xlat_v4_addr;
	struct ctrl_msg ap2cp_clatinfo_xlat_addr_0;
	struct ctrl_msg ap2cp_clatinfo_xlat_addr_1;
	struct ctrl_msg ap2cp_clatinfo_xlat_addr_2;
	struct ctrl_msg ap2cp_clatinfo_xlat_addr_3;
	struct ctrl_msg ap2cp_clatinfo_index;
#endif
	struct ctrl_msg ap2cp_kerneltime;	/* for DRAM_V1 and MAILBOX_SR */
	struct ctrl_msg ap2cp_kerneltime_sec;	/* for DRAM_V2 */
	struct ctrl_msg ap2cp_kerneltime_usec;	/* for DRAM_V2 */
	struct ctrl_msg ap2cp_handover_block_info;

#if IS_ENABLED(CONFIG_LINK_DEVICE_PCIE)
	/* Doorbell */
	unsigned int intval_ap2cp_msg;
	unsigned int intval_ap2cp_pcie_link_ack;

	/* MSI */
	u8 __iomem *msi_reg_base;
	bool msi_irq_enabled;
	int msi_irq_base;
	bool msi_irq_base_wake;
	u32 msi_irq_base_cpu;
	u32 msi_irq_q_cpu[PKTPROC_MAX_QUEUE];
#endif

	u32 __iomem *srinfo_base;
	u32 srinfo_size;
	u32 __iomem *clk_table;

	u32 ap_capability[AP_CP_CAP_PARTS];
	u32 cp_capability[AP_CP_CAP_PARTS];

	int (*pass_skb_to_net)(struct mem_link_device *mld, struct sk_buff *skb);
	int (*pass_skb_to_demux)(struct mem_link_device *mld, struct sk_buff *skb);

	struct pktproc_adaptor pktproc;
#if IS_ENABLED(CONFIG_CP_PKTPROC_UL)
	struct pktproc_adaptor_ul pktproc_ul;
#endif

	int pktproc_use_36bit_addr;

	int spi_bus_num;

	struct cpif_tpmon *tpmon;

	struct toe_ctrl_t *tc;

#if IS_ENABLED(CONFIG_CP_PKTPROC_CLAT)
	bool disable_hw_clat;
#endif

#if defined(CPIF_WAKEPKT_SET_MARK)
	atomic_t net_wakeup_count;
	atomic_t misc_wakeup_count;
#endif
};

#define to_mem_link_device(ld) \
		container_of(ld, struct mem_link_device, link_dev)
#define ld_to_mem_link_device(ld) \
		container_of(ld, struct mem_link_device, link_dev)
#define sbd_to_mem_link_device(sl) \
		container_of(sl, struct mem_link_device, sbd_link_dev)

#define MEM_IPC_MAGIC		0xAA
#define MEM_CRASH_MAGIC		0xDEADDEAD
#define MEM_BOOT_MAGIC		0x424F4F54
#define MEM_DUMP_MAGIC		0x44554D50

#define MAX_TABLE_COUNT 8

struct clock_table_info {
	char table_name[4];
	u32 table_count;
};

struct clock_table {
	char parser_version[4];
	u32 total_table_count;
	struct clock_table_info table_info[MAX_TABLE_COUNT];
};

/*============================================================================*/
static inline bool mem_type_shmem(enum mem_iface_type type)
{
	return (type & MEM_SHMEM_TYPE_MASK) ? true : false;
}

/*============================================================================*/
static inline bool int_valid(u16 x)
{
	return (x & MASK_INT_VALID) ? true : false;
}

static inline u16 mask2int(u16 mask)
{
	return mask | MASK_INT_VALID;
}

/*
 * @remark		This must be invoked after validation with int_valid().
 */
static inline bool cmd_valid(u16 x)
{
	return (x & MASK_CMD_VALID) ? true : false;

}

static inline bool chk_same_cmd(struct mem_link_device *mld, u16 x)
{
	if (mld->tx_flowctrl_cmd != x) {
		mld->tx_flowctrl_cmd = x;
		return false;
	}

	return true;
}

static inline u16 int2cmd(u16 x)
{
	return x & MASK_CMD_FIELD;
}

static inline u16 cmd2int(u16 cmd)
{
	return mask2int(cmd | MASK_CMD_VALID);
}

/*============================================================================*/
static inline struct circ_queue *cq(struct legacy_ipc_device *dev,
				    enum direction dir)
{
	return (dir == TX) ? &dev->txq : &dev->rxq;
}

static inline unsigned int get_txq_head(struct legacy_ipc_device *dev)
{
	return get_head(&dev->txq);
}

static inline void set_txq_head(struct legacy_ipc_device *dev, unsigned int in)
{
	set_head(&dev->txq, in);
}

static inline unsigned int get_txq_tail(struct legacy_ipc_device *dev)
{
	return get_tail(&dev->txq);
}

static inline void set_txq_tail(struct legacy_ipc_device *dev, unsigned int out)
{
	set_tail(&dev->txq, out);
}

static inline char *get_txq_buff(struct legacy_ipc_device *dev)
{
	return get_buff(&dev->txq);
}

static inline unsigned int get_txq_buff_size(struct legacy_ipc_device *dev)
{
	return get_size(&dev->txq);
}

static inline unsigned int get_rxq_head(struct legacy_ipc_device *dev)
{
	return get_head(&dev->rxq);
}

static inline void set_rxq_head(struct legacy_ipc_device *dev, unsigned int in)
{
	set_head(&dev->rxq, in);
}

static inline unsigned int get_rxq_tail(struct legacy_ipc_device *dev)
{
	return get_tail(&dev->rxq);
}

static inline void set_rxq_tail(struct legacy_ipc_device *dev, unsigned int out)
{
	set_tail(&dev->rxq, out);
}

static inline char *get_rxq_buff(struct legacy_ipc_device *dev)
{
	return get_buff(&dev->rxq);
}

static inline unsigned int get_rxq_buff_size(struct legacy_ipc_device *dev)
{
	return get_size(&dev->rxq);
}

static inline u16 msg_mask(struct legacy_ipc_device *dev)
{
	return dev->msg_mask;
}

static inline u16 req_ack_mask(struct legacy_ipc_device *dev)
{
	return dev->req_ack_mask;
}

static inline u16 res_ack_mask(struct legacy_ipc_device *dev)
{
	return dev->res_ack_mask;
}

static inline bool req_ack_valid(struct legacy_ipc_device *dev, u16 val)
{
	if (!cmd_valid(val) && (val & req_ack_mask(dev)))
		return true;
	else
		return false;
}

static inline bool res_ack_valid(struct legacy_ipc_device *dev, u16 val)
{
	if (!cmd_valid(val) && (val & res_ack_mask(dev)))
		return true;
	else
		return false;
}

static inline bool rxq_empty(struct legacy_ipc_device *dev)
{
	u32 head;
	u32 tail;
	unsigned long flags;

	spin_lock_irqsave(&dev->rxq.lock, flags);

	head = get_rxq_head(dev);
	tail = get_rxq_tail(dev);

	spin_unlock_irqrestore(&dev->rxq.lock, flags);

	return circ_empty(head, tail);
}

static inline bool txq_empty(struct legacy_ipc_device *dev)
{
	u32 head;
	u32 tail;
	unsigned long flags;

	spin_lock_irqsave(&dev->txq.lock, flags);

	head = get_txq_head(dev);
	tail = get_txq_tail(dev);

	spin_unlock_irqrestore(&dev->txq.lock, flags);

	return circ_empty(head, tail);
}

static inline int construct_ctrl_msg(struct ctrl_msg *cmsg, u32 *arr_from_dt,
					u8 __iomem *base)
{
	if (!cmsg)
		return -EINVAL;

	cmsg->type = arr_from_dt[0];
	switch (cmsg->type) {
	case MAILBOX_SR:
		cmsg->sr_num = arr_from_dt[1];
		break;
	case DRAM_V1:
	case DRAM_V2:
		cmsg->addr = (u32 __iomem *)(base + arr_from_dt[1]);
		break;
	case CMSG_TYPE_NONE:
		break;
	default:
		mif_err("ERR! wrong type for ctrl msg\n");
		return -EINVAL;
	}

	return 0;
}

static inline void init_ctrl_msg(struct ctrl_msg *cmsg)
{
	switch (cmsg->type) {
	case MAILBOX_SR:
		/* nothing to do */
		break;
	case DRAM_V1:
	case DRAM_V2:
		*cmsg->addr = 0;
		break;
	case GPIO:
		break;
	default:
		break;
	}

}
static inline u32 get_ctrl_msg(struct ctrl_msg *cmsg)
{
	u32 val = 0;

	switch (cmsg->type) {
	case MAILBOX_SR:
#if IS_ENABLED(CONFIG_MCU_IPC)
		val = cp_mbox_get_sr(cmsg->sr_num);
#endif
		break;
	case DRAM_V1:
	case DRAM_V2:
		val = ioread32(cmsg->addr);
		break;
	case GPIO:
		break;
	default:
		break;
	}

	return val;
}

static inline void set_ctrl_msg(struct ctrl_msg *cmsg, u32 msg)
{
	switch (cmsg->type) {
	case MAILBOX_SR:
#if IS_ENABLED(CONFIG_MCU_IPC)
		cp_mbox_set_sr(cmsg->sr_num, msg);
#endif
		break;
	case DRAM_V1:
	case DRAM_V2:
		iowrite32(msg, cmsg->addr);
		break;
	case GPIO:
		break;
	default:
		break;
	}
}

static inline u32 extract_ctrl_msg(struct ctrl_msg *cmsg, u32 mask, u32 pos)
{
	u32 val = 0;

	switch (cmsg->type) {
	case MAILBOX_SR:
#if IS_ENABLED(CONFIG_MCU_IPC)
		val = cp_mbox_extract_sr(cmsg->sr_num, mask, pos);
#endif
		break;
	case DRAM_V1:
	case DRAM_V2:
		val = (ioread32(cmsg->addr) >> pos) & mask;
		break;
	case GPIO:
		break;
	default:
		break;
	}

	return val;
}

static inline void update_ctrl_msg(struct ctrl_msg *cmsg, u32 msg, u32 mask, u32 pos)
{
	u32 val = 0;

	switch (cmsg->type) {
	case MAILBOX_SR:
#if IS_ENABLED(CONFIG_MCU_IPC)
		cp_mbox_update_sr(cmsg->sr_num, msg, mask, pos);
#endif
		break;
	case DRAM_V1:
	case DRAM_V2:
		val = ioread32(cmsg->addr);
		val &= ~(mask << pos);
		val |= (msg & mask) << pos;
		iowrite32(val, cmsg->addr);
		break;
	case GPIO:
		break;
	default:
		break;
	}
}

/*============================================================================*/
int msb_init(void);

struct mst_buff *msb_alloc(void);
void msb_free(struct mst_buff *msb);

void msb_queue_head_init(struct mst_buff_head *list);
void msb_queue_tail(struct mst_buff_head *list, struct mst_buff *msb);
void msb_queue_head(struct mst_buff_head *list, struct mst_buff *msb);

struct mst_buff *msb_dequeue(struct mst_buff_head *list);

void msb_queue_purge(struct mst_buff_head *list);

struct mst_buff *mem_take_snapshot(struct mem_link_device *mld,
				   enum direction dir);

/*============================================================================*/
static inline void send_ipc_irq(struct mem_link_device *mld, u16 val)
{
	if (likely(mld->send_ap2cp_irq))
		mld->send_ap2cp_irq(mld, val);
}

static inline void send_ipc_irq_debug(struct mem_link_device *mld, u16 val)
{
#if IS_ENABLED(CONFIG_MCU_IPC)
	if (mld->ap2cp_msg.type == MAILBOX_SR)
		cp_mbox_dump_sr();
#endif
	send_ipc_irq(mld, val);
#if IS_ENABLED(CONFIG_MCU_IPC)
	if (mld->ap2cp_msg.type == MAILBOX_SR)
		cp_mbox_dump_sr();
#endif
}

void mem_irq_handler(struct mem_link_device *mld, struct mst_buff *msb);

/*============================================================================*/
void __iomem *mem_vmap(phys_addr_t pa, size_t size, struct page *pages[]);
void mem_vunmap(void *va);

int mem_register_boot_rgn(struct mem_link_device *mld, phys_addr_t start,
			  size_t size);
void mem_unregister_boot_rgn(struct mem_link_device *mld);
int mem_setup_boot_map(struct mem_link_device *mld);

int mem_register_ipc_rgn(struct mem_link_device *mld, phys_addr_t start,
			 size_t size);
void mem_unregister_ipc_rgn(struct mem_link_device *mld);
void mem_setup_ipc_map(struct mem_link_device *mld);

struct mem_link_device *mem_create_link_device(enum mem_iface_type type,
					       struct modem_data *modem);

/*============================================================================*/
int mem_reset_ipc_link(struct mem_link_device *mld);
void mem_cmd_handler(struct mem_link_device *mld, u16 cmd);

/*============================================================================*/
#if IS_ENABLED(CONFIG_CP_PKTPROC_UL)
void pktproc_ul_q_stop(struct pktproc_queue_ul *q);
int pktproc_ul_q_check_busy(struct pktproc_queue_ul *q);
#endif

void sbd_txq_stop(struct sbd_ring_buffer *rb);
int sbd_txq_check_busy(struct sbd_ring_buffer *rb);

void txq_stop(struct mem_link_device *mld, struct legacy_ipc_device *dev);
int txq_check_busy(struct mem_link_device *mld, struct legacy_ipc_device *dev);

void tx_flowctrl_suspend(struct mem_link_device *mld);
void tx_flowctrl_resume(struct mem_link_device *mld);

void send_req_ack(struct mem_link_device *mld, struct legacy_ipc_device *dev);
void recv_res_ack(struct mem_link_device *mld, struct legacy_ipc_device *dev,
		  struct mem_snapshot *mst);

void recv_req_ack(struct mem_link_device *mld, struct legacy_ipc_device *dev,
		  struct mem_snapshot *mst);
void send_res_ack(struct mem_link_device *mld, struct legacy_ipc_device *dev);

/*============================================================================*/
void mem_handle_cp_crash(struct mem_link_device *mld, enum modem_state state);
void mem_forced_cp_crash(struct mem_link_device *mld);

/*============================================================================*/
void print_req_ack(struct mem_link_device *mld, struct mem_snapshot *mst,
		   struct legacy_ipc_device *dev, enum direction dir);
void print_res_ack(struct mem_link_device *mld, struct mem_snapshot *mst,
		   struct legacy_ipc_device *dev, enum direction dir);

void print_mem_snapshot(struct mem_link_device *mld, struct mem_snapshot *mst);
void print_dev_snapshot(struct mem_link_device *mld, struct mem_snapshot *mst,
			struct legacy_ipc_device *dev);

static inline struct sk_buff *mem_alloc_skb(unsigned int len)
{
	gfp_t priority;
	struct sk_buff *skb;

	priority = in_interrupt() ? GFP_ATOMIC : GFP_KERNEL;

	skb = alloc_skb(len + NET_SKB_PAD, priority);
	if (!skb) {
		mif_err("ERR! alloc_skb(len:%d + pad:%d, gfp:0x%x) fail\n",
			len, NET_SKB_PAD, priority);
#if IS_ENABLED(CONFIG_SEC_DEBUG_MIF_OOM)
		show_mem(SHOW_MEM_FILTER_NODES);
#endif
		return NULL;
	}

	skb_reserve(skb, NET_SKB_PAD);
	return skb;
}

/*============================================================================*/

#define NET_HEADROOM (NET_SKB_PAD + NET_IP_ALIGN)

#if IS_ENABLED(CONFIG_USB_ANDROID_SAMSUNG_COMPOSITE)
extern int is_rndis_use(void);
#endif

#endif /* __MODEM_LINK_DEVICE_MEMORY_H__ */
