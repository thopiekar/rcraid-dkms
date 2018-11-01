/****************************************************************************
 *
 * Copyright © 2006-2008 Ciprico Inc. All rights reserved.
 * Copyright © 2008-2013 Dot Hill Systems Corp. All rights reserved.
 * Copyright © 2015-2016 Seagate Technology LLC. All rights reserved.
 *
 * Use of this software is subject to the terms and conditions of the written
 * software license agreement between you and DHS (the "License"),
 * including, without limitation, the following (as further elaborated in the
 * License):  (i) THIS SOFTWARE IS PROVIDED "AS IS", AND DHS DISCLAIMS
 * ANY AND ALL WARRANTIES OF ANY KIND, WHETHER EXPRESS, IMPLIED, STATUTORY,
 * BY CONDUCT, OR OTHERWISE; (ii) this software may be used only in connection
 * with the integrated circuit product and storage software with which it was
 * designed to be used; (iii) this source code is the confidential information
 * of DHS and may not be disclosed to any third party; and (iv) you may not
 * make any modification or take any action that would cause this software,
 * or any other Dot Hill software, to fall under any GPL license or any other
 * open source license.
 *
 ****************************************************************************/

/*
 * Abstract:
 *
 *    all the stuff we need to know and to track for each physical adapter
 *    and the virtual adapter
 */
#ifndef _RC_ADAPTER_
#define _RC_ADAPTER_

struct rc_adapter_s;
typedef irqreturn_t (* rc_isr_func_t)(int irq, void *arg, struct pt_regs *regs);
typedef int (* rc_adapter_func_t)(struct rc_adapter_s *adapter);

/*
 * Hardware info for the adapter
 * FIXME: must stay in sync with RAIDCore core - should share common header
 */
#define MAX_PORTS_PER_HA    8
#define MAX_HBA             8
#define MAX_TOTAL_PORTS     (MAX_HBA * MAX_PORTS_PER_HA)
#define MAX_ARRAY           32
#define SECTOR_SIZE         512

#define PCI_CFG_SIZE    256
typedef struct rc_hw_info {
	int pci_bus;
	int pci_slot;
	int irq;
	int ismsi;
	void *vaddr; // device memory map address
	// Need to deal with 64bit paddrs.
	// These types come from linux/ioport.h
	unsigned long phys_addr;    // 32 bit bus address
	int mem_len; // len of memory map
	int adapter_number;
	unsigned char pci_config_space[2 * PCI_CFG_SIZE];   // Allow for extended pci config space
} rc_hw_info_t;

typedef    struct rc_mem_desc_s {
	void          *vaddr;
	dma_addr_t     dma_address;
	unsigned int   size;
} rc_mem_desc_t;

struct DmaMemoryNode {

    struct DmaMemoryNode *nextNode;

    void                *cpu_addr;
    dma_addr_t          dmaHandle;
    rc_uint32_t         bytes;
};

typedef struct rc_adapter_s {
	struct rc_version   *version;
	char                *name;
	int                  instance;
	rc_hw_info_t         hardware;
	struct pci_dev      *pdev;
	void                *mem; // memory allocated for the adapter
	rc_mem_desc_t        private_mem;
    atomic_t            checkInterrupt;
	struct DmaMemoryNode *dmaMemoryListHead;
	struct DmaMemoryNode *dmaMemoryListTail;
} rc_adapter_t;

/*
 * things need to implement srb_done/srb_queue tasklets
 */

typedef struct rc_srb_queue_s {
	spinlock_t             lock;
	rc_srb_t              *head;
	rc_srb_t              *tail;
	struct tasklet_struct  tasklet;
} rc_srb_queue_t;

typedef struct rc_mop_queue_s {
	spinlock_t   lock;
	rc_mem_op_t *head;
	rc_mem_op_t *tail;
} rc_mop_queue_t;


typedef struct rc_stats_s {
	uint        target_total[MAX_ARRAY];
	atomic_t    target_pending[MAX_ARRAY];
	uint        srb_total;          // SRBs sent to the OSIC
	atomic_t    srb_pending;        // SRBs sent to the OSIC
	uint        scb_total;          // SCBs from scsi layer
	atomic_t    scb_pending;        // SCBs from scsi layer
	uint        max_srbs_sent;      // ...into the core at one time in
	// rc_msg_srb_q_tasklet
	uint        max_intr_waiting;   // max interrupts that came in while sending
	// a batch of SRBs to the core
	uint        total_intr_waiting; // total
	uint        max_intr_delay;     // max #cycles an interrupt waited for us
	// to send SRBs to the core
} rc_stats_t;


/* Global driver info */
typedef struct rc_softstate  {
	int                    state;
	struct Scsi_Host      *host_ptr;
	int                    num_hba;
	/*
	 * private resources used by the OSIC
	 */
	int                    memsize_per_controller;
	int                    memsize_per_srb;
	int                    timer_interval;        /* in clock ticks */
	uint32_t               virtual_memory_size;
	void                  *virtual_memory;
	uint32_t               cache_memory_size;
	void                  *cache_memory;
	struct semaphore       init_sema;
	struct timer_list      timer;
	spinlock_t             osic_lock;
	int                    osic_locked;
	char                  *osic_lock_holder;
	struct timer_list      rc_timeout;
	struct semaphore       rc_timeout_sema;
	struct timer_list      msg_timeout;
	struct semaphore       msg_timeout_sema;
	struct timer_list      debug_timer;
	struct rc_srb_queue_s  srb_q;
	struct rc_srb_queue_s  srb_done;
	struct rc_mop_queue_s  mop_done;
	struct tasklet_struct  intr_tasklet;
    
    struct delayed_work resume_work;
    int is_suspended;
    uint                   adapter_is_suspended;
    
	atomic_t               intr_pending;
	rc_stats_t             stats;
} rc_softstate_t;

/*
 * State flags
 */
#define USE_OSIC      0x02
#define ENABLE_TIMER  0x04
#define PROCESS_INTR  0x08 /* ready to process interrupts */
#define INIT_DONE     0x10
#define SHUTDOWN	  0x20

/*
 * SHWL types (on board chip sets) supported/used
 *
 * WARNING: as these are modified, the common_shell linux install script
 *          needs to be changed as well.
 */
#define RC_SHWL_TYPE_CARD   0x00000001
#define RC_SHWL_TYPE_AHCI   0x00000002
#define RC_SHWL_TYPE_MPT    0x00000004
#define RC_SHWL_TYPE_MPT2   0x00000008
#define RC_SHWL_TYPE_ALL    0xffffffff

typedef struct rc_version {
	rc_adapter_func_t  init_func;
	rc_adapter_func_t  start_func;
	rc_adapter_func_t  shutdown_func;
	rc_isr_func_t      isr_func;
	char              *device_name;
	char              *vendor;
	char              *model;
	int                num_ports;
	int                window_size;
	int                which_bar;
	int                swl_type;
} rc_version_t;

#define rc_interrupt_adapter(adapter)				\
	adapter->adapter_funcs.interrupt_adapter(adapter)

#define rc_notify_adapter(adapter, adapter_event)			\
	adapter->adapter_funcs.notify_adapter(adapter, adapter_event)

#define rc_enable_interrupt(adapter, adapter_event, at_device_irq)	\
	adapter->adapter_funcs.enable_interrupt(adapter, adapter_event,	\
						at_device_irq)

#define rc_disable_interrupt(adapter, adapter_event, at_device_irq)	\
	adapter->adapter_funcs.disable_interrupt(adapter, adapter_event, \
						 at_device_irq)

#define rc_reset_device(adapter)			\
	adapter->adapter_funcs.reset_device(adapter)

#define read8     readb
#define read16    readw
#define read32    readl

#define write8    writeb
#define write16   writew
#define write32   writel

#endif // _RC_ADAPTER_
