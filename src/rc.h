/****************************************************************************
 *
 * Copyright © 2006-2008 Ciprico Inc. All rights reserved.
 * Copyright © 2008-2014 Dot Hill Systems Corp. All rights reserved.
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

#ifndef _BC_OSHEADERS_H_
#define _RC_OSHEADERS_H_

#include <linux/version.h>

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/sched.h>
#include <linux/pci.h>
#include <linux/spinlock.h>
#include <linux/slab.h>
#include <linux/completion.h>

#include <linux/blkdev.h>
#include <scsi/scsi.h>
#include <scsi/scsi_host.h>
#include <scsi/scsi_device.h>
#include <scsi/scsi_cmnd.h>
#include <scsi/scsi_tcq.h>

#include <linux/delay.h>
#include <linux/timer.h>
#include <linux/highmem.h>
#include <linux/interrupt.h>
#include <linux/proc_fs.h>
#include <linux/random.h>

#include <linux/acpi.h>
#include <linux/kthread.h>

/* Misc fixups for the 2.6 kernel */

/* 2.6 kernel dosen't include a typedef */
typedef struct scsi_host_template Scsi_Host_Template;

/* 2.6 kernel dosen't use io_request_lock */
#define GET_IO_REQUEST_LOCK
#define PUT_IO_REQUEST_LOCK
#define GET_IO_REQUEST_LOCK_IRQSAVE(i)
#define PUT_IO_REQUEST_LOCK_IRQRESTORE(i)


#include "rc_types_platform.h"
#include "rc_srb.h"
#include "rc_scsi.h"
#include "rc_msg_platform.h"
#include "rc_adapter.h"

typedef enum
{
	RC_PANIC = 0,
	RC_ALERT,
	RC_CRITICAL,
	RC_ERROR,
	RC_WARN,
	RC_NOTE,      // 5
	RC_INFO,
	RC_INFO2,
	RC_DEBUG,
	RC_DEBUG2,
	RC_DEBUG3,    // 10
	RC_TAIL
} rc_print_lvl_t;

#define RC_DEFAULT_ERR_LEVEL RC_INFO

void rc_printk(int flag, const char *fmt, ...)
	__attribute__ ((format (printf, 2, 3)));

/* Fast version of rc_printk for use in data path. */
#define RC_PRINTK(flag, fmt, ...)					\
	if (flag <= rc_msg_level) rc_printk(flag, fmt, __VA_ARGS__);

#ifdef RC_SW_DEBUG
#define RC_ASSERT(cond) BUG_ON(!(cond))
#else
#define RC_ASSERT(cond)
#endif

void rc_start_all_threads(void);
void rc_stop_all_threads(void);
void rc_event(rc_uint32_t type, rc_uint8_t bus, int update_mode);
int rc_event_init(void);
void rc_event_shutdown(void);

extern rc_softstate_t       rc_state;
extern rc_adapter_t       *rc_dev[];
extern int            rc_msg_level;

typedef struct _rc_work {
    struct _rc_work         *next;
    unsigned int            call_type;
    char                    *method;
    acpi_handle             handle;
    void                    *args;
} rc_work_t;

extern struct task_struct       *rc_wq;
extern rc_work_t                *acpi_work_item_head;
extern rc_work_t                *acpi_work_item_tail;
extern spinlock_t               acpi_work_item_lock;

//
// If we can't find the GPE from the _PSW method, then we'll default
// to this value. Doesn't seem to be any standard and it depends solely
// on the vendor.
//
#define RC_DEFAULT_ZPODD_GPE_NUMBER     6

acpi_status rc_acpi_evaluate_object(acpi_handle, char *, void *, int *);

extern unsigned int RC_ODDZDevAddr;
extern unsigned int RC_ODD_Device;
extern unsigned int RC_ODD_GpeNumber;

enum {
    RC_ODD_DEVICE_INVALID = 0,
    RC_ODD_DEVICE_ODDZ,
    RC_ODD_DEVICE_ODDL,
    RC_ODD_DEVICE_ODD8
};

//
// Pulled from rc_event.c
//
typedef struct rc_event_thread_s {
        struct task_struct  *thread;
        int                 running;
        int                 num_srb;
        rc_srb_queue_t      cfg_change_detect;
        rc_srb_queue_t      cfg_change_response;
        rc_uint32_t         targets[MAX_ARRAY];
} rc_event_thread_t;

extern rc_event_thread_t rc_event_thread;

//
// Declared in rc_config.c
//
extern struct semaphore rccfg_wait;

//
// Pulled from rc_mem_ops.c
//
#define RC_THREAD_BUF_CNT  16

typedef struct {
        rc_sg_list_t *sg;
        int          size;
} rc_thread_buf_t;

typedef struct rc_thread_s {
        struct task_struct *thread;
        int                running;
        volatile int       num_mop;
        rc_mem_op_t       *mop_head;
        rc_mem_op_t       *mop_tail;
        /* 0 is destination list, 1-15 are sources. */
        rc_thread_buf_t    buf[RC_THREAD_BUF_CNT];
    struct semaphore    stop_sema;
} rc_thread_t;


extern rc_thread_t rc_thread[];

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,13,0)
#define DEVICE_ACPI_HANDLE(dev) ((acpi_handle)ACPI_HANDLE(dev))
#endif

#endif // _RC_OSHEADERS_H_
