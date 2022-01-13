/****************************************************************************
 *
 * Copyright © 2006-2008 Ciprico Inc. All rights reserved.
 * Copyright © 2008-2014 Dot Hill Systems Corp. All rights reserved.
 * Copyright © 2051-2016 Seagate Technology LLC. All rights reserved.
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
 *    This module contains the code to handle events from the raid subsystem
 */
#include "rc.h"
#include "linux/kmod.h"
#include "linux/kthread.h"

static void rc_cfg_change_detect(rc_uint32_t type, rc_uint32_t bus, int flags);
static void rc_cfg_change_detect_tasklet( unsigned long arg);

static void rc_cfg_change_response(struct rc_srb_s *srb);
static void rc_send_inq(rc_uint32_t bus, rc_uint32_t target, rc_uint32_t lun, int update_mode);
static void rc_inq_callback (rc_srb_t *srb);
static int rc_notify_scsi_layer(int bus, int target, int lun, int present);
static int rc_event_kthread(void *arg);

extern int rc_srb_seq_num;

rc_event_thread_t rc_event_thread;
#if 0
static int rc_scsi_delay = 200;
#endif
int
rc_event_init(void)
{
	rc_event_thread_t *tp = &rc_event_thread;

	memset(tp, 0, sizeof(rc_event_thread_t));

	/*
	 * setup tasklet to catch config change events;
	 */

	tp->cfg_change_detect.head = (rc_srb_t *)0;
	tp->cfg_change_detect.tail = (rc_srb_t *)0;
	spin_lock_init(&tp->cfg_change_detect.lock);

	tasklet_init(&tp->cfg_change_detect.tasklet, rc_cfg_change_detect_tasklet,
		     (unsigned long)tp);

	/*
	 * response queue
	 * Use a kernel thread so that the thread can sleep
	 */
	tp->cfg_change_response.head = (rc_srb_t *)0;
	tp->cfg_change_response.tail = (rc_srb_t *)0;
	spin_lock_init(&tp->cfg_change_response.lock);

	tp->thread = kthread_run(rc_event_kthread, (void *) tp, "rc_event_kthread");

	if (IS_ERR(tp->thread)) {
		printk(KERN_ERR "rcraid: Unable to create event thread \n");
		return 0;
	}

	return 1;
}

void
rc_event_shutdown(void)
{
	rc_event_thread_t *tp;

	tp = &rc_event_thread;
	tasklet_kill(&tp->cfg_change_detect.tasklet);

	/* stop event thread */
	if (tp->thread) {
		kthread_stop(tp->thread);
	}
}


void
rc_event(rc_uint32_t type, rc_uint8_t bus, int update_mode)
{
	rc_srb_t *srb;
	unsigned long irql;
	rc_event_thread_t *tp;

	tp = &rc_event_thread;

	switch (type) {
	case RC_CTR_EVENT_CONFIG_CHANGE_DETECTED:
		rc_printk(RC_INFO, "rcraid: rc_event: config change detected on bus "
			  "%d\n", bus);
		break;

	case RC_CTR_EVENT_CONFIG_ARRAY_OFFLINE:
		rc_printk(RC_INFO, "rcraid: rc_event: array offline detected on bus "
			  "%d\n", bus);
		break;

	case RC_CTR_EVENT_CONFIG_DISK_OFFLINE:
		rc_printk(RC_INFO, "rcraid: rc_event: drive offline detected on bus "
			  "%d\n", bus);
		break;

	default:
		rc_printk(RC_WARN, "rcraid: rc_event: unknown event 0x%x detected on "
			  "bus %d\n", type, bus);
		return;
	}

	if ((srb = kmalloc( sizeof(rc_srb_t), GFP_ATOMIC)) == 0) {
		rc_printk(RC_WARN,"rc_event: could not alloc mem for srb\n");
		rc_printk(RC_WARN,"rc_event: config change not processed\n");
		return;
	}

	srb->bus      = bus;
	srb->function = type;
	srb->flags    = update_mode;
	srb->next     = (rc_srb_t *)0;

	spin_lock_irqsave(&tp->cfg_change_detect.lock, irql);

	if (tp->cfg_change_detect.head == (rc_srb_t *)0) {
		tp->cfg_change_detect.head = srb;
		tp->cfg_change_detect.tail = srb;
	} else {
		tp->cfg_change_detect.tail->next = srb;
		tp->cfg_change_detect.tail = srb;
	}
	spin_unlock_irqrestore(&tp->cfg_change_detect.lock, irql);

	tasklet_schedule(&tp->cfg_change_detect.tasklet);
}

static void
rc_cfg_change_detect_tasklet( unsigned long arg)
{
	unsigned long    irql;
	rc_srb_t    *srb;
	rc_uint32_t    type, bus;
	int        update_mode;
	rc_event_thread_t *tp;

	tp = (rc_event_thread_t *)arg;

	spin_lock_irqsave(&tp->cfg_change_detect.lock, irql);

	while (tp->cfg_change_detect.head) {
		/*
		 * take an srb off the list
		 */
		srb = tp->cfg_change_detect.head;
		tp->cfg_change_detect.head = srb->next;
		srb->next = (rc_srb_t *)0;

		spin_unlock_irqrestore(&tp->cfg_change_detect.lock, irql);

		type        = srb->function;
		bus         = srb->bus;
		update_mode = srb->flags;
		kfree(srb);

		rc_cfg_change_detect(type, bus, update_mode);
		spin_lock_irqsave(&tp->cfg_change_detect.lock, irql);
	}
	spin_unlock_irqrestore(&tp->cfg_change_detect.lock, irql);
}

static void
rc_cfg_change_detect(rc_uint32_t type, rc_uint32_t bus, int update_mode)
{
	unsigned int i;

	rc_printk(RC_DEBUG,"rc_cfg_change_detect type 0x%x bus %d\n", type, bus);

	for (i = 0; i <= RC_MAX_SCSI_TARGETS; i++) {
		rc_send_inq(bus, i, 0, update_mode);
	}
}

/*
 * Note: an inquiry could take 30 seconds or more when doing pass-through IO
 * and the scsi command has to time out for nonexistant devices.
 */
static void
rc_send_inq(rc_uint32_t bus, rc_uint32_t target, rc_uint32_t lun,
	    int update_mode)
{
	unsigned long   irql;
	rc_srb_t       *srb;
	int             size, sg_list_size;
	rc_softstate_t *state;
	rc_inq_data_t  *inq_data;
	inq_cmd_t      *inq_cmd;
	rc_sg_list_t   *sg;

	state = &rc_state;

	if (bus > 0  || target > RC_MAX_SCSI_TARGETS || lun > 0) {
		rc_printk(RC_INFO2, "rc_send_inq: invalid B/T/L %d/%d/%d\n",
			  bus, target, lun );
		return;
	}

	sg_list_size =  sizeof(rc_sg_list_t) +
		(RC_SG_MAX_ELEMENTS-1) * sizeof(rc_sg_elem_t);
	size = sizeof(rc_srb_t) + sg_list_size + state->memsize_per_srb;
	if ((srb = kmalloc(size, GFP_ATOMIC | GFP_DMA)) == 0) {
		rc_printk(RC_WARN, "%s: could not alloc %d bytes for srb\n",
			  __FUNCTION__, size);
		return;
	}
	memset(srb, 0, size);

	size = sizeof(inq_cmd_t) + sizeof(rc_inq_data_t);
	if ((inq_cmd = kmalloc(size, GFP_ATOMIC | GFP_DMA)) == 0) {
		rc_printk(RC_WARN, "%s: could not alloc mem for inquiry command and "
			  "data\n", __FUNCTION__);
		kfree(srb);
		return;
	}
	memset(inq_cmd, 0, size);

	inq_data = (rc_inq_data_t *) ((char *)inq_cmd + sizeof(inq_cmd_t));

	srb->function     = RC_SRB_EXECUTE_SCSI;
	srb->status       = RC_SRB_STATUS_SUCCESS;
	srb->bus          = bus;
	srb->target       = target;
	srb->lun          = lun;
	srb->flags        = RC_SRB_FLAGS_DATA_IN;
	srb->data_len     = sizeof(rc_inq_data_t);
	srb->cdb          = inq_cmd;
	srb->cdb_len      = sizeof(inq_cmd_t);
	srb->scsi_context = NULL;
	srb->sense        = NULL;
	srb->sense_len    = 0;
	srb->timeout      = 5;
	srb->callback     = rc_inq_callback;  // FIXME
	srb->sg_list      = (rc_sg_list_t *)&srb->private32[0];
	srb->dev_private  = (char *)srb->sg_list + sg_list_size;
	srb->next         = (rc_srb_t *)0;
	srb->seq_num      = rc_srb_seq_num++;

	if (update_mode == RC_SRB_LOCAL_UPDATE_ONLY)
		srb->flags |= RC_SRB_LOCAL_UPDATE_ONLY;

	/*
	 * Build the inquiry command.
	 * The command has already been memset to zero.
	 */
	inq_cmd->opcode = RC_INQUIRY;
	inq_cmd->len = sizeof(rc_inq_data_t);

	/*
	 * build the sg list
	 */
	sg = srb->sg_list;
	sg->sg_num_elem = 1;
	sg->sg_mem_type = RC_MEM_VADDR;
	sg->sg_elem[0].size = sizeof(rc_inq_data_t);
	sg->sg_elem[0].v_addr = inq_data;

	rc_printk(RC_DEBUG3, "rc_send_inq: seq_num: %d B/T/L %d/%d/%d\n",
		  srb->seq_num, bus, target, lun);

	/*
	 * Add the srb to the tail of the queue, schedule the q tasklet, and
	 * return.  Sending srb's through the OSIC can take a long time.  We don't
	 * want other CPU's spinning on the OSIC lock and just wasting cycles.  So
	 * we queue the srb's and just have one tasklet entering the OSIC.
	 */

	state->stats.scb_total++;
	state->stats.target_total[target]++;

	atomic_inc(&state->stats.scb_pending);
	atomic_inc(&state->stats.target_pending[target]);

	spin_lock_irqsave(&state->srb_q.lock, irql);

	if (state->srb_q.head == (rc_srb_t *)0) {
		state->srb_q.head = srb;
		state->srb_q.tail = srb;
	} else {
		state->srb_q.tail->next = srb;
		state->srb_q.tail = srb;
	}
	spin_unlock_irqrestore(&state->srb_q.lock, irql);

	tasklet_schedule(&state->srb_q.tasklet);
}

/*
 * Just queue the inquiry response to the rc_event_thread kernel thread
 */
static void
rc_inq_callback (rc_srb_t *srb)
{
	unsigned long irql;
	int         do_wakeup;
	rc_event_thread_t *tp = &rc_event_thread;


	rc_printk(RC_DEBUG3, "rc_inq_callback: target %d, \n", srb->target);

	srb->next = (rc_srb_t *)0;
	spin_lock_irqsave(&tp->cfg_change_response.lock, irql);

	if (tp->cfg_change_response.head == (rc_srb_t *)0) {
		tp->cfg_change_response.head = srb;
		tp->cfg_change_response.tail = srb;
	} else {
		tp->cfg_change_response.tail->next = srb;
		tp->cfg_change_response.tail = srb;
	}

	if (rc_event_thread.num_srb == 0)
		do_wakeup = 1;
	else
		do_wakeup = 0;
	rc_event_thread.num_srb++;

	spin_unlock_irqrestore(&tp->cfg_change_response.lock, irql);

	if (do_wakeup)
		wake_up_process(tp->thread);
}

static int
rc_event_kthread(void *rc_threadp)
{
	rc_event_thread_t    *tp;
	rc_srb_t         *srb;
	rc_softstate_t        *state;
	unsigned long        irql;

	state = &rc_state;
	tp    = (rc_event_thread_t*)rc_threadp;

	rc_printk(RC_DEBUG3, "rcraid: rc_event_kthread thread started\n");

	do {
		spin_lock_irqsave(&tp->cfg_change_response.lock, irql);

		while (tp->cfg_change_response.head) {
			/*
			 * take an srb off the list
			 */
			srb = tp->cfg_change_response.head;
			tp->cfg_change_response.head = srb->next;
			srb->next = (rc_srb_t *)0;
			tp->num_srb--;

			spin_unlock_irqrestore(&tp->cfg_change_response.lock, irql);
			rc_cfg_change_response(srb);
			spin_lock_irqsave(&tp->cfg_change_response.lock, irql);
		}

		spin_unlock_irqrestore(&tp->cfg_change_response.lock, irql);

		//rc_printk(RC_DEBUG3,"rc_event_thread: sleeping\n");
		tp->running = 0;

		schedule_timeout_interruptible(MAX_SCHEDULE_TIMEOUT);

		tp->running = 1;
		//rc_printk(RC_DEBUG3,"rc_event_kthread: awake\n");

	} while (!kthread_should_stop());

	rc_printk(RC_INFO, "rcraid: rc_event_kthread stopped\n");
	return 1;
}

static void
rc_cfg_change_response(struct rc_srb_s *srb)
{
	rc_uint32_t        bus, target, lun, status;
	rc_inq_data_t     *inq_data;
	rc_sg_list_t      *sg;
	char              *msg;
	int                local_update;
	int                present, was_present;
	int                locked = 0;
	rc_softstate_t    *state;
	rc_event_thread_t *tp;

	tp = &rc_event_thread;
	state = &rc_state;

	status   = srb->status;
	bus      = srb->bus;
	target   = srb->target;
	lun      = srb->lun;
	sg       = srb->sg_list;
	inq_data = (rc_inq_data_t *)sg->sg_elem[0].v_addr;

	if (srb->flags & RC_SRB_LOCAL_UPDATE_ONLY )
		local_update = 1;
	else
		local_update = 0;

	msg = "unknown status";
	present = 0;

	was_present =  tp->targets[target] & (1 << lun) ? 1 : 0;

	switch (status) {
	case RC_SRB_STATUS_SUCCESS:
		present = 1;
		msg = "device present";
		break;

	case RC_SRB_STATUS_NO_DEVICE:
		msg = "no device";
		break;

	case RC_SRB_STATUS_INVALID_LUN:
		msg = "invalid lun";
		break;

	case RC_SRB_STATUS_INVALID_TARGET_ID:
		msg = "invalid target";
		break;

	case RC_SRB_STATUS_INVALID_PATH_ID:
		msg = "invalid path";
		break;

	case RC_SRB_STATUS_DEVICE_LOCKED:
		msg = "no device, but locked by OS";
		locked = 1;

	default:
		break;
	}

	rc_printk(RC_DEBUG, "rc_cfg_change_response B/T/L %d/%d/%d  %s\n",
		  bus, target, lun, msg);

	/* device present status changed */
	if (was_present != present) {
		/*
		 * If the device has been removed but is locked (most likely due to a
		 * mount to this device), then we cannot remove scsi device; otherwise
		 * linux hangs trying to umount later.  If mounts are removed and/or
		 * array is deleted, then we will remove the scsi device.
		 */
		if (!present && locked) {
			rc_printk(RC_INFO,"rcraid:  B/T/L %d/%d/%d cannot be detached - "
				  "LOCKED\n", bus, target, lun);
			rc_printk(RC_INFO,"      : Please umount the filesystem\n");
		} else {
			if (present)
				tp->targets[target] |= (1 << lun);
			else
				tp->targets[target] &= ~(1 << lun);

			if (local_update == 0)
				rc_notify_scsi_layer(bus, target, lun, present);
		}
	}

	kfree(srb->cdb);    /* free inquiry command and response data */
	kfree(srb);
}

static int
rc_notify_scsi_layer(int channel, int target, int lun, int present)
{
	struct scsi_device *sdev;
	struct Scsi_Host *shost;
	int error = -ENXIO;

	rc_printk(RC_DEBUG, "%s:%d bus %d target %d lun %d present %d\n",__FUNCTION__,__LINE__,
		  channel,target,lun,present);

	if(!(shost = scsi_host_get(rc_state.host_ptr)))
		return error;

	if (present) {
#if 0
        if (rc_scsi_delay)
            mdelay(rc_scsi_delay);
#endif
		error = scsi_add_device(shost, channel, target, lun);
	} else {
		sdev = scsi_device_lookup(shost, channel, target, lun);
		if (sdev) {
#if 0
            if (rc_scsi_delay)
                mdelay(rc_scsi_delay);
#endif
			scsi_remove_device(sdev);
			scsi_device_put(sdev);
			error = 0;
		}
	}
	scsi_host_put(shost);
	return error;
}
