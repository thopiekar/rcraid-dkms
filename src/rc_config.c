/****************************************************************************
 *
 * Copyright 2010-2013 Dot Hill Systems Corp. All rights reserved.
 *
 ****************************************************************************/

#include <linux/version.h>

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/sched.h>
#include <linux/completion.h>

#include <linux/vmalloc.h>
#include <linux/slab.h>
#include <linux/pagemap.h>
#include <linux/interrupt.h>
#include <linux/compat.h>

#include <scsi/scsi.h>
#include <scsi/sg.h>

#include "rc_types_platform.h"
#include "rc_srb.h"
#include "rc_scsi.h"
#include "rc_msg_platform.h"
#include "rc_adapter.h"

#if LINUX_VERSION_CODE < KERNEL_VERSION(5,18,0)
#include <linux/genhd.h>
#endif

#ifndef GFP_NOWAIT
#define GFP_NOWAIT	(GFP_ATOMIC & ~__GFP_HIGH)
#endif // ! GFP_NOWAIT

extern struct mutex ioctl_mutex;

extern rc_softstate_t rc_state;
extern int rc_srb_seq_num;
extern void rc_msg_process_srb(rc_srb_t *);

struct semaphore rccfg_wait;

static void
rccfg_callback (rc_srb_t *srb)
{
	up(&rccfg_wait);
}

static int
rccfg_io(struct sg_io_hdr *hdr)
{

	rc_srb_t	*srb;
	rc_softstate_t	*state;
	int              sg_list_size;
	int		 size;
	void	        *data;
	rc_sg_list_t    *rc_sg;
	int		 i = 0;
	int		 err = 0;
	unsigned long    irql;
    rc_uint32_t target;

	mutex_lock(&ioctl_mutex);
	state = &rc_state;
	sg_list_size = sizeof(rc_sg_list_t) +
		(RC_SG_MAX_ELEMENTS-1) * sizeof(rc_sg_elem_t);
	size = sizeof(rc_srb_t) + sg_list_size + state->memsize_per_srb;

	if (hdr->dxfer_len)
		switch (hdr->dxfer_direction) {
		default:
			return -EINVAL;
		case SG_DXFER_TO_DEV:
			//writing = 1;
			break;
		case SG_DXFER_TO_FROM_DEV:
		case SG_DXFER_FROM_DEV:
			break;
		}

	if ((srb =  kmalloc( size, GFP_ATOMIC)) == 0) {
		if ((srb = kmalloc( size << 1, GFP_ATOMIC)) == 0) {
			printk("rc_msg_send_srb: could not alloc %d bytes for"
			       " srb\n", size);
			return -ENOMEM;
		}
	}
	memset(srb, 0, size);

    target = 24; // hack alert
	srb->function     = RC_SRB_EXECUTE_SCSI;
	srb->status       = RC_SRB_STATUS_SUCCESS;
	srb->bus          = 0;
	srb->target       = target;
	srb->lun          = 0;
	switch (hdr->dxfer_direction) {
	case SG_DXFER_TO_DEV:
		srb->flags    = RC_SRB_FLAGS_DATA_IN;
		break;
	case SG_DXFER_TO_FROM_DEV:
	case SG_DXFER_FROM_DEV:
		srb->flags    = RC_SRB_FLAGS_DATA_OUT;
		break;
	}

	if(!(srb->cdb = kmalloc(hdr->cmd_len, GFP_NOWAIT)) ) {
		err = -ENOMEM;
		goto out_srb_free;
	}

	if(copy_from_user(srb->cdb, hdr->cmdp, hdr->cmd_len)) {
		err = -EFAULT;
		goto out_cdb_free;
	}
	srb->cdb_len      = hdr->cmd_len;


	srb->data_len     = hdr->dxfer_len;
	srb->cdb_len      = hdr->cmd_len;
	srb->sense_len    = hdr->mx_sb_len;
	srb->scsi_context = NULL;
	srb->sg_list      = (rc_sg_list_t *)&srb->private32[0];
	srb->dev_private  = (char *)srb->sg_list + sg_list_size;
	srb->timeout      = 30;
	srb->seq_num      = rc_srb_seq_num++;
	srb->callback     = rccfg_callback;

	sema_init(&rccfg_wait, 0);

	if(!(data = vmalloc(hdr->dxfer_len))) {
		err = -ENOMEM;
		goto out_cdb_free;
	}
	memset(data, 0, hdr->dxfer_len);

	/* always copy data in from user */
	if(copy_from_user(data, hdr->dxferp, hdr->dxfer_len)) {
		printk("%s: copy_from_user failed\n",__FUNCTION__);
		err = -EFAULT;
		goto out_data_free;
	}

	rc_sg = srb->sg_list;
	rc_sg->sg_num_elem = 1;
	rc_sg->sg_mem_type = RC_MEM_VADDR;

	rc_sg->sg_elem[i].size = hdr->dxfer_len;
	rc_sg->sg_elem[i].v_addr = data;

	spin_lock_irqsave(&state->srb_q.lock, irql);
	if (state->srb_q.head == (rc_srb_t *)0) {
		state->srb_q.head = srb;
		state->srb_q.tail = srb;
	} else {
		state->srb_q.tail->next = srb;
		state->srb_q.tail = srb;
	}
	spin_unlock_irqrestore(&state->srb_q.lock, irql);

	state->stats.scb_total++;
	state->stats.target_total[target]++;

	atomic_inc(&state->stats.scb_pending);
	atomic_inc(&state->stats.target_pending[target]);

	tasklet_schedule(&state->srb_q.tasklet);

        // wait for callback completion
	down(&rccfg_wait);

	/* copy the srb status back into the sg hdr status
	 * this is appear to all that the user space uses
	 * for error reporting, but it might need sense data
	 * at some point
	 */
	hdr->status = srb->status;

	if(copy_to_user(hdr->dxferp, data, hdr->dxfer_len)) {
		printk("%s: copy_from_user failed\n",__FUNCTION__);
		err = -EFAULT;
		goto out_data_free;
	}

out_data_free:
	vfree(data);
out_cdb_free:
	kfree(srb->cdb);
out_srb_free:
	kfree(srb);

	mutex_unlock(&ioctl_mutex);
	return err;
}

struct sg_io_hdr32
{
	int interface_id;
	int dxfer_direction;
	unsigned char cmd_len;
	unsigned char mx_sb_len;
	unsigned short iovec_count;
	unsigned int dxfer_len;
	unsigned int dxferp;
	unsigned int cmdp;
	unsigned int sbp;
	unsigned int timeout;
	unsigned int flags;
	int pack_id;
	unsigned int usr_ptr;
	unsigned char status;
	unsigned char masked_status;
	unsigned char msg_status;
	unsigned char sb_len_wr;
	unsigned short host_status;
	unsigned short driver_status;
	int resid;
	unsigned int duration;
	unsigned int info;
};

static long
rccfg_ioctl(struct file * filp, unsigned int cmd, unsigned long arg)
{
	int err = 0;
	void __user *argp = (void __user *)arg;
	struct sg_io_hdr hdr;

	switch (cmd) {
	case SG_IO:

		err = -EFAULT;
		if (copy_from_user(&hdr, argp, sizeof(hdr)))
			break;
		err = rccfg_io(&hdr);
		if (err == -EFAULT)
			break;

		if (copy_to_user(argp, &hdr, sizeof(hdr)))
			err = -EFAULT;
		break;
	}
	return err;
}

#ifdef CONFIG_COMPAT
static long
rccfg_compat_ioctl(struct file * filp, unsigned int cmd, unsigned long arg)
{
	int err = 0;
	void __user *argp = (void __user *)arg;
	struct sg_io_hdr32 hdr;
	struct sg_io_hdr   hdr_native;

	switch (cmd) {
	case SG_IO:
		err = -EFAULT;
		if (copy_from_user(&hdr, argp, sizeof(hdr)))
			break;

		/*
		 * move all the fields over manually, specfically converting
		 * the 32 bit ptrs to native pointers
		 */
		hdr_native.interface_id		= hdr.interface_id;
		hdr_native.dxfer_direction	= hdr.dxfer_direction;
		hdr_native.cmd_len		= hdr.cmd_len;
		hdr_native.mx_sb_len		= hdr.mx_sb_len;
		hdr_native.iovec_count		= hdr.iovec_count;
		hdr_native.dxfer_len		= hdr.dxfer_len;
		hdr_native.dxferp		= compat_ptr(hdr.dxferp);
		hdr_native.cmdp			= compat_ptr(hdr.cmdp);
		hdr_native.sbp			= compat_ptr(hdr.sbp);
		hdr_native.timeout		= hdr.timeout;
		hdr_native.flags                = hdr.flags;
		hdr_native.pack_id		= hdr.pack_id;
		hdr_native.usr_ptr		= compat_ptr(hdr.usr_ptr);
		hdr_native.status               = hdr.status;
		hdr_native.masked_status	= hdr.masked_status;
		hdr_native.msg_status           = hdr.msg_status;
		hdr_native.sb_len_wr            = hdr.sb_len_wr;
		hdr_native.host_status		= hdr.host_status;
		hdr_native.driver_status	= hdr.driver_status;
		hdr_native.resid                = hdr.resid;
		hdr_native.duration             = hdr.duration;
		hdr_native.info                 = hdr.info;

		err = rccfg_io(&hdr_native);
		if (err == -EFAULT)
			break;

		hdr.status = hdr_native.status;
		if (copy_to_user(argp, &hdr, sizeof(hdr)))
			err = -EFAULT;
		break;
	}
	return err;
}
#else
#define rccfg_compat_ioctl NULL
#endif


struct file_operations rccfg_fops = {
	.owner		= THIS_MODULE,
	.unlocked_ioctl	= rccfg_ioctl,
	.compat_ioctl	= rccfg_compat_ioctl,
};

struct miscdevice rccfg_api_dev = {
	.minor  =  MISC_DYNAMIC_MINOR,
	.name   = "rc_api",
	.fops   = &rccfg_fops,
};
