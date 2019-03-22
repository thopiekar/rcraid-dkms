/****************************************************************************
 *
 * Copyright © 2006-2008 Ciprico Inc. All rights reserved.
 * Copyright © 2008-2015 Dot Hill Systems Corp. All rights reserved.
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

#include "rc.h"
#include "linux/sysrq.h"
#include "linux/nmi.h"
#include "asm/msr.h"
#include <linux/page-flags.h>
#include <linux/vmalloc.h>
#include "rc_ahci.h"

int  rc_setup_communications(void);
void rc_send_msg(struct rc_send_arg_s *p_send_arg);
void rc_msg_process_srb(rc_srb_t *srb);
void rc_receive_msg(void);
void rc_send_test(void);
int  rc_msg_send_srb(struct scsi_cmnd * scp);
void rc_msg_check_int_tasklet(unsigned long arg);

void rc_msg_send_srb_function (rc_softstate_t *state, int function_code);

#if LINUX_VERSION_CODE < KERNEL_VERSION(4,15,0)
void rc_msg_timer(unsigned long data);
#else
void rc_msg_timer(struct timer_list * t);
#endif

void rc_msg_timeout(int to);
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,15,0)
void rc_msg_timeout_done(unsigned long data);
#else
void rc_msg_timeout_done(struct timer_list * t);
#endif

void rc_msg_isr(rc_adapter_t *adapter);
void rc_msg_schedule_dpc(void);
void rc_msg_srb_done(struct rc_srb_s *srb);
void rc_msg_srb_complete(struct rc_srb_s *srb);
void rc_msg_build_sg(rc_srb_t *srb);
void rc_msg_map_phys_to_virt(struct map_memory_s *map);
void rc_msg_get_dma_memory(alloc_dma_address_t *dma_address);
void rc_msg_map_mem(struct map_memory_s *map);
void rc_msg_unmap_mem(struct unmap_memory_s *unmap);
void rc_msg_shutdown(rc_softstate_t *statep);
void rc_msg_access_ok(rc_access_ok_t accessOk);

void rc_msg_srb_q_tasklet(unsigned long arg);
void rc_msg_srb_done_tasklet(unsigned long arg);
void rc_dump_scp(struct scsi_cmnd * scp );
void rc_clear_stack(void);
int  rc_msg_stats(char *buf, int buf_size);
int  rc_mop_stats(char *buf, int buf_size);
void rc_wakeup_all_threads(void);

void
rc_add_dmaMemoryList(void *cpu_addr, dma_addr_t* dmaHandle, rc_uint32_t bytes,
			rc_adapter_t *adapter);

extern struct rc_interface_s RC_OurInterfaceStruct;

static struct rc_interface_s *rc_interface_header = &RC_OurInterfaceStruct;

int rc_srb_seq_num = 0;

static void rc_sysrq_intr (int key
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
			   ,struct pt_regs *pt_regs
#endif
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,36)
			   ,struct tty_struct * tty
#endif
			   );

static void rc_sysrq_state (int key
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
			    ,struct pt_regs *pt_regs
#endif
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,36)
			   ,struct tty_struct * tty
#endif
			   );

struct sysrq_key_op rc_skey_ops_intr = {
handler:    rc_sysrq_intr,
help_msg:   "rcraid-Force-interrupt",
action_msg: "forcing rcraid interrupt",
};

struct sysrq_key_op rc_skey_ops_dump = {
handler:    rc_sysrq_state,
help_msg:   "rcraid-Dump-state",
action_msg: "Dumping rcraid state",
};

#define check_lock(sp) {						\
		if (sp->osic_locked) {					\
			rc_printk(RC_WARN, "%s: osic already locked by %s\n", __FUNCTION__, \
				  sp->osic_lock_holder);		\
			panic("osic_lock already held\n");		\
		}							\
	}

void
rc_set_sense_data (char *sense, uint8_t sense_key, uint8_t sense_code,
		   uint8_t add_sense_code, uint8_t incorrect_len,
		   uint8_t bit_ptr, uint32_t field_ptr, uint32_t residue);

static char rc_stats_buf[1024];

#define TWO_TRIP_WRITE_RATE_DEFAULT    16
#define TWO_TRIP_WRITE_RATE_MIN         8
#define TWO_TRIP_WRITE_RATE_MAX      1024

static int TwoTripWriteRate = TWO_TRIP_WRITE_RATE_DEFAULT;
#ifdef module_param_named
module_param_named(TwoTripWriteRate, TwoTripWriteRate, int, 0);
#else
MODULE_PARM (TwoTripWriteRate, "i");
#endif
MODULE_PARM_DESC (TwoTripWriteRate, "min write rate (kb/s) to bypass cache");



#define ONE_TRIP_WRITE_RATE_DEFAULT   8
#define ONE_TRIP_WRITE_RATE_MIN       4
#define ONE_TRIP_WRITE_RATE_MAX     256

static int OneTripWriteRate = ONE_TRIP_WRITE_RATE_DEFAULT;
#ifdef module_param_named
module_param_named(OneTripWriteRate, OneTripWriteRate, int, 0);
#else
MODULE_PARM (OneTripWriteRate, "i");
#endif
MODULE_PARM_DESC (OneTripWriteRate, "min write rate (kb/s) to bypass cache");;


#define WRITE_BYPASS_THRESHOLD_DEFAULT     50
#define WRITE_BYPASS_THRESHOLD_MIN      4
#define WRITE_BYPASS_THRESHOLD_MAX    128

static int    WriteBypassThreshold = WRITE_BYPASS_THRESHOLD_DEFAULT;
#ifdef module_param_named
module_param_named(WriteBypassThreshold, WriteBypassThreshold, int, 0);
#else
MODULE_PARM (WriteBypassThreshold, "i");
#endif
MODULE_PARM_DESC (WriteBypassThreshold, "min write size (kb) to bypass cache");

#define ACTIVE_RAID5_FLUSHES_LIMIT_DEFAULT    64
#define ACTIVE_RAID5_FLUSHES_LIMIT_MIN         8
#define ACTIVE_RAID5_FLUSHES_LIMIT_MAX       128
static int ActiveRaid5FlushesLimit = ACTIVE_RAID5_FLUSHES_LIMIT_DEFAULT;

#ifdef module_param_named
module_param_named(ActiveRaid5FlushesLimit, ActiveRaid5FlushesLimit, int, 0);
#else
MODULE_PARM (ActiveRaid5FlushesLimit, "i");
#endif
MODULE_PARM_DESC (ActiveRaid5FlushesLimit, "Active Raid5 Flushes Limit");

static int ForcePhysAddr = 0;
#ifdef module_param_named
module_param_named(ForcePhysAddr, ForcePhysAddr, int, 0);
#else
MODULE_PARM (ForcePhysAddr, "i");
#endif
MODULE_PARM_DESC (ForcePhysAddr,
		  "force all memory references to use physical addresses");

static unsigned int DoSmart = 1;
#ifdef module_param_named
module_param_named(DoSmart, DoSmart, uint, 0444);
#else
MODULE_PARM (DoSmart, "i");
#endif
MODULE_PARM_DESC (DoSmart, "Do SMART polling");

#define SMART_POLL_INTERVAL_DEFAULT 900
static unsigned int SmartPollInterval = SMART_POLL_INTERVAL_DEFAULT;
#ifdef module_param_named
module_param_named(SmartPollInterval, SmartPollInterval, uint, 0444);
#else
MODULE_PARM (SmartPollInterval, "i");
#endif
MODULE_PARM_DESC (SmartPollInterval, "SMART poll interval in seconds");

/*
 * Simple wrapper function to map printf calls from the core to vprintk
 */
int32_t
rc_vprintf(uint32_t severity, const char *format, va_list ar)
{
	struct timeval tv;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,0,0)
	struct timespec ts;
#endif
        static int rc_saw_newline=1;

	if (severity > rc_msg_level)
		return 0;

        if (severity && rc_saw_newline) {
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 0, 0))
		getnstimeofday(&ts);
		tv.tv_sec = ts.tv_sec;
		tv.tv_usec = ts.tv_nsec / 1000;
#else
		do_gettimeofday(&tv);
#endif
		printk("rcraid: (%li.%06li) ", tv.tv_sec, tv.tv_usec);
	}

        rc_saw_newline = strchr(format, '\n') ? 1 : 0;

	return vprintk(format, ar);
}

/*
 *
 *   ROUTINE: rc_setup_communications
 *
 *        This routine sets up the communications with the code that is
 *        installed in CPU memory.
 *
 *    Returns:
 *        1 = Success
 *        0 = Failure
 */
int
rc_setup_communications(void)
{

	if ( (rc_interface_header->cookie_lo == RC_COOKIE_VALUE_LO)
	     && (rc_interface_header->cookie_hi == RC_COOKIE_VALUE_HI)
	     && (rc_interface_header->version == RC_INTERFACE_VERSION)
	     && (rc_interface_header->checksum == RC_INTERFACE_CHECKSUM)) {
		//
		//    We found the interface structure!! Fill in the values
		//    with what we will use!
		//
		rc_printk(RC_DEBUG, "send_function: offset: %p\n", rc_interface_header->send_function);
		rc_interface_header->receive_function = &rc_receive_msg;
        rc_interface_header->schedule_dpc_function = & rc_msg_schedule_dpc;
		return 1;
	}
	return 0;
}


void
rc_check_interrupt(rc_adapter_t* adapter)
{

	preempt_disable();
    rc_interface_header->check_interrupt_arg = adapter->private_mem.vaddr;

    (*rc_interface_header->check_interrupt_function)();
	preempt_enable();

}

/*
 *
 *   ROUTINE: rc_send_msg
 *
 *        This routine sends a command to the hardware communications
 *
 *    Returns:
 *        None
 *
 */
void
rc_send_msg(struct rc_send_arg_s *args)
{
	preempt_disable();
	rc_interface_header->send_arg = args;
	(*rc_interface_header->send_function)();
	preempt_enable();
}





/*
 *
 *   ROUTINE: rc_send_test
 *
 *        send a test command to the hardware communications
 *
 *    Returns:
 *        None
 *
 */
void
rc_send_test(void)
{
	struct rc_send_arg_s args;
	args.call_type = RC_CTS_TEST;
	rc_send_msg(&args);
}

void rc_msg_free_all_dma_memory(rc_adapter_t	*adapter);

/*
 *
 *   ROUTINE: rc_msg_suspend
 *
 *        Take actions when hibernating
 *
 *    Returns:
 *        None
 *
 */
void rc_msg_suspend(rc_softstate_t *state, rc_adapter_t* adapter)
{
    rc_send_arg_t   args;
    int             i;

    rc_printk(RC_NOTE, "%s\n",__FUNCTION__);

    state->is_suspended = 1;

    // flush the logical disk cache
    rc_printk(RC_ALERT, "rc_msg_suspend: flushing cache\n");
	rc_msg_send_srb_function(state, RC_SRB_FLUSH);

	rc_printk(RC_ALERT, "rc_msg_suspend: shutting down\n");
    rc_msg_send_srb_function(state, RC_SRB_SHUTDOWN);

	if ((state->state & ENABLE_TIMER) == ENABLE_TIMER)
	{
    	rc_printk(RC_INFO2, "rc_msg_shutdown: stop OSIC timer\n");
		state->state &= ~ENABLE_TIMER;
		del_timer_sync(&state->timer);
	}
    rc_printk(RC_ALERT, "rc_msg_suspend: pausing for 1/4 second\n");
    rc_msg_timeout(HZ>>2);

    // make sure all IOs are completed
	spin_lock(&state->osic_lock);
	check_lock(state);
	state->osic_locked = 1;
	state->osic_lock_holder = "rc_msg_suspend";

    for (i = state->num_hba - 1; i >= 0; i--)
    {
        if (rc_dev[i]->private_mem.vaddr) {
	        args.call_type = RC_CTS_STOP_ADAPTER;
            args.u.adapterMemory = rc_dev[i]->private_mem.vaddr;
    	    rc_send_msg(&args);
	    }
        else {
            rc_printk(RC_ERROR, "%s: no adapter memory :( \n",__FUNCTION__);
        }
    }
    state->osic_locked = 0;
	spin_unlock(&state->osic_lock);

    for (i = state->num_hba - 1; i >= 0; i--)
    {
        rc_msg_free_all_dma_memory(rc_dev[i]);
    }

}

/*
 *
 *   ROUTINE: rc_msg_resume
 *
 *        Take actions when resuming
 *
 *    Returns:
 *        None
 *
 */
void rc_msg_resume(rc_softstate_t *state, rc_adapter_t* adapter)
{
    rc_send_arg_t   args;
    int             i;

    rc_printk(RC_NOTE, "%s\n",__FUNCTION__);

    // don't lock, holding a spinlock while restarting the controller makes linux unhappy
    // since the timer and interrupts are disable this should be fine

    for (i = state->num_hba - 1; i >= 0; i--)
    {
        if (rc_dev[i]->private_mem.vaddr) {
	        args.call_type = RC_CTS_RESART_ADAPTER;
            args.u.adapterMemory = rc_dev[i]->private_mem.vaddr;
    	    rc_send_msg(&args);
	    }
        else {
            rc_printk(RC_ERROR, "%s: no adapter memory :( \n",__FUNCTION__);
        }
    }

	if ((state->state & ENABLE_TIMER) != ENABLE_TIMER)
	{
    	state->state |= ENABLE_TIMER;
		add_timer(&state->timer);
	}

    if (state->is_suspended)
    {
        rc_msg_send_srb_function(state, RC_SRB_RESTART);
        state->is_suspended = 0;
    }
}

void
rc_msg_shutdown( rc_softstate_t *statep)
{

	/*
	 * send a message to the OSIC to shutdown
	 * have to wait for it to stop doing IO.
	 */
	rc_printk(RC_INFO2, "rc_msg_shutdown: flushing cache OSIC\n");
	rc_msg_send_srb_function(statep, RC_SRB_FLUSH);

	rc_printk(RC_INFO2, "rc_msg_shutdown: shutting down OSIC\n");
	rc_msg_send_srb_function(statep, RC_SRB_SHUTDOWN);

	rc_printk(RC_INFO2, "rc_msg_shutdown: stop OSIC timer\n");
	statep->state &= ~ENABLE_TIMER;
	del_timer_sync(&statep->timer);
	rc_printk(RC_DEBUG, "rc_msg_shutdown: pausing for 1/4 second\n");
	rc_msg_timeout(HZ>>2);

	rc_printk(RC_INFO2, "rc_msg_shutdown: OSIC disabled \n");
	statep->state &= ~USE_OSIC;

	rc_stop_all_threads();
	rc_event_shutdown();

	rc_printk(RC_INFO2, "rc_msg_shutdown: shutting down srb tasklets\n");
	tasklet_kill(&statep->srb_done.tasklet);
	tasklet_kill(&statep->srb_q.tasklet);

	if (statep->virtual_memory) {
		rc_printk(RC_DEBUG, "rc_msg_shutdown: free virtual memory %p\n",
			  statep->virtual_memory);
		vfree(statep->virtual_memory);
		statep->virtual_memory_size = 0;
		statep->virtual_memory = (void *)0;
	}

	if (statep->cache_memory) {
		rc_printk(RC_DEBUG, "rc_msg_shutdown: free cache memory %p\n",
			  statep->cache_memory);
		vfree(statep->cache_memory);
		statep->cache_memory_size = 0;
		statep->cache_memory = (void *)0;
	}

	unregister_sysrq_key('f', &rc_skey_ops_intr);
	unregister_sysrq_key('d', &rc_skey_ops_dump);
}

/*
 * read or write PCI device configuration space
 *  - dword and byte (32 bit, 8 bit) support only
 *  - word (16 bit) support to be added if/when needed
 */
void rc_msg_pci_config(rc_pci_op_t *pci_op, rc_uint32_t call_type)
{
    struct pci_dev *pdev = (struct pci_dev *) NULL;
    rc_uint8_t tmp = 0x00;

    if (pci_op &&
        (pci_op->adapter < MAX_HBA) &&
        (pci_op->adapter < rc_state.num_hba) &&
        rc_dev[pci_op->adapter] &&
        (rc_dev[pci_op->adapter])->pdev) {
        // grab the pci device from the list of adapters
        pdev = (rc_dev[pci_op->adapter])->pdev;
        switch (call_type) {
        case RC_PCI_READ_CONFIG_BYTE:
            pci_op->status = pci_read_config_byte(pdev, pci_op->offset, &tmp);
            if (!pci_op->status) {
                pci_op->val = tmp;
            }
            break;
        case RC_PCI_READ_CONFIG_DWORD:
            pci_op->status = pci_read_config_dword(pdev, pci_op->offset, &(pci_op->val));
            break;
        case RC_PCI_WRITE_CONFIG_BYTE:
            tmp = (rc_uint8_t) (pci_op->val);
            pci_op->status = pci_write_config_byte(pdev, pci_op->offset, tmp);
            break;
        case RC_PCI_WRITE_CONFIG_DWORD:
            pci_op->status = pci_write_config_dword(pdev, pci_op->offset, pci_op->val);
            break;
        default:
            // set error status, anything non-zero will do
            pci_op->status = call_type;
            break;
        }
    }
    if (!pdev || !pci_op || pci_op->status) {
        if (!pdev && pci_op) {
            // set error status, anything non-zero will do
            pci_op->status = call_type;
        }
        rc_printk(RC_WARN, "%s: error: call_type: %d\n", __FUNCTION__, call_type);
    //} else {
        //RC_PRINTK(RC_WARN, "%s: call_type=%d, offset=0x%02X, val=0x%08X\n",
        //    __FUNCTION__, call_type, pci_op->offset, pci_op->val);
    }
}

/*
 *
 *
 *
 */
int rc_wq_handler(void *work)
{
    rc_work_t   *rc_work = (rc_work_t *) work;
    struct rc_receive_arg_s *args;

    set_current_state(TASK_INTERRUPTIBLE);
    while (!kthread_should_stop())
    {

	    //
	    // FIXME: We might not process these fast enough to be ready for the next request.
	    //        Make this into a queue and keep processing while the queue isn't empty.
	    //
	    while (acpi_work_item_head)
	    {
	        //set_current_state(TASK_RUNNING);
	        rc_work = (rc_work_t *) acpi_work_item_head;

	        args = (struct rc_receive_arg_s *) rc_work->args;

	        switch (rc_work->call_type)
	        {
	        case RC_ACPI_INVOKE:
                rc_printk(RC_INFO, "### %s(): Invoke ACPI method \"%s\"\n", __FUNCTION__, rc_work->method);
	            if (args->u.acpi.inPtr == NULL && args->u.acpi.outPtr == NULL &&
	                    args->u.acpi.inSize == 0 && args->u.acpi.outSize == 0)
	            {
//#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,2,1)
                    //
                    // Somewhere after 3.2.0, ACPI no longer enables GPE's if the device
                    // is WAKE capable. Instead, ACPI relies on the power management system
                    // to handle this. Since power management more or less requires the module
                    // to have a GPL license to call many of the required APIs, we need to
                    // deal with the GPE clear/enable here...
                    //
                    // Check if we're trying to turn off the power. If so, handle the GPE.
                    // (Don't want to deal with OS, especially Linux, calls in the blob. Push
                    // the call here so that everytime the interface changes we can add yet
                    // another hack... Sigh.)
                    //
                    if (strncmp((char *) &rc_work->method[strlen(rc_work->method) - 4], "_PS3", 4) == 0)
                    {
                        acpi_status ac_stat;

                        // Clear any pending notifcations
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,0,0)
                        ac_stat = acpi_clear_gpe(NULL, RC_ODD_GpeNumber, ACPI_NOT_ISR);
#else
                        ac_stat = acpi_clear_gpe(NULL, RC_ODD_GpeNumber);
#endif  /* LINUX_VERSION_CODE < KERNEL_VERSION(3,0,0) */
                        // Arm for notification
                        ac_stat = acpi_enable_gpe(NULL, RC_ODD_GpeNumber);
                    }
//#endif
	                // Invoke only
	                args->u.acpi.status =
	                        acpi_evaluate_object((acpi_handle) rc_work->handle, rc_work->method, NULL, NULL);
	            } else {
	                union acpi_object *out_object;
	                struct acpi_buffer outBuf = { ACPI_ALLOCATE_BUFFER, NULL };

	                if (args->u.acpi.inPtr)
	                {
	                    if (args->u.acpi.outPtr) {
	                        // Input and Output
	                        args->u.acpi.status =
	                                acpi_evaluate_object((acpi_handle) rc_work->handle, rc_work->method, args->u.acpi.inPtr, args->u.acpi.outPtr);
	                    } else {
	                        // Input, no Output
	                        args->u.acpi.status =
	                                acpi_evaluate_object((acpi_handle) rc_work->handle, rc_work->method, args->u.acpi.inPtr, NULL);
	                    }
	                } else {
	                    // Output, no Input
	                    args->u.acpi.status =
	                            acpi_evaluate_object((acpi_handle) rc_work->handle, rc_work->method, NULL, &outBuf);
	                    out_object = outBuf.pointer;
	                    if (ACPI_SUCCESS(args->u.acpi.status))
	                    {
	                        switch (out_object->type)
	                        {
	                        case ACPI_TYPE_INTEGER:
	                            args->u.acpi.outSize = min((u64) args->u.acpi.outSize, (u64) sizeof(u64));
	                            memcpy(args->u.acpi.outPtr, &out_object->integer.value, args->u.acpi.outSize);
	                            break;
	                        case ACPI_TYPE_BUFFER:
	                            args->u.acpi.outSize = min(args->u.acpi.outSize, out_object->buffer.length);
	                            memcpy(args->u.acpi.outPtr, out_object->buffer.pointer,
	                                    args->u.acpi.outSize);
	                            break;
	                        default:
	                            ;
	                        }
	                    }
                        if (outBuf.pointer)
                            kfree(outBuf.pointer);
	                }
	            }
	            break;
	        default:
	            // Only other call really is RC_QUEUE_WORK which is just a way
	            //  to have the callback executed. Falling through allows that.
	            ;
	        }

	        if (args->u.acpi.callback)
	        {
	            (*args->u.acpi.callback)(rc_work->args);
	        }

	        kfree((void *) rc_work->method);

	        spin_lock(&acpi_work_item_lock);
	        if (acpi_work_item_head == acpi_work_item_tail)
	        {
	            acpi_work_item_head = acpi_work_item_tail = NULL;
	        } else {
	            acpi_work_item_head = acpi_work_item_head->next;
	        }
	        spin_unlock(&acpi_work_item_lock);

	        kfree((void *) rc_work);        // Make sure this is rc_work as kthread passes NULL for parameter work!
	    }

        // (acpi_work_item)
        schedule();
        set_current_state(TASK_INTERRUPTIBLE);
    }
    set_current_state(TASK_RUNNING);

    return 0;
}

/*
 *
 *   ROUTINE: rc_receive_msg(void)
 *
 *        This routine receives a message call from the hardware.
 *
 *    Returns:
 *        None
 *
 */
void
rc_receive_msg(void)
{
	struct rc_receive_arg_s *args;
	rc_softstate_t *state;
	int delay;


	args = rc_interface_header->receive_arg;
	state = &rc_state;

	preempt_enable();
	switch (args->call_type) {
	case RC_CTR_TEST:
		rc_printk(RC_DEBUG, "rc_receive_msg Send/Receive test passed\n");
		break;

	case RC_CTR_SRB_DONE:
		rc_msg_srb_done(args->u.srb_done.srb);
		break;

	case RC_CTR_INIT_DONE:
		rc_printk(RC_DEBUG, "rc_receive_msg: init done callback\n");
		up(&state->init_sema);
		break;

	case RC_CTR_ASSERTION_FAILURE:
		break;

	case RC_CTR_EVENT:
		rc_event(args->u.event.rc_notification_type,
			 args->u.event.rc_bus_changed,
			 RC_SRB_GLOBAL_UPDATE);
		break;

    case RC_CTR_VMAP_MEMORY:
        rc_msg_map_phys_to_virt(&args->u.map_memory);
        break;

	case RC_CTR_MAP_MEMORY:
		rc_msg_map_mem(&args->u.map_memory);
		break;

    case RC_CTR_GET_DMA_ADDRESS:
        rc_msg_get_dma_memory(&args->u.get_dma_memory);
        break;

	case RC_CTR_UNMAP_MEMORY:
		rc_msg_unmap_mem(&args->u.unmap_memory);
		break;

	case RC_CTR_PRINT_VA:
		rc_vprintf(args->u.print_va.severity, args->u.print_va.format, args->u.print_va.va_l);
		break;

    case RC_CTR_SCHEDULE_DPC:
        rc_msg_schedule_dpc();
        break;

	case RC_CTR_WAIT_MICROSECONDS:
		delay = args->u.wait_microseconds.microseconds;
		//        rc_printk(RC_DEBUG2, "delay %d microseconds\n", delay);

		// Touch ALL cpu's touch_timestamp to avoid
		// erroneous Soft CPU lockup's.
		// Note touch_nmi_watchdog() implies touch_softlockup_watchdog() on all
		// architectures that support it, and does nothing on those that don't.
		// SUSE 10.1 exports it in header file, but fails to link it in...
		// Works on 10.2....  Hmmm - Well Spinlocks never observed on any
		// SUSE and this is a simple way to exclude it - so lets
		// just exclude it from all SUSE kernels.
#ifndef CONFIG_SUSE_KERNEL
		touch_nmi_watchdog();
#endif

		if (delay <= 1000) {
			preempt_disable();
			udelay(delay);
			preempt_enable();
			break;
		}
		//
		// We need to honor the osic_lock, which is implicitly telling us
		// there is a msg pending on OSIC. We don't want to suspend a thread
		// from bottom that the OSIC is waiting for a response from via a
		// timer.  However, we also cannot be doing huge udelays or we will
		// get soft lock errors (udelay will overflow with large delay times
		// and cause erratic behavior.).
		// This msg can be received from both top and bottom, which makes it
		// more confusing....
		// The only time the huge udelays seem to occur during an osic_lock is
		// during initialization.  The initial spinup that is not induced from
		// the OSIC.  So I have added a state to indicate the msg_init callback
		// has occured.
		// So: if the OSIC is locked and delay is reasonable - use udelay.
		//   : if the OSIC is locked and we are done with initialization - use
		//     udelay.
		//   : Otherwise use the msg_timeout timer.
		if ((state->osic_locked) && ((delay < 15000) ||
					     (state->state & INIT_DONE))) {
			preempt_disable();
			udelay(delay);
			preempt_enable();
		} else {
			int ticks;
			int usec_per_tick;

			usec_per_tick =  1000000/HZ;
			delay += usec_per_tick >> 1;    /* round up by 1/2 clock tick */
			ticks = delay / usec_per_tick;

			if (ticks == 0)
				ticks = 1;
			rc_msg_timeout(ticks);
		}
		break;

	case RC_CTR_MEMORY_OP:
		rc_msg_mem_op(args->u.mem_op);
		break;

    case RC_CTR_ACCESS_OK:
        rc_msg_access_ok(args->u.isAccessOk);
        break;

	case RC_PCI_READ_CONFIG_BYTE:
	case RC_PCI_READ_CONFIG_DWORD:
	case RC_PCI_WRITE_CONFIG_BYTE:
	case RC_PCI_WRITE_CONFIG_DWORD:
		rc_msg_pci_config(&(args->u.pci_op), args->call_type);
		break;

    case RC_ACPI_INVOKE:
        {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 13, 0)
            rc_adapter_t    *adapter = rc_dev[0];       // FIXME
            struct pci_dev  *pdev = adapter->pdev;
            acpi_handle     handle = DEVICE_ACPI_HANDLE(&pdev->dev);
            rc_work_t       *work;

            args->u.acpi.status = -1;
            if (rc_wq)
            {
                work = (rc_work_t *) kmalloc(sizeof(rc_work_t), GFP_KERNEL);
                if (work)
                {
                    int ret;

                    memset(work, 0, sizeof(rc_work_t));

                    work->call_type = args->call_type;
                    work->method = (char *) kmalloc(strlen(args->u.acpi.method) + 1, GFP_KERNEL);
                    work->handle = handle;
                    work->args = args;
                    if (work->method)
                    {
                        memcpy(work->method, args->u.acpi.method, strlen(args->u.acpi.method) + 1);
                        ret = 0;
                        //
                        // FIXME: acpi_work_item might not be NULL -- need to make a queue for these...
                        //
                        spin_lock(&acpi_work_item_lock);
                        if (acpi_work_item_tail != NULL)
                        {
                            acpi_work_item_tail->next = work;
                            acpi_work_item_tail = work;
                        } else {
                            acpi_work_item_head = acpi_work_item_tail = work;
                        }
                        spin_unlock(&acpi_work_item_lock);
                        wake_up_process(rc_wq);
                    } else {
                        // Failed to queue -- free memory
                        kfree((void *) work);
                    }
                }
            }
#endif  /* LINUX_VERSION_CODE >= KERNEL_VERSION(3, 13, 0) */
        }
        break;

    case RC_ACPI_REGISTER:
        {
            acpi_handle     handle;

            switch (RC_ODD_Device)
            {
            case RC_ODD_DEVICE_ODDZ:
                args->u.acpi.status = acpi_get_handle(NULL, "\\_SB.PCI0.SATA.ODDZ", &handle);
                break;
            case RC_ODD_DEVICE_ODDL:
                args->u.acpi.status = acpi_get_handle(NULL, "\\_SB.PCI0.SATA.ODDL", &handle);
                break;
            case RC_ODD_DEVICE_ODD8:
                args->u.acpi.status = acpi_get_handle(NULL, "\\_SB.PCI0.SATA.ODD8", &handle);
                break;
            }

            if (ACPI_SUCCESS(args->u.acpi.status))
            {
                //
                // When configured in RAID mode, the notify returns 0x80 -- that is a DEVICE
                //      notification. In AHCI mode, the return is 0x02 which is a SYSTEM notify.
                //
                args->u.acpi.status = acpi_install_notify_handler(handle, ACPI_DEVICE_NOTIFY,
                        args->u.acpi.context, NULL);
                // Apparently, Ubuntu 13.04 sends SYSTEM notifications...
                args->u.acpi.status |= acpi_install_notify_handler(handle, ACPI_SYSTEM_NOTIFY,
                        args->u.acpi.context, NULL);
            }
        }
        break;

	default:
		rc_printk(RC_WARN,"rc_receive_msg: unknown msg type 0x%x\n",
			  args->call_type);
		break;
	}
	preempt_disable();
}


/*
  Tasklets for resuming and suspending.  Work must be done in tasklets or Linux will hang

*/

void rc_msg_resume_work(void)
{
    rc_adapter_t* adapter;
    adapter = rc_dev[0];
    rc_printk(RC_ERROR, "%s: schedule resume tasklet\n",__FUNCTION__);
    rc_msg_resume(&rc_state, adapter);
}

void rc_msg_suspend_work(rc_adapter_t *adapter)
{
    rc_printk(RC_ERROR, "%s: schedule suspend tasklet\n",__FUNCTION__);
    rc_msg_suspend(&rc_state, adapter);
}


void rc_msg_init_tasklets(rc_softstate_t *state)
{
	tasklet_init(&state->srb_done.tasklet, rc_msg_srb_done_tasklet,
		     (unsigned long)state);
	tasklet_init(&state->srb_q.tasklet, rc_msg_srb_q_tasklet,
		     (unsigned long)state);
	tasklet_init(&state->intr_tasklet, rc_msg_check_int_tasklet,
		     (unsigned long)state);
}

void rc_msg_kill_tasklets(rc_softstate_t *state)
{
    tasklet_kill(&state->srb_done.tasklet);
    tasklet_kill(&state->srb_q.tasklet);
    tasklet_kill(&state->intr_tasklet);
}

/*
 * setup the whole message passing system
 */

int
rc_msg_init(rc_softstate_t *state)
{

	rc_send_arg_t    args;
	int         i, size,  period;
	rc_adapter_t    *adapter;

	/*
	 * find the initialization struct and fill in our function pointer
	 */
	if (rc_setup_communications() == 0) {
		rc_printk(RC_ERROR,"rc_msg_init: could not find init structure\n");
		return(1);
	}

	rc_send_test();

	/*
	 * setup lock and counter for processing pending interrupts
	 */
	atomic_set(&state->intr_pending, 0);

	/*
	 * setup tasklet for srb q processing;
	 */

	state->srb_q.head = (rc_srb_t *)0;
	state->srb_q.tail = (rc_srb_t *)0;
    spin_lock_init(&state->srb_q.lock);

    INIT_DELAYED_WORK(&state->resume_work, (void *) rc_msg_resume_work);

    /*
	 * setup tasklet for srb done processing;
	 */
	state->srb_done.head = (rc_srb_t *)0;
	state->srb_done.tail = (rc_srb_t *)0;
	spin_lock_init(&state->srb_done.lock);


    rc_msg_init_tasklets(state);

	state->mop_done.head = (rc_mem_op_t *)0;
	state->mop_done.tail = (rc_mem_op_t *)0;
	spin_lock_init(&state->mop_done.lock);

	state->osic_locked = 0;
	state->osic_lock_holder = "INIT";

	sema_init(&state->msg_timeout_sema, 0);

	rc_start_all_threads();
	rc_event_init();

	/*
	 *  send a  get_info msg to get setup info
	 */
	memset(&args, 0, sizeof(args));
	args.call_type = RC_CTS_GET_INFO;
	args.u.get_info.controller_count = state->num_hba;
	args.u.get_info.max_sg_map_elements = RC_SG_MAX_ELEMENTS;
	args.u.get_info.max_print_severity = rc_msg_level;
	get_random_bytes( &args.u.get_info.random_seed,
			  sizeof(args.u.get_info.random_seed));
	args.u.get_info.max_lba = 0xffffffffffffffffLL;

	if (TwoTripWriteRate < TWO_TRIP_WRITE_RATE_MIN)
		TwoTripWriteRate = TWO_TRIP_WRITE_RATE_MIN;
	if (TwoTripWriteRate > TWO_TRIP_WRITE_RATE_MAX)
		TwoTripWriteRate = TWO_TRIP_WRITE_RATE_MAX;

	args.u.get_info.parameter_mask |= (1 << rc_TwoTripWriteRate);
	args.u.get_info.tunable_parameters[rc_TwoTripWriteRate] =
		TwoTripWriteRate << 1;

	if (TwoTripWriteRate != TWO_TRIP_WRITE_RATE_DEFAULT)
		rc_printk(RC_INFO,"rcraid: set parameter TwoTripWriteRate = %d\n",
			  TwoTripWriteRate);

	if (OneTripWriteRate < ONE_TRIP_WRITE_RATE_MIN)
		OneTripWriteRate = ONE_TRIP_WRITE_RATE_MIN;
	if (OneTripWriteRate > ONE_TRIP_WRITE_RATE_MAX)
		OneTripWriteRate = ONE_TRIP_WRITE_RATE_MAX;

	args.u.get_info.parameter_mask |= (1 << rc_OneTripWriteRate);
	args.u.get_info.tunable_parameters[rc_OneTripWriteRate] =
		OneTripWriteRate << 1;

	if (OneTripWriteRate != ONE_TRIP_WRITE_RATE_DEFAULT)
		rc_printk(RC_INFO,"rcraid: set parameter OneTripWriteRate = %d\n",
			  OneTripWriteRate);

	if (WriteBypassThreshold < WRITE_BYPASS_THRESHOLD_MIN)
		WriteBypassThreshold = WRITE_BYPASS_THRESHOLD_MIN;

	if (WriteBypassThreshold > WRITE_BYPASS_THRESHOLD_MAX)
		WriteBypassThreshold = WRITE_BYPASS_THRESHOLD_MAX;

	args.u.get_info.parameter_mask |= (1 << rc_WriteBypassThreshold);
	args.u.get_info.tunable_parameters[rc_WriteBypassThreshold] =
		WriteBypassThreshold << 1;

	if (WriteBypassThreshold != WRITE_BYPASS_THRESHOLD_DEFAULT)
		rc_printk(RC_INFO,"rcraid: set parameter WriteBypassThreshold = %d\n",
			  WriteBypassThreshold);


	if (ActiveRaid5FlushesLimit < ACTIVE_RAID5_FLUSHES_LIMIT_MIN)
		ActiveRaid5FlushesLimit = ACTIVE_RAID5_FLUSHES_LIMIT_MIN;

	if  (ActiveRaid5FlushesLimit > ACTIVE_RAID5_FLUSHES_LIMIT_MAX)
		ActiveRaid5FlushesLimit = ACTIVE_RAID5_FLUSHES_LIMIT_MAX;

	args.u.get_info.parameter_mask |= (1 << rc_ActiveRaid5FlushesLimit);
	args.u.get_info.tunable_parameters[rc_ActiveRaid5FlushesLimit] =
		ActiveRaid5FlushesLimit;

	if (ActiveRaid5FlushesLimit != ACTIVE_RAID5_FLUSHES_LIMIT_DEFAULT)
		rc_printk(RC_INFO, "rcraid: set parameter ActiveRaid5FlushesLimit = "
			  "%d\n", ActiveRaid5FlushesLimit);

	if  (DoSmart > 1)
		DoSmart =  1;

	args.u.get_info.parameter_mask |= (1 << rc_DoSmart);
	args.u.get_info.tunable_parameters[rc_DoSmart] = DoSmart;

	if (DoSmart != 1)
		rc_printk(RC_INFO, "rcraid: set parameter DoSmart = %d, SMART polling"
			  " disabled\n", DoSmart);

	/* don't allow polling more than every minute */
	if (SmartPollInterval < 60)
		SmartPollInterval = 60;
	args.u.get_info.parameter_mask |= (1 << rc_SmartPollInterval);
	args.u.get_info.tunable_parameters[rc_SmartPollInterval] =
		SmartPollInterval;

	if (SmartPollInterval != SMART_POLL_INTERVAL_DEFAULT)
		rc_printk(RC_INFO, "rcraid: set parameter SmartPollInterval = %d "
			  "seconds\n", SmartPollInterval);

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,31)
    args.u.get_info.support4kNativeDisks = 0;
#else
    args.u.get_info.support4kNativeDisks = 1;
#endif

	rc_send_msg(&args);

	/* should sanity check sizes */
	state->memsize_per_controller = args.u.get_info.memory_size_per_controller;
	state->memsize_per_srb        = args.u.get_info.memory_size_per_srb;
	period = 1000 / HZ; /* milliseconds per interval */
	state->timer_interval =
		(args.u.get_info.timer_interval + period - 1) / period;


	/*
	 * now go allocate the private memory for each controller
	 */
	size = state->memsize_per_controller;
	for (i = 0; i < state->num_hba; i++) {
		void *addr;
		adapter = rc_dev[i];

		rc_printk(RC_DEBUG, "rc_msg_init: get private mem for controller %d\n",
			  i);
		if (adapter == (rc_adapter_t *)0) {
			rc_printk(RC_ERROR, "rc_msg_init null adapter\n");
		}
		addr = pci_alloc_consistent(adapter->pdev,
					    state->memsize_per_controller,
					    &adapter->private_mem.dma_address);

		if (addr == (void *)0) {
			rc_printk(RC_ERROR,"rc_msg_init: can not alloc %d bytes of per "
				  "controller memory\n", state->memsize_per_controller);
			/* need to free previously allocated.... */
			return(1);
		}

		adapter->private_mem.vaddr = addr;
		adapter->private_mem.size = state->memsize_per_controller;
		rc_printk(RC_DEBUG, "rc_msg_init: private mem controller %d @ %p len "
			  "%d\n", i, adapter->private_mem.vaddr,
			  adapter->private_mem.size );
	}


	/*
	 * now we can initialize each controller
	 */
	for (i = 0; i < state->num_hba; i++) {
		adapter = rc_dev[i];

		memset(&args, 0, sizeof(args));
		args.call_type = RC_CTS_INIT_CONTROLLER;
		args.u.init_controller.controller_memory = adapter->private_mem.vaddr;
		args.u.init_controller.controller_memory_id = RC_MEM_VADDR;
		args.u.init_controller.controller_handle = (void *)(adapter);
		args.u.init_controller.pci_config_space =
			adapter->hardware.pci_config_space;
		args.u.init_controller.pci_config_space_length = PCI_CFG_SIZE;
		args.u.init_controller.bar_memory[adapter->version->which_bar] =
			adapter->hardware.vaddr;
		args.u.init_controller.bar_length[adapter->version->which_bar] =
			adapter->hardware.mem_len;
        args.u.init_controller.context = (void *) adapter;
        args.u.init_controller.regread = rc_ahci_regread;
        args.u.init_controller.regwrite = rc_ahci_regwrite;

		// Fill in the rest
		rc_printk(RC_DEBUG, "rc_msg_init: init controller %d\n", i);
		rc_send_msg(&args);
		rc_printk(RC_DEBUG, "rc_msg_init: init controller %d Done\n", i);
		/* return status ??? */
	}


	/* Allocate virtual memory for the core*/
	state->virtual_memory_size =
		args.u.init_controller.virtual_memory_size_needed;

	if (state->virtual_memory_size) {
		if ((state->virtual_memory = vmalloc(state->virtual_memory_size)) == 0)
		{
			rc_printk(RC_ERROR, "rc_msg_init: can't allocate virtual memory, "
				  "size %d bytes\n", state->virtual_memory_size);
			/* need to free previously allocated.... */
			return(1);
		}
		memset(state->virtual_memory, 0, state->virtual_memory_size);
	}
	rc_printk(RC_DEBUG,"rc_msg_init: alloc %d @ %p for virtual memory\n",
		  state->virtual_memory_size, state->virtual_memory);


	/* Allocate cache memory for the core */
	state->cache_memory_size = args.u.init_controller.cache_memory_size_needed;

	if (args.u.init_controller.cache_memory_size_needed) {
		if ((state->cache_memory = vmalloc(state->cache_memory_size)) == 0) {
			rc_printk(RC_ERROR, "rc_msg_init: can't allocate %u bytes of cache"
				  " memory\n", state->cache_memory_size);
			/* need to free previously allocated.... */
			return(1);
		}
		memset(state->cache_memory, 0, state->cache_memory_size);
	}
	rc_printk(RC_DEBUG,"rc_msg_init: alloc %d @ %p for cache memory\n",
		  state->cache_memory_size,
		  state->cache_memory);


	/*
	 * do the final init for the OSIC
	 */
	state->state |= USE_OSIC;

	args.call_type = RC_CTS_FINAL_INIT;
	args.u.final_init.virtual_memory = state->virtual_memory;
	args.u.final_init.cache_memory = state->cache_memory;
	args.u.final_init.size_of_virtual_memory_allocated =
		state->virtual_memory_size;
	args.u.final_init.size_of_cache_memory_allocated = state->cache_memory_size;
	args.u.final_init.virtual_memory_id = RC_MEM_VADDR;
	args.u.final_init.cache_memory_id = RC_MEM_VADDR;
	args.u.final_init.timer_interval = state->timer_interval * period;

	rc_printk(RC_INFO2,"rc_msg_init: doing final init\n");

	sema_init(&state->init_sema, 0);

	// The FINAL_INIT takes a long time and occasionaly on Fedora the CPU we
	// come back on has changed - giving us a Spinlock Wrong CPU warning.
	// Since the periodic timer is not enabled yet, and PROCESS_INTR has not
	// been enabled, nobody else should be using the osic_lock.
	// Therefore, to avoid this we will not obtain the spin_lock during the
	// FINAL_INIT.  We will still set the osic_locked flag - check_lock will
	// panic if someone else attempts to obtain lock during this time - just
	// in case I have missed something.
//    spin_lock(&state->osic_lock);
	check_lock(state);
	state->osic_locked = 1;
	state->osic_lock_holder = "rc_msg_init";
	rc_send_msg(&args);
	state->osic_locked = 0;
//   spin_unlock(&state->osic_lock);



	state->state |= PROCESS_INTR;
    // we are ready to process interrupts
    // check all adapters to see if any are outstanding
    for (i=0; i < rc_state.num_hba; i++) {
        rc_msg_isr(rc_dev[i]);
    }

	/*
	 * intialize the periodic timer for the OSIC
	 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,15,0)
	init_timer(&state->timer);
	state->timer.expires = jiffies  + state->timer_interval ;
	state->timer.data = (unsigned long)state;
	state->timer.function = rc_msg_timer;
#else
	timer_setup(&state->timer, rc_msg_timer, 0);
	state->timer.expires = jiffies + state->timer_interval;
#endif
	state->state |= ENABLE_TIMER;

	add_timer(&state->timer);

    rc_printk(RC_INFO2,"rc_msg_init: timer started.... wait for callback \n");
	down(&state->init_sema);
	rc_printk(RC_INFO2,"rc_msg_init: we're back\n");

	state->state |= INIT_DONE;

	register_sysrq_key('f', &rc_skey_ops_intr);
	register_sysrq_key('d', &rc_skey_ops_dump);

	/*
	 * Inventory what devices are connected.
	 * The flag means only build the device table, but don't send
	 * anything to the upper scsi layer.
	 */
	rc_event(RC_CTR_EVENT_CONFIG_CHANGE_DETECTED,
		 (rc_uint8_t) 0,
		 RC_SRB_LOCAL_UPDATE_ONLY);
	/*
	 * OK.  we're ready to accept IO for the OSIC
	 */
	return(0);
}
void
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,15,0)
rc_msg_timer(unsigned long data)
#else
rc_msg_timer(struct timer_list * t)
#endif
{
	rc_softstate_t *state;
	rc_send_arg_t    args;

#if LINUX_VERSION_CODE < KERNEL_VERSION(4,15,0)
	state = (rc_softstate_t *)data;
#else
	state = from_timer(state, t, timer);
#endif

	if ((state->state & ENABLE_TIMER) == 0)
		return;

	/*
	 * set up timeout
	 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,15,0)
	init_timer(&state->timer);
	state->timer.expires = jiffies  + state->timer_interval;
	state->timer.data = (unsigned long)state;
	state->timer.function = rc_msg_timer;
#else
	timer_setup(&state->timer, rc_msg_timer,0);
	state->timer.expires = jiffies + state->timer_interval;
#endif
	add_timer(&state->timer);

	spin_lock(&state->osic_lock);
	check_lock(state);
	state->osic_locked = 1;
	state->osic_lock_holder = "rc_msg_timer";
	args.call_type = RC_CTS_TIMER_CALL;
	rc_send_msg(&args);
	state->osic_locked = 0;
	spin_unlock(&state->osic_lock);
}


/*
 * just increment intr_pending and wake up the tasklet
 */
void
rc_msg_isr( rc_adapter_t *adapter)
{

	rc_softstate_t *state;
	state = &rc_state;

    // Do not process any interrupts until PROCESS_INTR flag is set
    // indicating the bottom driver is ready
    // rc_msg_init will check all adapters for interrupts at that time


    if ( (state->state & PROCESS_INTR) != 0) {
        atomic_inc(&adapter->checkInterrupt);
        tasklet_schedule(&state->intr_tasklet);
    }



}


void rc_msg_check_int_tasklet(unsigned long arg)
{
    rc_softstate_t *state;
    rc_adapter_t* adapter;
    int i;
    state = (rc_softstate_t *)arg;




    if ( (state->state & PROCESS_INTR) != 0) {
        spin_lock(&state->osic_lock);
	    check_lock(state);
	    state->osic_locked = 1;
	    state->osic_lock_holder = "rc_msg_check_int_tasklet";
        for (i=0; i < state->num_hba; i++) {
            adapter = rc_dev[i];
            if (atomic_read(&adapter->checkInterrupt) !=0) {
                atomic_set(&adapter->checkInterrupt,0);
                rc_check_interrupt(adapter);
            }
        }

        state->osic_locked = 0;
        spin_unlock(&state->osic_lock);
    }



}


void
rc_msg_schedule_dpc()
{
    rc_softstate_t *state;
    state = &rc_state;


    if (atomic_read(&state->intr_pending) < 20) {
        atomic_inc(&state->intr_pending);

        tasklet_schedule(&state->srb_q.tasklet);
    }

}

int
rc_msg_send_srb(struct scsi_cmnd * scp)
{
	rc_softstate_t *state;
	rc_srb_t       *srb;
	int             size;
	int             bus, target, lun;
	unsigned long   irql;
	int             sg_list_size;

	state = &rc_state;

	bus    = scp->device->channel;
	target = scp->device->id;
	lun    = scp->device->lun;

	sg_list_size = sizeof(rc_sg_list_t) +
		(RC_SG_MAX_ELEMENTS-1) * sizeof(rc_sg_elem_t);
	size = sizeof(rc_srb_t) + sg_list_size + state->memsize_per_srb;

	/* Safety check so we don't walk over memory
	 * we are going to fail IO here?
	 * TODO: maybe this should be a retry?
	 */
	if (scsi_sg_count(scp) > RC_SG_MAX_ELEMENTS) {
		rc_printk(RC_ERROR, "rc_msg_send_srb:  scatter-gather list too large "
			  "(%d)\n", scsi_sg_count(scp));
 		scp->result = DID_NO_CONNECT << 16;
 		scp->scsi_done(scp);
		return 0;
	}

	/*
	 * Use GFP_ATOMIC instead of GP_KERNEL because we get called from the srb
	 * tasklet
	 */
	if ((srb =  kmalloc( size, GFP_ATOMIC)) == 0) {

		if ((srb = kmalloc( size << 1, GFP_ATOMIC)) == 0) {
			rc_printk(RC_DEBUG,"rc_msg_send_srb: could not alloc %d bytes for"
				  " srb\n", size);
			rc_msg_stats(rc_stats_buf, sizeof(rc_stats_buf));
			rc_printk(RC_DEBUG, rc_stats_buf);
			return SCSI_MLQUEUE_HOST_BUSY;
		}
	}
	memset(srb, 0, size);

	srb->function     = RC_SRB_EXECUTE_SCSI;
	srb->status       = RC_SRB_STATUS_SUCCESS;
	srb->bus          = bus;
	srb->target       = target;
	srb->lun          = lun;
	if ((scp->sc_data_direction == DMA_BIDIRECTIONAL) ||
	    (scp->sc_data_direction == DMA_NONE)          ||
	    (scp->sc_data_direction == DMA_FROM_DEVICE)) {
		srb->flags    = RC_SRB_FLAGS_DATA_IN;
	} else if (scp->sc_data_direction == DMA_TO_DEVICE) {
		srb->flags    = RC_SRB_FLAGS_DATA_OUT;
	}
	srb->data_len     = scsi_bufflen(scp);
	srb->cdb          = scp->cmnd;
	srb->cdb_len      = scp->cmd_len;
	srb->sense        = scp->sense_buffer;
	srb->sense_len    = sizeof(scp->sense_buffer);
	srb->scsi_context = scp;
	srb->sg_list      = (rc_sg_list_t *)&srb->private32[0];
	srb->dev_private  = (char *)srb->sg_list + sg_list_size;
#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,26)
	srb->timeout      = scp->timeout_per_command/HZ;
#else
	srb->timeout      = scp->request->timeout/HZ;
#endif
	srb->seq_num      = rc_srb_seq_num++;

	/* the scsi_cmnd pointer points at our srb, at least until the command is
	 * aborted
	 */
	scp->SCp.ptr = (void *)srb;

	rc_msg_build_sg(srb);

	// rc_printk(RC_DEBUG2, "\nrc_msg_send_srb: seq_num: %d B/T/L %d/%d/%d\n",
	//           srb->seq_num, bus, target, lun);
	// rc_dump_scp(scp);

	/*
	 * Add the srb to the tail of the queue, schedule the q tasklet, and return.
	 * Sending srb's through the OSIC can take a long time. We don't want other
	 * CPU's spinning on the OSIC lock and just wasting cycles.  So we queue
	 * the srb's and just have one tasklet entering the OSIC.
	 */

	// STATS

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

	return 0;
}

void
rc_msg_process_srb(rc_srb_t *srb )
{
	rc_softstate_t    *state;
	rc_send_arg_t    args;
	struct scsi_cmnd    *scp;
	int        queued;

	state = &rc_state;

	// rc_printk(RC_DEBUG2, "\nrc_msg_process_srb: seq_num: %d "
	//           "B/T/L %d/%d/%d\n", srb->seq_num, srb->bus, srb->target,
	//           srb->lun);

	scp = (struct scsi_cmnd *) srb->scsi_context;

	if (srb->callback == NULL &&  scp == NULL) {
		/* the command was aborted so forget about it... */

		// STATS
		state->stats.srb_total++;
		atomic_dec(&state->stats.target_pending[srb->target]);
		atomic_dec(&state->stats.scb_pending);

		rc_printk(RC_WARN, "rc_msg_process_srb: seq_num %d Aborted\n",
			  srb->seq_num);
		srb->seq_num = -1;
		kfree(srb);
		return;
	}

	if (scp)
		scp->result = 0xffffffff;

	spin_lock(&state->osic_lock);
	check_lock(state);
	state->osic_locked = 1;
	state->osic_lock_holder = "rc_msg_process_srb";

	// STATS
	atomic_inc(&state->stats.srb_pending);
	state->stats.srb_total++;

	args.call_type = RC_CTS_SEND_SRB;
	args.u.send_srb.srb = srb;
	args.u.send_srb.queued = 0;
	rc_send_msg(&args);
	queued = args.u.send_srb.queued;

	state->osic_locked = 0;
	spin_unlock(&state->osic_lock);

	/*
	 * expected state is queued == 1
	 */

	if (queued) {
		// rc_printk(RC_DEBUG2, "rc_msg_process_srb: seq_num %d Queued\n",
		//           srb->seq_num);
		return;
	}
	/*
	 * be carefull about using srb as it may already have been freed
	 * if the srb wasn't queued, we can still use srb.
	 */

	if (srb->seq_num == -1) {
		rc_printk(RC_WARN, "rc_msg_process_srb: seq_num %d  already freed\n",
			  srb->seq_num);
		return;
	}

	/*
	 * unexpected status PENDING
	 */
	if (srb->status == RC_SRB_STATUS_PENDING) {
		rc_printk(RC_WARN, "rc_msg_process_srb: seq_num %d STATUS_PENDING\n",
			  srb->seq_num);
		rc_dump_scp((struct scsi_cmnd *) srb->scsi_context);
		return;
	}

	if (!queued && (srb->status == RC_SRB_STATUS_DEVICE_OFFLINE)) {
		if (scp && scp->device)
			scsi_device_set_state(scp->device, SDEV_OFFLINE);
		rc_printk(RC_WARN, "%s: Bus/Target/Lun %d/%d/%d OFFLINE\n",
			  __FUNCTION__, srb->bus, srb->target, srb->lun);
		rc_msg_srb_done(srb);
		return;
	}

	/*
	 * just queue up the srb and let the tasklet handle the completion
	 */
	rc_msg_srb_done(srb);
	return;
}

/*
 *
 * First check for any interrupts that need to be processed.
 * Next send any memory operattion responses
 * Then queue to the OSIC any pending SRBs
 */

void
rc_msg_srb_q_tasklet(  unsigned long arg)
{
	unsigned long    irql;
	rc_srb_t    *srb;
	rc_softstate_t *state;
	rc_stats_t    *sp;
	rc_send_arg_t    args;
	int        progress;
	rc_mem_op_t    *mop;
	// STATS
	unsigned int stat_last_pending;
	unsigned int stat_nsrbs;
	unsigned int stat_intr_low, stat_intr_hi;

	state = (rc_softstate_t *)arg;
	sp = &state->stats;

	/*
	 * If our card shares interrupts with another card,
	 * we may get scheduled because of an interrupt on that card.
	 * If the OSIC isn't ready to process interrupts yet,
	 * just defer running the tasklet
	 */
	if (( state->state & PROCESS_INTR) == 0) {
		tasklet_schedule(&state->srb_q.tasklet);
		return;
	}


	/*
	 * Keep checking for more work as long as we did anything
	 */
	do {
		/*
		 * Process any pending interrupts
		 */

		progress = 0;
		stat_last_pending = 0;
		stat_intr_low = 0;
		if (atomic_read(&state->intr_pending) !=0) {
			atomic_set(&state->intr_pending, 0);
			stat_last_pending = atomic_read(&state->intr_pending);
			++progress;
			spin_lock(&state->osic_lock);
			check_lock(state);
			state->osic_locked = 1;
			state->osic_lock_holder = "rc_msg_srb_q_tasklet:intr";
			args.call_type = RC_CTS_PERFORM_DPC;
			rc_send_msg(&args);
			state->osic_locked = 0;
			spin_unlock(&state->osic_lock);

		}


		/*
		 * process all pending MORBs
		 */
		spin_lock_irqsave(&state->mop_done.lock, irql);

		while (state->mop_done.head) {
			/*
			 * take an morb off the list
			 */
			mop = state->mop_done.head;
			state->mop_done.head = mop->next;
			mop->next = (rc_mem_op_t *)0;
			spin_unlock_irqrestore(&state->mop_done.lock, irql);
			spin_lock(&state->osic_lock);
			check_lock(state);
			state->osic_locked = 1;
			state->osic_lock_holder = "rc_msg_srb_q_tasklet: morb";
			args.call_type = RC_CTS_MEMORY_OP_RESP;
			args.u.mem_op_resp.id = mop->id;
			args.u.mem_op_resp.status = 1;
			rc_send_msg(&args);

			state->osic_locked = 0;
			spin_unlock(&state->osic_lock);
			spin_lock_irqsave(&state->mop_done.lock, irql);
			progress++;
		}
		spin_unlock_irqrestore(&state->mop_done.lock, irql);

		/*
		 * process all pending SRBs
		 */
		spin_lock_irqsave(&state->srb_q.lock, irql);

		stat_nsrbs = 0;
		while (state->srb_q.head) {
			/*
			 * take an srb off the list
			 */
			srb = state->srb_q.head;
			state->srb_q.head = srb->next;
			srb->next = (rc_srb_t *)0;
			spin_unlock_irqrestore(&state->srb_q.lock, irql);
			++progress;
			++stat_nsrbs;

			rc_msg_process_srb(srb);
			spin_lock_irqsave(&state->srb_q.lock, irql);
			if (stat_last_pending != atomic_read(&state->intr_pending)) {
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,10,0) // (4,2,0)
				if (!stat_intr_low)
					rdtscl(stat_intr_low);
#endif	/* LINUX_VERSION_CODE < KERNEL_VERSION(3,10,0) // (4,2,0) */
			}
		}
		// STATS
		/*
		 * compute:
		 *     max number of srbs sent to the core at once
		 *    max interrupts that came in while in the core
		 *    total interrupts that came in while in the core
		 *    max time an interrupt waited for the core
		 */
		if (sp->max_srbs_sent < stat_nsrbs)
			sp->max_srbs_sent = stat_nsrbs;
		if (sp->max_intr_waiting < stat_last_pending) {
			sp->max_intr_waiting = stat_last_pending;
		}
		sp->total_intr_waiting += stat_last_pending;
		if (stat_intr_low) {
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,10,0) // (4,2,0)
			rdtscl(stat_intr_hi);
#endif	/* LINUX_VERSION_CODE < KERNEL_VERSION(3,10,0) // (4,2,0) */
			stat_intr_hi -= stat_intr_low;
			if (sp->max_intr_delay < stat_intr_hi)
				sp->max_intr_delay = stat_intr_hi;
		}

        spin_unlock_irqrestore(&state->srb_q.lock, irql);

	} while (progress);

}

void
rc_msg_srb_done_tasklet(  unsigned long arg)
{
	unsigned long irql;
	rc_srb_t *srb;
	rc_softstate_t *state;

	state = (rc_softstate_t *)arg;

	spin_lock_irqsave(&state->srb_done.lock, irql);

	while (state->srb_done.head) {
		/*
		 * take an srb off the list
		 */
		srb = state->srb_done.head;
		state->srb_done.head = srb->next;
		srb->next = (rc_srb_t *)0;

		spin_unlock_irqrestore(&state->srb_done.lock, irql);
		rc_msg_srb_complete(srb);
		spin_lock_irqsave(&state->srb_done.lock, irql);
	}

	spin_unlock_irqrestore(&state->srb_done.lock, irql);

}

void
rc_msg_srb_done(rc_srb_t *srb)
{
	rc_softstate_t *state;
	unsigned long irql;

	state = &rc_state;

	srb->next = (rc_srb_t *)0;

	// rc_printk(RC_DEBUG2, "rc_msg_srb_done: seq_num %d  queued\n",
	//           srb->seq_num);

	/*
	 * OSIC lock is already held entering this function
	 */
	// STATS
	atomic_dec(&state->stats.srb_pending);

	/*
	 * add to tail of srb queue
	 */

	spin_lock_irqsave(&state->srb_done.lock, irql);

	if (state->srb_done.head == (rc_srb_t *)0) {
		state->srb_done.head = srb;
		state->srb_done.tail = srb;
	} else {
		state->srb_done.tail->next = srb;
		state->srb_done.tail = srb;
	}
	spin_unlock_irqrestore(&state->srb_done.lock, irql);

	tasklet_schedule(&state->srb_done.tasklet);
}

void
rc_msg_srb_complete(struct rc_srb_s *srb)
{
	struct scsi_cmnd    *scp;
	rc_softstate_t *state;

	state = &rc_state;

	if ( srb->function == RC_SRB_SHUTDOWN
         || srb->function == RC_SRB_RESTART
	     || srb->function == RC_SRB_FLUSH )
	{
		rc_softstate_t *state;

		state = &rc_state;
		rc_printk(RC_DEBUG,"%s: shutdown complete\n", __FUNCTION__);
		kfree(srb);
		up(&state->init_sema);
		return;
	}

	// STATS
	atomic_dec(&state->stats.target_pending[srb->target]);
	atomic_dec(&state->stats.scb_pending);

	if (srb->callback) {
		srb->callback(srb);
		/* kfree will be done much later */
        return;
	}

	if ((scp = (struct scsi_cmnd *)srb->scsi_context) == NULL) {
		/* the request was aborted, so forget about it... */
		rc_printk(RC_WARN, "%s: seq_num %d  Aborted\n", __FUNCTION__,
			  srb->seq_num);
		srb->seq_num = -1;
		kfree(srb);
		return;
	}

	scp->SCp.ptr = NULL;

	if (srb->status == RC_SRB_STATUS_SUCCESS) {
		 //rc_printk(RC_DEBUG2, "%s: seq_num %d SUCCESS\n", __FUNCTION__,
		 //          srb->seq_num);
		scp->result = DID_OK << 16 | COMMAND_COMPLETE << 8 | GOOD;

		GET_IO_REQUEST_LOCK_IRQSAVE(irql);
		scp->scsi_done(scp);
		PUT_IO_REQUEST_LOCK_IRQRESTORE(irql);
		srb->seq_num = -1;
		kfree(srb);
		return;
	}

	/*
	 * no device
	 */
	if (srb->status == RC_SRB_STATUS_NO_DEVICE              ||
	    srb->status == RC_SRB_STATUS_INVALID_LUN            ||
	    srb->status == RC_SRB_STATUS_INVALID_TARGET_ID      ||
	    srb->status == RC_SRB_STATUS_INVALID_PATH_ID)
	{
		rc_printk(RC_DEBUG2, "%s: seq_num %d STATUS_NO_DEVICE\n", __FUNCTION__,
			  srb->seq_num);
		scp->result = DID_BAD_TARGET << 16;
		GET_IO_REQUEST_LOCK_IRQSAVE(irql);
		scp->scsi_done(scp);
		PUT_IO_REQUEST_LOCK_IRQRESTORE(irql);
		srb->seq_num = -1;
		kfree(srb);
		return;
	}

	/*
	 * Something went wrong.  May need to check specific error codes
	 */
	rc_printk(RC_DEBUG2, "%s: seq_num %d ERROR 0x%x\n", __FUNCTION__,
		  srb->seq_num, srb->status);
	/*
	 * dump the command that failed
	 * Note: Will only see if debug level >= 8.
	 */
	rc_dump_scp(scp);

	scp->result = DID_OK << 16 | COMMAND_COMPLETE << 8 | CHECK_CONDITION;

	if (! (srb->flags & RC_SRB_FLAGS_SENSEVALID)) {
		if (srb->status == RC_SRB_STATUS_INVALID_REQUEST) {
			// build up generic illegal request failure sense data
			rc_set_sense_data((char *)scp->sense_buffer, SENKEY_ILLEGAL,
					  SENCODE_INVALID_MESSAGE_ERROR,
					  ASENCODE_INVALID_MESSAGE_ERROR, 0, 0, 0, 0);
		} else {
			// build up generic target failure sense data
			rc_set_sense_data((char *)scp->sense_buffer, SENKEY_HW_ERR,
					  SENCODE_INTERNAL_TARGET_FAILURE,
					  ASENCODE_INTERNAL_TARGET_FAILURE, 0, 0, 0, 0);
		}


	}

	GET_IO_REQUEST_LOCK_IRQSAVE(irql);
	scp->scsi_done (scp);
	PUT_IO_REQUEST_LOCK_IRQRESTORE(irql);
	srb->seq_num = -1;
	kfree(srb);

}

/*
 * send a flush cache or shutdown srb to the OSIC.
 * Wait for IO to stop
 */

void
rc_msg_send_srb_function (rc_softstate_t *state, int function)
{
	rc_srb_t      *srb;
	rc_send_arg_t  args;
	int            size;
	int            queued;
	int            sg_list_size;

	rc_printk(RC_DEBUG, "rc_msg_send_srb_function\n");

	sg_list_size = sizeof(rc_sg_list_t) +
		(RC_SG_MAX_ELEMENTS-1) * sizeof(rc_sg_elem_t);
	size = sizeof(rc_srb_t) + sg_list_size + state->memsize_per_srb;

	/*
	 * Use GFP_ATOMIC instead of GP_KERNEL because we get called from the srb
	 * tasklet
	 */
	if ((srb = kmalloc( size, GFP_ATOMIC)) == 0) {
		rc_printk(RC_WARN, "rc_msg_send_srb_function: could not alloc %d "
			  "bytes for srb\n", size);
		return;
	}
	memset(srb, 0, size);

	srb->function     = function;
	srb->status       = RC_SRB_STATUS_SUCCESS;
	srb->flags        = RC_SRB_FLAGS_UNSPECIFIED_DIRECTION;
	srb->sg_list      = (rc_sg_list_t *)&srb->private32[0];
	srb->dev_private  = (char *)srb->sg_list + sg_list_size;
	srb->seq_num       = rc_srb_seq_num;
	rc_srb_seq_num++;

	/*
	 * on a uniprocessor kernel, the spin_locks disappear and we have no
	 * lock to prevent reentering the OSIC.
	 * we can't allow rc_msg_timer to execute while we're in the middle of
	 * this call to the OSIC, as it also enters the OSIC.
	 * So, disable the bottom-half handlers, which will prevent the msg_timer
	 * from running.  After the bh handlers are re-enabled, they'll get
	 * executed later
	 */
	local_bh_disable();
	spin_lock(&state->osic_lock);
	check_lock(state);
	state->osic_locked = 1;
	state->osic_lock_holder = "rc_msg_send_srb_function";
	args.call_type = RC_CTS_SEND_SRB;
	args.u.send_srb.srb = srb;
	args.u.send_srb.queued = 0;
	rc_send_msg(&args);
	queued = args.u.send_srb.queued;

	state->osic_locked = 0;
	spin_unlock(&state->osic_lock);
	local_bh_enable();

	/*
	 * expected state is queued == 1
	 */

	if (!queued) {
		rc_printk(RC_WARN, "rc_msg_send_srb_function: seq_num %d  NOT Queued\n",
			  srb->seq_num);
		kfree(srb);
		return;
	}

	/*
	 * wait for the srb_function to complete
	 */
	rc_printk(RC_DEBUG, "rc_msg_send_srb_function: waiting for completion\n");
	down(&state->init_sema);
	rc_printk(RC_DEBUG, "rc_msg_send_srb_function: complete\n");
	return;
}



/*
 * build a scatter/gather list of Virtual addresses to send to OSIC.
 * We only do this when all sg elements are already mapped into the kernel.
 * We've already checked that this is the case.
 */

void __inline__
rc_msg_build_sg_virt( rc_srb_t *srb)
{

	int        i;
	struct scatterlist *sg;
	struct scsi_cmnd    *scp;
	rc_sg_list_t    *rc_sg;

	scp = (struct scsi_cmnd *)srb->scsi_context;
	RC_ASSERT(scp != NULL);

	rc_sg = srb->sg_list;
	rc_sg->sg_num_elem = scsi_sg_count(scp);
	rc_sg->sg_mem_type = RC_MEM_VADDR;

	scsi_for_each_sg(scp, sg, scsi_sg_count(scp), i) {
		rc_sg->sg_elem[i].size = sg->length;
		rc_sg->sg_elem[i].v_addr = page_address(sg_page(sg)) + sg->offset;
	}
}

/*
 * build a scatter/gater list of physical address to send to the OSIC.
 */
void __inline__
rc_msg_build_sg_phys( rc_srb_t *srb)
{

	int        i;
	struct scatterlist *sg;
	dma_addr_t    dma_addr;
	struct scsi_cmnd    *scp;
	rc_sg_list_t    *rc_sg;

	scp = (struct scsi_cmnd *)srb->scsi_context;
	RC_ASSERT(scp != NULL);

	rc_sg = srb->sg_list;
	rc_sg->sg_num_elem = scsi_sg_count(scp);
	rc_sg->sg_mem_type = RC_MEM_PADDR | RC_MEM_USE_EXTERNAL_LIST_COPY;

	scsi_for_each_sg(scp, sg, scsi_sg_count(scp), i) {
		dma_addr = sg_phys(sg);
		rc_sg->sg_elem[i].dma_paddr = dma_addr;
		rc_sg->sg_elem[i].size =  sg->length;
	}
}


void
rc_msg_build_sg( rc_srb_t *srb)
{
	struct scsi_cmnd    *scp;
	rc_sg_list_t    *rc_sg;
	int        has_highmem;

#if defined(CONFIG_HIGHMEM)
	int             i;
	struct scatterlist *sg;
#endif

	scp = (struct scsi_cmnd *)srb->scsi_context;
	RC_ASSERT(scp != NULL);

	rc_sg = srb->sg_list;

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,24)
	if (scp->use_sg == 0) {       /* has to be a virtual address */
		rc_sg->sg_num_elem = 1;
		rc_sg->sg_elem[0].size   = scp->request_bufflen;
		rc_sg->sg_elem[0].v_addr = scp->request_buffer;
		rc_sg->sg_mem_type = RC_MEM_VADDR;

		// RC_PRINTK(RC_DEBUG2, "rc_msg_build_sg: scp: 0x%p use_sg 0, buf "
		//           "0x%8x:%8x, len %d\n", scp,
		//           (rc_uint32_t)(rc_sg->sg_elem[0].dma_paddr >> 32),
		//           (rc_uint32_t)(rc_sg->sg_elem[0].dma_paddr & 0xffffffff),
		//           rc_sg->sg_elem[0].size);
		return;
	}
#endif

	has_highmem = 0;

	if (ForcePhysAddr && scp->device->id != 16) {
		rc_msg_build_sg_phys(srb);
		return;
	}

	/*
	 * Run through all the elements and see if they are all in low memory
	 */
#ifdef CONFIG_HIGHMEM
	sg = scsi_sglist(scp);
	scsi_for_each_sg(scp, sg, scsi_sg_count(scp), i) {
		if (PageHighMem(sg_page(sg))) {
			has_highmem = 1;
			break;
		}
	}
#endif
	if (has_highmem)
		rc_msg_build_sg_phys(srb);
	else
		rc_msg_build_sg_virt(srb);

	return;
}

void
rc_msg_map_phys_to_virt(struct map_memory_s *map)
{
    rc_adapter_t    *adapter = (rc_adapter_t *) map->dev_handle;

    map->address = 0;
    if (adapter->hardware.irq == 0) {
        rc_printk(RC_ERROR, "%s: WARNING adapter shutdown\n", __FUNCTION__);
    } else if ((map->memory_id & MEM_TYPE) == RC_MEM_PADDR) {
        map->address = (rc_uint64_t) (uintptr_t) phys_to_virt(map->physical_address);
    }
}


void
rc_add_dmaMemoryList(void *cpu_addr, dma_addr_t* dmaHandle, rc_uint32_t bytes,
			rc_adapter_t *adapter)
{
    struct DmaMemoryNode *newNode;

    //newNode = vmalloc(sizeof(struct DmaMemoryNode));
    newNode = kmalloc(sizeof(struct DmaMemoryNode), GFP_KERNEL);

    if (newNode)
    {
        newNode->cpu_addr = cpu_addr;
        newNode->dmaHandle = *dmaHandle;
        newNode->bytes = bytes;
        newNode->nextNode = NULL;

        if(adapter->dmaMemoryListHead) {
            adapter->dmaMemoryListTail->nextNode = newNode;
        }
        else {
            adapter->dmaMemoryListHead = newNode;
        }
        adapter->dmaMemoryListTail = newNode;
    }

}


void
rc_msg_get_dma_memory(alloc_dma_address_t *dma_address)
{
    dma_addr_t* 	    dmaHandle;
    rc_adapter_t	    *adapter;

    adapter = (rc_adapter_t *) dma_address->dev_handle;
    dmaHandle = (dma_addr_t*) &dma_address->dmaHandle;


    dma_address->cpu_addr = pci_alloc_consistent(adapter->pdev,dma_address->bytes, dmaHandle );

    if (dma_address->cpu_addr)
    {
    	rc_add_dmaMemoryList(dma_address->cpu_addr, dmaHandle, dma_address->bytes, adapter);
    }
}

void
rc_msg_free_dma_memory(rc_adapter_t	*adapter, void *cpu_addr, dma_addr_t dmaHandle, rc_uint32_t bytes)
{
    pci_free_consistent(adapter->pdev, bytes, cpu_addr, dmaHandle);
}

void
rc_msg_free_all_dma_memory(rc_adapter_t	*adapter)
{

    struct DmaMemoryNode *dmaNode;

    dmaNode = adapter->dmaMemoryListHead;

    while (dmaNode) {
        rc_msg_free_dma_memory(
		adapter,
		dmaNode->cpu_addr,
		dmaNode->dmaHandle,
		dmaNode->bytes
		);

        adapter->dmaMemoryListHead = adapter->dmaMemoryListHead->nextNode;
        kfree(dmaNode);
        dmaNode = adapter->dmaMemoryListHead;
    }

    if (adapter->dmaMemoryListHead == NULL)
    {
	    //adapter->dmaMemoryListTail = NULL;
    }
}

void
rc_msg_map_mem(struct map_memory_s *map)
{
	rc_adapter_t	*adapter;
	void		*vaddr;
	int		len;
	void		*private_start;        // Adapter private memory
	void		*private_end;
	char		*type = "?";
	size_t		len_mapped;
	unsigned long	offset;
	struct page	*page;

	adapter = (rc_adapter_t *) map->dev_handle;

	if (adapter->hardware.irq == 0) {
		rc_printk(RC_ERROR, "%s: WARNING adapter shutdown\n", __FUNCTION__);
		return;
	}
	/* Physical memory type, needs no translation. */

	if ((map->memory_id & MEM_TYPE) == RC_MEM_PADDR) {
		map->physical_address = map->address;
	} else if ((map->memory_id & MEM_TYPE) == RC_MEM_VADDR) {
		vaddr = (void *)(rc_uint_ptr_t)map->address;
		len   = (size_t)map->number_bytes;

		private_start = adapter->private_mem.vaddr;
		private_end  =  private_start + adapter->private_mem.size;

		/*
		 * check to see if the request is in the device private memory region.
		 */
		if ((vaddr >= private_start) && (vaddr < private_end) ) {
			if (vaddr + len >=  private_end) {
				rc_printk(RC_WARN, "rc_msg_map_mem: invalid address range: %p len %d\n", vaddr, len);
				map->number_bytes = 0;
				goto out;
			}

			offset = vaddr - private_start;
			map->physical_address = (rc_uint64_t) adapter->private_mem.dma_address + offset;
			map->number_bytes = (rc_uint64_t)len;
			type = "p";
		} else {
			if (((unsigned long)vaddr >= VMALLOC_START) &&
			    ((unsigned long)vaddr < VMALLOC_END)) {
				page = vmalloc_to_page(vaddr);
				type = "v";
			} else {
				page = virt_to_page(vaddr);
				type = "k";
			}

			offset = (unsigned long)vaddr & (PAGE_SIZE-1);
			len_mapped = PAGE_SIZE - offset;
			if (len < len_mapped)
				len_mapped = len;

			map->physical_address = dma_map_page(&adapter->pdev->dev, page, offset, len_mapped, PCI_DMA_BIDIRECTIONAL);
            if (dma_mapping_error(&adapter->pdev->dev, map->physical_address))
            {
                map->number_bytes = 0;
            } else {
			    map->number_bytes = (rc_uint64_t)len_mapped;
            }
		}
    } else if ((map->memory_id & MEM_TYPE) == RC_MEM_DMA) {
        vaddr = (void *)(rc_uint_ptr_t)map->address;

        map->physical_address = dma_map_single(&adapter->pdev->dev, vaddr, map->number_bytes, PCI_DMA_BIDIRECTIONAL);
        if (dma_mapping_error(&adapter->pdev->dev, map->physical_address))
        {
            map->number_bytes = 0;
        }
	} else  {
		/* No known memory type is coding error */
		rc_printk(RC_ERROR, "rc_msg_map_mem: invalid memory type %x\n", map->memory_id);
		BUG();
	}

out:
/*
	RC_PRINTK(RC_DEBUG3, "rc_msg_map_mem: addr[%s] 0x%x:%x paddr 0x%x:%x "
		  "number_bytes %d\n", type,
		  (rc_uint32_t)(map->address >> 32),
		  (rc_uint32_t)(map->address & 0xffffffff),
		  (rc_uint32_t)(map->physical_address >> 32),
		  (rc_uint32_t)(map->physical_address & 0xffffffff),
		  (rc_uint32_t)map->number_bytes );
*/
  	return;

}


void
rc_msg_unmap_mem(struct unmap_memory_s *unmap)
{

	rc_adapter_t     *adapter;
	dma_addr_t    dma_addr;
	int        len;
	dma_addr_t    private_start;        // Adapter private memory
	dma_addr_t    private_end;
	rc_uint64_t    paddr;
	int seq_num;

	adapter = (rc_adapter_t *) unmap->dev_handle;
	paddr = unmap->physical_address;
	len   = (rc_uint32_t)unmap->number_bytes;

	if (unmap->srb)
		seq_num = unmap->srb->seq_num;
	else
		seq_num = -1;

	dma_addr = (dma_addr_t) paddr;
	private_start = adapter->private_mem.dma_address;
	private_end  =  private_start + adapter->private_mem.size;

	RC_PRINTK(RC_DEBUG3, "rc_msg_unmap_mem: paddr 0x%x:%x number_bytes "
		  "%d\n", (rc_uint32_t)(unmap->physical_address >> 32),
		  (rc_uint32_t)(unmap->physical_address & 0xffffffff),
		  (rc_uint32_t)unmap->number_bytes);

	/*
	 * check to see if the request is in the device private memory region.
	 */
	if ((dma_addr >= private_start) && (dma_addr < private_end) ) {
		if (dma_addr + len >=  private_end) {
			rc_printk(RC_WARN, "rc_msg_unmap_mem: invalid address range: "
				  "0x%llx len %d\n", (rc_uint64_t)dma_addr, len);
			unmap->number_bytes = 0;
			return;
		}
		unmap->number_bytes  = (rc_uint64_t)len;
		return;
	}

	dma_unmap_page(&adapter->pdev->dev, dma_addr, len, DMA_BIDIRECTIONAL);
	unmap->number_bytes = (rc_uint64_t)len;
	return;


}


void
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,15,0)
rc_msg_timeout_done(unsigned long data)
#else
rc_msg_timeout_done(struct timer_list * t)
#endif
{
	rc_softstate_t *state;

#if LINUX_VERSION_CODE < KERNEL_VERSION(4,15,0)
	state = (rc_softstate_t *)data;
	init_timer(&state->msg_timeout);
#else
	state = from_timer(state, t, msg_timeout);
	timer_setup(&state->msg_timeout, rc_msg_timeout_done, 0);
#endif
	up(&state->msg_timeout_sema);
}

void
rc_msg_timeout( int to)
{
	rc_softstate_t *state;

	state = &rc_state;
	/*
	 * set up timeout
	 */

#if LINUX_VERSION_CODE < KERNEL_VERSION(4,15,0)
	init_timer(&state->msg_timeout);
	state->msg_timeout.expires = jiffies  + to;
	state->msg_timeout.data = (unsigned long)state;
	state->msg_timeout.function = rc_msg_timeout_done;
#else
	timer_setup(&state->msg_timeout, rc_msg_timeout_done, 0);
	state->msg_timeout.expires = jiffies  + to;
#endif
	add_timer(&state->msg_timeout);
	down(&state->msg_timeout_sema);

}

void
rc_msg_access_ok(rc_access_ok_t accessOk)
{

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 0, 0)
    accessOk.returnStatus = access_ok( accessOk.access_location, accessOk.access_size);
#else
    accessOk.returnStatus = access_ok( VERIFY_WRITE , accessOk.access_location, accessOk.access_size);
#endif /* LINUX_VERSION_CODE */
	
}


void
rc_set_sense_data (char    *sense,
		   uint8_t    sense_key,
		   uint8_t    sense_code,
		   uint8_t    add_sense_code,
		   uint8_t    incorrect_len,
		   uint8_t    bit_ptr,
		   uint32_t    field_ptr,
		   uint32_t    residue)
{
	sense[0] = 0xF0;    // Sense data Valid, err code 70h (current error)
	sense[1] = 0;        // Segment number, always zero

	if (incorrect_len) {
		sense[2] = sense_key | 0x20;    // Set the ILI bit | sense key
		sense[3] = BYTE3 (residue);
		sense[4] = BYTE2 (residue);
		sense[5] = BYTE1 (residue);
		sense[6] = BYTE0 (residue);
	} else
		sense[2] = sense_key;    // Sense key

	if (sense_key == SENKEY_ILLEGAL)
		sense[7] = 10;        // Additional sense length
	else
		sense[7] = 6;        // Additional sense length

	sense[12] = sense_code;        // Additional sense code
	sense[13] = add_sense_code;    // Additional sense code qualifier
	if (sense_key == SENKEY_ILLEGAL) {
		sense[15] = 0;

		if (sense_code == SENCODE_INVALID_PARAM_FIELD)
			sense[15] = 0x80;    // Std sense key specific field
		// Illegal parameter is in the parameter block

		if (sense_code == SENCODE_INVALID_CDB_FIELD)
			sense[15] = 0xc0;    // Std sense key specific field
		// Illegal parameter is in the CDB block
		sense[15] |= bit_ptr;
		sense[16] = field_ptr >> 8;    // MSB
		sense[17] = field_ptr;        // LSB
	}
}


// STATS
int
rc_msg_stats(char *buf, int buf_size)
{
	int cnt, len, i;
	rc_softstate_t *state;
	rc_stats_t *sp;
	char *cp;

	state = &rc_state;
	sp = &state->stats;

	cnt = 0;
	cp = buf;

	len = snprintf(cp, buf_size,
		       "srb total  %d  pending %d scb total %d pending %d\n",
		       sp->srb_total, atomic_read(&sp->srb_pending),
		       sp->scb_total, atomic_read(&sp->scb_pending));

	cp  += len;
	cnt += len;

	len = snprintf(cp, buf_size - cnt, "target  ");
	cp  += len;
	cnt += len;

	for (i = 0; i < MAX_ARRAY; i++) {
		len = snprintf(cp, buf_size - cnt, "%5d ", i);
		cp += len;
		cnt += len;
	}

	len = snprintf(cp, buf_size - cnt, "\nTotal  ");
	cp  += len;
	cnt += len;

	for (i = 0; i < MAX_ARRAY; i++) {
		len = snprintf(cp, buf_size - cnt,  "%5d ", sp->target_total[i]);
		cp += len;
		cnt += len;
	}


	len = snprintf(cp, buf_size - cnt, "\nPending  ");
	cp  += len;
	cnt += len;

	for (i = 0; i < MAX_ARRAY; i++) {
		len = snprintf(cp, buf_size - cnt, "%5d ",
			       atomic_read(&sp->target_pending[i]));
		cp += len;
		cnt += len;
	}

	len = snprintf(cp, buf_size - cnt,
		       "\nOSIC lock %d holder %s intr_pending %d\n",
		       state->osic_locked, state->osic_lock_holder,
		       atomic_read(&state->intr_pending));
	cp  += len;
	cnt += len;

	len = snprintf(cp, buf_size - cnt, "max SRBs sent %u max intr waiting %u "
		       "total intr waiting %u max intr delay %u\n",
		       sp->max_srbs_sent, sp->max_intr_waiting,
		       sp->total_intr_waiting, sp->max_intr_delay);
	sp->max_srbs_sent = sp->max_intr_waiting = sp->total_intr_waiting =
		sp->max_intr_delay = 0;
	cp  += len;
	cnt += len;
	*cp = '\0';

	return(cnt);
}

static void rc_sysrq_intr (int key
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
			   ,struct pt_regs *pt_regs
#endif
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,36)
			   ,struct tty_struct * tty
#endif
                           )
{
	rc_softstate_t *state;

	state = &rc_state;
	/* this really could become an atomic counter now */
	atomic_inc(&state->intr_pending);

	/* only bump msg level if debug was set when module loaded */
	if (rc_msg_level >= RC_INFO2)
		rc_msg_level += 1;

	tasklet_schedule(&state->srb_q.tasklet);
	rc_wakeup_all_threads();
	rc_printk(RC_ALERT, "scheduling tasklet interrupt\n");
}

static void rc_sysrq_state (int key
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
			    ,struct pt_regs *pt_regs
#endif
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,36)
			   ,struct tty_struct * tty
#endif
			   )
{

	rc_msg_stats(rc_stats_buf, sizeof(rc_stats_buf));
	rc_printk(RC_ALERT, rc_stats_buf);
	rc_mop_stats(rc_stats_buf, sizeof(rc_stats_buf));
	rc_printk(RC_ALERT, rc_stats_buf);
}

size_t
Min(size_t a, size_t b)
{
    return (a < b) ? a : b;
}

acpi_status
rc_acpi_evaluate_object(acpi_handle handle, char *method, void *ret, int *size)
{
    acpi_status         ac_stat;
    struct acpi_buffer  buffer;

    if (!handle || method == NULL || ret == NULL)
        return AE_BAD_PARAMETER;

    memset(&buffer, 0, sizeof(buffer));
    buffer.length = ACPI_ALLOCATE_BUFFER;

    ac_stat = acpi_evaluate_object(handle, method, NULL, &buffer);

    if (ACPI_SUCCESS(ac_stat))
    {
        union acpi_object   *acpi_obj;

        acpi_obj = buffer.pointer;

        if (acpi_obj)
        {
            int                 sz;

            sz = *size;

            switch (acpi_obj->type)
            {
            case ACPI_TYPE_INTEGER:
                sz = Min(sz, sizeof(acpi_obj->integer.value));
                memcpy(ret, &acpi_obj->integer.value, sz);
                break;
            case ACPI_TYPE_BUFFER:
                sz = Min(sz, acpi_obj->buffer.length);
                memcpy(ret, acpi_obj->buffer.pointer, sz);
                break;
            case ACPI_TYPE_PACKAGE:
                sz = Min(sz, buffer.length);
                memcpy(ret, acpi_obj, sz);
                break;
            default:
                rc_printk(RC_WARN, "### %s(): ACPI method \"%s\" returned object type = %d\n",
                        __FUNCTION__, method, acpi_obj->type);
                break;
            }

            *size = sz;
        }

        if (buffer.pointer)
            kfree(buffer.pointer);
    }

    return ac_stat;
}
