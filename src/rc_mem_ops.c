/****************************************************************************
 *
 * Copyright © 2006-2008 Ciprico Inc. All rights reserved.
 * Copyright © 2008-2015 Dot Hill Systems Corp. All rights reserved.
 * Copyright © 2015-2016 Seagate Technology LLC. All rights reserved.
 * Copyright © 2019, Advanced Micro Devices, Inc. All rights reserved.
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

#include "linux/vmalloc.h"
#include "linux/wait.h"
#include "linux/sched.h"
#include "linux/string.h"
#include "linux/kthread.h"
#include "asm/page.h"        /* need pfn_to_page() and other things */
#ifdef CONFIG_SPARSEMEM
#include "linux/mmzone.h"    /* pfn_to_page() could be here instead */
#endif
#ifdef CONFIG_DISCONTIGMEM
#include "asm/mmzone.h"        /* ...or even here */
#endif
#if 0
#include "asm/i387.h"
#endif  /* 0 */

#ifdef CONFIG_HIGHMEM
#include "asm/highmem.h"
#endif

#include "linux/signal.h"

#include "rc.h"

rc_thread_t rc_thread[NR_CPUS];

int rc_kthread(void *arg);

void rc_start_all_threads(void);
void rc_stop_all_threads(void);
void rc_wakeup_all_threads(void);

int rc_start_thread(int cpu);
void rc_stop_thread(int cpu);
void rc_wakeup_thread(int cpu);

int rc_mem_copy_list ( rc_sg_list_t *dst, rc_sg_list_t *src, int byte_count);
int rc_mem_clear_list(rc_sg_list_t *dst, int byte_count);
int rc_kthread_mem_copy(rc_thread_t *tp, rc_mem_op_t *mop);
int rc_kthread_mem_user_copy(rc_thread_t *tp, rc_mem_op_t *mop);
int rc_sync_mem_copy(rc_uint64_t dst, rc_uint32_t dst_id,
		     rc_uint64_t src, rc_uint32_t src_id,
		     rc_uint32_t byte_count);
int rc_sync_mem_clear(rc_uint64_t dst, rc_uint32_t dst_id,
		      rc_uint32_t byte_count);

/* 2.6 added a new macro defined here for older kernel versions. */
#ifndef for_each_online_cpu
#define for_each_online_cpu(cpu)      for (cpu = 0; cpu < smp_num_cpus; cpu++)
#endif

/* 2.6 removed cpu logical maps */
#ifndef cpu_logical_map
#define cpu_logical_map(cpu)        cpu
#endif

/*
 * 2.6.8 changed cpu mask from an unsigned long to a strucure, macros for
 * backwards compatibility
 */
#ifndef cpu_set
#define cpu_set(cpu, dst)       dst |= 1UL << cpu
#endif

#ifndef cpus_clear
#define cpus_clear(dst)         dst = 0UL
#endif

void
rc_start_all_threads(void)
{
	int cpu;

	for_each_online_cpu(cpu)
		rc_start_thread(cpu);
}

void
rc_stop_all_threads(void)
{
	int cpu;

	for_each_online_cpu(cpu)
		rc_stop_thread(cpu);
}

void
rc_wakeup_all_threads(void)
{
	int cpu;

	for_each_online_cpu(cpu)
		rc_wakeup_thread(cpu);
}

int
rc_start_thread(int cpu)
{
	rc_thread_t *tp;

	tp = &rc_thread[cpu];

	memset(tp,0, sizeof(rc_thread_t));

    sema_init(&tp->stop_sema, 0);

	tp->thread = kthread_create(rc_kthread, (void *) tp, "rc_kthread/%d", cpu);
	if (IS_ERR(tp->thread)) {
		printk(KERN_ERR "rcraid: Unable to create thread for cpu %d.\n", cpu);
		return 0;
	}
	/* lock this thread to the given cpu to allow sg list processing */
	kthread_bind(tp->thread, cpu);
	if (!wake_up_process(tp->thread)) {
		printk(KERN_ERR "rcraid: Unable to wake up thread for cpu %d.\n", cpu);
		return 0;
	}
	return 1;
}

void
rc_stop_thread(int cpu)
{
	rc_thread_t *tp;

	tp = &rc_thread[cpu];
	if (tp->thread) {
        mb();
		kthread_stop(tp->thread);
        down(&tp->stop_sema);
	}
}

void
rc_wakeup_thread(int cpu)
{
	rc_thread_t *tp;

	tp = &rc_thread[cpu];
	wake_up_process(tp->thread);
}

int
rc_kthread(void *rc_threadp)
{

	int        cpu, status, i;
	rc_thread_t    *tp;
	rc_mem_op_t     *mop;
	rc_softstate_t    *state;
	unsigned long    irql;

	state = &rc_state;

	tp = (rc_thread_t*)rc_threadp;

	cpu = tp - rc_thread;

	set_user_nice(current, -10);
	sigfillset(&current->blocked);

	/* make sure we're on our cpu */
	if (smp_processor_id() != cpu)
		BUG();

	rc_printk(RC_DEBUG, "rc_kthread: cpu %d thread started\n", cpu);

	do {
		preempt_disable();
		local_irq_save(irql);
		while (tp->mop_head) {
			mop = tp->mop_head;
			tp->mop_head = mop->next;
			mop->next = (rc_mem_op_t *)0;

			tp->num_mop--;

			local_irq_restore(irql);
			preempt_enable();

			switch (mop->opcode) {

			case RC_OP_MEM_LIST_COPY:
				status = rc_kthread_mem_copy(tp, mop);
				break;

			case RC_OP_MEM_LIST_XOR:
			case RC_OP_MEM_LIST_CMP:
			case RC_OP_MEM_CLEAR:
			case RC_OP_MEM_COPY:
				rc_printk(RC_ERROR, "rc_msg_mem_op: Unsupported memory "
					  "operation- %d\n", mop->opcode);
				status = 0;
				break;
            case RC_OP_MEM_USER_COPY:
                status = rc_kthread_mem_user_copy(tp, mop);
                break;
			default:
				rc_printk(RC_ERROR, "rc_msg_mem_op: Invalid memory "
					  "operation- %d\n", mop->opcode);
				status = 0;

			}

			mop->status = status;
			/*
			 * Send a successfull response to the OSIC
			 * We just internally queue the response and
			 * let the tasklet send the response. Otherwise,
			 * we'll block on the OSIC lock.
			 */
			spin_lock_irqsave(&state->mop_done.lock, irql);

			if (state->mop_done.head == (rc_mem_op_t *)0) {
				state->mop_done.head = mop;
				state->mop_done.tail = mop;
			} else {
				state->mop_done.tail->next = mop;
				state->mop_done.tail = mop;
			}
			spin_unlock_irqrestore(&state->mop_done.lock, irql);

			tasklet_schedule(&state->srb_q.tasklet);

			preempt_disable();
			local_irq_save(irql);

		}

		local_irq_restore(irql);
		preempt_enable();

		RC_PRINTK(RC_DEBUG3,"rc_kthread: cpu %d thread sleeping\n", cpu);
		tp->running = 0;

		set_current_state(TASK_INTERRUPTIBLE);
		if (!tp->mop_head)
			schedule_timeout(MAX_SCHEDULE_TIMEOUT);


		tp->running = 1;
		RC_PRINTK(RC_DEBUG3,"rc_kthread: cpu %d thread awake\n", cpu);

	} while (!kthread_should_stop());

	for (i = 0; i < RC_THREAD_BUF_CNT; i++) {
		if (tp->buf[i].size) {
			kfree(tp->buf[i].sg);
		}
	}

	rc_printk(RC_INFO,"rc_kthread: cpu %d thread stopped\n", cpu);

    mb();

    up(&tp->stop_sema);

	return 1;
}


static __inline__ int
rc_sg_list_size( rc_sg_list_t *src)
{
	int i, size;

	size = 0;

	if (!src) {
		rc_printk(RC_ERROR,"rc_sg_list_size: src == 0\n");
		return 0;
	}

	for (i = 0; i < src->sg_num_elem; i++)
		size += src->sg_elem[i].size;
	return size;
}


#undef min
#define min(a,b) ((a) < (b) ? (a) : (b))


/*
 * clear memory described by dst list.
 * if the map function fails, the whole clear fails
 */
int
rc_mem_clear_list(rc_sg_list_t *dst, int byte_count)
{
	int    dst_idx;
	int    dst_cnt;
	int    dst_mapped;    /* amount of sg list element currently mapped */
	int    dst_offset;    /* offset into current element */
	dma_addr_t    dst_dma_addr;
	unsigned char    *dst_vaddr;
	struct page     *dst_page;
	int    xfer_cnt;
	int    residual;
	int    offset, size, pfn;

	dst_cnt = rc_sg_list_size(dst);

	if (byte_count > dst_cnt) {
		rc_printk(RC_ERROR, "rc_mem_clear_list, count %d dst_count %d\n",
			  byte_count, dst_cnt);
		BUG();
	}

	dst_idx       = 0;
	dst_mapped    = 0;
	dst_offset    = 0;
	dst_dma_addr  = 0;
	dst_vaddr     = (unsigned char *)0;
	dst_page      = (struct page *)0;

	residual = byte_count;


	while (residual > 0) {

		if (dst_mapped == 0) {
			if ((dst->sg_mem_type & MEM_TYPE) == RC_MEM_PADDR) {
				dst_dma_addr = dst->sg_elem[dst_idx].dma_paddr + dst_offset;
				offset = dst_dma_addr & (PAGE_SIZE-1);

				size = dst->sg_elem[dst_idx].size - dst_offset;
				size = min(size, PAGE_SIZE - offset);

				pfn = dst_dma_addr >> PAGE_SHIFT;
				dst_page = pfn_to_page(pfn);
                dst_vaddr = kmap_atomic(dst_page);
				if (dst_vaddr == 0)
					return 0;

				dst_vaddr += offset;
				dst_mapped = size;
				dst_offset = dst_offset + size;
				RC_PRINTK(RC_DEBUG3, "dst[%d]: dma_addr 0x%llx size 0x%x "
					  "map_addr 0x%x vaddr %px offset 0x%x mapped 0x%x\n",
					  dst_idx, (rc_uint64_t)dst_dma_addr, size,
					  pfn << PAGE_SHIFT, dst_vaddr, dst_offset, dst_mapped);
			}
			else if ((dst->sg_mem_type & MEM_TYPE) == RC_MEM_VADDR) {
				dst_vaddr = dst->sg_elem[dst_idx].v_addr;
				dst_mapped = dst->sg_elem[dst_idx].size;
				rc_printk(RC_DEBUG3, "dst[%d]: vaddr %px size: %x\n", dst_idx,
					  dst_vaddr, dst_mapped);
			}
			else
				BUG();
		}

		xfer_cnt = min(residual, dst_mapped);

		RC_PRINTK(RC_DEBUG3, "rc_mem_clear_list: clearing %x bytes at %px\n",
			  xfer_cnt, dst_vaddr);

		memset(dst_vaddr, 0, xfer_cnt);

		dst_mapped -= xfer_cnt;
		residual   -= xfer_cnt;

		if (dst_mapped == 0) {
			if ((dst->sg_mem_type & MEM_TYPE) == RC_MEM_PADDR) {
				if (dst_page)
                    kunmap_atomic(dst_vaddr);
				dst_page = (struct page *)0;
				if (dst_offset == dst->sg_elem[dst_idx].size) {
					dst_idx++;
					dst_offset = 0;
				}
			} else  if ((dst->sg_mem_type & MEM_TYPE) == RC_MEM_VADDR) {
				dst_idx++;
			} else
				BUG();
		} else
			dst_vaddr += xfer_cnt;

	}

	if (dst_page)
        kunmap_atomic(dst_vaddr);
	if (residual)
		rc_printk(RC_PANIC, "rc_mem_clear_list: residual is not 0 at end "
			  "of loop\n");

	return 1;
}


/*
 * copy memory described by src list to dst list.
 * if the map function fails, the whole copy fails
 * and it will have to be rescheduled
 */
int
rc_mem_copy_list ( rc_sg_list_t *dst, rc_sg_list_t *src, int byte_count)
{

	int    src_idx,    dst_idx;
	int    src_cnt,    dst_cnt;
	/* amount of sg list element currently mapped */
	int    src_mapped, dst_mapped;
	/* offset into current element */
	int    src_offset, dst_offset;
	dma_addr_t    src_dma_addr,  dst_dma_addr;
	unsigned char    *src_vaddr,  *dst_vaddr;
	struct page     *src_page, *dst_page;
	int    xfer_cnt;
	int    residual;
	int    offset, size, pfn;
	int	ret = 1;
	void	*vret;

	src_cnt = rc_sg_list_size(src);
	dst_cnt = rc_sg_list_size(dst);

	if (byte_count > src_cnt) {
		rc_printk(RC_ERROR, "rc_mem_copy_list, count %d src_count %d\n",
			  byte_count, src_cnt);
		BUG();
	}

	if (byte_count > dst_cnt) {
		rc_printk(RC_ERROR, "rc_mem_copy_list, count %d dst_count %d\n",
			  byte_count, dst_cnt);
		BUG();
	}

	src_idx       = 0;
	dst_idx       = 0;
	src_mapped    = 0;
	dst_mapped    = 0;
	src_offset    = 0;
	dst_offset    = 0;
	src_dma_addr  = 0;
	dst_dma_addr  = 0;
	src_vaddr     = (unsigned char *)0;
	dst_vaddr     = (unsigned char *)0;
	src_page      = (struct page *)0;
	dst_page      = (struct page *)0;

	residual = byte_count;


	while (residual > 0) {

		if (src_mapped == 0) {
			if ((src->sg_mem_type & MEM_TYPE) == RC_MEM_PADDR) {
				src_dma_addr = src->sg_elem[src_idx].dma_paddr + src_offset;
				offset = src_dma_addr & (PAGE_SIZE-1);

				size = src->sg_elem[src_idx].size - src_offset;
				size = min(size, PAGE_SIZE - offset);

				pfn = src_dma_addr >> PAGE_SHIFT;
				src_page = pfn_to_page(pfn);
                src_vaddr = kmap_atomic(src_page);
				if (src_vaddr == 0) {
					if (dst_page)
                        kunmap_atomic(dst_vaddr);
					ret = 0;
					goto out;
				}

				src_vaddr += offset;
				src_mapped = size;
				src_offset = src_offset + size;
				RC_PRINTK(RC_DEBUG3, "%s: src[%d]: dma_addr 0x%llx size 0x%x "
					  "map_addr 0x%x vaddr %px offset 0x%x mapped 0x%x\n",
					  __FUNCTION__,
					  src_idx, (rc_uint64_t)src_dma_addr, size,
					  pfn << PAGE_SHIFT, src_vaddr, src_offset, src_mapped);
			}
			else if ((src->sg_mem_type & MEM_TYPE) == RC_MEM_VADDR) {
				src_vaddr = src->sg_elem[src_idx].v_addr;
				src_mapped = src->sg_elem[src_idx].size;
				RC_PRINTK(RC_DEBUG3, "%s: src[%d]: vaddr %px size %x\n",
					  __FUNCTION__,
					  src_idx,
					  src_vaddr, src_mapped);
			}
			else
				BUG();

		}

		if (dst_mapped == 0) {
			if ((dst->sg_mem_type & MEM_TYPE) == RC_MEM_PADDR) {
				dst_dma_addr = dst->sg_elem[dst_idx].dma_paddr + dst_offset;
				offset = dst_dma_addr & (PAGE_SIZE-1);

				size = dst->sg_elem[dst_idx].size - dst_offset;
				size = min(size, PAGE_SIZE - offset);

				pfn = dst_dma_addr >> PAGE_SHIFT;
				dst_page = pfn_to_page(pfn);
                dst_vaddr = kmap_atomic(dst_page);
				if (dst_vaddr == 0) {
					if (src_page)
                        kunmap_atomic(src_vaddr);
					ret = 0;
					goto out;
				}

				dst_vaddr += offset;
				dst_mapped = size;
				dst_offset = dst_offset + size;
				RC_PRINTK(RC_DEBUG3, "%s: dst[%d]: dma_addr 0x%llx size 0x%x "
					  "map_addr 0x%x vaddr %px offset 0x%x mapped 0x%x\n",
					  __FUNCTION__,
					  dst_idx, (rc_uint64_t)dst_dma_addr, size,
					  pfn << PAGE_SHIFT, dst_vaddr, dst_offset,
					  dst_mapped);
			}
			else if ((dst->sg_mem_type & MEM_TYPE) == RC_MEM_VADDR) {
				dst_vaddr = dst->sg_elem[dst_idx].v_addr;
				dst_mapped = dst->sg_elem[dst_idx].size;
				rc_printk(RC_DEBUG3, "%s: dst[%d]: vaddr %px size: 0x%x\n",
					  __FUNCTION__,
					  dst_idx,
					  dst_vaddr, dst_mapped);
			}
			else
				BUG();
		}


		xfer_cnt = min(src_mapped, dst_mapped);
		xfer_cnt = min(residual, xfer_cnt);
		RC_PRINTK(RC_DEBUG3, "%s: moving %x bytes from %px to %px\n",
			  __FUNCTION__,
			  xfer_cnt,src_vaddr, dst_vaddr);

		// Note: preempt disable may no longer be needed now that a straight
		//       memcpy() is used instead of MMX assembly for the copy
		vret = memcpy( dst_vaddr, src_vaddr, xfer_cnt);
		if (!ret)
			BUG();

		src_mapped -= xfer_cnt;
		dst_mapped -= xfer_cnt;
		residual   -= xfer_cnt;

		if (src_mapped == 0) {
			if ((src->sg_mem_type & MEM_TYPE) == RC_MEM_PADDR) {
				if (src_page)
                    kunmap_atomic(src_vaddr);
				src_page = (struct page *)0;
				if (src_offset == src->sg_elem[src_idx].size) {
					src_idx++;
					src_offset = 0;
				}
			} else if ((src->sg_mem_type & MEM_TYPE) == RC_MEM_VADDR) {
				src_idx++;
			} else
				BUG();
		} else
			src_vaddr += xfer_cnt;

		if (dst_mapped == 0) {
			if ((dst->sg_mem_type & MEM_TYPE) == RC_MEM_PADDR) {
				if (dst_page)
                    kunmap_atomic(dst_vaddr);
				dst_page = (struct page *)0;
				if (dst_offset == dst->sg_elem[dst_idx].size) {
					dst_idx++;
					dst_offset = 0;
				}
			} else  if ((dst->sg_mem_type & MEM_TYPE) == RC_MEM_VADDR) {
				dst_idx++;
			} else
				BUG();
		} else
			dst_vaddr += xfer_cnt;

	}
out:

	if (src_page)
        kunmap_atomic(src_vaddr);
	if (dst_page)
        kunmap_atomic(dst_vaddr);
	if (residual)
		rc_printk(RC_PANIC, "rc_mem_copy_list: residual is not 0 at end of "
			  "loop\n");

	return ret;
}

#ifndef virt_addr_valid
#define virt_addr_valid(kaddr)   pfn_valid(__pa(kaddr) >> PAGE_SHIFT)
#endif

/*
 * Sanity check a single address, making sure it is reasonable
 * based on the memory type.  Returns 1 if the address is good
 * or 0 if an error is found.
 */
static rc_uint32_t
rc_check_addr_one(rc_uint64_t addr, rc_uint32_t id, rc_uint32_t    byte_count)
{
	rc_uint32_t status = 0;

	id &= MEM_TYPE;

	if (id != RC_MEM_VADDR && id != RC_MEM_PADDR) {
		rc_printk(RC_ERROR,
			  "rc_check_addr_one: Bad memory type %x\n", id);
		return status;
	}

	if (id == RC_MEM_VADDR) {
		void *vaddr = ((addr_elem_t)addr).virt_addr;
		if ( ((unsigned long)vaddr < VMALLOC_START ||
		      (unsigned long)vaddr >= VMALLOC_END) &&
		     !virt_addr_valid(vaddr) ) {
			rc_printk(RC_PANIC,
				  "rc_check_addr_one: Invalid virtual address - %px\n",
				  vaddr);
			return status;
		}
	}
	else { /* Physical address */
		rc_uint64_t paddr = ((addr_elem_t)addr).phys_addr;
		unsigned long long pfn = paddr >> PAGE_SHIFT;
		if (!pfn_valid(pfn)) {
			rc_printk(RC_PANIC,
				  "rc_check_addr_one: Invalid physical address - 0x%llx\n",
				  paddr);
			return status;
		}
	}

	status = 1;
	return status;

}


/*
 * Sanity checks a address list structure making sure all the
 * addresses are reasonable based on the memory type. Returns
 * 1 if the list is good or 0 if an error is found.
 */
static rc_uint32_t
rc_check_addr_list(rc_addr_list_t *addr_list)
{
	int i;
	rc_uint16_t mem_type;
	rc_addr_list_t *ap;
	rc_uint32_t status = 0;

	mem_type = addr_list->mem_id & MEM_TYPE;

	if (mem_type != RC_MEM_VADDR && mem_type != RC_MEM_PADDR) {
		rc_printk(RC_ERROR, "rc_check_addr_list: Bad memory type %x\n",
			  mem_type);
		return status;
	}

	for (ap = addr_list; ap; ap = ap->next) {
		if (ap->mem_elem_count > RC_SINGLE_IO_ADDRESS_COUNT) {
			rc_printk(RC_ERROR, "rc_check_addr_list: Element count too large "
				  "- %u\n", addr_list->mem_elem_count);
			return status;
		}

		for (i = 0; i < ap->mem_elem_count; i++) {

			if (mem_type == RC_MEM_VADDR) {
				void *vaddr = ap->sg_list[i].v_addr;
				if ( ((unsigned long)vaddr < VMALLOC_START ||
				      (unsigned long)vaddr >= VMALLOC_END) &&
				     !virt_addr_valid(vaddr) ) {
					rc_printk(RC_PANIC, "rc_check_addr_list: Invalid virtual "
						  "address - %px\n", vaddr);
					return status;
				}
			}
			else { /* Physical address */
				unsigned long long pfn = addr_list->sg_list[i].dma_paddr >>
					PAGE_SHIFT;
				if (!pfn_valid(pfn)) {
					rc_printk(RC_PANIC, "rc_check_addr_list: Invalid physical "
						  "address - 0x%llu\n", ap->sg_list[i].dma_paddr);
					return status;
				}
			}
		}
	}

	status = 1;
	return status;
}


static int __inline__
rc_addr_list_elements( rc_addr_list_t  *ap)
{
	int num_elem;

	num_elem = 0;
	while (ap) {
		num_elem += ap->mem_elem_count;
		ap = ap->next;
	}
	return num_elem;
}


/*
 * determine the size of the src and dst arrays.
 * allocate memory big enough to hold all the lists
 * convert the lists from a chained list to arrays
 * enqueue the mememory operation to be done by the rcraid mem_op thread
 */
void
rc_msg_mem_op(rc_mem_op_t *mop)
{
	int cpu, i;
	unsigned long irql;
	rc_thread_t *tp;
	rc_uint32_t status = 1;

	if (rc_msg_level >= RC_DEBUG) {

		switch (mop->opcode) {

		case RC_OP_MEM_LIST_COPY:
			status &= rc_check_addr_list(mop->mem.list->dst_addr_list);
			status &= rc_check_addr_list(mop->mem.list->src.addr_list);
			break;

		case RC_OP_MEM_LIST_XOR:
			status &= rc_check_addr_list(mop->mem.list->dst_addr_list);
			for (i = 0; i < mop->mem.list->array_count; i++)
				status &=
					rc_check_addr_list(mop->mem.list->src.addr_list_array[i]);
			rc_printk(RC_ERROR, "rc_msg_mem_op: Unsupported memory operation- "
				  "%d\n", mop->opcode);
			status = 0;
			break;

		case RC_OP_MEM_LIST_CMP:
			for (i = 0; i < mop->mem.list->array_count; i++)
				status &=
					rc_check_addr_list(mop->mem.list->src.addr_list_array[i]);
			rc_printk(RC_ERROR, "rc_msg_mem_op: Unsupported memory operation- "
				  "%d\n", mop->opcode);
			status = 0;
			break;

		case RC_OP_MEM_COPY:
			status &= rc_check_addr_one(mop->mem.cp.dst, mop->mem.cp.dst_id,
						    mop->mem.cp.byte_count);
			status &= rc_check_addr_one(mop->mem.cp.src, mop->mem.cp.src_id,
						    mop->mem.cp.byte_count);
			rc_printk(RC_DEBUG2, "RC_MEM_COPY: dst %llx id %x src %llx id %x "
				  "count %x status1 %i\n",
				  mop->mem.cp.dst, mop->mem.cp.dst_id,
				  mop->mem.cp.src, mop->mem.cp.src_id,
				  mop->mem.cp.byte_count, status);
			break;

		case RC_OP_MEM_CLEAR:
			status &= rc_check_addr_one(mop->mem.clr.dst, mop->mem.clr.dst_id,
						    mop->mem.clr.byte_count);
			rc_printk(RC_DEBUG2, "RC_MEM_CLEAR: dst %llx id %x count %x "
				  "status1 %i\n", mop->mem.clr.dst, mop->mem.clr.dst_id,
				  mop->mem.clr.byte_count, status);
			break;
        case RC_OP_MEM_USER_COPY:
            rc_printk(RC_DEBUG2, "RC_MEM_USER_COPY\n");
            break;
		default:
			rc_printk(RC_ERROR, "rc_msg_mem_op: Invalid memory operation- %d\n",
				  mop->opcode);
			status = 0;

		}
	}

	mop->status = status;

	if (status) {

		switch (mop->opcode) {

		case RC_OP_MEM_COPY:
			mop->status = rc_sync_mem_copy(mop->mem.cp.dst, mop->mem.cp.dst_id,
						       mop->mem.cp.src, mop->mem.cp.src_id,
						       mop->mem.cp.byte_count);
			RC_PRINTK(RC_DEBUG2, "RC_MEM_COPY: status2 %d\n", mop->status);
			break;

		case RC_OP_MEM_CLEAR:
			mop->status = rc_sync_mem_clear(mop->mem.clr.dst,
							mop->mem.clr.dst_id,
							mop->mem.clr.byte_count);
			RC_PRINTK(RC_DEBUG2, "RC_MEM_COPY: status2 %d\n", mop->status);
			break;
        case RC_OP_MEM_USER_COPY:
		default:
			/*
			 * queue the morb to the correct cpu
			 */
			preempt_disable();
			cpu = smp_processor_id();
			tp = &rc_thread[cpu];

			local_irq_save(irql);

			mop->next = 0;
			if (tp->mop_head == (rc_mem_op_t *)0) {
				tp->mop_head = mop;
				tp->mop_tail = mop;
			} else {
				tp->mop_tail->next = mop;
				tp->mop_tail = mop;
			}

			if (tp->num_mop == 0)
				wake_up_process(tp->thread);

			tp->num_mop++;

			local_irq_restore(irql);
			preempt_enable();
			break;
		}
	}
}

// STATS
int
rc_mop_stats(char *buf, int buf_size)
{
	int cnt, len;
	rc_softstate_t *state;
	rc_stats_t *sp;
	char *cp;
	int cpu;

	state = &rc_state;
	sp = &state->stats;


	cnt = 0;
	cp = buf;

	len = snprintf(cp, buf_size - cnt, "\nExternal memory opeations\nCPU     ");
	cp  += len;
	cnt += len;

	for_each_online_cpu(cpu)  {
		len = snprintf(cp, buf_size - cnt, "%8d  ", cpu);
		cp  += len;
		cnt += len;
	}

	len = snprintf(cp, buf_size - cnt, "\nPending  ");
	cp  += len;
	cnt += len;

	for_each_online_cpu(cpu)  {
		len = snprintf(cp, buf_size - cnt, "%8d  ", rc_thread[cpu].num_mop);
		cp  += len;
		cnt += len;
	}

	len = snprintf(cp, buf_size - cnt, "\n");
	cp  += len;
	cnt += len;
	*cp = '\0';

	return cnt;
}

/*
 * Set up for a synchronous copy and call rc_mem_copy_list
 * to do the work.  Return 1 on success, 0 if the copy failed
 * for any reason.
 */
int
rc_sync_mem_copy(rc_uint64_t dst, rc_uint32_t dst_id,
		 rc_uint64_t src, rc_uint32_t src_id,
		 rc_uint32_t byte_count)
{
	rc_sg_list_t dst_sg, src_sg;

	dst_sg.sg_mem_type = dst_id;
	dst_sg.sg_num_elem = 1;
	dst_sg.sg_elem[0].size = byte_count;
	if ((dst_id & MEM_TYPE) == RC_MEM_VADDR)
		dst_sg.sg_elem[0].v_addr = ((addr_elem_t)dst).virt_addr;
	else /* physical */
		dst_sg.sg_elem[0].dma_paddr = ((addr_elem_t)dst).phys_addr;

	src_sg.sg_mem_type = src_id;
	src_sg.sg_num_elem = 1;
	src_sg.sg_elem[0].size = byte_count;
	if ((src_id & MEM_TYPE) == RC_MEM_VADDR)
		src_sg.sg_elem[0].v_addr = ((addr_elem_t)src).virt_addr;
	else /* physical */
		src_sg.sg_elem[0].dma_paddr = ((addr_elem_t)src).phys_addr;

	return rc_mem_copy_list (&dst_sg, &src_sg, byte_count);
}

/*
 * Set up for a synchronous clear and call rc_mem_clear_list
 * to do the work.  Return 1 on success, 0 if the copy failed
 * for any reason.
 */
int
rc_sync_mem_clear(rc_uint64_t dst, rc_uint32_t dst_id,
		  rc_uint32_t byte_count)
{
	rc_sg_list_t dst_sg;

	dst_sg.sg_mem_type = dst_id;
	dst_sg.sg_num_elem = 1;
	dst_sg.sg_elem[0].size = byte_count;
	if ((dst_id & MEM_TYPE) == RC_MEM_VADDR)
		dst_sg.sg_elem[0].v_addr = ((addr_elem_t)dst).virt_addr;
	else /* physical */
		dst_sg.sg_elem[0].dma_paddr = ((addr_elem_t)dst).phys_addr;

	return rc_mem_clear_list (&dst_sg, byte_count);
}

/* Flattens an address list and copies it to a scatter/gather list. */
rc_sg_list_t *
rc_mem_sg_list(rc_addr_list_t *ap,
	       rc_uint32_t starting_elem,
	       rc_uint32_t offset,
	       rc_thread_buf_t *buf)
{

	int i, indx = 0;
	int elems = rc_addr_list_elements(ap);
	int size = sizeof(rc_sg_list_t) + (elems - 1) * sizeof(rc_sg_elem_t);
	rc_sg_list_t *sg;

	/* (re)allocate the buffer if it isn't big enough */
	if (size > buf->size) {
		buf->size += 2048;
		kfree(buf->sg);
		buf->sg = kmalloc(buf->size, GFP_KERNEL);
	}

	sg = buf->sg;

	sg->sg_mem_type = ap->mem_id;

	if (starting_elem || offset) {

		rc_sg_elem_t *ep= &ap->sg_list[starting_elem];

		if ((sg->sg_mem_type & MEM_TYPE) == RC_MEM_PADDR) {
			sg->sg_elem[indx].dma_paddr = ep->dma_paddr + offset;
		}
		else {
			sg->sg_elem[indx].v_addr = ep->v_addr + offset;
		}

		sg->sg_elem[indx].size = ep->size - offset;
		starting_elem++;
		indx++;
	}

	while(ap) {
		for ( i = starting_elem ; i < ap->mem_elem_count; i++) {
			sg->sg_elem[indx].addr = ap->sg_list[i].addr;
			sg->sg_elem[indx].size = ap->sg_list[i].size;
			indx++;
			offset = 0;
		}
		ap = ap->next;
		starting_elem = 0;

	}
	sg->sg_num_elem = indx;

	return sg;
}


/*
 * Converts a memory operation structure to a memory copy request structure.
 */
int
rc_kthread_mem_copy(rc_thread_t    *tp, rc_mem_op_t *mop)
{
	rc_sg_list_t *dst, *src;
	int byte_count;

	dst = rc_mem_sg_list(mop->mem.list->dst_addr_list,
			     mop->mem.list->dst_element,
			     mop->mem.list->dst_offset,
			     &tp->buf[0]);

	src = rc_mem_sg_list(mop->mem.list->src.addr_list,
			     mop->mem.list->src_element,
			     mop->mem.list->src_offset,
			     &tp->buf[1]);

	byte_count = mop->mem.list->sector_count << 9;

	return rc_mem_copy_list (dst, src, byte_count);
}

int
rc_kthread_mem_user_copy(rc_thread_t *tp, rc_mem_op_t *mop)
{
	void* src;
    void* dst;
    unsigned long numberOfBytes=0, byte_count=0;;

    src = (void *) (uintptr_t) mop->mem.cp.src;
    dst = (void *) (uintptr_t) mop->mem.cp.dst;
    byte_count = mop->mem.cp.byte_count;

    numberOfBytes = copy_to_user( dst, src, byte_count);

    if (numberOfBytes) {
        RC_PRINTK(RC_ERROR, "rc_kthread_mem_user_copy: %d\n", (int)numberOfBytes);
        return 0;
    }
    else {
        return 1;
    }
}
