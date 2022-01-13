/****************************************************************************
 *
 * Copyright © 2006-2008 Ciprico Inc. All rights reserved.
 * Copyright © 2008-2015 Dot Hill Systems Corp. All rights reserved.
 * Copyright © 2015-2016 Seagate Technology LLC. All rights reserved.
 * Copyright © 2019-2020 Advanced Micro Devices, Inc. All rights reserved.
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

#include "version.h"

#include "rc.h"
#include "version.h"
#include "build_number.h"
#include "rc_pci_ids.h"
#include <linux/hdreg.h>
#include <linux/reboot.h>
#include <linux/pci.h>
#include <linux/interrupt.h>
#include <asm/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/spinlock_types.h>
#include <linux/sysctl.h>
#include <linux/pm_runtime.h>

#define RC_DRIVER_VERSION       RC_VERSION_STR
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,10,0)
#if !defined(RC_DRIVER_BUILD_DATE)
#define RC_DRIVER_BUILD_DATE    __DATE__
#endif  /* !defined(RC_DRIVER_BUILD_DATE) */
#else
#if !defined(RC_DRIVER_BUILD_DATE)
extern const char *RC_DRIVER_BUILD_DATE;
#endif	/* !defined(RC_DRIVER_BUILD_DATE) */
#endif	/* KERNEL_VERSION(5,0,0) */
#define RC_DRIVER_BUILD_TIME    __TIME__
#define RC_DRIVER_NAME          "rcraid"
#define RC_MAX_CMD_Q_DEPTH      1024
#define RC_DEFAULT_CMD_Q_DEPTH  512
#define RC_MAX_TAG_Q_DEPTH      255
#define RC_DEFAULT_TAG_Q_DEPTH  16

//#define RC_SUPPORT_V60_DRIVERS_ON_INTEL_PLATFORMS
//#define RC_SUPPORT_V60_DRIVERS_ON_HUDSON_PLATFORMS

// FIXME: some older kernels still supported by RAIDCore do not have
//        DMA_BIT_MASK().  Remove once support for them has been dropped.
#ifndef DMA_BIT_MASK
#define DMA_BIT_MASK(n) (((n) == 64) ? ~0ULL : ((1ULL<<(n))-1))
#endif

MODULE_AUTHOR(VER_COMPANYNAME_STR);
MODULE_DESCRIPTION("AMD-RAID controller");
MODULE_LICENSE("Proprietary");

static int debug = 0;
#ifdef module_param_named
module_param_named(debug, debug, int, 0444);
#else
MODULE_PARM (debug, "i");
#endif
MODULE_PARM_DESC (debug, "debug print level");

static int cmd_q_depth = RC_DEFAULT_CMD_Q_DEPTH;
#ifdef module_param_named
module_param_named(cmd_q_depth, cmd_q_depth, int, 0444);
#else
MODULE_PARM (cmd_q_depth, "i");
#endif
MODULE_PARM_DESC (cmd_q_depth, "total command queue depth");

static int tag_q_depth = RC_DEFAULT_TAG_Q_DEPTH;
#ifdef module_param_named
module_param_named(tag_q_depth, tag_q_depth, int, 0444);
#else
MODULE_PARM (tag_q_depth, "i");
#endif
MODULE_PARM_DESC (tag_q_depth, "individual tagged command queue depth");

static int max_xfer = 2048; // AHCI PRD limit is 224K or 448 sectors
#ifdef module_param_named
module_param_named(max_xfer, max_xfer, int, 0444);
#else
MODULE_PARM (max_xfer, "i");
#endif
MODULE_PARM_DESC (max_xfer, "max sectors per transfer");

// Default is to use all supported motherboard chip sets and adapter cards.
static int use_swl = RC_SHWL_TYPE_ALL;
#ifdef module_param_named
module_param_named(use_swl, use_swl, int, 0444);
#else
MODULE_PARM (use_swl, "i");
#endif
MODULE_PARM_DESC (use_swl, "Specify SWL chipsets");

// Set the number of adapters for spanning. Issues with "hot insert"
// so force this to be passed every time...
static int rc_adapter_count = 999;      // Bogus value
#ifdef module_param_named
module_param_named(rc_adapter_count, rc_adapter_count, int, 0444);
#else
MODULE_PARM (rc_adapter_count, "i");
#endif  /* module_param_named */
MODULE_PARM_DESC (rc_adapter_count, "Specify number of spanned adapters");

//
// Delay prior to suspend improves chance of success (from failing
//  under load on second cycle to working until manually stopped).
//  Add as parameter so we can vary without rebuilding/installing.
//
static int rc_suspend_delay = 5000;
#ifdef module_param_named
module_param_named(rc_suspend_delay, rc_suspend_delay, int, 0444);
#else
MODULE_PARM(rc_suspend_delay, "i");
#endif  /* module_param_named */
MODULE_PARM_DESC(rc_suspend_delay, "suspend delay");

#define VERSION_STRING_LEN      256
static char                     version_string[VERSION_STRING_LEN];

/*
 * Globals
 */
int                rc_config_debug = 0;
rc_softstate_t     rc_state;
int                rc_cntl_num = 0;
int                rc_msg_level = RC_DEFAULT_ERR_LEVEL;
rc_adapter_t      *rc_dev[MAX_HBA];
struct mutex	   ioctl_mutex;

static unsigned adapter_count = 0;     /* Number of adapters on the PCI bus,
					  Used to determine when the last
					  adapter has been initialized. */

extern struct miscdevice rccfg_api_dev;

extern unsigned int RC_EnableDIPM;
extern unsigned int RC_EnableHIPM;
extern unsigned int RC_EnableAN;
extern unsigned int RC_EnableNCQ;
extern unsigned int RC_EnableZPODD;

#define RCRAID_DEFAULT_DIPM	0x00000000;  /* Turn OFF DIPM for all ports by default for Linux */
#define RCRAID_DEFAULT_HIPM	0x00000000;  /* Turn OFF HIPM for all ports by default for Linux */
#define RCRAID_DEFAULT_AN	0x00000001;  /* Turn ON Asynchronous Notification by default for Linux */
#define RCRAID_DEFAULT_NCQ	0x00000001;  /* Turn ON NCQ by default for Linux */
#define RCRAID_DEFAULT_ZPODD	0x00000000; /* Turn ON Zero Power Optical Disk Device by default for Linux */

struct task_struct      *rc_wq = NULL;
rc_work_t               *acpi_work_item_head = NULL;
rc_work_t               *acpi_work_item_tail = NULL;
spinlock_t              acpi_work_item_lock;
extern int              rc_wq_handler(void *work);

/*
 * function prototypes
 */
static int  rc_eh_abort_cmd(struct scsi_cmnd * scmd);
static int  rc_eh_dev_reset(struct scsi_cmnd * scmd);
static int  rc_eh_bus_reset(struct scsi_cmnd * scmd);
static int  rc_eh_hba_reset(struct scsi_cmnd * scmd);

void        rc_shutdown_adapter(rc_adapter_t *adapter);
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 1, 0)
int         rc_ioctl(struct scsi_device * scsi_dev_ptr, int cmd, void *arg);
#else
int         rc_ioctl(struct scsi_device * scsi_dev_ptr, unsigned int cmd, void *arg);
#endif


void        rc_dump_scp(struct scsi_cmnd * scp);
const char *rc_info(struct Scsi_Host *host_ptr);
void        rc_timeout(int to);
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,14,0)
void        rc_timeout_done(unsigned long data);
#else
void        rc_timeout_done(struct timer_list *t);
#endif
static int  rc_slave_cfg(struct scsi_device *sdev);
int         rc_bios_params(struct scsi_device *sdev, struct block_device *bdev,
			   sector_t capacity, int geom[]);

int         rc_queue_cmd_lck(struct scsi_cmnd * scp, void (*CompletionRoutine) (struct scsi_cmnd *));

#ifdef RC_AHCI_SUPPORT
// Additions for AHCI driver
static inline void rc_ahci_disable_irq(rc_adapter_t *adapter);
int         rc_ahci_init(rc_adapter_t *adapter);
int         rc_ahci_start(rc_adapter_t *adapter);
int         rc_ahci_shutdown(rc_adapter_t *adapter);
irqreturn_t rc_ahci_isr(int irq, void *arg, struct pt_regs *regs);
#endif // RC_AHCI_SUPPORT

// Additions for NVME driver
int         rc_nvme_init(rc_adapter_t *adapter);
int         rc_nvme_start(rc_adapter_t *adapter);
int         rc_nvme_shutdown(rc_adapter_t *adapter);
irqreturn_t rc_nvme_isr(int irq, void *arg, struct pt_regs *regs);

void       rc_remove_proc(void);

void rc_msg_isr( rc_adapter_t *adapter);
int  rc_msg_send_srb(struct scsi_cmnd * scp);
int  rc_msg_init(rc_softstate_t *state);
void rc_msg_shutdown(rc_softstate_t *statep);
int  rc_reboot_notify(struct notifier_block *nb, ulong event, void *buf);
int  rc_msg_stats(char *buf, int buf_size);
int  rc_mop_stats(char *buf, int buf_size);
void rc_send_msg(struct rc_send_arg_s *p_send_arg);
void rc_msg_resume(rc_softstate_t *state, rc_adapter_t* adapter);


struct rc_pci_bar {
	struct {
		rc_uint32_t low;
		rc_uint32_t high;
	} addr;
	rc_uint32_t len;
	rc_uint32_t flags;
};

#ifdef RC_AHCI_SUPPORT
static rc_version_t rc_ahci_version =
{
	.init_func = rc_ahci_init,
	.start_func = rc_ahci_start,
	.shutdown_func = rc_ahci_shutdown,
	.isr_func = rc_ahci_isr,
	.device_name = "rcraid",
	.vendor = VER_COMPANYNAME_STR,
	.model = VER_AHCI_STR,
	.num_ports = 6,
	.window_size = 0,
	.which_bar = 5,
	.swl_type = RC_SHWL_TYPE_AHCI
};
#endif

static rc_version_t rc_nvme_version =
{
	.init_func = rc_nvme_init,
	.start_func = rc_nvme_start,
	.shutdown_func = rc_nvme_shutdown,
	.isr_func = rc_nvme_isr,
	.device_name = "rcraid",
	.vendor = VER_COMPANYNAME_STR,
	.model = VER_NVME_STR,
	.num_ports = 6,
	.window_size = 0,
	.which_bar = 0,
	.swl_type = RC_SHWL_TYPE_NVME
};

#define PCI_CLASS_NVME_STORAGE_EXPRESS 0x010802

static struct pci_device_id rcraid_id_tbl[] = {
	{
		.vendor = RC_PD_VID_AMD,
		.device = AMD_NVME_DID,
		.subvendor = PCI_ANY_ID,
		.subdevice = PCI_ANY_ID,
		.class = (PCI_CLASS_NVME_STORAGE_EXPRESS),
		.class_mask = 0xffffff,
		.driver_data = (unsigned long)&rc_nvme_version
	},

#ifdef RC_AHCI_SUPPORT
	{
		.vendor = RC_PD_VID_AMD,
		.device = RC_PD_DID_BRISTOL,
		.subvendor = PCI_ANY_ID,
		.subdevice = PCI_ANY_ID,
		.class = 0,
		.class_mask = 0,
		.driver_data = (unsigned long)&rc_ahci_version
	},
	{
		.vendor = RC_PD_VID_AMD,
		.device = RC_PD_DID_SUMMIT,
		.subvendor = PCI_ANY_ID,
		.subdevice = PCI_ANY_ID,
		.class = 0,
		.class_mask = 0,
		.driver_data = (unsigned long)&rc_ahci_version
	},
	{
		.vendor = RC_PD_VID_AMD,
		.device = RC_PD_DID_PROMONTORY,
		.subvendor = PCI_ANY_ID,
		.subdevice = PCI_ANY_ID,
		.class = 0,
		.class_mask = 0,
		.driver_data = (unsigned long)&rc_ahci_version
	},

	{
		.vendor = RC_PD_VID_AMD,
		.device = RC_PD_DID_SUMMIT_SP3,
		.subvendor = PCI_ANY_ID,
		.subdevice = PCI_ANY_ID,
		.class = 0,
		.class_mask = 0,
		.driver_data = (unsigned long)&rc_ahci_version
	},
#endif // RC_AHCI_SUPPORT

	{0,}
};

typedef struct rc_bios_disk_parameters
{
	int heads;
	int sectors;
	int cylinders;
} rc_bios_disk_parameters_t;

static DEF_SCSI_QCMD(rc_queue_cmd)

static Scsi_Host_Template driver_template = {
	.module =                  THIS_MODULE,
	.name =                    RC_DRIVER_NAME,
	.proc_name =               RC_DRIVER_NAME,
	.proc_dir =                NULL,
	.info =                    rc_info,
	.ioctl =                   rc_ioctl,
	.queuecommand =            rc_queue_cmd,
	.bios_param =              rc_bios_params,
	.can_queue =               1,
	.this_id =                 -1,
	.sg_tablesize =            1,
	.max_sectors =             128, // 64K
	.eh_abort_handler =        rc_eh_abort_cmd,
	.eh_device_reset_handler = rc_eh_dev_reset,
	.eh_bus_reset_handler =    rc_eh_bus_reset,
	.eh_host_reset_handler =   rc_eh_hba_reset,
#if LINUX_VERSION_CODE < KERNEL_VERSION(5,0,0)
	.use_clustering =          ENABLE_CLUSTERING,
#else
#endif	/* (5,0,0) */
	.slave_configure =         rc_slave_cfg,
};

//
// UEFI NVARM Access
//

static struct efi *get_efi(void)
{
#ifdef  CONFIG_EFI
        return &efi;
#else
        return NULL;
#endif  /* CONFIG_EFI */
}

NVME_TRAP_DEVICE        NvmeTrapDeviceVar[MAX_HBA - 1];         // One HBA reserved for AHCI(/ATAPI)

static efi_status_t
RC_Unmap_VidDid(
    uint32_t    Location,
    uint16_t    *VID,
    uint16_t    *DID
    )
{
    int32_t     I;

    for (I = 0; I < (MAX_HBA - 1); I++)
    {
        uint32_t    *p = (uint32_t *) &NvmeTrapDeviceVar[I].Bus;

        if (Location == *p)
        {
            if (VID)
                *VID = NvmeTrapDeviceVar[I].VendorId;
            if (DID)
                *DID = NvmeTrapDeviceVar[I].DeviceId;
            return EFI_SUCCESS;
        }
    }

    return EFI_NOT_FOUND;
}

/*
 * rc_init_module()
 *
 *    One time initialization stuff, called from the module init function
 *    in 2.6 kernels.
 */
static void
rc_init_module(void)
{
	extern char       *rc_ident;

	rc_printk(RC_NOTE, "%s %s raid driver version %s build_number %s built "
		  "%s\n", VER_COMPANYNAME_STR, RC_DRIVER_NAME,
                  RC_DRIVER_VERSION, RC_BUILD_NUMBER, RC_DRIVER_BUILD_DATE);
	rc_printk(RC_NOTE, "%s %s\n", RC_DRIVER_NAME, rc_ident);

        //
        // Attempt to find NVMe original VID/DID table
        //
        memset(NvmeTrapDeviceVar, 0, sizeof(NvmeTrapDeviceVar));
        if (get_efi())
        {
                efi_status_t    Status;
                unsigned long   VarSize;
                unsigned int    nvmeTrapDeviceAttrib = 0;
                efi_guid_t      gAmdNvmeTrapDeviceVarGuid = NVME_TRAP_DEVICE_VAR_GUID;

                VarSize = sizeof(NvmeTrapDeviceVar);
                {
                        // EFI_NOT_FOUND if treminating 0 isn't present...
                        static efi_char16_t NVME_TRAP_DEVICE_NAME[] = {
                                'N', 'V', 'M', 'E', '_', 'T', 'R', 'A', 'P', '_', 'D', 'E', 'V', 'I', 'C', 'E', 0 };

                        Status = get_efi()->get_variable(
                                NVME_TRAP_DEVICE_NAME, // Name,
                                &gAmdNvmeTrapDeviceVarGuid, // GUID
                                &nvmeTrapDeviceAttrib, // Attribute
                                &VarSize,
                                &NvmeTrapDeviceVar[0]
                                );
                }
        }

	/*
	 * enforce reasonable limits on module parameters
	 */
	if (cmd_q_depth < 0)
		cmd_q_depth = 1;
	if (cmd_q_depth > RC_MAX_CMD_Q_DEPTH)
		cmd_q_depth = RC_MAX_CMD_Q_DEPTH;

	if (tag_q_depth < 0)
		tag_q_depth = 1;
	if (tag_q_depth > cmd_q_depth)
		tag_q_depth = cmd_q_depth;

	if (max_xfer < 8)
		max_xfer = 8;

	use_swl |= RC_SHWL_TYPE_CARD; // always support cards

	rc_printk(RC_NOTE, "rcraid: cmd_q_depth %d, tag_q_depth %d, max_xfer "
                  "%d, use_swl 0x%x\n", cmd_q_depth, tag_q_depth, max_xfer,
                  use_swl);

	rc_msg_level += debug;
	if (rc_msg_level < 0)
		rc_msg_level = 0;

	/* Initialize the global state variable */
	memset(&rc_state, 0, sizeof(rc_softstate_t));
	spin_lock_init(&rc_state.osic_lock);
	//rc_state.osic_lock = __SPIN_LOCK_UNLOCKED(rc_state.osic_lock);
	sema_init(&rc_state.rc_timeout_sema, 0);

	/* Initialize Power Management DIPM & HIPM settings */
	RC_EnableDIPM = RCRAID_DEFAULT_DIPM;
	RC_EnableHIPM = RCRAID_DEFAULT_HIPM;
    RC_EnableAN = RCRAID_DEFAULT_AN;
    RC_EnableNCQ = RCRAID_DEFAULT_NCQ;
    RC_EnableZPODD = RCRAID_DEFAULT_ZPODD;

    // Setup ACPI work handler
    {
        char    thread_name[64];

        sprintf(thread_name, "%s ACPI thread", RC_DRIVER_NAME);
        rc_wq = kthread_run(rc_wq_handler, NULL, thread_name);
        spin_lock_init(&acpi_work_item_lock);
    }
}

/*
 * Takes into account 64-bit bars.
 * If linux changes how pci_resource_ works, this code will have to change
 */
static void
rc_get_bar(struct pci_dev *dev, int which_bar, struct rc_pci_bar *result)
{
	int i, index;
	struct rc_pci_bar bar;

	memset(&bar, 0, sizeof(bar));

	for (i = 0, index = 0;  i <= which_bar; i++, index++) {
		bar.addr.low  = pci_resource_start(dev,index);
		bar.len       = pci_resource_len(dev,index);
		bar.flags     = pci_resource_flags(dev,index);

		if (bar.flags & 0x4) {
			index++;
			bar.addr.high =  pci_resource_start(dev,index);
		} else {
			bar.addr.high = 0;
		}

		rc_printk(RC_DEBUG, "bar %d addr 0x%x.%x len %d flags 0x%x\n",
			  i, bar.addr.high, bar.addr.low, bar.len, bar.flags);
	}

	*result = bar;
}

/*
 * rc_init_adapter()
 *
 *    Initialize a single adapter, returns 0 on success or a negative
 *    error code on failure.
 */
#ifdef NO_IRQ_HANDLER_T
typedef irqreturn_t (*irq_handler_t)(int, void *, struct pt_regs *);
#define RC_IRQF SA_INTERRUPT | SA_SHIRQ
#else
#define RC_IRQF IRQF_SHARED         // IRQF_DISABLED | IRQF_SHARED
#endif

static int
rc_init_adapter(struct pci_dev *dev, const struct pci_device_id *id)
{
	rc_adapter_t          *adapter;
	rc_hw_info_t          *hw;
	int                   i;
	struct rc_pci_bar     bar;
	//int                   rc;
	uint32_t              Loc;

	rc_printk(RC_DEBUG, "%s: Matched %.04x/%.04x/%.04x/%.04x\n", __FUNCTION__,
		  id->vendor, id->device, id->subvendor, id->subdevice);

	if (rc_state.num_hba == MAX_HBA) {
		rc_printk(RC_ERROR, "%s: Exceeded maximum adapter count of %d",
			  __FUNCTION__, MAX_HBA);
		return -ENODEV;
	}

	if (pci_enable_device(dev))
	{
		return -ENODEV;
	}

	adapter = kmalloc(sizeof(rc_adapter_t), GFP_KERNEL);
	if (adapter == NULL) {
		rc_printk(RC_ERROR, "%s: can't alloc memory\n", __FUNCTION__);
		return -ENODEV;
	}

	memset(adapter, 0, sizeof (*adapter));
	adapter->pdev = dev;
	adapter->instance = rc_state.num_hba;
	adapter->version = (rc_version_t *)id->driver_data;
	adapter->name = adapter->version->device_name;
	adapter->hardware.adapter_number = rc_state.num_hba;

	//rc = pcim_iomap_regions_request_all(dev, 1 << adapter->version->which_bar, "rcraid");
	//rc_printk(RC_ERROR, "%s(): rc = 0x%x\n", __FUNCTION__, rc);

	hw = &adapter->hardware;
	hw->pci_bus = dev->bus->number;
	hw->pci_slot = PCI_SLOT(dev->devfn);
	rc_get_bar(dev, adapter->version->which_bar, &bar);
	hw->phys_addr = bar.addr.low;
	hw->mem_len = bar.len;

	if (hw->phys_addr == 0) {   /* No pci memory */
		rc_printk(RC_ERROR, RC_DRIVER_NAME ": %s %s with pci IDs %x/%x/%x/%x "
			  "has no pci address space assigned\n",
			  adapter->version->vendor, adapter->version->model,
			  id->vendor, id->device, id->subvendor, id->subdevice);
		rc_printk(RC_ERROR, "Check BIOS PCI settings\n");
		rc_shutdown_adapter(adapter);
		return -ENODEV;
	}

	/*
	 * make a copy of pci config space
	 */
	for (i = 0; i < (2 * PCI_CFG_SIZE); i++) {  // Allow for extended pci config space
		pci_read_config_byte(dev, i, &hw->pci_config_space[i]);
	}

	Loc = ((((hw->pci_func & 0xFF) << 8) | (hw->pci_slot & 0xFF)) << 8) | (hw->pci_bus & 0xFF);

	hw->orig_vendor_id = (uint16_t) 0;
	hw->orig_device_id = (uint16_t) 0;

	(void) RC_Unmap_VidDid(Loc, &hw->orig_vendor_id, &hw->orig_device_id);

	/*
	 * set dma_mask to 64 bit capabilities but if that fails, try 32 bit
	 */
	if (!pci_set_dma_mask(dev, DMA_BIT_MASK(64)) &&
	    !pci_set_consistent_dma_mask(dev, DMA_BIT_MASK(64))) {
		rc_printk(RC_NOTE, RC_DRIVER_NAME ": %s 64 bit DMA enabled\n",
			  __FUNCTION__);
	} else if (!pci_set_dma_mask(dev, DMA_BIT_MASK(32)) &&
		   !pci_set_consistent_dma_mask(dev, DMA_BIT_MASK(32))) {
		rc_printk(RC_NOTE, RC_DRIVER_NAME ": %s 64 bit DMA disabled\n",
			  __FUNCTION__);
	} else {
		rc_printk(RC_ERROR, RC_DRIVER_NAME ": %s failed to "
			  "set usable DMA mask\n", __FUNCTION__);
		rc_shutdown_adapter(adapter);
		return -ENODEV;
	}

	/*
	 * map in the adapter MMIO space
	 */
	adapter->hardware.vaddr = (void *) ioremap(hw->phys_addr, hw->mem_len);
	rc_printk(RC_ERROR, "%s(): hardware.vaddr = %px\n", __FUNCTION__, adapter->hardware.vaddr);

	if (adapter->hardware.vaddr == NULL) {
		rc_printk(RC_ERROR, RC_DRIVER_NAME ": %s can't map %s "
			  "adapter %d at 0x%lx\n", __FUNCTION__,
			  adapter->name, adapter->instance, hw->phys_addr);
		rc_shutdown_adapter(adapter);
		return -ENODEV;
	}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 13, 0)
    // Check for device before calling init_func to ensure RC_EnableZPODD properly set.
    {
        acpi_handle                 handle = DEVICE_ACPI_HANDLE(&adapter->pdev->dev);
        acpi_status                 ac_stat;
        unsigned int                devStat = (unsigned int) -1;
        unsigned int                devAddr = (unsigned int) -1;
        unsigned char               package[96];
        int                         size;

        //
        // Initialize default GPE number. If there's a _PRW() method, we'll update
        // this with what the BIOS ACPI reports.
        //
        // _PRW() should return a package with two integers -- 1st is GPE number (expect 6),
        //      2nd is lowest sleep state (expect 3)
        //
        RC_ODD_GpeNumber = RC_DEFAULT_ZPODD_GPE_NUMBER;

        //
        // ._STA
        //
        // Bit 5-31 Reserved
        // Bit 4    Battery present
        // Bit 3    Device functioning properly
        // Bit 2    Device should be shown in UI
        // Bit 1    Device is enabled
        // Bit 0    Device is present
        //
        size = sizeof(devStat);
        ac_stat = rc_acpi_evaluate_object(handle, "ODDZ._STA", &devStat, &size);
        if (ACPI_SUCCESS(ac_stat) && (devStat & 0xB) == 0xB)
        {
            RC_ODD_Device = RC_ODD_DEVICE_ODDZ;

            size = sizeof(devAddr);
            ac_stat = rc_acpi_evaluate_object(handle, "ODDZ._ADR", &devAddr, &size);
            if (ACPI_SUCCESS(ac_stat))
            {
                RC_ODDZDevAddr = devAddr;

                size = sizeof(package);
                ac_stat = rc_acpi_evaluate_object(handle, "ODDZ._PRW", package, &size);
                if (ACPI_SUCCESS(ac_stat))
                {
                    union acpi_object *acpi_obj = (union acpi_object *) package;
                    union acpi_object *aObj = &acpi_obj->package.elements[0];

                    if (acpi_obj->type == ACPI_TYPE_PACKAGE && acpi_obj->package.count == 2)
                    {
                        RC_ODD_GpeNumber = (unsigned int) aObj->integer.value;
                    }
                }
            }
        } else {
            size = sizeof(devStat);
            ac_stat = rc_acpi_evaluate_object(handle, "ODDL._STA", &devStat, &size);

            if (ACPI_SUCCESS(ac_stat) && (devStat & 0xB) == 0xB)
            {
                RC_ODD_Device = RC_ODD_DEVICE_ODDL;

                size = sizeof(devAddr);
                ac_stat = rc_acpi_evaluate_object(handle, "ODDL._ADR", &devAddr, &size);
                if (ACPI_SUCCESS(ac_stat))
                {
                    RC_ODDZDevAddr = devAddr;

                    size = sizeof(package);
                    ac_stat = rc_acpi_evaluate_object(handle, "ODDL._PRW", package, &size);
                    if (ACPI_SUCCESS(ac_stat))
                    {
                        union acpi_object *acpi_obj = (union acpi_object *) package;
                        union acpi_object *aObj = &acpi_obj->package.elements[0];

                        if (acpi_obj->type == ACPI_TYPE_PACKAGE && acpi_obj->package.count == 2)
                        {
                            RC_ODD_GpeNumber = (unsigned int) aObj->integer.value;
                        }
                    }
                }
            } else {
                size = sizeof(devStat);
                ac_stat = rc_acpi_evaluate_object(handle, "ODD8._STA", &devStat, &size);

                if (ACPI_SUCCESS(ac_stat) && (devStat & 0xB) == 0xB)
                {
                    RC_ODD_Device = RC_ODD_DEVICE_ODD8;

                    size = sizeof(devAddr);
                    ac_stat = rc_acpi_evaluate_object(handle, "ODD8._ADR", &devAddr, &size);
                    if (ACPI_SUCCESS(ac_stat))
                    {
                        RC_ODDZDevAddr = devAddr;

                        size = sizeof(package);
                        ac_stat = rc_acpi_evaluate_object(handle, "ODD8._PRW", package, &size);
                        if (ACPI_SUCCESS(ac_stat))
                        {
                            union acpi_object *acpi_obj = (union acpi_object *) package;
                            union acpi_object *aObj = &acpi_obj->package.elements[0];

                            if (acpi_obj->type == ACPI_TYPE_PACKAGE && acpi_obj->package.count == 2)
                            {
                                RC_ODD_GpeNumber = (unsigned int) aObj->integer.value;
                            }
                        }
                    }
                } else {
                    RC_EnableZPODD = 0;
                }
            }
        }
    }
    if (RC_ODD_Device > RC_ODD_DEVICE_ODD8)
        RC_ODD_Device = RC_ODD_DEVICE_INVALID;

    if (!RC_EnableZPODD)
    {
        rc_printk(RC_INFO, "### %s(): RC_EnableZPODD = %d\n", __FUNCTION__, RC_EnableZPODD);
    } else {
        char ODD_Devices[4] = { '?', 'Z', 'L', '8' };
        rc_printk(RC_INFO, "### %s(): RC_EnableZPODD = %d, RC_ODD_Device = ODD%c, RC_ODDZDevAddr = 0x%x, RC_ODD_GpeNumber = %d\n",
                __FUNCTION__, RC_EnableZPODD, ODD_Devices[RC_ODD_Device], RC_ODDZDevAddr, RC_ODD_GpeNumber);
    }
#endif  /* LINUX_VERSION_CODE >= KERNEL_VERSION(3, 13, 0) */

	{
		dma_addr_t	mem_dma;
		void		*mem;

		mem = dmam_alloc_coherent(&dev->dev, 1024 * 1024, &mem_dma, GFP_KERNEL);
	}

	/* Call initialization routine */
	rc_printk(RC_DEBUG, "%s: Initializing hardware...\n", __FUNCTION__);
	if ((*adapter->version->init_func)(adapter) != 0) {
		/* Device initialization failed */
		rc_printk(RC_ERROR, RC_DRIVER_NAME ":%d Device initialization failed\n",
			  adapter->instance);
		rc_shutdown_adapter(adapter);
		return -ENODEV;
	}

	/* attach the interrupt */

    // Some NVMe devices only work with MSI-X interrupts. Use
    // the devm_ to properly set everything up...

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,8,1)
    if (request_irq(dev->irq, (void *)adapter->version->isr_func,
            RC_IRQF,
            adapter->version->model, (void *)adapter ) < 0) {
#else
    // repurpose 'i' here
    i = dev->irq;
    if (dev->msix_enabled)
	i = adapter->hardware.msix_ent.vector;
    if (devm_request_irq(&adapter->pdev->dev,
            i,       // pci_irq_vector(dev, 0),
            (void *) adapter->version->isr_func,
            0,      // RC_IRQF,
            RC_DRIVER_NAME,
            (void *) adapter) < 0) {
#endif  /* LINUX_VERSION_CODE < KERNEL_VERSION(3,8,1) */
		rc_printk(RC_ERROR, RC_DRIVER_NAME ":%d request_irq failed\n",
			  adapter->instance);
		rc_shutdown_adapter(adapter);
		return -ENODEV;
	} else
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,8,1)
		rc_printk(RC_ERROR, RC_DRIVER_NAME ":%d irq %d\n",
			  adapter->instance, dev->irq);
#else
		rc_printk(RC_ERROR, RC_DRIVER_NAME ":%d irq %d\n",
			  adapter->instance, i);	// dev->irq);     // pci_irq_vector(dev, 0));
#endif  /* LINUX_VERSION_CODE < KERNEL_VERSION(3,8,1) */
dev_info(&dev->dev, "msi_enabled %d, msix_enabled %d\n", dev->msi_enabled, dev->msix_enabled);


#if LINUX_VERSION_CODE < KERNEL_VERSION(3,8,1)
    hw->irq = dev->irq;
#else
	hw->irq = i;	// dev->irq;     // pci_irq_vector(dev, 0);
#endif  /* LINUX_VERSION_CODE < KERNEL_VERSION(3,8,1) */

	if ((*adapter->version->start_func)(adapter) != 0) {
		/* Device initialization failed */
		rc_printk(RC_ERROR, RC_DRIVER_NAME ":%d Device start failed\n",
			  adapter->instance);
		rc_shutdown_adapter(adapter);
		return -ENODEV;
	}

	pci_set_drvdata(dev, adapter);
	rc_dev[rc_state.num_hba++] = adapter;

	rc_printk(RC_NOTE, RC_DRIVER_NAME ": card %d: %s %s\n\n", adapter->instance,
		  adapter->version->vendor, adapter->version->model);

    pci_save_state(dev);

	return 0;
}


/*
 * rc_init_host()
 *
 *    Final stage of initialization is called after the last adapter
 *    has been initialized. and the core started. finializes the host
 *    parameters.
 */
static int
rc_init_host(struct pci_dev *pdev)
{
	int error;
	struct Scsi_Host    *host_ptr;

	/* start the raid core. */
	if (0 != (error = rc_msg_init(&rc_state)))
	{
		return error;
	}

	rc_printk(RC_DEBUG, "rc_init_host: calling scsi_host_alloc\n");
	host_ptr = scsi_host_alloc (&driver_template, 32);
	if (!host_ptr)
	{
		return -ENOMEM;
	}

	if (rc_state.state & USE_OSIC) {
		host_ptr->max_id = RC_MAX_SCSI_TARGETS;
		host_ptr->max_channel = 1;
		host_ptr->can_queue = cmd_q_depth;
		host_ptr->sg_tablesize = 32;
		host_ptr->max_sectors = max_xfer;
		host_ptr->cmd_per_lun = 32;  // untagged queue depth
	} else {
		host_ptr->max_id = 9;
		host_ptr->max_channel = rc_state.num_hba - 1;
		host_ptr->can_queue = 1;
		host_ptr->sg_tablesize = 1;
		host_ptr->max_sectors = 128;
		host_ptr->cmd_per_lun = 1;  // untagged queue depth
	}

	host_ptr->max_lun = 1;
	host_ptr->irq = 0;
	host_ptr->base = 0;
	host_ptr->max_cmd_len = 16;
	host_ptr->unique_id = 0;
	host_ptr->this_id = -1;  /* SCSI Id for the adapter itself */

	driver_template.present = 1;    /* one virtual adapter */

	error = scsi_add_host(host_ptr, &pdev->dev);

	if (error) {
		rc_printk(RC_ERROR, "Failed to add scsi host");
		scsi_host_put(host_ptr);
		return error;
	}

	rc_state.host_ptr = host_ptr;
    rc_state.is_suspended = 0;
	scsi_scan_host(host_ptr);

	rc_printk(RC_DEBUG, "rc_init_host: completed\n");
	return 0;
}


/*
 * rcraid_probe_one()
 *
 *    Called once by the 2.6 kernel for each adapter discovered.
 *    Returns 0 if successful or a negative error code if the
 *    device couldn't be initalized.
 */
static int
rcraid_probe_one(struct pci_dev *dev, const struct pci_device_id *id)
{
	int  ret = -ENODEV;
	struct pci_device_id *probe_id;
	struct pci_dev *probe_dev;

	/* Count the number adapters on the bus that we will claim. */
	rc_printk(RC_DEBUG, "%s rcraid ENTER\n", __FUNCTION__);
	if (!adapter_count) {
        rc_printk(RC_NOTE, "%s: counting supported adapters\n", __FUNCTION__);
		for (probe_id = &rcraid_id_tbl[0]; probe_id->vendor != 0; probe_id++) {
			probe_dev = NULL;
			while ((probe_dev = pci_get_subsys(probe_id->vendor,
							   probe_id->device,
							   probe_id->subvendor,
							   probe_id->subdevice,
							   probe_dev))) {


				if (probe_id->vendor == PCI_ANY_ID) {
					if (probe_id->class != probe_dev->class) {
						//rc_printk(RC_NOTE, "%s: not rcraid device vendor = 0x%x device 0x%x, class 0x%x\n", __FUNCTION__, probe_dev->vendor,probe_dev->device,probe_dev->class);
						continue;
					}
				}

				rc_printk(RC_NOTE, "%s: matched supported adapter - vendor = 0x%x device 0x%x\n", __FUNCTION__,
					  probe_dev->vendor,
					  probe_dev->device
					  );


				/* Found an adapter */
				if (use_swl &
				    ((rc_version_t *)probe_id->driver_data)->swl_type) {
					adapter_count++;
				}
			}
		}
		rc_printk(RC_NOTE, "%s: Total adapters matched %u\n", __FUNCTION__,
			  adapter_count);
	}

	if (use_swl & ((rc_version_t *)id->driver_data)->swl_type) {
		ret = rc_init_adapter(dev, id);
	}
	if (ret < 0)
	{
	    return ret;
	}

	/*
	 * Finished with all of the adapters, start the core and
	 * initialize the one virtual scsi host.  The PCI device information for
	 * the last adapter initialized will be used for all arrays.
	 */
	if ((adapter_count && rc_adapter_count == rc_state.num_hba) ||
        (rc_adapter_count == 999 && adapter_count == rc_state.num_hba)) {
		int err;

		err = rc_init_host(dev);
		if (!err) {
			if (misc_register(&rccfg_api_dev))
				rc_printk(RC_ERROR, "%s: failed to register rc_api\n",__FUNCTION__);
			mutex_init(&ioctl_mutex);
		} else {
			return err;
		}
	}
	return ret;
}


/*
 *  rc_shutdown_host()
 *
 *      Shuts down the core.
 *
 */
void
rc_shutdown_host(struct Scsi_Host *host_ptr)
{
	if ((rc_state.state & USE_OSIC) == 0) return;


	rc_state.state |= SHUTDOWN;

	rc_printk(RC_DEBUG, "rc_shutdown_host\n" );
	rc_msg_shutdown(&rc_state);
}



/*
 *  rc_shutdown_adapter()
 *
 *    Shuts down an adapter card and frees all of its' resources.
 *
 */
void
rc_shutdown_adapter(rc_adapter_t *adapter)
{
	if (adapter == (rc_adapter_t *) 0) {
		rc_printk(RC_DEBUG, "%s: NULL adapter\n", __FUNCTION__);
		return;
	}

	rc_printk(RC_DEBUG, "%s: adapter %d addr 0x%px\n",
		  __FUNCTION__, adapter->instance, adapter);

	/* Call the adapter specific shutdown function. */
	if (adapter->version->shutdown_func)
		(*adapter->version->shutdown_func)(adapter);

	rc_printk(RC_DEBUG, "%s: free_irq\n", __FUNCTION__);
	if (adapter->hardware.irq)
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,8,1)
        free_irq(adapter->hardware.irq, adapter);
#else
        devm_free_irq(&adapter->pdev->dev, adapter->hardware.irq, adapter);

    if (adapter->pdev->msi_enabled)
        pci_disable_msi(adapter->pdev);
    if (adapter->pdev->msix_enabled)
        pci_disable_msix(adapter->pdev);
#endif  /* LINUX_VERSION_CODE < KERNEL_VERSION(3,8,1) */

	rc_printk(RC_DEBUG, "%s: unmap MMIO space 0x%px\n",
		  __FUNCTION__, adapter->hardware.vaddr);
	if (adapter->hardware.vaddr)
		iounmap((void *)adapter->hardware.vaddr);

	rc_printk(RC_DEBUG, "%s: free private_mem 0x%px\n",
		  __FUNCTION__, adapter->private_mem.vaddr);
	if (adapter->private_mem.vaddr)  {
		pci_free_consistent(adapter->pdev,
				    rc_state.memsize_per_controller,
				    adapter->private_mem.vaddr,
				    adapter->private_mem.dma_address);
	}

	/* pci_disable_device(adapter->pdev); */
	rc_printk(RC_DEBUG, "%s: free adapter 0x%px\n",
		  __FUNCTION__, adapter);
	kfree(adapter);
}

void rc_stop_all_threads(void);
void rc_start_all_threads(void);
void rc_msg_suspend_work(rc_adapter_t *adapter);
void rc_msg_init_tasklets(rc_softstate_t *state);
void rc_msg_kill_tasklets(rc_softstate_t *state);
void rc_msg_suspend(rc_softstate_t *state, rc_adapter_t* adapter);
void rc_msg_free_all_dma_memory(rc_adapter_t   *adapter);

#ifdef  CONFIG_PM
/*
 * rcraid_suspend_one()
 *
 *     Called to suspend.  Should do as little work as possible
 *     Any long running tasks should be done in the tasklet to prevent Linux from hanging
 *
 */
static int rcraid_suspend_one(struct pci_dev *pdev, pm_message_t mesg)
{

    rc_softstate_t  *state;
    rc_adapter_t	*adapter;
    int             i;

    if (pdev == NULL)
    {
        return 0;
    }

#if 1
    //
    // Looks like a race condition somewhere... this delay
    // seems to solve the issue with suspend/hibernate cycles.
    // Placement of the delay seems to matter -- after
    // scsi_block_requests() doesn't work...
    //
    msleep(rc_suspend_delay);
#endif	/* 1 */

    //
    // Get adapter associated with this pci_dev
    //
    adapter = pci_get_drvdata(pdev);
    if (!adapter)
    {
	    return 0;
    }

    //
    // If not master adapter (instance 0), then do nothing
    //
    if (adapter->instance != 0)
    {
        return 0;
    }

    //
    // Inform the SCSI layer to stop sending requests down...
    //
    scsi_block_requests(rc_state.host_ptr);

    //
    // If we made it here then we're the master adapter/controller
    //

    state = &rc_state;

    rc_printk(RC_NOTE, RC_DRIVER_NAME ": suspend pdev %px\n",
        pdev);

    pdev->dev.power.power_state = mesg;

    rc_printk(RC_ERROR, "%s: event=%d \n",__FUNCTION__, mesg.event);

    //
    // Shutdown the core
    //

    rc_msg_suspend_work(adapter);

    rc_stop_all_threads();

    rc_msg_kill_tasklets(state);

    rc_event_shutdown();

    //
    // Shutdown the slave controllers first
    //

    //
    // For each slave (spanned) adapter
    //
    for (i = rc_state.num_hba -1; i > 0; i--)
    {
        adapter = rc_dev[i];

        pci_save_state(adapter->pdev);

        pci_disable_device(adapter->pdev);

        pci_set_power_state(adapter->pdev, pci_choose_state(adapter->pdev, mesg));

        adapter->pdev->dev.power.power_state = mesg;

        state->adapter_is_suspended |= (1 << adapter->instance);
    }

    //
    // (Re)get adapter associated with this pci_dev.
    //      We will be the master adapter if we're here...
    //
    adapter = pci_get_drvdata(pdev);

    //
    // and now shutdown the master...
    //
    pci_save_state(pdev);

	pci_disable_device(pdev);

	pci_set_power_state(pdev, pci_choose_state(pdev, mesg));

    state->adapter_is_suspended |= (1 << adapter->instance);

    return 0;
}


/*
 * rcraid_resume_one()
 *
 *     Called to resume from hibernation.  Should do as little work as possible
 *     Any long running tasks should be done in the tasklet to prevent Linux from hanging
 *     Note: Linux calls this even when going into hibernation
 *
 */
static int rcraid_resume_one(struct pci_dev *pdev)
{
    rc_adapter_t    *adapter;
    rc_softstate_t  *state;
    int             i;

	rc_printk(RC_NOTE, RC_DRIVER_NAME ": resume pdev %px\n",
		  pdev);

    //
    // Get adapter associated with this pci_dev
    //
    adapter = pci_get_drvdata(pdev);
    if (!adapter)
    {
	    return 0;
    }

    //
    // If not master adapter (instance 0), then do nothing
    //
    if (adapter->instance != 0)
    {
        return 0;
    }

    //
    // If we made it here then we're the master adapter/controller
    //

    state = &rc_state;

    //
    // Bring up the slave controllers first
    //

    //
    // For each slave (spanned) adapter
    //
    for (i = rc_state.num_hba - 1; i > 0; i--)
    {
        adapter = rc_dev[i];

        pci_set_power_state(adapter->pdev, PCI_D0);

        pci_enable_wake(adapter->pdev, PCI_D0, 0);

        pci_restore_state(adapter->pdev);

        if (pci_enable_device(adapter->pdev))
        {
            // IGNORE RESULT
        }

        if (adapter->version->start_func)
        {
            (*adapter->version->start_func)(adapter);
        }

        state->adapter_is_suspended &= ~(1 << adapter->instance);
    }

    //
    // (Re)get adapter associated with this pci_dev.
    //      We will be the master adapter if we're here...
    //
    adapter = pci_get_drvdata(pdev);

    //
    // Now bring up the master
    //
    pci_set_power_state(pdev, PCI_D0);

    pci_enable_wake(pdev, PCI_D0, 0);

	pci_restore_state(pdev);

    if (pcim_enable_device(pdev))
    {
        // IGNORE RESULT
    }

    //
    // and restart the core...
    //
    if (adapter->version->start_func)
    {
		(*adapter->version->start_func)(adapter);
    }

    rc_msg_init_tasklets(state);

    rc_start_all_threads();

    rc_event_init();

    schedule_delayed_work(&state->resume_work,250);

    scsi_unblock_requests(rc_state.host_ptr);

    state->adapter_is_suspended &= ~(1 << adapter->instance);

    return 0;
}
#endif  /* CONFIG_PM */

/*
 * rcraid_shutdown_one()
 *
 *     Dereferences the adapter structure from the device and calls
 *     the interal shutdown routine. Called by the PCI driver layer
 *     once for each adapter card.
 */
void
rcraid_shutdown_one(
	struct pci_dev *pdev
	)
{
	rc_adapter_t  *adapter = pci_get_drvdata(pdev);
	if (rc_state.state & USE_OSIC) {
		rc_remove_proc();
		rc_shutdown_host(rc_state.host_ptr);
	}

	rc_shutdown_adapter(adapter);
}

#ifdef RC_AHCI_SUPPORT
#define ICH6_REG_OFFSET_GHC     0x04    // Global HBA Control register
#define AHCI_GHC_IE             (1 << 1)  // global IRQ enable
/*
 * disable HW interrupts on all ports on an adapter
 */
static inline void
rc_ahci_disable_irq(rc_adapter_t *adapter)
{
	void __iomem *mmio = adapter->hardware.vaddr;
	u32 ctl;

	ctl = readl(mmio + ICH6_REG_OFFSET_GHC );
	ctl &= ~AHCI_GHC_IE;
	writel(ctl, mmio + ICH6_REG_OFFSET_GHC);
	readl(mmio + ICH6_REG_OFFSET_GHC); /* flush */
}

/*
 * Additions for Intel/AMD (and other) AHCI motherboards.
 */
int rc_ahci_init(rc_adapter_t *adapter)
{
	rc_ahci_disable_irq(adapter);

	// try using msi (0 return means success)
	if (pci_enable_msi(adapter->pdev)) {
		rc_printk(RC_WARN, "rc_ahci_init: pci_enable_msi failed\n");
	} else {
		adapter->hardware.ismsi = 1;
        }

	return 0;
}

int rc_ahci_start(rc_adapter_t *adapter)
{
	u16 cmd;

	// The RAIDCore BIOS (and INT13 driver) may turn off access to
	// the chip by clearing PCI Bus Master, Memory, and IO bits.  These
	// need to be reenabled so the driver has access to the chip.
	pci_set_master(adapter->pdev);
	pci_read_config_word(adapter->pdev, PCI_COMMAND, &cmd);
	cmd |= (PCI_COMMAND_IO | PCI_COMMAND_MEMORY);
	pci_write_config_word(adapter->pdev, PCI_COMMAND, cmd);

	return 0;
}

int rc_ahci_shutdown(rc_adapter_t *adapter)
{
	rc_printk(RC_WARN, "%s\n",__FUNCTION__);
    // devm_ functions handle the shutdown. If
    // we do this, we'll see kernel errors/bugs.
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,8,1)
	if  (adapter->hardware.irq) {
		free_irq(adapter->hardware.irq, adapter);
		adapter->hardware.irq = 0;
		if (adapter->hardware.ismsi) {
		  pci_disable_msi(adapter->pdev);
		  adapter->hardware.ismsi = 0;
		}
	}
#endif  /* LINUX_VERSION_CODE < KERNEL_VERSION(3,8,1) */
	return 0;
}

irqreturn_t rc_ahci_isr(int irq, void *arg, struct pt_regs *regs)
{
	rc_adapter_t    *adapter = (rc_adapter_t *) arg;

    // We disable interrupts in the bottom driver now, doing it in 2 places is bad
    // especially if the bottom driver does not see any interrupts when it checks
	//rc_ahci_disable_irq(adapter);
    if (rc_state.state & USE_OSIC) rc_msg_isr(adapter);

	return IRQ_HANDLED;
}
#endif // RC_AHCI_SUPPORT

// nvme functions

static inline void
rc_nvme_disable_irq(rc_adapter_t *adapter)
{
	rc_printk(RC_DEBUG, "%s\n",__FUNCTION__);
}

int rc_nvme_init(rc_adapter_t *adapter)
{
	rc_printk(RC_DEBUG, "%s\n",__FUNCTION__);

	rc_nvme_disable_irq(adapter);

    if (adapter->pdev->msix_cap)
    {
        adapter->hardware.msix_ent.entry = 0;

        if (pci_enable_msix_exact(adapter->pdev, &adapter->hardware.msix_ent, 1))
        {
            // Non-zero - failed
            dev_err(&adapter->pdev->dev, "rc_nvme_init: pci_enable_msix_exact failed\n");
        } else {
            // Zero - succeeded
            //dev_err(&adapter->pdev->dev, "rc_nvme_init: pci_enable_msi_exact succeeded - vector %d\n", adapter->hardware.msix_ent.vector);
        }
    } else if (adapter->pdev->msi_cap) {
	    if (pci_enable_msi(adapter->pdev)) {
		    dev_err(&adapter->pdev->dev, "rc_nvme_init: pci_enable_msi failed\n");
	    } else {
		    adapter->hardware.ismsi = 1;
        }
    }

	return 0;
}

int rc_nvme_start(rc_adapter_t *adapter)
{
	u16 cmd;

	rc_printk(RC_DEBUG, "%s\n",__FUNCTION__);

	// The RAIDCore BIOS (and INT13 driver) may turn off access to
	// the chip by clearing PCI Bus Master, Memory, and IO bits.  These
	// need to be reenabled so the driver has access to the chip.
	pci_set_master(adapter->pdev);
	pci_read_config_word(adapter->pdev, PCI_COMMAND, &cmd);
	cmd |= (PCI_COMMAND_IO | PCI_COMMAND_MEMORY);
	pci_write_config_word(adapter->pdev, PCI_COMMAND, cmd);

	return 0;
}

int rc_nvme_shutdown(rc_adapter_t *adapter)
{
    // devm_ functions handle the shutdown. If
    // we do this, we'll see kernel errors/bugs.
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,8,1)
	rc_printk(RC_WARN, "%s\n",__FUNCTION__);
	if  (adapter->hardware.irq) {
		free_irq(adapter->hardware.irq, adapter);
		adapter->hardware.irq = 0;
		if (adapter->hardware.ismsi) {
		  pci_disable_msi(adapter->pdev);
		  adapter->hardware.ismsi = 0;
		}
	}
#endif  /* LINUX_VERSION_CODE < KERNEL_VERSION(3,8,1) */
	return 0;
}

irqreturn_t rc_nvme_isr(int irq, void *arg, struct pt_regs *regs)
{
	rc_adapter_t    *adapter = (rc_adapter_t *) arg;

    // We disable interrupts in the bottom driver now, doing it in 2 places is bad
    // especially if the bottom driver does not see any interrupts when it checks
	//rc_ahci_disable_irq(adapter);
    if (rc_state.state & USE_OSIC) rc_msg_isr(adapter);

	return IRQ_HANDLED;
}

/*
 * rc_queue_cmd()
 *
 * Queues a SCSI command
 */


int rc_queue_cmd_lck (struct scsi_cmnd * scp, void (*CompletionRoutine) (struct scsi_cmnd *))
{
	scp->scsi_done = CompletionRoutine;
	//#define FAIL_ALL_IO 0

#ifdef FAIL_ALL_IO
	scp->result = DID_NO_CONNECT << 16;
	scp->scsi_done(scp);
	return 0;
#endif

#if 0
	rc_printk(RC_DEBUG2, "\trc_queue_cmd B/T/L %d/%d/%d\n",
		  scp->device->channel, scp->device->id, scp->device->lun);
#endif

    // If we are suspended(controller is not restarted) block any IO from coming in
    if (  (rc_state.is_suspended == 1) || ( (rc_state.state & SHUTDOWN) == SHUTDOWN) ) {
        return SCSI_MLQUEUE_DEVICE_BUSY;
    }

	return(rc_msg_send_srb(scp));
}

/*
 * rc_eh_abort_cmd()
 *
 *  Abort command if possible.
 */
int
rc_eh_abort_cmd (struct scsi_cmnd * scp)
{
	rc_srb_t *srb;

	rc_printk(RC_ERROR, "rc_eh_abort_cmd: scp: 0x%px bus %d target %d\n",
		  scp, scp->device->channel, scp->device->id);
	// rc_config_debug = 1;

	srb = (rc_srb_t *)scp->SCp.ptr;
	if (srb != NULL) {
		rc_printk(RC_DEBUG, "\tsrb: 0x%px seq_num %d function %x status %x "
			  "flags %x b/t/l %d/%d/%d\n", srb, srb->seq_num, srb->function,
			  srb->status, srb->flags, srb->bus, srb->target, srb->lun);
		srb->scsi_context = NULL;
		scp->SCp.ptr = NULL;
	} else {
		rc_printk(RC_WARN, "rc_eh_abort_cmd: srb already completed\n");
		// most likely here because we already processed srb
		// (rc_msg_srb_complete)
		return (FAILED);
	}

	rc_dump_scp(scp);

	return (SUCCESS);
}


int
rc_eh_dev_reset (struct scsi_cmnd *scp)
{

	rc_printk(RC_ERROR, "rc_eh_dev_reset: scp 0x%px bus %d target %d\n",
		  scp, scp->device->channel, scp->device->id);
	rc_config_debug = 1;
	rc_dump_scp(scp);

	return (SUCCESS);
}

int
rc_eh_bus_reset (struct scsi_cmnd *scp)
{
	rc_printk(RC_ERROR, "rc_eh_bus_reset: scp 0x%px bus %d target %d\n",
		  scp, scp->device->channel, scp->device->id);
	rc_config_debug = 1;
	rc_dump_scp(scp);

	return (SUCCESS);
}

int
rc_eh_hba_reset (struct scsi_cmnd *scp)
{
	rc_printk(RC_ERROR, "rc_eh_hba_reset: scp 0x%px bus %d target %d\n",
		  scp, scp->device->channel, scp->device->id);
	rc_dump_scp(scp);

	return (SUCCESS);
}

//=============================================================================
//

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,10,0)

static int
rc_proc_read(char *buf, char **start, off_t offset, int buf_size, int *peof, void *data);

static int
rc_proc_write(struct file *file, const char __user *buf, unsigned long count, void *data);

void rc_init_proc(void)
{
    rc_proc_entry_t *pe;
    struct proc_dir_entry *pde;
    int i;

    if (!driver_template.proc_dir)
    {
        rc_printk(RC_ERROR, "rc_info: no entry for /proc/rcraid yet\n");
    } else {
        for (i = 0, pe = rc_proc_dir_entries;
             i < sizeof(rc_proc_dir_entries) / sizeof(rc_proc_entry_t);
             ++i, ++pe) {
            if (!(pde = create_proc_entry(pe->name, pe->mode,
                              driver_template.proc_dir)))
                rc_printk(RC_ERROR, "rc_info: can't create entry for "
                      "/proc/rcraid/%s\n", pe->name);
            else {
                pe->proc_dir = pde;
                pde->read_proc = rc_proc_read;
                pde->write_proc = rc_proc_write;
                pde->data = (void *)pe;
            }
        }
    }
}

#else

static int rc_proc_show_int(struct seq_file *sfile, void *v)
{
    seq_printf(sfile, "%d\n", *((int *) sfile->private));
    return 0;
}

static int rc_proc_show_hex(struct seq_file *sfile, void *v)
{
    seq_printf(sfile, "%X\n", *((int *) sfile->private));
    return 0;
}

static ssize_t
rc_proc_write_dipm(struct file *file, const char __user *buffer,
                   size_t count, loff_t *off)
{
    int			    err;
    unsigned long	num;

    if (!capable(CAP_SYS_ADMIN) || !capable(CAP_SYS_RAWIO))
        return -EACCES;
    err = kstrtoul_from_user(buffer, count, 16, &num);
    if (err)
        return err;
    if (num >= 0 && num <= 0xFFFFFFFF)
        RC_EnableDIPM = num;
    return count;
}

static int
rc_proc_dipm_open(struct inode *inode, struct file *file)
{
    return single_open(file, rc_proc_show_hex, &RC_EnableDIPM);
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(5,6,0)
static const struct file_operations rc_proc_dipm_fops = {
    .owner      = THIS_MODULE,
    .open       = rc_proc_dipm_open,
    .read       = seq_read,
    .llseek     = seq_lseek,
    .write	    = rc_proc_write_dipm,
    .release    = single_release,
};
#else
static const struct proc_ops rc_proc_dipm_fops = {
	.proc_open	= rc_proc_dipm_open,
	.proc_read	= seq_read,
	.proc_lseek	= seq_lseek,
	.proc_write	= rc_proc_write_dipm,
	.proc_release	= single_release,
};
#endif


static ssize_t
rc_proc_write_hipm(struct file *file, const char __user *buffer,
                   size_t count, loff_t *off)
{
    int			    err;
    unsigned long	num;

    if (!capable(CAP_SYS_ADMIN) || !capable(CAP_SYS_RAWIO))
        return -EACCES;
    err = kstrtoul_from_user(buffer, count, 16, &num);
    if (err)
        return err;
    if (num >= 0 && num <= 0xFFFFFFFF)
        RC_EnableHIPM = num;
    return count;
}

static int
rc_proc_hipm_open(struct inode *inode, struct file *file)
{
    return single_open(file, rc_proc_show_hex, &RC_EnableHIPM);
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(5,6,0)
static const struct file_operations rc_proc_hipm_fops = {
    .owner      = THIS_MODULE,
    .open       = rc_proc_hipm_open,
    .read       = seq_read,
    .llseek     = seq_lseek,
    .write	    = rc_proc_write_hipm,
    .release    = single_release,
};
#else
static const struct proc_ops rc_proc_hipm_fops = {
	.proc_open	= rc_proc_hipm_open,
	.proc_read	= seq_read,
	.proc_lseek	= seq_lseek,
	.proc_write	= rc_proc_write_hipm,
	.proc_release	= single_release,
};
#endif


static ssize_t
rc_proc_write_debug(struct file *file, const char __user *buffer,
                    size_t count, loff_t *off)
{
    int             err;
    unsigned long   num;

    if (!capable(CAP_SYS_ADMIN) || !capable(CAP_SYS_RAWIO))
        return -EACCES;
    err = kstrtoul_from_user(buffer, count, 0, &num);
    if (err)
        return err;
    if (num >= 0 && num < RC_TAIL)
    {
        rc_send_arg_t   args;

        rc_msg_level = num;
        memset(&args, 0, sizeof(args));
        args.call_type = RC_CTS_SET_MSG_LEVEL;
        args.u.max_print_severity = rc_msg_level;
        rc_send_msg(&args);
    }
    return count;
}

static int
rc_proc_debug_show(struct seq_file *sfile, void *v)
{
    seq_printf(sfile, "%d %d %d\n", rc_msg_level, RC_PANIC, RC_TAIL - 1);
    return 0;
}

static int
rc_proc_debug_open(struct inode *inode, struct file *file)
{
    return single_open(file, rc_proc_debug_show, NULL);
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(5,6,0)
static const struct file_operations rc_proc_debug_fops = {
    .owner      = THIS_MODULE,
    .open       = rc_proc_debug_open,
    .read       = seq_read,
    .llseek     = seq_lseek,
    .write      = rc_proc_write_debug,
    .release    = single_release,
};
#else
static const struct proc_ops rc_proc_debug_fops = {
	.proc_open	= rc_proc_debug_open,
	.proc_read	= seq_read,
	.proc_lseek	= seq_lseek,
	.proc_write	= rc_proc_write_debug,
	.proc_release	= single_release,
};
#endif


static ssize_t
rc_proc_write_an(struct file *file, const char __user *buffer,
                 size_t count, loff_t *off)
{
    int			err;
    unsigned long	num;

    if (!capable(CAP_SYS_ADMIN) || !capable(CAP_SYS_RAWIO))
        return -EACCES;
    err = kstrtoul_from_user(buffer, count, 16, &num);
    if (err)
        return err;
    if (num >= 0 && num < 0xFFFFFFFF)
    {
        rc_send_arg_t   args;

        RC_EnableAN = num;
        memset(&args, 0, sizeof(args));
        args.call_type = RC_CTS_CHANGE_PARAM;
        args.u.change_param.param = RC_CTS_PARAM_AN;
        args.u.change_param.value = RC_EnableAN;
        rc_send_msg(&args);
    }
    return count;
}

static int
rc_proc_an_open(struct inode *inode, struct file *file)
{
    return single_open(file, rc_proc_show_hex, &RC_EnableAN);
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(5,6,0)
static const struct file_operations rc_proc_an_fops = {
    .owner      = THIS_MODULE,
    .open       = rc_proc_an_open,
    .read       = seq_read,
    .llseek     = seq_lseek,
    .write	= rc_proc_write_an,
    .release    = single_release,
};
#else
static const struct proc_ops rc_proc_an_fops = {
	.proc_open	= rc_proc_an_open,
	.proc_read	= seq_read,
	.proc_lseek	= seq_lseek,
	.proc_write	= rc_proc_write_an,
	.proc_release	= single_release,
};
#endif


static ssize_t
rc_proc_write_zpodd(struct file *file, const char __user *buffer,
                    size_t count, loff_t *off)
{
    int			err;
    unsigned long	num;

    if (!capable(CAP_SYS_ADMIN) || !capable(CAP_SYS_RAWIO))
        return -EACCES;
    err = kstrtoul_from_user(buffer, count, 16, &num);
    if (err)
        return err;
    if (num >= 0 && num <= 1)
    {
        rc_send_arg_t   args;

        //RC_EnableZPODD = num;
        memset(&args, 0, sizeof(args));
        args.call_type = RC_CTS_CHANGE_PARAM;
        args.u.change_param.param = RC_CTS_PARAM_ZPODD;
        args.u.change_param.value = RC_EnableZPODD;
        rc_send_msg(&args);
    }
    //RC_EnableZPODD = num;
    return count;
}

static int
rc_proc_zpodd_open(struct inode *inode, struct file *file)
{
    return single_open(file, rc_proc_show_hex, &RC_EnableZPODD);
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(5,6,0)
static const struct file_operations rc_proc_zpodd_fops = {
    .owner      = THIS_MODULE,
    .open       = rc_proc_zpodd_open,
    .read       = seq_read,
    .llseek     = seq_lseek,
    .write	    = rc_proc_write_zpodd,
    .release    = single_release,
};
#else
static const struct proc_ops rc_proc_zpodd_fops = {
	.proc_open	= rc_proc_zpodd_open,
	.proc_read	= seq_read,
	.proc_lseek	= seq_lseek,
	.proc_write	= rc_proc_write_zpodd,
	.proc_release	= single_release,
};
#endif


static ssize_t
rc_proc_write_delay(struct file *file, const char __user *buffer,
                    size_t count, loff_t *off)
{
    int			        err;
    unsigned long       num;

    if (!capable(CAP_SYS_ADMIN) || !capable(CAP_SYS_RAWIO))
        return -EACCES;
    err = kstrtoul_from_user(buffer, count, 0, &num);
    if (err)
        return err;
    rc_suspend_delay = num;
    return count;
}

static int
rc_proc_delay_open(struct inode *inode, struct file *file)
{
    return single_open(file, rc_proc_show_int, &rc_suspend_delay);
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(5,6,0)
static const struct file_operations rc_proc_delay_fops = {
    .owner      = THIS_MODULE,
    .open       = rc_proc_delay_open,
    .read       = seq_read,
    .llseek     = seq_lseek,
    .write	    = rc_proc_write_delay,
    .release    = single_release,
};
#else
static const struct proc_ops rc_proc_delay_fops = {
	.proc_open	= rc_proc_delay_open,
	.proc_read	= seq_read,
	.proc_lseek	= seq_lseek,
	.proc_write	= rc_proc_write_delay,
	.proc_release	= single_release,
};
#endif


static int
rc_proc_version_show(struct seq_file *sfile, void *v)
{
    seq_printf(sfile, "V%s %s\n", RC_DRIVER_VERSION, RC_BUILD_NUMBER);
    return 0;
}

static int
rc_proc_version_open(struct inode *inode, struct file *file)
{
    return single_open(file, rc_proc_version_show, NULL);
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(5,6,0)
static const struct file_operations rc_proc_version_fops = {
    .owner      = THIS_MODULE,
    .open       = rc_proc_version_open,
    .read       = seq_read,
    .llseek     = seq_lseek,
    .release    = single_release,
};
#else
static const struct proc_ops rc_proc_version_fops = {
	.proc_open	= rc_proc_version_open,
	.proc_read	= seq_read,
	.proc_lseek	= seq_lseek,
	.proc_release	= single_release,
};
#endif


static int
rc_proc_stats_show(struct seq_file *sfile, void *v)
{
    char *kbuf;

    kbuf = kmalloc(8192, GFP_KERNEL);

    if (kbuf)
    {
	    int	ret;

        ret = rc_msg_stats(kbuf, 8192);
	    ret = rc_mop_stats((kbuf + ret), 8192 - ret);

        seq_printf(sfile, "%s\n", kbuf);

	    kfree(kbuf);
    }
    return 0;
}

static int
rc_proc_stats_open(struct inode *inode, struct file *file)
{
    return single_open(file, rc_proc_stats_show, NULL);
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(5,6,0)
static const struct file_operations rc_proc_stats_fops = {
	.owner      = THIS_MODULE,
	.open       = rc_proc_stats_open,
	.read       = seq_read,
	.llseek     = seq_lseek,
	.release    = single_release,
};
#else
static const struct proc_ops rc_proc_stats_fops = {
	.proc_open	= rc_proc_stats_open,
	.proc_read	= seq_read,
	.proc_lseek	= seq_lseek,
	.proc_release	= single_release,
};
#endif


static struct proc_dir_entry *proc_parent = NULL;

static const struct rc_proc_entry {
    const char                    *name;
#if LINUX_VERSION_CODE < KERNEL_VERSION(5,6,0)
    const struct file_operations  *fops;
#else
    const struct proc_ops  *fops;
#endif
} rc_proc_entries[] = {
    { "dipm", &rc_proc_dipm_fops },
    { "hipm", &rc_proc_hipm_fops },
    { "version", &rc_proc_version_fops },
    { "debug", &rc_proc_debug_fops },
    { "an", &rc_proc_an_fops },
    { "zpodd", &rc_proc_zpodd_fops },
    { "suspend_delay", &rc_proc_delay_fops },
    { "stats", &rc_proc_stats_fops },
    { NULL, NULL }
};

void rc_init_proc(void)
{
    // Only install once
    if (proc_parent == NULL)
    {
        proc_parent = proc_mkdir("rcraid", NULL);

        if (proc_parent)
        {
            const struct rc_proc_entry    *rpe = rc_proc_entries;

            while (rpe && rpe->name)
            {
                struct proc_dir_entry   *pde;

                pde = proc_create(rpe->name, 0, proc_parent, rpe->fops);
                rpe++;
            }
        }
    }
}

/*
 *  rc_remove_proc()
 *
 *      Removes all of the proc filesystem entries.
 *
 */
void
rc_remove_proc(void)
{
    if (proc_parent)
    {
        const struct rc_proc_entry	*rpe = rc_proc_entries;

        while (rpe && rpe->name)
        {
            remove_proc_entry(rpe->name, proc_parent);
            rpe++;
        }

        remove_proc_entry("rcraid", NULL);

        proc_parent = NULL;
    }
}

#endif  /* LINUX_VERSION_CODE >= KERNEL_VERSION(3,10,0) */
//
//=============================================================================

/*
 * rc_info()
 *
 *    Returns the host adapter name
 *    Also creates any additional /proc/rcraid/... entries that we need.
 *    This is done here instead of in rc_detect() or bc*init() because those
 *    are all called before scsi_register_host() creates /proc/rcraid.
 */
const char *
rc_info (struct Scsi_Host *host_ptr)
{

    rc_init_proc();

	return(VER_COMPANYNAME_STR " " VER_PRODUCT_STR);
}


/*
 * rc_bios_params;
 *
 *  Return the Heads/Sectors/Cylinders BIOS Disk Parameters for Disk.
 *  The default disk geometry is 64 Heads, 32 Sectors, and the appropriate
 *  number of Cylinders so as not to exceed drive capacity.  In order for
 *  disks equal to or larger than 1 GB to be addressable by the BIOS
 *  without exceeding the BIOS limitation of 1024 Cylinders, Extended
 *  Translation should be enabled.   With Extended Translation enabled,
 *  drives between 1 GB inclusive and 2 GB exclusive are given a disk
 *  geometry of 128 Heads and 32 Sectors, and drives above 2 GB inclusive
 *  are given a disk geometry of 255 Heads and 63 Sectors.  However, if
 *  the BIOS detects that the Extended Translation setting does not match
 *  the geometry in the partition table, then the translation inferred
 *  from the partition table will be used by the BIOS, and a warning may
 *  be displayed.
 */
int
rc_bios_params (struct scsi_device *sdev,
		struct block_device *bdev,
		sector_t capacity,
		int geom[])
{
	rc_bios_disk_parameters_t *param;

	param = (rc_bios_disk_parameters_t *) geom;

	rc_printk (RC_DEBUG, "rc_bios_disk_parameters\n");

	// Assuming extended translation is enabled - #REVISIT#
	if (capacity >= 2 * 1024 * 1024)    // 1 GB in 512 byte Sectors
	{
		if (capacity >= 4 * 1024 * 1024)    // 2 GB in 512 byte sectors
		{
			param->heads = 255;
			param->sectors = 63;
		} else {
			param->heads = 128;
			param->sectors = 32;
		}
	} else {
		param->heads = 64;
		param->sectors = 32;
	}

	sector_div(capacity, param->heads * param->sectors);
	param->cylinders = capacity;

	return (0);
}

/*
 * rc_slave_cfg()
 *
 *  Selects queue depths for each Target Device based on the host adapter's
 *  total capacity and the queue depth supported by the target Device.
 *  A queue depth of one automatically disables tagged queueing.
 */
static int
rc_slave_cfg(struct scsi_device *sdev)
{
    rc_send_arg_t   args;
    rc_softstate_t    *state;

    memset(&args, 0, sizeof(args));
    state = &rc_state;

    spin_lock(&state->osic_lock);
    check_lock(state);
    state->osic_locked = 1;
    state->osic_lock_holder = "rc_slave_cfg";


    args.call_type = RC_CTS_SCSI_INFO;
    args.u.rc_scsi_info.bus = sdev->channel;
    args.u.rc_scsi_info.target = sdev->id;
    args.u.rc_scsi_info.lun = sdev->lun;
    rc_send_msg(&args);

    state->osic_locked = 0;
    spin_unlock(&state->osic_lock);
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 10, 0)
	if (sdev->tagged_supported)
		scsi_adjust_queue_depth(sdev, MSG_ORDERED_TAG, tag_q_depth);
	else
		scsi_adjust_queue_depth(sdev, 0, 1);
#endif  /* LINUX_VERSION_CODE < KERNEL_VERSION(3, 10, 0) */

    if ((args.u.rc_scsi_info.value & 1) == 1)
    {
        sdev->use_10_for_ms = 1;
        //
        // EXT-46803
        //     Address issue with ill behaved app (brasero)
        // that's using Block I/O parameter to do bypass I/O.
        //
        // Set max sectors per transfer to 256 for ATAPI devices.
        //
        blk_queue_max_hw_sectors(sdev->request_queue, 256);
    }

	return 0;
}


#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 1, 0)
int
rc_ioctl (struct scsi_device * scsi_dev_ptr,
	  int cmd,
	  void *arg)
#else
int
rc_ioctl (struct scsi_device * scsi_dev_ptr,
	  unsigned int cmd,
	  void *arg)
#endif
{
	char direction = 'w';

	if (_IOC_DIR(cmd) & _IOC_READ)
		direction = 'r';

	rc_printk(RC_DEBUG, "rc_ioctl: type %i nr %i dir %c\n", _IOC_TYPE(cmd),
		  _IOC_NR(cmd), direction);
	return(-ENOTTY);
}





static char rc_print_buf[1024];
void
rc_printk(int flag, const char *fmt, ...)
{
	static int newline_in_last_fmt = 1;
	va_list args;

	va_start(args, fmt);
	(void) vsnprintf(rc_print_buf, sizeof(rc_print_buf), fmt, args);
	va_end(args);

	if (flag == 0) {
		panic(rc_print_buf);
		return;
	}

	if (flag > rc_msg_level)
		return;

	flag = flag > 7 ? 7 : flag;

	if (newline_in_last_fmt)
		printk("<%d>%s", flag, rc_print_buf);
	else
		printk("%s", rc_print_buf);

	newline_in_last_fmt = strchr(fmt, '\n') ? 1 : 0;
}

void
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,14,0)
rc_timeout_done(unsigned long data)
{
	rc_softstate_t *state;

	state = (rc_softstate_t *)data;
	init_timer(&state->rc_timeout);
	up(&state->rc_timeout_sema);
}
#else
rc_timeout_done(struct timer_list *t)
{
	rc_softstate_t	*state = from_timer(state, t, rc_timeout);
	timer_setup(&state->rc_timeout, rc_timeout_done, 0);
	up(&state->rc_timeout_sema);
}
#endif

void
rc_timeout(int to)
{
	rc_softstate_t *state;

	state = &rc_state;
	/*
	 * set up timeout
	 */

#if LINUX_VERSION_CODE < KERNEL_VERSION(4,14,0)
	init_timer(&state->rc_timeout);
	state->rc_timeout.expires = jiffies  + to;
	state->rc_timeout.data = (unsigned long)state;
	state->rc_timeout.function = rc_timeout_done;
	add_timer(&state->rc_timeout);
#else
	timer_setup(&state->rc_timeout, rc_timeout_done, 0);
	mod_timer(&state->rc_timeout, jiffies + to);
	printk("%s(): timer_setup(): state %px, timeout %ld\n", __FUNCTION__, state, jiffies + to);
#endif
	down(&state->rc_timeout_sema);
}


void
rc_dump_scp(struct scsi_cmnd * scp)
{

	unsigned char cmd;
	rc_scb_t *scb;
	int bus, target, lun;
	unsigned int lba, sector_count;
	unsigned long long  lba16;
	int         i;
	dma_addr_t      dma_addr;
	struct scatterlist  *sg;

	if (scp == NULL)
		return;
	else return;

	scb = (rc_scb_t *) scp->cmnd;
	cmd = scb->scsi6.opcode;

	bus = scp->device->channel;
	target = scp->device->id;
	lun = scp->device->lun;

	rc_printk(RC_DEBUG2, "SCP/SCSI command 0x%02x B/T/L %d/%d/%d\n", cmd,
		  bus, target, lun);

	switch (cmd) {
	case RC_WRITE_6:
	case RC_READ_6:
		lba = ((scb->scsi6.addr[0] & 0x1f) << 16) | (scb->scsi6.addr[1] << 8)
			| scb->scsi6.addr[2];
		sector_count = scb->scsi6.len;
		rc_printk(RC_DEBUG, "lba: %d len %d\n", lba, sector_count);
		break;

	case RC_WRITE_10:
	case RC_READ_10:
		lba = (scb->scsi10.addr[0] << 24) | (scb->scsi10.addr[1] << 16)
			| (scb->scsi10.addr[2] << 8) | scb-> scsi10.addr[3];
		sector_count = (scb->scsi10.len[0] << 8) | scb->scsi10.len[1];
		rc_printk(RC_DEBUG, "lba: %d len %d\n", lba, sector_count);
		break;

	case RC_WRITE_16:
	case RC_READ_16:
		lba16 = ((uint64_t)scb->scsi16.addr[0] << 56)
			| ((uint64_t)scb->scsi16.addr[1] << 48)
			| ((uint64_t)scb->scsi16.addr[2] << 40)
			| ((uint64_t)scb->scsi16.addr[3] << 32)
			| ((uint64_t)scb->scsi16.addr[4] << 24)
			| ((uint64_t)scb->scsi16.addr[5] << 16)
			| ((uint64_t)scb->scsi16.addr[6] << 8)
			|  (uint64_t)scb->scsi16.addr[7];
		sector_count = (scb->scsi16.len[0] << 8) | scb->scsi16.len[1];
		rc_printk(RC_DEBUG, "lba: %lld len %d\n", lba16, sector_count);
		break;

	default:
		break;
	}

	rc_printk(RC_DEBUG2, "    scp: 0x%px sg 0x%px, sg_count %d, len %d\n",
		  scp, scsi_sglist(scp), scsi_sg_count(scp), scsi_bufflen(scp));
	scsi_for_each_sg(scp, sg, scsi_sg_count(scp), i) {
		dma_addr = sg_phys(sg);
		rc_printk(RC_DEBUG,
#if defined(CONFIG_HIGHMEM64G) ||  defined(CONFIG_X86_64)
			  "    page: 0x%px  offset: 0x%x addr: 0x%016llx len %d\n",
#else
			  "    page: 0x%px  offset: 0x%x addr: 0x%08x len %d\n",
#endif
			  sg_page(sg), sg->offset, dma_addr, sg->length);
	}
	rc_printk(RC_DEBUG2,"\n");
}

static int rc_notify_reboot(struct notifier_block *this, unsigned long code,
			    void *x)
{
	if ((code == SYS_DOWN) || (code == SYS_HALT) || (code == SYS_POWER_OFF))
		printk(KERN_INFO "%s: stopping all RAIDCore (tm) devices.\n", __FUNCTION__);
	return NOTIFY_DONE;
}

static struct notifier_block rc_notifier = {
	.notifier_call  = rc_notify_reboot,
	.next           = NULL,
	.priority       = INT_MAX, /* before any real devices */
};


MODULE_DEVICE_TABLE(pci, rcraid_id_tbl);

static struct pci_driver rcraid_pci_driver = {
	.name       = RC_DRIVER_NAME,
	.id_table   = rcraid_id_tbl,
	.probe      = rcraid_probe_one,
	.shutdown   = rcraid_shutdown_one,

#ifdef  CONFIG_PM
	.suspend		= rcraid_suspend_one,
	.resume			= rcraid_resume_one,
#endif  /* CONFIG_PM */

};

#if LINUX_VERSION_CODE <= KERNEL_VERSION(3, 17, 0)
u32
rc_ahci_regread(void *context, u32 offset)
{
#if !defined(CONFIG_64BIT)
    return *((volatile u32 *) (u32) offset);
#else
    return *((volatile u32 *) (u64) offset);
#endif  /* !defined(CONFIG_64BIT) */
}

void
rc_ahci_regwrite(void *context, u32 offset, u32 value)
{
#if !defined(CONFIG_64BIT)
    *((volatile u32 *) (u32) offset) = value;
#else
    *((volatile u32 *) (u64) offset) = value;
#endif  /* !defined(CONFIG_64BIT) */
}
#else
u32
rc_ahci_regread(void *context, u32 offset)
{
    rc_adapter_t    *adapter = (rc_adapter_t *) context;
    void __iomem    *mmio;

    if (adapter == NULL)
        return (u32) -1;

    mmio = adapter->hardware.vaddr;

    if (mmio)
        return readl(mmio + offset);
    else
        return (u32) -1;
}

void
rc_ahci_regwrite(void *context, u32 offset, u32 value)
{
    rc_adapter_t    *adapter = (rc_adapter_t *) context;
    void __iomem    *mmio = NULL;

    if (adapter)
        mmio = adapter->hardware.vaddr;

    if (mmio)
        writel(value, mmio + offset);
}
#endif  /* LINUX_VERSION_CODE < KERNEL_VERSION(3, 17, 0) */

static struct ctl_table rcraid_table[] = {
	{
	  .procname	= "dipm",
	  .data		= &RC_EnableDIPM,
	  .maxlen	= sizeof(unsigned int),
	  .mode		= 0644,
	  .proc_handler	= &proc_dointvec
	},
	{
	  .procname	= "hipm",
	  .data		= &RC_EnableHIPM,
	  .maxlen	= sizeof(unsigned int),
	  .mode		= 0644,
	  .proc_handler	= &proc_dointvec
	},
	{
	  .procname	= "an",
	  .data		= &RC_EnableAN,
	  .maxlen	= sizeof(unsigned int),
	  .mode		= 0644,
	  .proc_handler	= &proc_dointvec
	},
    {
	  .procname	= "ncq",
	  .data		= &RC_EnableNCQ,
	  .maxlen	= sizeof(unsigned int),
	  .mode		= 0644,
	  .proc_handler	= &proc_dointvec
	},
    {
      .procname = "zpodd",
      .data     = &RC_EnableZPODD,
	  .maxlen	= sizeof(unsigned int),
	  .mode		= 0644,
	  .proc_handler	= &proc_dointvec
	},
    {
      .procname = "suspend_delay",
      .data     = &rc_suspend_delay,
	  .maxlen	= sizeof(unsigned int),
	  .mode		= 0644,
	  .proc_handler	= &proc_dointvec
	},
    {
      .procname = "version",
      .data     = version_string,
      .maxlen   = VERSION_STRING_LEN,
      .mode     = 0644,
      .proc_handler = &proc_dostring,
    },
	{ }
};

static struct ctl_table rcraid_dir_table[] = {
	{ .procname	= "rcraid",
	  .mode		= 0555,
	  .child	= rcraid_table },
	{ }
};

//
// Running into issues (first seen on Debian) where '/proc/scsi' doesn't exist.
// This results in kernel module panics which can leave things somewhat
// unresolved. There's no interface in procfs that allows one to determine
// if a node already exists and the one method found (using vfs_path_lookup())
// relies on GPL symbols which are not compatible with our proprietary module.
//
// Move the tree(s) from scsi/rcraid to rcraid to avoid this issue.
//

#if 0
static struct ctl_table rcraid_scsi_dir_table[] = {
	{ .procname	= "scsi",
	  .mode		= 0555,
	  .child	= rcraid_dir_table },
	{ }
};
#endif  /* 0 */

static struct ctl_table rcraid_root_table[] = {
	{ .procname	= "dev",
	  .mode		= 0555,
#if 0
	  .child	= rcraid_scsi_dir_table },
#else
	  .child	= rcraid_dir_table },
#endif  /* 0 */
	{ }
};

static struct ctl_table_header *rcraid_sysctl_hdr;

static int __init rcraid_init(void)
{
	int err = 0;
    int retries = 100;

	/*
	 * make sure this is NULL, use this to check if the core
	 * finishes init and thus registers the config device
	 */
	rccfg_api_dev.this_device = NULL;
	rc_init_module();

	register_reboot_notifier(&rc_notifier);
	err = pci_register_driver(&rcraid_pci_driver);

	if (err != 0)
		return err;

    //
    // Setup version string for sysctl access
    //
    memset(version_string, 0, VERSION_STRING_LEN);
    snprintf(version_string, VERSION_STRING_LEN, "V%s %s", RC_DRIVER_VERSION, RC_BUILD_NUMBER);
	rcraid_sysctl_hdr = register_sysctl_table(rcraid_root_table);
	if (rcraid_sysctl_hdr == NULL)
		return -ENOMEM;

    // Allow sr_mod.ko to present any SR devices before exiting.
    // Addresses timing window with linuxrc installer.
    while (((rc_state.state & INIT_DONE) != INIT_DONE) && --retries)
        msleep(100);

    ssleep(5);

    rc_init_proc();

	return 0;
}

static void __exit rcraid_exit(void)
{
    int i;

    rc_printk(RC_DEBUG, "rcraid_exit\n");

    unregister_reboot_notifier(&rc_notifier);

	if (rc_state.host_ptr) {
		scsi_remove_host(rc_state.host_ptr);
		rc_shutdown_host(rc_state.host_ptr);
		scsi_host_put(rc_state.host_ptr);
		rc_state.host_ptr = NULL;
	}

	for (i = 0; i < rc_state.num_hba; i++) {
		rc_shutdown_adapter(rc_dev[i]);
	}

    if (rccfg_api_dev.this_device)
        misc_deregister(&rccfg_api_dev);

    pci_unregister_driver(&rcraid_pci_driver);

    unregister_sysctl_table(rcraid_sysctl_hdr);

    rc_remove_proc();
}

module_init(rcraid_init);
module_exit(rcraid_exit);
