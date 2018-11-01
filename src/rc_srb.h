/****************************************************************************
 *
 * Copyright © 2006-2008 Ciprico Inc. All rights reserved.
 * Copyright © 2008-2013 Dot Hill Systems Corp. All rights reserved.
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

#ifndef RC_SRB_H
#define RC_SRB_H

#include "rc_types_platform.h"

#define RC_SG_MAX_ELEMENTS 33

//
// SRB types
//
#define RC_SRB_EXECUTE_SCSI         0x00
#define RC_SRB_CLAIM_DEVICE         0x01
#define RC_SRB_IO_CONTROL           0x02
#define RC_SRB_RECEIVE_EVENT        0x03
#define RC_SRB_RELEASE_QUEUE        0x04
#define RC_SRB_ATTACH_DEVICE        0x05
#define RC_SRB_RELEASE_DEVICE       0x06
#define RC_SRB_SHUTDOWN             0x07
#define RC_SRB_FLUSH                0x08
#define RC_SRB_ABORT_COMMAND        0x10
#define RC_SRB_RELEASE_RECOVERY     0x11
#define RC_SRB_RESET_BUS            0x12
#define RC_SRB_RESET_DEVICE         0x13
#define RC_SRB_TERMINATE_IO         0x14
#define RC_SRB_FLUSH_QUEUE          0x15
#define RC_SRB_REMOVE_DEVICE        0x16
#define RC_SRB_WMI                  0x17
#define RC_SRB_LOCK_QUEUE           0x18
#define RC_SRB_UNLOCK_QUEUE         0x19
#define RC_SRB_RESET_LOGICAL_UNIT   0x20
#define RC_SRB_RESTART              0x21

//
// SRB Status
//

#define RC_SRB_STATUS_PENDING                 0x00
#define RC_SRB_STATUS_SUCCESS                 0x01
#define RC_SRB_STATUS_ABORTED                 0x02
#define RC_SRB_STATUS_ABORT_FAILED            0x03
#define RC_SRB_STATUS_ERROR                   0x04
#define RC_SRB_STATUS_BUSY                    0x05
#define RC_SRB_STATUS_INVALID_REQUEST         0x06
#define RC_SRB_STATUS_INVALID_PATH_ID         0x07
#define RC_SRB_STATUS_NO_DEVICE               0x08
#define RC_SRB_STATUS_TIMEOUT                 0x09
#define RC_SRB_STATUS_SELECTION_TIMEOUT       0x0A
#define RC_SRB_STATUS_COMMAND_TIMEOUT         0x0B
#define RC_SRB_STATUS_MESSAGE_REJECTED        0x0D
#define RC_SRB_STATUS_BUS_RESET               0x0E
#define RC_SRB_STATUS_PARITY_ERROR            0x0F
#define RC_SRB_STATUS_REQUEST_SENSE_FAILED    0x10
#define RC_SRB_STATUS_NO_HBA                  0x11
#define RC_SRB_STATUS_DATA_OVERRUN            0x12
#define RC_SRB_STATUS_UNEXPECTED_BUS_FREE     0x13
#define RC_SRB_STATUS_PHASE_SEQUENCE_FAILURE  0x14
#define RC_SRB_STATUS_BAD_SRB_BLOCK_LENGTH    0x15
#define RC_SRB_STATUS_REQUEST_FLUSHED         0x16
#define RC_SRB_STATUS_INVALID_LUN             0x20
#define RC_SRB_STATUS_INVALID_TARGET_ID       0x21
#define RC_SRB_STATUS_BAD_FUNCTION            0x22
#define RC_SRB_STATUS_ERROR_RECOVERY          0x23
#define RC_SRB_STATUS_NOT_POWERED             0x24
#define RC_SRB_STATUS_SGLE_ERROR              0x25
#define RC_SRB_STATUS_DEVICE_LOCKED           0x26
#define RC_SRB_STATUS_DEVICE_OFFLINE          0x27

//
// SRB Flags
//
#define RC_SRB_FLAGS_NO_DATA_TRANSFER         0x00000000
#define RC_SRB_FLAGS_DATA_IN                  0x00000001
#define RC_SRB_FLAGS_DATA_OUT                 0x00000002
#define RC_SRB_FLAGS_UNSPECIFIED_DIRECTION    (RC_SRB_FLAGS_DATA_IN |	\
					       RC_SRB_FLAGS_DATA_OUT)
#define RC_SRB_FLAGS_RAW                      0x00000004
#define RC_SRB_FLAGS_SENSEVALID               0x00000008

#define RC_SRB_GLOBAL_UPDATE                  0
#define RC_SRB_LOCAL_UPDATE_ONLY              0x01000000

/*
 * SRB layout
 *
 * The entire SRB is allocated in one chunk.
 * The sg_map and the device private space are allocated out of the 'private'
 * space.
 *
 *  ---------------------------
 * |                           |
 * |        SRB                |
 * |                           |
 *  ---------------------------
 * |                           |
 * |        sg_map             |
 * |                           |
 *  ---------------------------
 * |                           |
 * |   device private space    |
 * |                           |
 *  ---------------------------
 *
 */



typedef union {
	void         *virt_addr;
	rc_uint64_t    phys_addr;
} addr_elem_t;

#define dma_paddr    addr.phys_addr
#define v_addr        addr.virt_addr

#ifndef RC_PACKED
#define RC_PACKED
#endif

typedef struct RC_PACKED rc_sg_elem_s {
	addr_elem_t    addr;
	rc_uint32_t    size;     /* set to machine natural size */
} rc_sg_elem_t;

typedef struct rc_sg_list_s {
	rc_uint16_t    sg_mem_type;    /* Memory ID to pass to routines */
	rc_uint16_t    reserved;
	rc_uint32_t    sg_num_elem;
	rc_sg_elem_t   sg_elem[1];    /* variable size array */
} rc_sg_list_t;

typedef struct rc_srb_s {
	struct rc_srb_s *next;
	rc_uint32_t      function;
	rc_uint32_t      status;
	rc_uint32_t      bus;
	rc_uint32_t      target;
	rc_uint32_t      lun;
	rc_uint32_t      flags;
	rc_uint32_t      seq_num;
	rc_uint32_t      data_len;
	rc_uint32_t      cdb_len;
	void            *cdb;
	rc_uint32_t      sense_len;
	void            *sense;
	rc_sg_list_t    *sg_list;
	void            *scsi_context;
	void            *dev_private;
	void            (* callback)(struct rc_srb_s *);
	rc_uint32_t      timeout;
	rc_uint32_t      private32[1]; // This must be the last entry.
	// The sg_list (rc_sg_list_t) is allocated along with
	// this structure and private32 is the pointer used to
	// beginning of the sg_list.
} rc_srb_t;


#endif /*RC_SRB_H*/
