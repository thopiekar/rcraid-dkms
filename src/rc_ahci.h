/****************************************************************************
 *
 * Copyright © 2006-2008 Ciprico Inc. All rights reserved.
 * Copyright © 2008-2015 Dot Hill Systems Corp. All rights reserved.
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

#ifdef RC_AHCI_SUPPORT
// *FIX* I'm not sure why the linux top layer requires it's own
// interrupt handler, initialization, etc.  This should be done at
// the bottom layer (only..and in fact already is done at the bottom.)
// Also, these definitions are taken as a subset of the original driver,
// which is obviously not ideal.
#define ICH6_REG_OFFSET_GHC     0x04    // Global HBA Control register
#define AHCI_GHC_IE             2       // Interrupt enable


#define ICH6_REGREAD(_r, _o)        readl(_r + _o)
#define ICH6_REGWRITE(_r, _o, _v)   writel(_v, _r + _o)

u32 rc_ahci_regread(void *context, u32 offset);
void rc_ahci_regwrite(void *context, u32 offset, u32 value);

#endif // RC_AHCI_SUPPORT
