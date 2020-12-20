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

#ifndef RC_TYPES_PLATFORM_H
#define RC_TYPES_PLATFORM_H
#ifndef __BCM_BASETYPES_H__

typedef unsigned char		rc_uint8_t;
typedef unsigned short		rc_uint16_t;
typedef unsigned int		rc_uint32_t;
typedef int			rc_int32_t;

#ifndef __LINUX__
typedef unsigned __int64 rc_uint64_t;
#else
typedef unsigned long long  rc_uint64_t;
#endif

typedef unsigned long       rc_uint_ptr_t;

#endif /* __BCM_BASETYPES_H__ */
#endif /* RC_TYPES_H */
