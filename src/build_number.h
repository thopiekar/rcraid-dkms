/****************************************************************************\
*
*   MODULE: rc_build_number.h
*
*		Header file to define the build number variables.
*
*   Copyright (c) 2000-2004, RAIDCore, Inc.
*   Copyright (c) 2005-2006, Broadcom Corporation.  All rights reserved.
*   Copyright (c) 2006-2008, Ciprico Inc.  All rights reserved.
*   Copyright (c) 2008-2013, Dot Hill Systems Corp.  All rights reserved.
*
\****************************************************************************/

#ifndef _RC_BUILD_NUMBER_H
#define _RC_BUILD_NUMBER_H

// maximum build number string length
// - this is actually 20 due to the genflash utility
// - see fulcrum/utils/genflash/genflash.cpp
// - unfortunately, other areas could be affected by this size, and
//   reducing it may be bad (alignment, structure size, etc.)
#define RC_MAX_BUILD_NUMBER_SIZE 32

// maximum mon/day/year string length
#define RC_MAX_BUILD_MONDAYYEAR 32

// to get around bios issues that do not concern this module
// use of externs are okay in this header file
#ifdef extern
#define _RC_BUILD_NUMBER_H_EXTERN extern
#undef extern
#endif


#ifdef __cplusplus
#include "rc_types.h"
RC_EXTERN const RC_CHAR *RC_BUILD_NUMBER_MARK_BEGIN;
RC_EXTERN const RC_CHAR *RC_BUILD_NUMBER;
RC_EXTERN const RC_CHAR *RC_BUILD_NUMBER_MARK_END;
RC_EXTERN const RC_CHAR *RC_BUILD_OEM;
RC_EXTERN RC_UINT16 RC_BUILD_COUNT;
RC_EXTERN RC_UINT16 RC_BUILD_REL_REVISION;
RC_EXTERN const RC_CHAR *RC_BUILD_MONDAYYEAR;
#else
extern char *RC_BUILD_NUMBER;
extern unsigned short RC_BUILD_COUNT;
extern char *RC_BUILD_MONDAYYEAR;
#endif

// be sure these this length matches that of the strings
// RC_BUILD_NUMBER_MARK_[BEGIN,END] in rc_build_number.cpp
// (length does not include null terminator)
#define RC_BUILD_NUMBER_MARK_LENGTH     7

// put back what was undone
#ifdef _RC_BUILD_NUMBER_H_EXTERN
#define extern _RC_BUILD_NUMBER_H_EXTERN
#endif


#endif //_RC_BUILD_NUMBER_H
