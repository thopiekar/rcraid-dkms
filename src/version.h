/****************************************************************************\
*
*   MODULE: rc_version.h
*
*	Header file which defines the common version information.
*
*   Copyright (c) 2000-2004, RAIDCore, Inc.
*   Copyright (c) 2005-2006, Broadcom Corporation.  All rights reserved.
*   Copyright (c) 2006-2008, Ciprico Inc.  All rights reserved.
*   Copyright (c) 2008-2014, Dot Hill Systems Corp.  All rights reserved.
*   Copyright (c) 2015 Seagate All rights reserved.
*
\****************************************************************************/ 


#ifndef _RC_VERSION_H
#define _RC_VERSION_H

#define RC_MAX_VER_MAJOR_SIZE 4
#define RC_BUILD_VER_MAJOR 8

#define RC_MAX_VER_MINOR_SIZE 4
#define RC_BUILD_VER_MINOR 1

#define RC_MAX_VER_PATCH_SIZE 8
#define RC_BUILD_VER_PATCH 0

#define RC_STRINGIFY(s) #s

#define RC_VERSION_STR RC_MK_BUILD_VER(RC_BUILD_VER_MAJOR, RC_BUILD_VER_MINOR, RC_BUILD_VER_PATCH)

#define RC_MK_BUILD_VER(RC_BUILD_VER_MAJOR, RC_BUILD_VER_MINOR, RC_BUILD_VER_PATCH) \
                  RC_STRINGIFY(RC_BUILD_VER_MAJOR)"." \
                  RC_STRINGIFY(RC_BUILD_VER_MINOR)"." \
                  RC_STRINGIFY(RC_BUILD_VER_PATCH) 

#define VER_COMPANYNAME_STR       "AMD, Inc."
#define VER_PRODUCT_STR           "AMD-RAID"
#define VER_PRODUCT_STR_NO_TM     "AMD-RAID"
#define VER_PRODUCT_STR_WIN_DESC  "AMD-RAID Controller [storport] Device Driver"

#define VER_GUI_PRODUCT_STR_NO_TM "RAIDXpert2"

#define VER_AHCI_STR              "AHCI"
#define VER_MPT_STR               "LSI1068"
#define VER_MPT2_STR              "LSI2008"
#define VER_LEGALCOPYRIGHT_STR    "Copyright (c)2012-2016 Advanced Micro Devices, Inc."

#define VER_LEGALCOPYRIGHT_YEARS  "2012-2016"
#define VER_PRODUCTVERSION_STR    RC_BUILD_NUMBER
#define VER_LEGALTRADEMARKS_STR   ""

#define VER_FILEVERSION           RC_BUILD_VER_MAJOR, RC_BUILD_VER_MINOR, RC_BUILD_VER_PATCH, RC_BUILD_COUNT
#define VER_FILEVERSION_STR       RC_BUILD_NUMBER


#endif //_RC_VERSION_H
