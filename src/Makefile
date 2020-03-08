#
# Copyright © 2006-2008 Ciprico Inc. All rights reserved.
# Copyright © 2008-2013 Dot Hill Systems Corp. All rights reserved.
#
# Use of this software is subject to the terms and conditions of the written
# software license agreement between you and DHS (the "License"),
# including, without limitation, the following (as further elaborated in the
# License):  (i) THIS SOFTWARE IS PROVIDED "AS IS", AND DHS DISCLAIMS
# ANY AND ALL WARRANTIES OF ANY KIND, WHETHER EXPRESS, IMPLIED, STATUTORY,
# BY CONDUCT, OR OTHERWISE; (ii) this software may be used only in connection
# with the integrated circuit product and storage software with which it was
# designed to be used; (iii) this source code is the confidential information
# of DHS and may not be disclosed to any third party; and (iv) you may not
# make any modification or take any action that would cause this software,
# or any other Dot Hill software, to fall under any GPL license or any other
# open source license.
#

RC_HOST=$(shell /bin/hostname)
RC_USER=$(shell whoami)
RC_DATE=$(shell /bin/date)
RC_BUILD_DATE=$(shell /bin/date +'%b %d %Y')
PLATFORM=$(shell uname -i)

EXTRA_CFLAGS += -D__LINUX__
EXTRA_CFLAGS += -DRC_AHCI_SUPPORT -DRC_AMD_AHCI -DRC_AHCI_AUTOSENSE
EXTRA_CFLAGS += -DRC_RAW_SPTD
EXTRA_CFLAGS += -DRC_RAW_PASSTHROUGH
EXTRA_CFLAGS += -DRC_LSI1068
EXTRA_CFLAGS += -DRC_MPT2
EXTRA_CFLAGS += -DRC_DETECT_AND_BLOCK_PROMISE_RAID
EXTRA_CFLAGS += -DRC_DRIVER_BUILD_DATE='"${RC_BUILD_DATE}"'

# Build against an installed kernel object tree.
# Either set the kernel version here or pass in the version.
# Defaults to building for the currently running system.
# examples:  make KVERS=2.6.20-1.2933.fc6
#            make KVERS=2.6.18-1.25-smp

ifndef KVERS
	KVERS=$(shell uname -r)
endif

# either set path to the kernel build tree here or pass in the directory
ifndef KDIR
	KDIR    := /lib/modules/$(KVERS)/build
endif

ifdef KBUILD_SRC
	ifneq ($(shell grep --quiet "irq_handler_t" $(srctree)/include/linux/interrupt.h && echo yes), yes)
		EXTRA_CFLAGS += -DNO_IRQ_HANDLER_T
	endif
endif

ifneq ($(RUN_DEPMOD),)
	DEPMOD := /sbin/depmod -a
else
	DEPMOD := true
endif

PWD    := $(shell pwd)

.PHONY: install

all: module
	$(MAKE) -C $(KDIR) M=$(PWD)

module:
	$(MAKE) -C $(KDIR) M=$(PWD)

clean:
	$(RM) -f *.o *.ko vers.c .*.cmd .*.d
	$(RM) -f rcraid.mod.c Module.symvers Modules.symvers
	$(RM) -rf .tmp_versions Module.markers modules.order

install:
	@if [ -e /etc/redhat-release ]; then                            \
	    echo "performing Redhat install";                           \
	    (sh ./install_rh $(KVERS));                                 \
	elif [ -e /etc/SuSE-release ]; then                             \
	    echo "performing SuSE install";                             \
	    (sh ./install_suse $(KVERS));                               \
	else                                                            \
	    echo "performing generic install";                          \
	    cp rcraid.ko /lib/modules/$(KVERS)/kernel/drivers/scsi;     \
	    $(DEPMOD) -a                                                \
	fi

obj-m := rcraid.o

rcraid-objs := rc_init.o \
               rc_msg.o \
               rc_mem_ops.o \
               rc_event.o \
               rc_config.o \
               rcblob.${PLATFORM}.o \
	       vers.o

.PHONY:	$(obj)/vers.c
$(obj)/vers.c:
	@echo "char *rc_ident = \"built on $(RC_HOST) by $(RC_USER) on $(RC_DATE)\";" > $@

# hack to avoid warning about missing .rcblob.cmd file when modpost tries to
# find all the sources
.PHONY: $(obj)/rcblob.${PLATFORM}.o
$(obj)/rcblob.${PLATFORM}.o:
	ln -sf `basename $@ .o` $@
	@( echo "cmd_$@ := true"; echo "dep_$@ := \\"; echo "	$@ \\"; echo "" ) > $(obj)/.`basename $@`.cmd
