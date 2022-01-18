#
# Copyright © 2006-2008 Ciprico Inc. All rights reserved.
# Copyright © 2008-2013 Dot Hill Systems Corp. All rights reserved.
# Copyright © 2020 Advanced Micro Devices, Inc. All rights reserved.
#
# Use of this software is subject to the terms and conditions of the written
# software license agreement between you and AMD (the "License"),
# including, without limitation, the following (as further elaborated in the
# License):  (i) THIS SOFTWARE IS PROVIDED "AS IS", AND AMD DISCLAIMS
# ANY AND ALL WARRANTIES OF ANY KIND, WHETHER EXPRESS, IMPLIED, STATUTORY,
# BY CONDUCT, OR OTHERWISE; (ii) this software may be used only in connection
# with the integrated circuit product and storage software with which it was
# designed to be used; (iii) this source code is the confidential information
# of AMD and may not be disclosed to any third party; and (iv) you may not
# make any modification or take any action that would cause this software,
# or any other Advanced Micro Devices software, to fall under any GPL license
# or any other open source license.
#

#
# ifeq/ifneq/etc. HAVE to start at position 0 otherwise
#	Make attempts to treat them as a shell command and
#	you'll get lots of errors and spend a good time
#	scratching your head trying to figure out what's
#	going on -- I know I did...
#

RC_HOST=$(shell /bin/hostname)
RC_USER=$(shell whoami)
RC_DATE=$(shell /bin/date)
RC_BUILD_DATE=$(shell /bin/date +'%b %d %Y')
RCCERT=/var/lib/rccert/certs
PLATFORM=$(shell uname -m)
ifeq ($(PLATFORM), unknown)
    PLATFORM=$(shell uname -m)
endif

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

PWD    := $(shell pwd)

#SB_ENABLED	:= $(shell /bin/mokutil --sb-state | cut -f2 -d\ )

ifeq ($(shell [ -e /etc/lsb-release ] && echo 1 || echo 0), 1)
    SIGN_TOOL=/usr/src/linux-headers-$(KVERS)/scripts/sign-file
else
    ifeq ($(shell test -e "/etc/redhat-release" && echo 1 || echo 0), 1)
	SIGN_TOOL=/usr/src/kernels/$(KVERS)/scripts/sign-file
    else
        SIGN_TOOL=echo
    endif
endif

.PHONY: install bbanner ibanner

#
# First check to see if Secure boot is enabled. If not, we don't need
# to sign any modules.
#
# If so, then make sure we're running as root. We don't want normal
# user having access to private signing key, installing /lib/modules,
# and/or modifying the initramfs -- all necessary to install rcraid.ko
#
# If running as root, check if local signing certificate exists. If not,
# create one.
#
# Sign the module using the local certificate.
#

all: bbanner
	$(MAKE) -C $(KDIR) M=$(shell pwd) modules
#ifeq ($(shell /bin/mokutil --sb-state | cut -f2 -d\ ), enabled)
#ifneq ($(shell id -u), 0)
#	@echo "#"
#	@echo "# WARNING:"
#	@echo "#"
#	@echo "# Not building as root -- skipping signing for Secure Boot."
#	@echo "# Please rebuild as root or run 'make sign' as root before"
#	@echo "# installing."
#	@echo "#"
#else
#ifneq ($(shell [ -e ../certs/module_signing_key.der ] && echo 1 || echo 0), 1)
#	(cd ..; ./mk_certs)
#endif
#ifeq ($(shell [ -e ../certs/module_signing_key.der ] && echo 1 || echo 0), 1)
#	$(SIGN_TOOL) \
#		sha512 ../certs/module_signing_key.priv \
#		../certs/module_signing_key.der rcraid.ko
#else
#	@echo "#"
#	@echo "# ERROR:"
#	@echo "#"
#	@echo "# Signing certificate ../certs/module_signing_key.der does not exist!"
#	@echo "# Module NOT signed -- installing unsigned module will cause reboot to fail!"
#	@echo "#"
#endif
#endif
#endif
	@echo "Signing module using local certificate"
	@$(PWD)/../mk_certs $(SIGN_TOOL) $(PWD)/rcraid.ko $(RCCERT) $(KVERS)

sign:	rcraid.ko
#ifeq ($(shell /bin/mokutil --sb-state | cut -f2 -d\ ), enabled)
#ifneq ($(shell id -u), 0)
#	@echo "#"
#	@echo "# ERROR:"
#	@echo "#"
#	@echo "# Not running as root. Modules must be signed as root!"
#	@echo "# Module NOT signed -- installing unsigned module will cause reboot to fail!"
#	@echo "#"
#	@exit 1
#endif
#ifneq ($(shell [ -e ../certs/module_signing_key.der ] && echo 1 || echo 0), 1)
#	(cd ..; ./mk_certs)
#endif
#ifeq ($(shell [ -e ../certs/module_signing_key.der ] && echo 1 || echo 0), 1)
#	$(SIGN_TOOL) \
#		sha512 ../certs/module_signing_key.priv \
#		../certs/module_signing_key.der rcraid.ko
#else
#	@echo "#"
#	@echo "# ERROR:"
#	@echo "#"
#	@echo "# Signing certificate ../certs/module_signing_key.der does not exist!"
#	@echo "# Module NOT signed -- installing unsigned module will cause reboot to fail!"
#	@echo "#"
#endif
#endif
	@echo "Signing module using local certificate"
	@$(PWD)/../mk_certs $(SIGN_TOOL) $(PWD)/rcraid.ko $(RCCERT) $(KVERS)

clean:
	@rm -f *.o *.ko vers.c .*.cmd .*.d *.mod
	@rm -f rcraid.mod.c Module.symvers Modules.symvers modules.order
	@rm -rf .tmp_versions

install: ibanner
	@if [ -e /etc/redhat-release ]; then                             \
	     echo "performing Redhat install";                           \
	     (sh ./install_rh $(KVERS));                                 \
	 else                                                            \
	     echo "performing generic install";                          \
	     cp rcraid.ko /lib/modules/$(KVERS)/kernel/drivers/scsi;     \
	     depmod -a                                                   \
	 fi

bbanner:
	@echo ------------------------------------------------------------
	@echo - building for kernel $(KVERS)
	@echo ------------------------------------------------------------

ibanner:
	@echo ------------------------------------------------------------
	@echo - installing for kernel $(KVERS)
	@echo ------------------------------------------------------------

obj-m := rcraid.o

rcraid-objs := rc_init.o rc_msg.o rc_mem_ops.o rc_event.o rc_config.o rcblob.$(PLATFORM).o \
	       vers.o

.PHONY:	$(obj)/vers.c
$(obj)/vers.c:
	@echo "char *rc_ident = \"built on $(RC_HOST) by $(RC_USER) on $(RC_DATE)\";" > $@
# hack to avoid warning about missing .rcblob.cmd file when modpost tries to
# find all the sources if below $(obj)/rcblob.${PLATFORM}.o does not work
	@echo "$(PLATFORM) := 1" > $(obj)/.rcblob.x86_64.o.cmd

# hack to avoid warning about missing .rcblob.cmd file when modpost tries to
# find all the sources
.PHONY: $(obj)/rcblob.${PLATFORM}.o
$(obj)/rcblob.${PLATFORM}.o:
	ln -sf `basename $@ .o` $@
	@( echo "cmd_$@ := true"; echo "dep_$@ := \\"; echo "	$@ \\"; echo "" ) > $(obj)/.`basename $@`.cmd
