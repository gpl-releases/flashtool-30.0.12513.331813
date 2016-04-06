#  GPL LICENSE SUMMARY
#
#  Copyright(c) 2009-2011 Intel Corporation. All rights reserved.
#
#  This program is free software; you can redistribute it and/or modify
#  it under the terms of version 2 of the GNU General Public License as
#  published by the Free Software Foundation.
#
#  This program is distributed in the hope that it will be useful, but
#  WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
#  General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
#  The full GNU General Public License is included in this distribution
#  in the file called LICENSE.GPL.
#
#  Contact Information:
#    Intel Corporation
#    2200 Mission College Blvd.
#    Santa Clara, CA  97052
########################################################################

#-----
# versioning
#-----
# Official build generates the follow version environment variables:
#       COMP_VER1=1
#       COMP_VER2=2
#       COMP_VER3=3
#       COMP_VER4=4
#       COMP_VER=1-2-3-0004
#
# If official build is not being done, we use bogus "development" build number
COMP_VER4 ?= 9999999
COMP_VER  ?= <Unofficial Build>
EXTRA_CFLAGS+= -DLINUX -DVER1=$(COMP_VER1) -DVER2=$(COMP_VER2) -DVER3=$(COMP_VER3) -DVER4=$(COMP_VER4)     -DVER="\"$(COMP_VER)\"" -DCOMPONENTNAME="\"$(COMPONENTNAME)\""
EXTRA_CFLAGS+=-I$(FLASHTOOL_ROOT)/include -I$(BOOTLOADER_ROOT)/include

ifneq ($(KERNELRELEASE),)

obj-m :=intel_ce_flash.o 


else

PWD := $(shell pwd)
TARGETS := intel_ce_flash.ko 


.PHONY: all clean

all: $(TARGETS)

%.ko: %.c %.h
#	$(MAKE) -C $(KSRC) SUBDIRS=$(PWD) modules
	$(MAKE) -C $(KERNEL_BUILD_DIR) SUBDIRS=$(PWD) modules

endif

clean:
	rm -f *.o *.ko *.mod.c .*o.cmd .*ko.cmd
	rm -rf .tmp_versions [Mm]odule*
