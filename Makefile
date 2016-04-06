#//*****************************************************************************
#// This file is provided under a dual BSD/LGPLv2.1 license.  When using 
#// or redistributing this file, you may do so under either license.
#//
#// LGPL LICENSE SUMMARY
#//
#// Copyright(c) <2008-2012>. Intel Corporation. All rights reserved.
#//
#// This program is free software; you can redistribute it and/or modify 
#// it under the terms of version 2.1 of the GNU Lesser General Public 
#// License as published by the Free Software Foundation.
#//
#// This library is distributed in the hope that it will be useful, but 
#// WITHOUT ANY WARRANTY; without even the implied warranty of 
#// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU 
#// Lesser General Public License for more details.
#//
#// You should have received a copy of the GNU Lesser General Public 
#// License along with this library; if not, write to the Free Software 
#// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 
#// USA. The full GNU Lesser General Public License is included in this 
#// distribution in the file called LICENSE.LGPL.
#//
#// Contact Information:
#//     Intel Corporation
#//     2200 Mission College Blvd.
#//     Santa Clara, CA  97052
#//
#// BSD LICENSE
#//
#// Copyright (c) <2008-2012>. Intel Corporation. All rights reserved.
#//
#// Redistribution and use in source and binary forms, with or without 
#// modification, are permitted provided that the following conditions 
#// are met:
#//
#//   - Redistributions of source code must retain the above copyright 
#//     notice, this list of conditions and the following disclaimer.
#//   - Redistributions in binary form must reproduce the above copyright 
#//     notice, this list of conditions and the following disclaimer in 
#//     the documentation and/or other materials provided with the 
#//     distribution.
#//   - Neither the name of Intel Corporation nor the names of its 
#//     contributors may be used to endorse or promote products derived 
#//     from this software without specific prior written permission.
#//
#// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
#// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
#// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR 
#// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
#// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
#// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
#// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, 
#// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY 
#// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
#// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
#// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#//*****************************************************************************

ifndef BUILD_ROOT
	export BUILD_ROOT=$(PWD)/../
endif

<default>:
	@echo "below targets are supported:"
	@echo "	all: build flashtool"
	@echo "	clean: clean flashtool"


include ./Makefile.inc

FLASHTOOL_SUB_DIRS= \
flashtool \
docs

.PHONY: all
all: bld doc test

.PHONY: bld 
bld:
	@echo ">>>Building flashtool"
	@echo "PATH: $(PATH)"
	$(foreach SUBDIR, $(FLASHTOOL_SUB_DIRS), $(MAKE) -C $(SUBDIR) all;)

.PHONY: install
install: install_dev install_target
	@echo ">>>Installed flashtool development and target files"

.PHONY: install_dev
install_dev:
	@echo "Copying flashtool development files to $(BUILD_DEST)"
	@[ -d "$(BUILD_DEST)/include" ] || mkdir -pv "$(BUILD_DEST)/include"
	@cp -v include/common.h $(BUILD_DEST)/include
	@cp -v include/config.h $(BUILD_DEST)/include
	@cp -v include/flashapi.h $(BUILD_DEST)/include
	@if [ -f README_flashtool.txt ]; then \
	cp -v README_flashtool.txt $(BUILD_DEST); \
	fi
	@[ -d "$(BUILD_DEST)/lib/modules" ] || mkdir -pv "$(BUILD_DEST)/lib/modules"
	@cp -v lib/libflashapi.a $(BUILD_DEST)/lib
	@cp -v lib/modules/intel_ce_flash.ko $(BUILD_DEST)/lib/modules
	@[ -d "$(BUILD_DEST)/bin" ] || mkdir -pv $(BUILD_DEST)/bin
	@cp -v bin/oflash $(BUILD_DEST)/bin
	@cp -v bin/sburn $(BUILD_DEST)/bin
	@cp -v bin/sread $(BUILD_DEST)/bin

.PHONY: install_target
install_target:
	@echo "Copying flashtool target files to $(FSROOT)"
	@[ -d "$(FSROOT)/lib/modules" ] || mkdir -pv "$(FSROOT)/lib/modules"
	@cp -v lib/modules/intel_ce_flash.ko $(FSROOT)/lib/modules
	@if [ -f init_flashtool ]; then \
		mkdir -p $(FSROOT)/etc/init.d; \
		cp -vf init_flashtool $(FSROOT)/etc/init.d/flashtool; \
	fi
	

.PHONY: debug 
debug: dbld doc test

.PHONY: dbld 
dbld:
	@echo ">>>Building debug version of flashtool"
	@echo ">>>Debug version is same as normal version"
	@echo ">>>Building flashtool"
	@echo "PATH: $(PATH)"
	$(foreach SUBDIR, $(FLASHTOOL_SUB_DIRS), $(MAKE) -C $(SUBDIR) all;)

.PHONY: doc
doc:
	@echo ">>>Building doc for flashtool"
	make -C docs

.PHONY: doc-clean
doc-clean:
	@echo ">>>Clean doc for flashtool"
	make clean -C docs

.PHONY: test
test:
	@echo ">>>Do nothing"

.PHONY: clean
clean:
	@echo ">>>Cleaning flashtool"
	rm -f ./bin/*
	$(foreach SUBDIR, $(FLASHTOOL_SUB_DIRS), $(MAKE) -C $(SUBDIR) clean;)
	make uninstall

.PHONY: uninstall
uninstall: uninstall_dev uninstall_target
	@echo "Uninstalled flashtool development and target files"

uninstall_dev:
	@echo "Removing flashtool development files from $(BUILD_DEST)"
	@rm -f $(BUILD_DEST)/include/common.h
	@rm -f $(BUILD_DEST)/include/config.h
	@rm -f $(BUILD_DEST)/include/flashapi.h
	@if [ -f README_flashtool.txt ]; then \
	rm -f $(BUILD_DEST)/README_flashtool.txt; \
	fi
	@rm -f $(BUILD_DEST)/lib/libflashapi.a
	@rm -f $(BUILD_DEST)/lib/modules/intel_ce_flash.ko
	@rm -f $(BUILD_DEST)/bin/oflash
	@rm -f $(BUILD_DEST)/bin/sburn
	@rm -f $(BUILD_DEST)/bin/sread
	
uninstall_target:
	@echo "Removing flashtool target files from $(FSROOT)"
	@rm -f $(FSROOT)/lib/modules/intel_ce_flash.ko
