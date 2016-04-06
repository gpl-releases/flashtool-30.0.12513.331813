#//*****************************************************************************
#// This file is provided under a dual BSD/LGPLv2.1 license.  When using 
#// or redistributing this file, you may do so under either license.
#//
#// LGPL LICENSE SUMMARY
#//
#// Copyright(c) <2008-2010>. Intel Corporation. All rights reserved.
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
#// Copyright (c) <2008-2010>. Intel Corporation. All rights reserved.
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


TOPDIR=$(shell pwd)/..
FLASHTOOL_ROOT=$(TOPDIR)/flashtool

all:
	make -C driver FLASHTOOL_ROOT=$(FLASHTOOL_ROOT)
	make -C lib FLASHTOOL_ROOT=$(FLASHTOOL_ROOT) all
	make -C tools FLASHTOOL_ROOT=$(FLASHTOOL_ROOT) all
	@mkdir -p $(TOPDIR)/bin
	@cp -f driver/*.ko 	$(TOPDIR)/bin
	@cp -f lib/*.a 		$(TOPDIR)/bin
	@cp -f tools/sburn 			$(TOPDIR)/bin
	@cp -f tools/sread 			$(TOPDIR)/bin
	@cp -f tools/oflash 			$(TOPDIR)/bin
	@cp -f tools/conf.* 			$(TOPDIR)/bin
	@mkdir -p $(TOPDIR)/lib
	@mkdir -p $(TOPDIR)/lib/modules
	@cp -f driver/*.ko  $(TOPDIR)/lib/modules/
	@mkdir -p $(TOPDIR)/include
	@cp -f include/common.h $(TOPDIR)/include
	@cp -f include/config.h $(TOPDIR)/include
	@cp -f include/flashapi.h $(TOPDIR)/include
	@cp -f lib/libflashapi.a $(TOPDIR)/lib	
clean:
	make -C driver clean
	make -C lib clean
	make -C tools clean
