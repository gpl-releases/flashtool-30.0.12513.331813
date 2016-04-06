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
CFLAGS+= -DLINUX -DVER1=$(COMP_VER1) -DVER2=$(COMP_VER2) -DVER3=$(COMP_VER3) -DVER4=$(COMP_VER4)     -DVER="\"$(COMP_VER)\"" -DCOMPONENTNAME="\"$(COMPONENTNAME)\""
CFLAGS+=-I$(FLASHTOOL_ROOT)/include -I$(BOOTLOADER_ROOT)/include -I$(FLASHTOOL_ROOT)/driver

OBJS=oflash sburn sread
LIB=../lib/libflashapi.a

.PHONY: all clean

all: $(OBJS)

oflash: oflash.o config.o
	$(CC) -o $@ $^ $(LIB)
sburn: sburn.o config.o
	$(CC) -o $@ $^ $(LIB)
sread: sread.o config.o
	$(CC) -o $@ $^ $(LIB)

%.o : %.c
	$(CC) -c $(CFLAGS) -o $@ $<
clean:
	rm -f *.o $(OBJS)
