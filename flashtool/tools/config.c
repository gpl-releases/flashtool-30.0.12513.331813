//*****************************************************************************
// This file is provided under a dual BSD/LGPLv2.1 license.  When using 
// or redistributing this file, you may do so under either license.
//
// LGPL LICENSE SUMMARY
//
// Copyright(c) <2008-2012>. Intel Corporation. All rights reserved.
//
// This program is free software; you can redistribute it and/or modify 
// it under the terms of version 2.1 of the GNU Lesser General Public 
// License as published by the Free Software Foundation.
//
// This library is distributed in the hope that it will be useful, but 
// WITHOUT ANY WARRANTY; without even the implied warranty of 
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU 
// Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public 
// License along with this library; if not, write to the Free Software 
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 
// USA. The full GNU Lesser General Public License is included in this 
// distribution in the file called LICENSE.LGPL.
//
// Contact Information:
//     Intel Corporation
//     2200 Mission College Blvd.
//     Santa Clara, CA  97052
//
// BSD LICENSE
//
// Copyright (c) <2008-2012>. Intel Corporation. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions 
// are met:
//
//   - Redistributions of source code must retain the above copyright 
//     notice, this list of conditions and the following disclaimer.
//   - Redistributions in binary form must reproduce the above copyright 
//     notice, this list of conditions and the following disclaimer in 
//     the documentation and/or other materials provided with the 
//     distribution.
//   - Neither the name of Intel Corporation nor the names of its 
//     contributors may be used to endorse or promote products derived 
//     from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR 
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, 
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY 
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//*****************************************************************************


#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <linux/types.h>
#include <unistd.h>
#include <getopt.h>
#include "intel_ce_flash.h"
#include "flash_private_data.h"
#include "partition.h"
#include "config.h"

int filesize(char * file)
{
	struct stat file_stat;
	if(stat(file, &file_stat))
	{
		//printf("Error getting size of file %s!\n", file);
		return -1;
	}
	return (file_stat.st_size);
}

int filesize_rounded(char * file, int blocksize)
{
	int length;
	
	length = filesize(file);
	if(length < 0)
	{
		return -1;
	}
	if(length % blocksize)
	{
		// Round to block size
		length += (blocksize-(length%blocksize));
		//printf("blocksize=0x%x\n", blocksize);
		//printf("length rounded up to %d(0x%x) bytes\n", length, length);
	}
	return length;
}

int get_type(char * type)
{
	if(!type)
		return TYPE_INVALID;
	if(!strcmp(type, "BOOTLOADER"))
	{
		return TYPE_BLDR;
	}
	else if(!strcmp(type, "KERNEL"))
	{
		return TYPE_KERNEL;
	}
	else if(!strcmp(type, "FPD"))
	{
		return TYPE_FPD;
	}
	else if(!strcmp(type, "RAMDISK"))
	{
		return TYPE_RAMDISK;
	}
	else if(!strcmp(type, "CONFIG"))
	{
		return TYPE_CONFIGBLOCK;
	}
	else if(!strcmp(type, "END"))
	{
		return TYPE_END;
	}
	else if(!strcmp(type, "NULL"))
	{
		return TYPE_NULL;
	}
	else if(!strcmp(type, "IMAGEDATA"))
	{
		return TYPE_IMGDATA;
	}
	else if(!strcmp(type, "APPDATA"))
	{
		return TYPE_APPDATA;
	}
	else return TYPE_INVALID;
}

int get_next_config(FILE * fp, config_item * pItem)
{
	char filename[128], typestr[32];
	int offset, length = 0, type;

	if(!fp)
		return 1;
	fscanf(fp, "%31s", typestr);
	typestr[31] = '\0';
	fscanf(fp, "%127s", filename);
	filename[127] = '\0';
	fscanf(fp, "%x", &offset);

	//printf("Type: %s\n", typestr);
	//printf("File: %s\n", filename);
	//printf("Offset: 0x%x\n", offset);
	type = get_type(typestr);
	if(TYPE_NULL != type && TYPE_END != type && TYPE_INVALID != type)
	{
		length = filesize_rounded(filename, INTEL_CE_P30FLASH_BLOCK);
		//printf("Length: 0x%x\n", length);
	}
	if(TYPE_INVALID == type)
	{
		printf("Invalid type: %s\n", typestr);
		return 0;
	}
	
	strncpy(pItem->filename, filename, 128);
	pItem->offset = offset;
	pItem->length = length;
	pItem->type = type;
	
	return 0;
}
