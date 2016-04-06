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


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <getopt.h>
#include "common.h"
#include "partition.h"
#include "config.h"
#include "flashapi.h"


#define OFLASH_HANDLE_MAGIC 0x6873666f
#define OFLASH_HANDLE_MAGIC_SPI_NOR 0x61167851



static int file_length(char * file)
{
	struct stat file_stat;
	if(stat(file, &file_stat))
	{
		//printf("Error getting size of file %s!\n", file);
		return -1;
	}
	return (file_stat.st_size);
}



static int write_file_single(char * file, int offset)
{
	int fd2;
	OFLASH_HANDLE handle;
	int ret = 0, bytesRemain;
	unsigned char * buf;
	int length;
	intel_ce_flash_info_t flashinfo;

	/* Test SPI NOR Flash Device */
	handle.pad[0]	= OFLASH_HANDLE_MAGIC_SPI_NOR;
	
	if(OFLASH_OK != oflash_open(&handle, OFLASH_PAR_ALL))
	{
		printf("error opening device\n");
		return 1;
	}
	oflash_getinfo(&handle, &flashinfo);

	length = file_length(file);
	if(length < 0)
	{
		//printf("Error getting file size\n");
		oflash_close(&handle);
		return 1;
	}
	fd2 = open(file, O_RDONLY);
	if(fd2 < 0)
	{
		printf("Error opening file %s for read\n", file);
		oflash_close(&handle);
		return 1;
	}
	buf = (unsigned char *)malloc(flashinfo.blocksize);
	if(!buf)
	{
		printf("Error allocating buffer\n");
		ret = 1;
		goto out;
	}
	lseek(fd2, 0, SEEK_SET);
	bytesRemain = length;
	while(bytesRemain > 0)
	{
		memset(buf, 0, flashinfo.blocksize);
		if(bytesRemain >= flashinfo.blocksize)
		{

			read(fd2, buf, flashinfo.blocksize);
			printf("\n**********************************\n");
			printf("\nWriting block at 0x%x.length 0x%x..\n", offset,flashinfo.blocksize);
			printf("**********************************\n");
			oflash_seek(&handle, offset, OFLASH_SEEK_SET);
			oflash_write(&handle, buf, flashinfo.blocksize);
			offset += flashinfo.blocksize;
			bytesRemain -= flashinfo.blocksize;
		}
		else
		{
			printf("Read from file...\n");
			read(fd2, buf, bytesRemain);
			printf("\n**********************************\n");
			printf("\nWriting block at 0x%x.length 0x%x..\n", offset,bytesRemain);
			printf("\n**********************************\n");


			oflash_seek(&handle, offset, OFLASH_SEEK_SET);
			oflash_write(&handle, buf, bytesRemain);
			offset += bytesRemain;
			bytesRemain = 0;
		}
	}
	printf("Flash write complete!\n");

	free(buf);
out:
	oflash_close(&handle);
	close(fd2);
	return ret;
}
int main(int argc, char **argv)
{
	char *filename;
	int offset;
	if(argc !=3)
	{
		printf("please supply correct parameters\n");
		exit(1);
	}
	filename = strdup(argv[1]);
	offset = atoi(argv[2]);
	write_file_single(filename, offset);
	free(filename);
	printf("burn finished\n");
}
