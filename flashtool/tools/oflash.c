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

#ifdef LINUX
char *version_string = "#@# oflash " VER;
#endif

#define DEVICE_NODE		"/dev/flash"

char file_conf[128] = "";
#if 0
char file_bootloader[128] = "";
char file_kernel[128] = "";
char file_sysrd[128] = "";
char file_apprd[128] = "";
#endif
int batch = 0;
int blocksize;

void usage(char * app)
{
	printf("Usage: %s <options...>\n", app);
	printf("Options:\n");
	printf("\t-c <config_file>\n");
	printf("\t-v|--version: version\n");
	printf("\t-h: help\n");
}

void version()
{
	printf( "%s ", version_string );
}

int write_file1(char * file, int offset)
{
	int fd2;
	OFLASH_HANDLE handle;
	int ret = 0, bytesRemain;
	unsigned char * buf;
	int length;
	intel_ce_flash_info_t flashinfo;
	
	if(OFLASH_OK != oflash_open(&handle, OFLASH_PAR_ALL))
	{
		printf("error opening device\n");
		return 1;
	}
	oflash_getinfo(&handle, &flashinfo);

	length = filesize_rounded(file, flashinfo.blocksize);
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
		printf("Read from file...\n");
		read(fd2, buf, flashinfo.blocksize);

		printf("Writing block at 0x%x...\n", offset);
		oflash_seek(&handle, offset, OFLASH_SEEK_SET);
		oflash_write(&handle, buf, flashinfo.blocksize);

		offset += flashinfo.blocksize;
		bytesRemain -= flashinfo.blocksize;
	}
	printf("Flash write complete!\n");

	free(buf);
out:
	oflash_close(&handle);
	close(fd2);
	return ret;
}

int write_batch(char * conf)
{
	FILE * fp;
	config_item item;
	
	int ret = 0;
	
	if(!conf)
	{
		printf("Invalid config file name!\n");
		return 1;
	}
	if((fp = fopen(conf, "r")) == NULL)
	{
		printf("Error openning config file: %s\n", conf);
		return 1;
	}
	memset(&item, 0, sizeof(config_item));
	do {
		item.offset = -1;
		if(!get_next_config(fp, &item))
		{
			if((item.offset) >= 0 && item.type != TYPE_END
				&& item.type != TYPE_NULL && item.type != TYPE_INVALID)
			{
				printf("File: %s\n", item.filename);
				printf("Offset: 0x%x\n", item.offset);
				printf("Length: 0x%x\n", item.length);
				if(write_file1(item.filename, item.offset))
				{
					printf("Error writing file %s, abort!\n", item.filename);
					ret = 1;
					break;
				}
			}
		}
		else {
			printf("Error getting next config item, abort!\n");
			ret = 1;
			break;
		}
	} while(ret == 0 && item.type != TYPE_END && item.offset >= 0 && !feof(fp));
	fclose(fp);
	return ret;
}

int main(int argc, char **argv)
{
	int ret;

	{
		int i;
		for(i = 1; i < argc; i++)
		{
			if(!strncmp(argv[i], "-v", 16) || !strncmp(argv[i], "--version", 16))
			{
				version();
				exit(0);
			}
		}
	}

	while((ret = getopt(argc, argv, "c:h")) != -1)
	{
		switch(ret)
		{
			case 'h':
				usage(argv[0]);
				return 0;
			case 'c':
				strncpy(file_conf, optarg, 127);
				file_conf[127] = '\0';
				batch = 1;
				break;
			default:
				usage(argv[0]);
				return 1;
		}
	}

	if(!batch || !strlen(file_conf))
	{
		printf("Please specify the configuration file!\n");
		return 1;
	}
	else
	{
		printf("Config file: %s\n", file_conf);
		return write_batch(file_conf);
	}
}
