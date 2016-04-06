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
#include "flash_private_data.h"
#include "flash_shim.h"
//#define FLASHAPI_DEBUG
#define _OFLASH_CORE
//#include "flashapi.h"
#include "_flashapi.h"

#define DEVICE_NODE		"/dev/flash" //Device node for NOR flash in Expansion bus
char SPI_NOR_DEVICE_NODE[256]; //Device node for NOR flash in SPI bus


#define START_OF_BLOCK(i)	((pHandle->handle.magic == OFLASH_HANDLE_MAGIC_EXP_NOR)?device_map.map[i].start:(BLOCKSIZE*i))
#define END_OF_BLOCK(i)	((pHandle->handle.magic == OFLASH_HANDLE_MAGIC_EXP_NOR)?device_map.map[i].end:(BLOCKSIZE*(i+1)-1))

#define BLOCKSIZE		(flash_info.blocksize)
#define BLOCKSIZE2		(flash_info.blocksize2)
#define INVALID_HANDLE (pHandle->handle.magic != OFLASH_HANDLE_MAGIC_EXP_NOR) && (pHandle->handle.magic != OFLASH_HANDLE_MAGIC_SPI_NOR)

static memory_map_t device_map;
static intel_ce_flash_info_t flash_info;
static u8 * internal_buf = NULL;
static flash_private_data_t fpd;
static int fpd_loaded = 0;

int ioctl(int fd, int command, ...);
typedef int (*oflash_ioctl_type)(int fd, int cmd, ...); 
static oflash_ioctl_type oflash_ioctl	= NULL;


OFLASH_STATUS oflash_open(OFLASH_HANDLE * pHandle, OFLASH_PARTYPE parType)
{
	int fd;
	int flash_type = 0;
	if(!pHandle)
	{
		PERROR("NULL pointer pHandle?\n");
		return OFLASH_ERR;
	}
	pHandle->handle.fd = -1;
	pHandle->handle.parType = 0;
//	printf("\npHandle 0x%x\n",pHandle->handle.magic);
	flash_type = (pHandle->handle.magic == OFLASH_HANDLE_MAGIC_SPI_NOR)?1:0;
	if (flash_type && detect_mtd_dev_node(SPI_NOR_DEVICE_NODE))
		goto ERR_NO_DEV_NODE;
	fd = open((flash_type?SPI_NOR_DEVICE_NODE:DEVICE_NODE),O_RDWR);
	if(fd < 0)
		goto ERR_NO_DEV_NODE;

	/* 
	 * 1. NOR flash access will go to system call "ioctl" 
	 * 2. SPI NOR flash access will be direct to MTD library
	 */
	if (pHandle->handle.magic == OFLASH_HANDLE_MAGIC_SPI_NOR)
		oflash_ioctl	= (oflash_ioctl_type)&flash2mtd_ioctl;
	else
	{
		oflash_ioctl	= (oflash_ioctl_type)&ioctl;
		/* Initialize the handler once it's uninitialized */
		pHandle->handle.magic = OFLASH_HANDLE_MAGIC_EXP_NOR;
	}		
	if((*oflash_ioctl)(fd, INTEL_CE_FLASH_GET_MAP, (unsigned long)&device_map))
	{
		PERROR("error getting device memory map\n");
		close(fd);
		return OFLASH_ERR;
	}
	if((*oflash_ioctl)(fd, INTEL_CE_FLASH_READINFO, (unsigned long)&flash_info))
	{
		PERROR("error getting device info\n");
		close(fd);
		return OFLASH_ERR;
	}
	internal_buf = (u8 *)malloc(BLOCKSIZE);
	if(!internal_buf)
	{
		PERROR("error allocating internal buffer\n");
		close(fd);
		return OFLASH_ERR;
	}
	
	pHandle->handle.fd = fd;
	pHandle->handle.parType = parType;
	pHandle->handle.offset = 0;
	fpd_loaded = 0;

	return OFLASH_OK;
ERR_NO_DEV_NODE:
	printf("Error opening device %s\n", flash_type?"SPI FLASH":"EXP P30 FLASH");
	printf("Please ensure that:\n");
	printf("- The flash device is not opened twice at same time\n");
	printf("- flash driver is properly loaded\n");
	printf("- Device node %s is created\n", flash_type?SPI_NOR_DEVICE_NODE:DEVICE_NODE);
	return OFLASH_ERR;
}

OFLASH_STATUS oflash_close(OFLASH_HANDLE * pHandle)
{
	if(!pHandle)
	{
		PERROR("null handle\n");
		return OFLASH_ERR;
	}
	if((pHandle->handle.fd) <= 0)
	{
		PERROR("invalid or non-initialized handle\n");
		return OFLASH_ERR;
	}
	if (INVALID_HANDLE)
	{
		PERROR("invalid or non-initialized handle\n");
		return OFLASH_ERR;
	}
	close(pHandle->handle.fd);
	if(internal_buf)
		free(internal_buf);
	pHandle->handle.fd = -1;
	pHandle->handle.magic = 0;

	return OFLASH_OK;
}

OFLASH_STATUS oflash_getinfo(OFLASH_HANDLE * pHandle, intel_ce_flash_info_t * pInfo)
{
	int fd;

	if(!pHandle || !pInfo)
	{
		PERROR("null handle\n");
		return OFLASH_ERR;
	}
	if(pHandle->handle.fd <= 0)
	{
		PERROR("invalid handle\n");
		return OFLASH_ERR;
	}
	if (INVALID_HANDLE)
	{
		PERROR("invalid or non-initialized handle\n");
		return OFLASH_ERR;
	}
	fd = pHandle->handle.fd;
	if((*oflash_ioctl)(fd, INTEL_CE_FLASH_READINFO, (unsigned long)pInfo))
	{
		return OFLASH_ERR;
	}
	
	return OFLASH_OK;
}

static inline int _calc_blockno(OFLASH_HANDLE *pHandle,int offset)
{
	int i;

	if (pHandle->handle.magic == OFLASH_HANDLE_MAGIC_SPI_NOR)
		return (offset/BLOCKSIZE);
	else
	{
		for(i = 0; i < device_map.memory_map_size; i++)
		{
			if((device_map.map[i].start) <= offset && offset <= (device_map.map[i].end))
			{
				return i;
			}
		}
	}
	return -1;
}

OFLASH_STATUS _get_partition_limit(OFLASH_HANDLE *pHandle, int * pStart, int *pLength)
{
	int ret = OFLASH_OK;
	intel_ce_flash_cmd_t rwcmd;
	//flash_private_data_t fpd;
	
	if(!pHandle)
	{
		PERROR("null handle\n");
		return OFLASH_ERR;
	}
	if(pHandle->handle.fd <= 0)
	{
		PERROR("invalid handle\n");
		return OFLASH_ERR;
	}
	if(INVALID_HANDLE)
	{
		PERROR("invalid or non-initialized handle\n");
		return OFLASH_ERR;
	}
	if(!pStart || !pLength)
	{
		PERROR("null pointer parameter\n");
		return OFLASH_ERR;
	}
	
	switch(pHandle->handle.parType)
	{
		case OFLASH_PAR_ALL:
			*pStart = 0;
			*pLength = flash_info.size_in_bytes;
			break;
		case OFLASH_PAR_FPD:
			*pStart = OFFSET_FPD;
			*pLength = SIZE_FPD;
			break;
		case OFLASH_PAR_IMAGEDATA:
		case OFLASH_PAR_APPDATA:
			if(!fpd_loaded)
			{
				rwcmd.buf = internal_buf;
				rwcmd.blockno = _calc_blockno(pHandle,OFFSET_FPD);
				if((*oflash_ioctl)(pHandle->handle.fd, INTEL_CE_FLASH_READ_BLOCK, (unsigned long)&rwcmd))
				{
					PERROR("error reading FPD at block %d\n", rwcmd.blockno);
					ret = OFLASH_ERR;
					goto out;
				}
				memcpy(&fpd, internal_buf, sizeof(flash_private_data_t));
				fpd_loaded = 1;
			}
			if(OFLASH_PAR_IMAGEDATA == (pHandle->handle.parType))
			{
				*pStart = fpd.offset_imagedata;
				*pLength = fpd.size_imagedata;
			}
			else {
				*pStart = fpd.offset_customdata;
				*pLength = fpd.size_customdata;
			}
			break;
		default:
			PERROR("invalid partition type: %d\n", pHandle->handle.parType);
			ret = OFLASH_ERR;
			break;
	}
out:
	DPRINTF("start=0x%x, length=0x%x\n", *pStart, *pLength);
	return ret;
}

OFLASH_STATUS oflash_read(OFLASH_HANDLE * pHandle, u8 * pDst, int length)
{
	int fd;
	OFLASH_STATUS ret = OFLASH_OK;
	intel_ce_flash_cmd_t rwcmd;
	int block_start, block_end, i;
	int par_base, par_size;
	int start, count, totalread = 0;
	int offset;
	u8 * buf = internal_buf;

	if(!pHandle)
	{
		PERROR("null handle\n");
		return OFLASH_ERR;
	}
	if(!pDst)
	{
		PERROR("null destination buffer\n");
		return OFLASH_ERR;
	}
	if(pHandle->handle.fd <= 0)
	{
		PERROR("invalid handle\n");
		return OFLASH_ERR;
	}
	if(INVALID_HANDLE)
	{
		PERROR("invalid or non-initialized handle\n");
		return OFLASH_ERR;
	}
	fd = pHandle->handle.fd;
	if(OFLASH_OK != _get_partition_limit(pHandle, &par_base, &par_size))
	{
		PERROR("error getting partition limit\n");
		return OFLASH_ERR;
	}
	DPRINTF("par_base=0x%x, par_size=0x%x\n", par_base, par_size);
	if((pHandle->handle.offset) < 0 || (pHandle->handle.offset) + length > par_size)
	{
		PERROR("access out of partition region: offset=0x%x, length=0x%x!\n", pHandle->handle.offset, length);
		return OFLASH_ERR;
	}
	DPRINTF("offset = 0x%08x\n", pHandle->handle.offset);
	offset = (pHandle->handle.offset);
	offset += par_base;
	if(offset >= flash_info.size_in_bytes)
	{
		PERROR("offset out of device region!");
		return OFLASH_ERR;
	}
	rwcmd.buf = pDst;
	rwcmd.offset = offset;
	rwcmd.length = length;
	if((*oflash_ioctl)(fd, INTEL_CE_FLASH_READ_BYTES, (unsigned long)&rwcmd))
	{
		PERROR("error reading at 0x%x\n", offset);
		ret = OFLASH_ERR;
		goto out;
	}
	totalread = length;
	DPRINTF("finish!\n");
out:
	if(OFLASH_OK == ret)
	{
		DPRINTF("totalread=0x%08x\n", totalread);
		pHandle->handle.offset += totalread;
		DPRINTF("new offset = 0x%08x\n", pHandle->handle.offset);
	}
	return ret;
}
OFLASH_STATUS oflash_secret_read(OFLASH_HANDLE * pHandle, u8 * pDst, int length, int offset)
{
	int fd;
	OFLASH_STATUS ret = OFLASH_OK;
	intel_ce_flash_cmd_t rwcmd;
	u8 * buf = internal_buf;

	if(!pHandle)
	{
		PERROR("null handle\n");
		return OFLASH_ERR;
	}
	if(!pDst)
	{
		PERROR("null destination buffer\n");
		return OFLASH_ERR;
	}
	if(pHandle->handle.fd <= 0)
	{
		PERROR("invalid handle\n");
		return OFLASH_ERR;
	}
	if(INVALID_HANDLE)
	{
		PERROR("invalid or non-initialized handle\n");
		return OFLASH_ERR;
	}
	fd = pHandle->handle.fd;
	rwcmd.buf = pDst;
	rwcmd.offset = offset;
	rwcmd.length = length;
	if((*oflash_ioctl)(fd, INTEL_CE_FLASH_READ_SECRET_BYTES, (unsigned long)&rwcmd))
	{
		PERROR("error reading at 0x%x\n", offset);
		ret = OFLASH_ERR;
		goto out;
	}
	DPRINTF("finish!\n");
out:
	if(OFLASH_OK == ret)
	{
	}
	return ret;
}

OFLASH_STATUS oflash_write(OFLASH_HANDLE * pHandle, u8 * pSrc, int length)
{
	int fd;
	OFLASH_STATUS ret = OFLASH_OK;
	intel_ce_flash_cmd_t rwcmd;
	int block_start, block_end, i;
	int par_base, par_size;
	int start, count, totalwrite = 0;
	int offset;
	u8 * buf = internal_buf;

	if(!pHandle)
	{
		PERROR("null handle\n");
		return OFLASH_ERR;
	}
	if(pHandle->handle.fd <= 0)
	{
		PERROR("invalid handle\n");
		return OFLASH_ERR;
	}
	if(INVALID_HANDLE)
	{
		PERROR("invalid or non-initialized handle\n");
		return OFLASH_ERR;
	}
	fd = pHandle->handle.fd;
	if(OFLASH_OK != _get_partition_limit(pHandle, &par_base, &par_size))
	{
		PERROR("error getting partition limit\n");
		return OFLASH_ERR;
	}
	DPRINTF("par_base=0x%x, par_size=0x%x\n", par_base, par_size);
	if((pHandle->handle.offset) < 0 || (pHandle->handle.offset) + length > par_size)
	{
		PERROR("access out of partition region: offset=0x%x, length=0x%x!\n", pHandle->handle.offset, length);
		return OFLASH_ERR;
	}
	DPRINTF("offset = 0x%08x\n", pHandle->handle.offset);
	offset = (pHandle->handle.offset);
	offset += par_base;
	if(offset >= flash_info.size_in_bytes)
	{
		PERROR("offset out of device region!");
		return OFLASH_ERR;
	}

	block_start = _calc_blockno(pHandle,offset);
	block_end = _calc_blockno(pHandle,offset + length -1);
	DPRINTF("block_start = %d, block_end = %d\n", block_start, block_end);
	{
		// Need to check if FPD is updated
		int fpd_block = _calc_blockno(pHandle,OFFSET_FPD);
		if(block_start <= fpd_block && fpd_block <= block_end)
			fpd_loaded = 0;
	}
	if((block_start < 0) || (block_start > (MAX_MEMORY_MAP - 1)) || (block_end < 0) || (block_end > (MAX_MEMORY_MAP - 1)))
		return OFLASH_ERR;
	for(i = block_start; i <= block_end; i++)
	{
		DPRINTF("read out block %d\n", i);
		rwcmd.buf = buf;
		rwcmd.blockno = i;
		rwcmd.length = flash_info.blocksize;

		if((*oflash_ioctl)(fd, INTEL_CE_FLASH_READ_BLOCK, (unsigned long)&rwcmd))
		{
			PERROR("error reading out block %d\n", i);
			ret = OFLASH_ERR;
			goto out;
		}
		start = 0;
		count = END_OF_BLOCK(i)	- START_OF_BLOCK(i) + 1;
		if(i == block_start)
		{
			start = (offset - START_OF_BLOCK(i));
			count = END_OF_BLOCK(i)	 - offset + 1;
		}
		if(i == block_end)
		{
			if(i != block_start)
			{
				start = 0;
				count = offset + length - START_OF_BLOCK(i);
			}
			else {
				count = length;
			}
		}
		DPRINTF("start=0x%x, count=0x%x\n", start, count);
		memcpy(buf + start, pSrc, count);
		pSrc += count;
		totalwrite += count;
		
		DPRINTF("erasing block %d\n", i);
		if((*oflash_ioctl)(fd, INTEL_CE_FLASH_ERASE, (unsigned long)START_OF_BLOCK(i)))
		{
			PERROR("error erasing at 0x%x\n", START_OF_BLOCK(i));
			ret = OFLASH_ERR;
			goto out;
		}
		
		DPRINTF("writing block %d\n", i);
		if((*oflash_ioctl)(fd, INTEL_CE_FLASH_WRITE_BLOCK, (unsigned long)&rwcmd))
		{
			PERROR("error writing block %d\n", i);
			ret = OFLASH_ERR;
			goto out;
		}
	}
	DPRINTF("finish!\n");
out:
	if(OFLASH_OK == ret)
	{
		DPRINTF("totalwrite=0x%08x\n", totalwrite);
		pHandle->handle.offset += totalwrite;
		DPRINTF("new offset = 0x%08x\n", pHandle->handle.offset);
	}
	return ret;
}
OFLASH_STATUS oflash_secret_write(OFLASH_HANDLE * pHandle, u8 * pSrc, int length, int offset)
{
	int fd;
	OFLASH_STATUS ret = OFLASH_OK;
	intel_ce_flash_cmd_t rwcmd;
	u8 * buf = internal_buf;

	if(!pHandle)
	{
		PERROR("null handle\n");
		return OFLASH_ERR;
	}
	if(!pSrc)
	{
		PERROR("null destination buffer\n");
		return OFLASH_ERR;
	}
	if(pHandle->handle.fd <= 0)
	{
		PERROR("invalid handle\n");
		return OFLASH_ERR;
	}
	if(INVALID_HANDLE)
	{
		PERROR("invalid or non-initialized handle\n");
		return OFLASH_ERR;
	}
	fd = pHandle->handle.fd;
	rwcmd.buf = pSrc;
	rwcmd.offset = offset;
	rwcmd.length = length;
	if((*oflash_ioctl)(fd, INTEL_CE_FLASH_WRITE_SECRET_BYTES, (unsigned long)&rwcmd))
	{
		PERROR("error writing at 0x%x\n", offset);
		ret = OFLASH_ERR;
		goto out;
	}
	DPRINTF("finish!\n");
out:
	if(OFLASH_OK == ret)
	{
	}
	return ret;
}


#if 0
OFLASH_STATUS oflash_erase(OFLASH_HANDLE * pHandle, int offset)
{
	int fd;
	OFLASH_STATUS ret = OFLASH_OK;
	int par_base, par_size;

	if(!pHandle)
	{
		return OFLASH_ERR;
	}
	if(pHandle->handle.fd <= 0)
	{
		return OFLASH_ERR;
	}
	fd = pHandle->handle.fd;
	if(OFLASH_OK != _get_partition_limit(pHandle, &par_base, &par_size))
	{
		PERROR("error getting partition limit\n");
		return OFLASH_ERR;
	}
	DPRINTF("par_base=0x%x, par_size=0x%x\n", par_base, par_size);
	offset += par_base;
	if(offset >= flash_info.size_in_bytes)
	{
		PERROR("offset out of device region!");
		return OFLASH_ERR;
	}
	
	DPRINTF("erase offset=0x%x\n", offset);	
	if((*oflash_ioctl)(fd, INTEL_CE_FLASH_ERASE, offset))
	{
		PERROR("error erasing at 0x%x\n", offset);
		return OFLASH_ERR;
	}
	
	return OFLASH_OK;
}
#endif

OFLASH_STATUS oflash_seek(OFLASH_HANDLE * pHandle, int offset, OFLASH_SEEK_TYPE seek_type)
{
	int fd;
	OFLASH_STATUS ret = OFLASH_OK;
	int par_base, par_size;
	int seek_base, newoffset;
	
	if(!pHandle)
	{
		PERROR("null handle\n");
		return OFLASH_ERR;
	}
	if(pHandle->handle.fd <= 0)
	{
		PERROR("invalid handle\n");
		return OFLASH_ERR;
	}
	if(INVALID_HANDLE)
	{
		PERROR("invalid or non-initialized handle\n");
		return OFLASH_ERR;
	}
	fd = pHandle->handle.fd;
	if(OFLASH_OK != _get_partition_limit(pHandle, &par_base, &par_size))
	{
		PERROR("error getting partition limit\n");
		return OFLASH_ERR;
	}
	DPRINTF("par_base=0x%x, par_size=0x%x\n", par_base, par_size);
	DPRINTF("old offset = 0x%08x\n", pHandle->handle.offset);

	switch(seek_type)
	{
		case OFLASH_SEEK_SET:
			seek_base = 0;
			break;
		case OFLASH_SEEK_CUR:
			seek_base = pHandle->handle.offset;
			break;
		case OFLASH_SEEK_END:
			seek_base = par_size;
			break;
		default:
			PERROR("Invalid seek type: %d\n", seek_type);
			return OFLASH_ERR;
	}
	newoffset = seek_base + offset;
	DPRINTF("seek_base=0x%08x, newoffset=0x%08x\n", seek_base, newoffset);
	if(newoffset < 0 || newoffset >= par_size)
	{
		PERROR("new offset (0x%08x) out of partition region!", newoffset);
		ret = OFLASH_ERR;
	}
	pHandle->handle.offset = newoffset;
	DPRINTF("set new offset: 0x%08x\n", pHandle->handle.offset);

out_seek:
	return ret;
}

OFLASH_STATUS oflash_getpartitionsize(OFLASH_HANDLE * pHandle, int * pParSize)
{
	int fd;
	OFLASH_STATUS ret = OFLASH_OK;
	int par_size, par_base;

	if(!pHandle)
	{
		PERROR("null handle\n");
		return OFLASH_ERR;
	}
	if(pHandle->handle.fd <= 0)
	{
		PERROR("invalid handle\n");
		return OFLASH_ERR;
	}
	if(INVALID_HANDLE)
	{
		PERROR("invalid or non-initialized handle\n");
		return OFLASH_ERR;
	}
	if(!pParSize)
	{
		PERROR("null pointer parameter\n");
		return OFLASH_ERR;
	}
	if(OFLASH_OK != _get_partition_limit(pHandle, &par_base, &par_size))
	{
		PERROR("error getting partition limit\n");
		return OFLASH_ERR;
	}
	DPRINTF("par_base=0x%x, par_size=0x%x\n", par_base, par_size);
	
	(*pParSize) = par_size;
	return ret;
}

OFLASH_STATUS oflash_getpartitiontype(OFLASH_HANDLE * pHandle, OFLASH_PARTYPE * pParType)
{
	int fd;
	OFLASH_STATUS ret = OFLASH_OK;

	if(!pHandle)
	{
		PERROR("null handle\n");
		return OFLASH_ERR;
	}
	if(pHandle->handle.fd <= 0)
	{
		PERROR("invalid handle\n");
		return OFLASH_ERR;
	}
	if(INVALID_HANDLE)
	{
		PERROR("invalid or non-initialized handle\n");
		return OFLASH_ERR;
	}
	if(!pParType)
	{
		PERROR("null pointer parameter\n");
		return OFLASH_ERR;
	}
	(*pParType) = (pHandle->handle.parType);
	
	return ret;
}

OFLASH_STATUS oflash_getcurrentoffset(OFLASH_HANDLE * pHandle, int * pCurOffset)
{
	int fd;
	OFLASH_STATUS ret = OFLASH_OK;

	if(!pHandle)
	{
		PERROR("null handle\n");
		return OFLASH_ERR;
	}
	if(pHandle->handle.fd <= 0)
	{
		PERROR("invalid handle\n");
		return OFLASH_ERR;
	}
	if(INVALID_HANDLE)
	{
		PERROR("invalid or non-initialized handle\n");
		return OFLASH_ERR;
	}
	if(!pCurOffset)
	{
		PERROR("null pointer parameter\n");
		return OFLASH_ERR;
	}
	
	(*pCurOffset) = (pHandle->handle.offset);
	return ret;
}
