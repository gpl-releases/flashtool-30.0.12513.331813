//*****************************************************************************
// This file is provided under a dual BSD/LGPLv2.1 license.  When using 
// or redistributing this file, you may do so under either license.
//
// LGPL LICENSE SUMMARY
//
// Copyright(c) <2011-2012>. Intel Corporation.
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
// Copyright (c) <2011-2012>. Intel Corporation. 
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

/*
 * Flashtool shim layer
 * Transform access from flashtool library to mtd library
*/
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <dirent.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <mtd/mtd-user.h>
#include "common.h"

#include "mtdlib.h"
#include "flash_shim.h"


struct mtd_info_user mtdinfo;


#define SECRET_BLK_NUM 			2
#define SECRET_START_BLKNO 		(MTD_BLOCK_NUMBER - SECRET_BLK_NUM -1)
#define SECRET_END_BLKNO 		(MTD_BLOCK_NUMBER -1)

#define MTD_BLOCK_SIZE			(mtdinfo.erasesize)
#define CAL_BLOCK_NUMBER(addr)	((unsigned int)(addr)/MTD_BLOCK_SIZE)
#define CAL_BLOCK_OFFSET(addr)	((unsigned int)(addr)%MTD_BLOCK_SIZE)
#define MTD_BLOCK_NUMBER		((unsigned int)mtdinfo.size/MTD_BLOCK_SIZE)

/* Detect mtd device node in sysfs */
#define MAX_PATH_LENGTH		256
#define MAX_FILE_LENGTH		274
#define SYSFS_MTD_DIR        	"/sys/class/mtd"
#define MTD_NAME_PATT    	"mtd%d"
#define DEV_DIR    		"/dev"
#define TARGET_MTD_DEVICE_NAME 	"nmyx25"

static int read_data_from_file(const char *file, void *buf,int buf_len,int dst_len)
{
   int fd, rd;
   if (!file)
	return -1;
   if (!buf)
	return -1;

   if ( (buf_len - 1) < dst_len) 
	return -1;

   fd = open(file, O_RDONLY);
   if (fd == -1)
      return -1;

   rd = read(fd, buf, dst_len);
   if (rd == -1) 
   {
      errmsg("cannot read \"%s\"", file);
      goto out_error;
   }

   ((char *)buf)[rd] = '\0';

   if (close(fd)) 
   {
       errmsg("close failed on \"%s\"", file);
       return -1;
   }

   return rd;

out_error:
   close(fd);
   return -1;
}
int detect_mtd_dev_node(char *dev_node)
{
	DIR *sysfs_mtd;
	struct dirent *dirent;	
	int errno;
	int present = 0;
	/*
	 * We have to scan the MTD sysfs directory to identify how many MTD
	 * devices are present.
	 */
	sysfs_mtd = opendir(SYSFS_MTD_DIR);
	if (!sysfs_mtd) 
	{
		if (errno == ENOENT) 
		{
			errno = ENODEV;
			return -1;
		}
		return errmsg("cannot open \"%s\"", SYSFS_MTD_DIR);
	}

	while (1) 
	{
		int mtd_num, ret;
		char tmp_buf[MAX_PATH_LENGTH];
		char file[MAX_FILE_LENGTH];
		char file_content[MAX_PATH_LENGTH];

		errno = 0;
		dirent = readdir(sysfs_mtd);
		if (!dirent)
			break;

		if (strlen(dirent->d_name) >= (MAX_PATH_LENGTH-1)) 
		{
			errmsg("invalid entry in %s: \"%s\"",
			       SYSFS_MTD_DIR, dirent->d_name);
			errno = EINVAL;
			goto out_close;
		}

		ret = sscanf(dirent->d_name, MTD_NAME_PATT"%s",
			     &mtd_num, tmp_buf);
		if (ret == 1) 
		{
			snprintf(file,MAX_FILE_LENGTH,"%s/%s/%s",SYSFS_MTD_DIR,dirent->d_name,"name");
			if (read_data_from_file(file,file_content,sizeof(file_content),strlen(TARGET_MTD_DEVICE_NAME))>0)
			{
				/*
					Check whether nmyx25 mtd device is present
				*/
				if (!strcmp(file_content,TARGET_MTD_DEVICE_NAME))
				{
					sprintf(dev_node,"%s/%s","/dev",dirent->d_name);
					infomsg("Got file name %s, [%s], %s\n",file,file_content,dev_node);
					present = 1;
					break;
				}
			}
		}
	}

	if (!dirent && errno) {
		errmsg("readdir failed on \"%s\"", SYSFS_MTD_DIR);
		goto out_close;
	}

	if (closedir(sysfs_mtd))
		return errmsg("closedir failed on \"%s\"", SYSFS_MTD_DIR);

	if (present)
		return 0;
	else
	{
		printf("No SPI NOR flash MTD device node found in \"%s\" \n", SYSFS_MTD_DIR);
		return -1;
	}

out_close:
	closedir(sysfs_mtd);
	return -1;
}

static int	flash2mtd_erase(int fd, unsigned long addr)
{
	int eb	=	CAL_BLOCK_NUMBER(addr);
	
	infomsg("erase block: blk number %d\n",eb);

	if (mtd_erase(&mtdinfo,fd,eb))
	{
		errmsg("erase block: blk number %d\n",eb);
		return -1;
	}

	return 0;
}
static int flash2mtd_readinfo(int fd,intel_ce_flash_info_t * info)
{

	if (mtd_get_info(&mtdinfo,fd))
		return -1;
	info->parameter		= mtdinfo.flags;
	info->size_in_bytes	= mtdinfo.size;
	info->blocksize		= MTD_BLOCK_SIZE;
	info->blocksize2	= 0;
	infomsg("read info: size_in_bytes %dMB, block_size 0x%xKB, block_size2 0x%xKB, parameter %d\n",info->size_in_bytes>>20,
		info->blocksize>>10,info->blocksize2>>10,info->parameter);

	return 0;
}
static int flash2mtd_read_block(int fd, intel_ce_flash_cmd_t *rwcmd)
{
	int len		=	MTD_BLOCK_SIZE;
	char *buf	=	rwcmd->buf;
	int eb 		=	rwcmd->blockno;
	int offs	=	0;
	infomsg("rwcmd length 0x%x, eb 0x%d, offs 0x%x\n",len,eb,offs);
	if (mtd_read(&mtdinfo,fd,eb,offs,buf,len))
	{
		errmsg("mtd_read");
		return -1;
	}
	else
		infomsg("read block: blk number %d, offs 0x%x, len0x%x\n",eb,offs,len);
	return 0;
}
static int flash2mtd_read_bytes(int fd, intel_ce_flash_cmd_t *rwcmd)
{
	int len		=	rwcmd->length;
	char *buf	=	rwcmd->buf;
	int eb 		=	CAL_BLOCK_NUMBER(rwcmd->offset);
	int offs	=	CAL_BLOCK_OFFSET(rwcmd->offset);
	infomsg("rwcmd length 0x%x, eb 0x%d, offs 0x%x\n",len,eb,offs);

	
	if (mtd_read(&mtdinfo,fd,eb,offs,buf,len))
	{
		errmsg("mtd_read");
		return -1;
	}
	else
		infomsg("read bytes: blk number %d, offs 0x%x, len0x%x\n",eb,offs,len);
	return 0;
}
static int flash2mtd_read_secret_bytes_internal(int fd, intel_ce_flash_cmd_t *rwcmd)
{
	int len		=	rwcmd->length;
	char *buf	=	rwcmd->buf;
	int eb 		=	SECRET_START_BLKNO + CAL_BLOCK_NUMBER(rwcmd->offset);
	int offs	=	CAL_BLOCK_OFFSET(rwcmd->offset);


	if (mtd_read(&mtdinfo,fd,eb,offs,buf,len))
	{
		errmsg("%s: blk number %d, offs 0x%x, len0x%x\n",__FUNCTION__,eb,offs,len);
		return -1;
	}
	else
		infomsg("%s: blk number %d, offs 0x%x, len0x%x\n",__FUNCTION__,eb,offs,len);
	return 0;
}
static int flash2mtd_write_block_buf(int fd, intel_ce_flash_cmd_t *rwcmd)
{
	int err;
	int len		=	MTD_BLOCK_SIZE;
	char *buf	=	rwcmd->buf;
	int eb 		=	rwcmd->blockno;
	int offs	=	0;
	infomsg("rwcmd length 0x%x, eb 0x%d, offs 0x%x\n",len,eb,offs);

	err = mtd_write(&mtdinfo,fd,eb,offs,buf,len);
	if (err)
	{	
		errmsg("%s: blk number %d, offs 0x%x, len0x%x\n",__FUNCTION__,eb,offs,len);
		return -1;
	}
	return 0;
}
static int flash2mtd_write_secret_bytes_internal(int fd, intel_ce_flash_cmd_t *rwcmd)
{
	int err,i;
	int len		=	rwcmd->length;
	char *buf	=	rwcmd->buf;
	int statblk =	SECRET_START_BLKNO + CAL_BLOCK_NUMBER(rwcmd->offset);
	int endblk	=	SECRET_START_BLKNO + CAL_BLOCK_NUMBER(rwcmd->offset+rwcmd->length);
	char *tmp_buf	= malloc(MTD_BLOCK_SIZE*(endblk-statblk+1));
	char *p 	= 	tmp_buf;
	infomsg("rwcmd length 0x%x, startblk 0x%d, endblk 0x%x\n",len,statblk,endblk);
	for (i=statblk;i<=endblk;i++)
	{
		err=mtd_read(&mtdinfo,fd,i,0,p,MTD_BLOCK_SIZE);
		if (err)
			goto err_out;
		p+=MTD_BLOCK_SIZE;	
	}
	memcpy(tmp_buf+rwcmd->offset,buf,len);
	
	p	=	tmp_buf;
	for (i=statblk;i<=endblk;i++)
	{
		err=mtd_erase(&mtdinfo,fd,i);
		if (err)
			goto err_out;
		err=mtd_write(&mtdinfo,fd,i,0,p,MTD_BLOCK_SIZE);
		if (err)
			goto err_out;
		p+=MTD_BLOCK_SIZE;	
	}
	
	free(tmp_buf);
	return 0;
err_out:
	errmsg("%s: blk number %d, len0x%x\n",__FUNCTION__,statblk,len);
	free(tmp_buf);
	return -1;
}
static int flash2mtd_get_map(int fd, memory_map_t *arg)
{
	return 0;
}
int flash2mtd_ioctl(int fd, unsigned long cmd, unsigned long arg)
{
	int status = -EINVAL;

	infomsg("cmd=%lu arg=0x%08lx\n", cmd, arg);
	
	switch(cmd)
	{
		case INTEL_CE_FLASH_ERASE:
			status = flash2mtd_erase(fd, arg);
			break;
		case INTEL_CE_FLASH_READINFO:
			status = flash2mtd_readinfo(fd,(intel_ce_flash_info_t *)arg);
			break;
		case INTEL_CE_FLASH_READ_BLOCK:
			status = flash2mtd_read_block(fd, (intel_ce_flash_cmd_t *)arg);
			break;
		case INTEL_CE_FLASH_READ_BYTES:
			status = flash2mtd_read_bytes(fd, (intel_ce_flash_cmd_t *)arg);
			break;
		case INTEL_CE_FLASH_READ_SECRET_BYTES:
			status = flash2mtd_read_secret_bytes_internal(fd, (intel_ce_flash_cmd_t *)arg);
			break;
		case INTEL_CE_FLASH_WRITE_SECRET_BYTES:
			status = flash2mtd_write_secret_bytes_internal(fd, (intel_ce_flash_cmd_t *)arg);
			break;
		case INTEL_CE_FLASH_WRITE_BLOCK:
			status = flash2mtd_write_block_buf(fd, (intel_ce_flash_cmd_t *)arg);
			break;
		case INTEL_CE_FLASH_GET_MAP:
			status = flash2mtd_get_map(fd, (memory_map_t *)arg);
			break;
		default:
			errmsg("error cmd %lu\n",cmd);
			break;
	}

	return status;
}

