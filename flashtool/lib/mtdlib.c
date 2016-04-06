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

/* Simple MTD library */


#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <time.h>
#include <sys/ioctl.h>
#include <sys/mount.h>

#include <mtd/mtd-user.h>

#include "mtdlib.h"


static int mtd_valid_erase_block(const struct mtd_info_user *mtd, int eb)
{
	int eb_cnt = mtd->size/mtd->erasesize;
	if (eb < 0 || eb >= eb_cnt) {
		errmsg("bad eraseblock number %d, mtd has %d eraseblocks",
		       eb, eb_cnt);
		return -1;
	}
	return 0;
}

int mtd_get_info(struct mtd_info_user *mtd, int fd)
{

	if (fd < 0) {
		perror("open");
		exit(1);
	}
	memset((void *)mtd, 0, sizeof(*mtd));

	if (ioctl(fd, MEMGETINFO, mtd)) {
		perror("MEMGETINFO");
		exit(-1);
	}
	infomsg("size 0x%x, erasesize 0x%x",mtd->size, mtd->erasesize);
	return 0;
}


int mtd_read(const struct mtd_info_user *mtd, int fd, int eb, int offs,
	     void *buf, int len)
{
	int ret, rd = 0;
	off_t seek;

	infomsg("fd %d, eb %d, offs 0x%x, len 0x%x\n",fd,eb,offs,len);
	if(!buf)
		return -1;
	ret = mtd_valid_erase_block(mtd, eb);
	if (ret)
		return ret;

	/* Seek to the beginning of the eraseblock */
	seek = (off_t)eb * mtd->erasesize + offs;
	if (!fd)
		errmsg("bad file name");
	
	if (offs < 0 || seek + len > mtd->size) {
		errmsg("bad offset %d or length %d, mtd%d size is %d",
		       offs, len, mtd->flags, mtd->size);

		return -1;
	}

	if (lseek(fd, seek, SEEK_SET) != seek)
		return errmsg("cannot seek mtd%d to offset %llu",
				  mtd->flags, (unsigned long long)seek);

	while (rd < len) {
		ret = read(fd, buf, len);
		if (ret < 0)
			return errmsg("cannot read %d bytes from mtd (eraseblock %d, offset %d)",
					  len,  eb, offs);
		rd += ret;
		buf += ret;
	}

	return 0;
}
	
int mtd_write(const struct mtd_info_user *mtd, int fd, int eb, int offs,
			  void *buf, int len)
{
	int ret;
	off_t seek;
	infomsg("fd %d, eb %d, offs 0x%x, len 0x%x\n",fd,eb,offs,len);
	if(!buf)
		return -1;
	ret = mtd_valid_erase_block(mtd, eb);
	if (ret)
		return ret;
	
	if (offs < 0 || offs + len > mtd->erasesize) {
		errmsg("bad offset %d or length %d, mtd%d eraseblock size is %d",
			   offs, len, mtd->flags, mtd->erasesize);

		return -1;
	}
	if (offs % mtd->writesize) {
		errmsg("write offset %d is not aligned to mtd%d min. I/O size %d",
			   offs, mtd->flags, mtd->writesize);

		return -1;
	}
	if (len % mtd->writesize) {
		errmsg("write length %d is not aligned to mtd%d min. I/O size %d",
			   len, mtd->flags, mtd->writesize);

		return -1;
	}
	
	/* Seek to the beginning of the eraseblock */
	seek = (off_t)eb * mtd->erasesize + offs;
	if (lseek(fd, seek, SEEK_SET) != seek)
		return errmsg("cannot seek mtd%d to offset %llu",
				  mtd->flags, (unsigned long long)seek);

	ret = write(fd, buf, len);
	if (ret != len)
		return errmsg("cannot write %d bytes to mtd%d (eraseblock %d, offset %d)",
				  len, mtd->flags, eb, offs);
	
	return 0;
}
	
int mtd_erase(const struct mtd_info_user *mtd, int fd, int eb)
{
	int ret;

	struct erase_info_user ei;
	
	ret = mtd_valid_erase_block(mtd, eb);
	if (ret)
		return ret;
		
	ei.start = eb * mtd->erasesize;
	ei.length = mtd->erasesize;
	ret = ioctl(fd, MEMERASE, &ei);
	if (ret < 0)
		return errmsg("cannot erase block %d to mtd%d (offset %d)",
				  eb, mtd->flags, ei.start);
	return 0;
}


