/*
*This file is provided under a dual BSD/GPLv2 license.  When using or
*redistributing this file, you may do so under either license.
*
*GPL LICENSE SUMMARY
*
*Copyright (c) 2005-2009. Intel Corporation. All rights reserved.
*
*This program is free software; you can redistribute it and/or modify
*it under the terms of version 2 of the GNU General Public License as
*published by the Free Software Foundation.
*
*This program is distributed in the hope that it will be useful, but
*WITHOUT ANY WARRANTY; without even the implied warranty of
*MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
*General Public License for more details.
*
*You should have received a copy of the GNU General Public License
*along with this program; if not, write to the Free Software
*Foundation, Inc., 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
*The full GNU General Public License is included in this distribution
*in the file called LICENSE.GPL.
*
*Contact Information:
*Intel Corporation
*2200 Mission College Blvd.
*Santa Clara, CA  97052
*
*BSD LICENSE
*
*Copyright (c) 2005-2009. Intel Corporation. All rights reserved.
*All rights reserved.
*
*Redistribution and use in source and binary forms, with or without
*modification, are permitted provided that the following conditions
*are met:
*
*  * Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*  * Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in
*    the documentation and/or other materials provided with the
*    distribution.
*  * Neither the name of Intel Corporation nor the names of its
*    contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
*THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
*A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
*OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
*SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
*LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
*DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
*THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
*(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/


#ifndef _INTEL_CE_FLASH_H
#define _INTEL_CE_FLASH_H

#define INTEL_CE_FLASH_MAJOR	223

/* Intel P30 Flash block size*/ 
#define INTEL_CE_P30FLASH_BLOCK		0x20000
#define INTEL_CE_P30FLASH_BLOCK2	0x8000

/* Intel P30 Flash commands and status */ 
#define FLASH_BLOCK_ERASE	0x20
#define FLASH_PROGRAM		0x40
#define FLASH_UNLOCK		0x60
#define FLASH_STATUS_READY	0x80
#define FLASH_CMD_CONFIRM	0xD0
#define FLASH_READ_STATUS	0x70
#define FLASH_READ_ARRAY_CMD	0xFF
#define FLASH_READ_INFO		0x90
#define FLASH_PROGRAM_BUF	0xE8

/* Spansion  Flash commands and status */ 
#define DEV_MULTI(cmd)						(cmd)

#define FLASH_AUTOSELECT_CMD				DEV_MULTI(0x90)
#define FLASH_CFI_QUERY_CMD					DEV_MULTI(0x98)
#define FLASH_CHIP_ERASE_CMD				DEV_MULTI(0x10)
#define FLASH_ERASE_SETUP_CMD				DEV_MULTI(0x80)
#define FLASH_PROGRAM_CMD					DEV_MULTI(0xA0)
#define FLASH_RESET_CMD						DEV_MULTI(0xF0)
#define FLASH_SECSI_SECTOR_ENTRY_CMD		DEV_MULTI(0x88)
#define FLASH_SECSI_SECTOR_EXIT_SETUP_CMD	DEV_MULTI(0x90)
#define FLASH_SECSI_SECTOR_EXIT_CMD			DEV_MULTI(0x00)
#define FLASH_SECTOR_ERASE_CMD				DEV_MULTI(0x30)
#define FLASH_UNLOCK_BYPASS_ENTRY_CMD		DEV_MULTI(0x20)
#define FLASH_UNLOCK_BYPASS_PROGRAM_CMD		DEV_MULTI(0xA0)
#define FLASH_UNLOCK_BYPASS_RESET_CMD1		DEV_MULTI(0x90)
#define FLASH_UNLOCK_BYPASS_RESET_CMD2		DEV_MULTI(0x00)
#define FLASH_UNLOCK_DATA1					DEV_MULTI(0xAA)
#define FLASH_UNLOCK_DATA2					DEV_MULTI(0x55)
#define FLASH_WRITE_BUFFER_ABORT_RESET_CMD	DEV_MULTI(0xF0)
#define FLASH_WRITE_BUFFER_LOAD_CMD			DEV_MULTI(0x25)
#define FLASH_WRITE_BUFFER_PGM_CONFIRM_CMD	DEV_MULTI(0x29) 
#define FLASH_SUSPEND_CMD					DEV_MULTI(0xB0)
#define FLASH_RESUME_CMD					DEV_MULTI(0x30)
#define FLASH_SET_CONFIG_CMD				DEV_MULTI(0xD0)
#define FLASH_READ_CONFIG_CMD				DEV_MULTI(0xC6)
#define FLASH_STATUS_REG_READ_CMD			DEV_MULTI(0x70)


#define DQ1_MASK   DEV_MULTI(0x02)  /* DQ1 mask for all interleave devices */
#define DQ2_MASK   DEV_MULTI(0x04)  /* DQ2 mask for all interleave devices */
#define DQ5_MASK   DEV_MULTI(0x20)  /* DQ5 mask for all interleave devices */
#define DQ6_MASK   DEV_MULTI(0x40)  /* DQ6 mask for all interleave devices */


#define FLASH_BANK_WIDTH 2
#define FLASH_OFFSET(o) (FLASH_BANK_WIDTH * (o))
/* Spansion Flash commands related addresses */
#define FLASH_UNLOCK_ADDR1   FLASH_OFFSET(0x00000555)
#define FLASH_UNLOCK_ADDR2   FLASH_OFFSET(0x000002AA)

/* Spansion Flash status */
#define FLASH_DEV_NOT_BUSY	0x0
#define FLASH_DEV_BUSY		0x1

#include "common.h"

#ifdef INTEL_CE_FLASH_DEBUG
#   define DPRINTK(fmt, args...) printk("%s: " fmt, __FUNCTION__ , ## args)
#else
#   define DPRINTK(fmt, args...)
#endif

#define CFI_MFR_ANY 0xFFFF
#define CFI_MFR_AMD 0x0001
#define CFI_MFR_ATMEL 0x001F
#define CFI_MFR_ST  0x0020 	/* STMicroelectronics */

#define CFI_ID_ANY  0xFFFF

#define P_ID_NONE               0x0000
#define P_ID_INTEL_EXT          0x0001
#define P_ID_AMD_STD            0x0002
#define P_ID_INTEL_STD          0x0003
#define P_ID_AMD_EXT            0x0004
#define P_ID_WINBOND            0x0006
#define P_ID_ST_ADV             0x0020
#define P_ID_MITSUBISHI_STD     0x0100
#define P_ID_MITSUBISHI_EXT     0x0101
#define P_ID_SST_PAGE           0x0102
#define P_ID_INTEL_PERFORMANCE  0x0200
#define P_ID_INTEL_DATA         0x0210
#define P_ID_RESERVED           0xffff
/* Flash basic Operations */
#define FLASH_WR(base, offset, value) (*(typeof(base))((unsigned long)(base) + offset)) = (value)
#define FLASH_RD(base, offset) (*(typeof(base))((unsigned long)(base) + offset))

#define INTEL_CE_PERROR(fmt, args...) printk("[ERROR] %s: " fmt, __FUNCTION__ , ## args)
#define MAX_BUFSIZE		32

// Basic Query Structure 
struct cfi_ident {
	__u8  qry[3];
	__u16 P_ID;
	__u16 P_ADR;
	__u16 A_ID;
	__u16 A_ADR;
	__u8  VccMin;
	__u8  VccMax;
	__u8  VppMin;
	__u8  VppMax;
	__u8  WordWriteTimeoutTyp;
	__u8  BufWriteTimeoutTyp;
	__u8  BlockEraseTimeoutTyp;
	__u8  ChipEraseTimeoutTyp;
	__u8  WordWriteTimeoutMax;
	__u8  BufWriteTimeoutMax;
	__u8  BlockEraseTimeoutMax;
	__u8  ChipEraseTimeoutMax;
	__u8  DevSize;
	__u16 InterfaceDesc;
	__u16 MaxBufWriteSize;
	__u8  NumEraseRegions;
	__u32 EraseRegionInfo[4]; /* Not host ordered */
} __attribute__((packed));

struct cfi_fixup_table
{
    __u16 mfr;
    __u16 id;
    void (*fixup)(struct cfi_ident *cfi_ident_fixup, void* param);
    void* param;

};

// Flahs chip type 
typedef enum
{
	ID_NONE ,
	INTEL_EXT,
	AMD_STD,
	INTEL_STD,
	AMD_EXT,
	WINBOND,
	ST_ADV,
	MITSUBISHI_STD,
	MITSUBISHI_EXT,
	SST_PAGE,
	INTEL_PERFORMANCE,
	INTEL_DATA,
	RESERVED 
}ChipType;

unsigned long  INTEL_CE_FLASH_BASE_VIRT;
unsigned long  INTEL_CE_EXP_TIMING_CS0;

#endif
