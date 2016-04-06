/*
 * GPL LICENSE SUMMARY
 * Copyright (c) 2008-2012, Intel Corporation and its suppliers.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 *
 */

#include <linux/version.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/pci.h>
#include <linux/pci_ids.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/miscdevice.h>
#include <linux/mm.h>
#include <linux/vmalloc.h>
#include <linux/delay.h>
#include <linux/wait.h>
#include <linux/sched.h>

#if LINUX_VERSION_CODE == KERNEL_VERSION(2, 6, 23)
#include <asm/semaphore.h>
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 28)
#include <linux/semaphore.h>
#endif

#include <asm/uaccess.h>
//#include <asm/hardware.h>

#ifdef LINUX
char *version_string = "#@# intel-ce-flash.ko " VER;
#endif

//#define INTEL_CE_FLASH_DEBUG

#include "intel_ce_flash.h"
#define SECRET_BLK_NUM 2	//we reserved 2 blocks as secret area
#define SECRET_START memory_map[memory_map_count - SECRET_BLK_NUM].start
#define SECRET_END memory_map[memory_map_count - 1].end

#define DOWN_RW_SEM  do{\
		if(down_interruptible(&rw_sem))\
		{\
			INTEL_CE_PERROR("error in getting semaphore\n");\
			return -ERESTARTSYS;\
		}\
	}while(0)

#define UP_RW_SEM do{\
		up(&rw_sem);\
	}while(0)
static struct semaphore rw_sem;
static int bitmode = 8;
ChipType chip_type = ID_NONE;

memory_map_item_t memory_map[MAX_MEMORY_MAP];
static int memory_map_count = 0;
static int write_buffer_size = 1;

static intel_ce_flash_info_t flash_info;

static struct cfi_ident intel_ce_cfi_ident;

static void fixup_st_M29W800DT(struct cfi_ident *cfi_ident_fixup, void *param)
{
    uint32_t t;

    t = cfi_ident_fixup->EraseRegionInfo[0];
    cfi_ident_fixup->EraseRegionInfo[0] = cfi_ident_fixup->EraseRegionInfo[3];
    cfi_ident_fixup->EraseRegionInfo[3] = t;
    t = cfi_ident_fixup->EraseRegionInfo[1];
    cfi_ident_fixup->EraseRegionInfo[1] = cfi_ident_fixup->EraseRegionInfo[2];
    cfi_ident_fixup->EraseRegionInfo[2] = t;
}

static struct cfi_fixup_table cfi_generic_fixup[] =
{
    { 0x0020, 0x22D7, fixup_st_M29W800DT, NULL },
    { 0, 0, NULL, NULL }
};

void cfi_fixup(struct cfi_ident *cfi_ident_fixup, struct cfi_fixup_table *table, uint16_t mfr, uint16_t id)
{
    struct cfi_fixup_table *f;

    for (f=table; f->fixup; f++) {
        if (((f->mfr == CFI_MFR_ANY) || (f->mfr == mfr)) &&
            ((f->id  == CFI_ID_ANY)  || (f->id  == id))) {
            f->fixup(cfi_ident_fixup, f->param);
        }
    }
}



int intel_ce_flash_reset(void)
{
	u16 volatile * flashBase = (u16 *)INTEL_CE_FLASH_BASE_VIRT;
	
	DPRINTK("reset\n");
	switch (chip_type)
	{
		case INTEL_EXT:
			FLASH_WR(flashBase, FLASH_OFFSET(0), FLASH_READ_ARRAY_CMD);
		break;
		case AMD_STD:
			FLASH_WR(flashBase, FLASH_OFFSET(0), FLASH_RESET_CMD);
		break;
		default:
		DPRINTK("unrecognized chip\n");
			return -EINVAL;		
	}
	return 0;
}
/*Only for p30 flash use */
static int intel_ce_flash_reset_block_p30(volatile u16 * blockaddr)
{
	if (INTEL_EXT != chip_type)
		return -EINVAL;
	DPRINTK("reset\n");

    	*blockaddr = FLASH_READ_ARRAY_CMD;	// return to data mode
	return 0;
}

static int _flash_probe(void)
{
	int i;
	volatile u16 * flashBase = (u16 *)INTEL_CE_FLASH_BASE_VIRT;
	int dev_size = 0;
	u16 mfr_id, dev_id;


	/* CFI Query */
	FLASH_WR(flashBase, FLASH_OFFSET(0x55), FLASH_CFI_QUERY_CMD);
	msleep(1);
	for(i = 0; i < sizeof(struct cfi_ident); i++)
	{
		((u8 *)&intel_ce_cfi_ident)[i] = 
			(u8)FLASH_RD(flashBase, FLASH_OFFSET(0x10 + i));
	}
	intel_ce_flash_reset();


	if(intel_ce_cfi_ident.qry[0] != 'Q' || 
		intel_ce_cfi_ident.qry[1] != 'R' || 
		intel_ce_cfi_ident.qry[2] != 'Y')
	{
		printk(KERN_DEBUG "qry: %02x %02x %02x\n",
			intel_ce_cfi_ident.qry[0],
			intel_ce_cfi_ident.qry[1],
			intel_ce_cfi_ident.qry[2]);
		return -ENODEV;
	}
	switch (intel_ce_cfi_ident.P_ID)
	{
		case P_ID_INTEL_EXT:
			chip_type = INTEL_EXT;
			DPRINTK("Intel Extended flash found\n");
			/* Read manufacturer ID and device ID */
			*flashBase = FLASH_READ_INFO;
			mfr_id = *((u16 *)flashBase);
			dev_id = *((u16 *)(flashBase + 1));
			intel_ce_flash_reset();
			break;
		case P_ID_AMD_STD:
			chip_type = AMD_STD;
			DPRINTK("Spansion flash found\n");			
			/* Read manufacturer ID and device ID */
			FLASH_WR(flashBase, FLASH_UNLOCK_ADDR1, FLASH_UNLOCK_DATA1);
			FLASH_WR(flashBase, FLASH_UNLOCK_ADDR2, FLASH_UNLOCK_DATA2);

			FLASH_WR(flashBase, FLASH_UNLOCK_ADDR1, FLASH_AUTOSELECT_CMD);

			mfr_id = FLASH_RD(flashBase, FLASH_OFFSET(0x0));
			dev_id = FLASH_RD(flashBase, FLASH_OFFSET(0x1));		       
			intel_ce_flash_reset();
			break;
		default:
			chip_type = ID_NONE;
			printk(KERN_DEBUG "Unknown P_ID: %d\n", intel_ce_cfi_ident.P_ID);
			return -EIO;
	}


	cfi_fixup(&intel_ce_cfi_ident, cfi_generic_fixup, mfr_id, dev_id);

	/* do byte swapping */
	intel_ce_cfi_ident.P_ID = le16_to_cpu(intel_ce_cfi_ident.P_ID);
	intel_ce_cfi_ident.P_ADR = le16_to_cpu(intel_ce_cfi_ident.P_ADR);
	intel_ce_cfi_ident.A_ID = le16_to_cpu(intel_ce_cfi_ident.A_ID);
	intel_ce_cfi_ident.A_ADR = le16_to_cpu(intel_ce_cfi_ident.A_ADR);
	intel_ce_cfi_ident.InterfaceDesc = 
		le16_to_cpu(intel_ce_cfi_ident.InterfaceDesc);
	intel_ce_cfi_ident.MaxBufWriteSize = 
		le16_to_cpu(intel_ce_cfi_ident.MaxBufWriteSize);
	/*get the write_buffer_size in byte mode according to MaxBufWriteSize*/	
	for (i = 0; i < intel_ce_cfi_ident.MaxBufWriteSize; i++){
		write_buffer_size *= 2;
	}
	/*get the write_buffer_size in word mode if flash support buffer write operation*/
	if(intel_ce_cfi_ident.MaxBufWriteSize){
		write_buffer_size = write_buffer_size/2;
	}
	for (i = 0; i < intel_ce_cfi_ident.NumEraseRegions; i++) 
	{
		intel_ce_cfi_ident.EraseRegionInfo[i] = 
			le32_to_cpu(intel_ce_cfi_ident.EraseRegionInfo[i]);
	}

	/* initialize flash_info */
	flash_info.blocksize = 0;
	/* FIXME */
	/* What if there are three different kinds of block size? */
	flash_info.blocksize2 = ~0;
	for (i = 0; i < intel_ce_cfi_ident.NumEraseRegions; i++) 
	{
		unsigned long ernum, ersize;
		ersize = ((intel_ce_cfi_ident.EraseRegionInfo[i] >> 8) & ~0xff);
		ernum = (intel_ce_cfi_ident.EraseRegionInfo[i] & 0xffff) + 1;

		if (flash_info.blocksize < ersize) 
		{
			flash_info.blocksize = ersize;
		}

		if (flash_info.blocksize2 > ersize) 
		{
			flash_info.blocksize2 = ersize;
		}

		dev_size += ersize * ernum;
	}
	if (flash_info.blocksize == flash_info.blocksize2)
		flash_info.blocksize2 = 0;
	
	flash_info.size_in_bytes = (1 << intel_ce_cfi_ident.DevSize);
	if (dev_size != flash_info.size_in_bytes)
	{
		printk("Chip size inconsistent 0x%x != 0x%x.\n",
			dev_size, flash_info.size_in_bytes);
		return -EIO;
	}
	
	return 0;
}
// P30 flash read status
inline u8 intel_ce_flash_readstatus_p30(void)
{
	volatile u16 * flashBase = (u16 *)INTEL_CE_FLASH_BASE_VIRT;
	u16 status;
	
	FLASH_WR(flashBase, FLASH_OFFSET(0x0), FLASH_READ_STATUS);
	status = FLASH_RD(flashBase, FLASH_OFFSET(0x0));
	
	return (u8)status;
}
static inline u16 _get_status(int offset)
{
	u16 volatile * flashBase = (u16 *)INTEL_CE_FLASH_BASE_VIRT;
	return FLASH_RD(flashBase, offset);
}
//Spansion flash get status
static int intel_ce_flash_readstatus_spansion(int offset)
{
	u16 s1, s2, s3;
	u16 t;

	s1 = _get_status(offset);
	s2 = _get_status(offset);
	s3 = _get_status(offset);

	t = ((s1 ^ s2) &		/* Toggles between read1 and read2 */ 
		(s2 ^ s3) &			/* Toggles between read2 and read3 */
		DQ6_MASK );			/* Check for DQ6 only */

	if(t)
	{
		return FLASH_DEV_BUSY;
	}
	else
	{
		s1 = _get_status(offset);
		s2 = _get_status(offset);

		if ( ((s1 ^ s2) & DQ2_MASK) == 0)     
			return FLASH_DEV_NOT_BUSY;	/* All devices DQ2 not toggling */

		return FLASH_DEV_BUSY;			/* Wait for all devices DQ2 toggling */
	}
}

static inline void intel_ce_flash_waitready_p30(void)
{
	volatile u16 * flashBase = (u16 *)INTEL_CE_FLASH_BASE_VIRT;
	
	while(0 == ((*flashBase) & FLASH_STATUS_READY))
		yield();
}

static inline void intel_ce_flash_waitready_spainsion(int offset)
{
	u32 retry = ~0;

	for(; retry != 0; retry--)
	{
		if(FLASH_DEV_NOT_BUSY == intel_ce_flash_readstatus_spansion(offset))
			break;
		yield();
	}
}

void intel_ce_flash_waitready(int offset)
{
	switch(chip_type)
	{
	case INTEL_EXT:
		intel_ce_flash_waitready_p30();
		break;
	case AMD_STD:
		intel_ce_flash_waitready_spainsion(offset);
		break;
	default:
		INTEL_CE_PERROR("Unrecongnzied chip\n");
		intel_ce_flash_reset();
	}	
}
/*
 * Please hold semaphore before calling this function.
 */
 //erase one block on P30 flash
int _erase_one_block_p30(int offset)
{
	u16 volatile * flashBase = (u16 *)(INTEL_CE_FLASH_BASE_VIRT+offset);
	u8 status = 0;
	
	if (INTEL_EXT != chip_type)
		return -EINVAL;

	DPRINTK("unlocking 0x%08x\n", offset);
    	*flashBase = FLASH_UNLOCK;
    	*flashBase = FLASH_CMD_CONFIRM;
    //DPRINTK("waiting for ready...\n");
    	status = intel_ce_flash_readstatus_p30();
    	intel_ce_flash_waitready(offset);
    	intel_ce_flash_reset_block_p30(flashBase);

	DPRINTK("erasing 0x%08x\n", offset);
    	*flashBase = FLASH_BLOCK_ERASE;
    	*flashBase = FLASH_CMD_CONFIRM;
    
    //DPRINTK("waiting for ready...\n");
    	status = intel_ce_flash_readstatus_p30();
    	intel_ce_flash_waitready(offset);
    
    	status = intel_ce_flash_readstatus_p30();
    	intel_ce_flash_reset_block_p30(flashBase);

	DPRINTK("status=0x%x\n", status);

    	return status;
}

 //erase one block on spansion flash
int _erase_one_block_spansion(int offset)
{
	u16 volatile * flashBase = (u16 *)INTEL_CE_FLASH_BASE_VIRT;

	if (AMD_STD!= chip_type)
		return -EINVAL;

	DPRINTK("unlocking 0x%08x\n", offset);
	FLASH_WR(flashBase, FLASH_UNLOCK_ADDR1, FLASH_UNLOCK_DATA1);
	FLASH_WR(flashBase, FLASH_UNLOCK_ADDR2, FLASH_UNLOCK_DATA2);

	FLASH_WR(flashBase, FLASH_UNLOCK_ADDR1, FLASH_ERASE_SETUP_CMD);

	FLASH_WR(flashBase, FLASH_UNLOCK_ADDR1, FLASH_UNLOCK_DATA1);
	FLASH_WR(flashBase, FLASH_UNLOCK_ADDR2, FLASH_UNLOCK_DATA2);

	DPRINTK("erasing 0x%08x\n", offset);
	FLASH_WR(flashBase, offset, FLASH_SECTOR_ERASE_CMD);
	
	//DPRINTK("waiting for ready...\n");
	intel_ce_flash_waitready(offset);
	
	intel_ce_flash_reset();

	if(FLASH_DEV_NOT_BUSY != intel_ce_flash_readstatus_spansion(offset))
	{
		u16 status = 0;
		
		status = _get_status(offset);
		DPRINTK("status=0x%x\n", status);
		return -EIO;
	}
	DPRINTK("end spansion 0x%08x\n", offset);
	return 0;
}
int _erase_one_block(int offset)
{
	u8 status = 0;
	int ret = -EINVAL;
	switch (chip_type)
	{
		case INTEL_EXT:
			status = _erase_one_block_p30(offset);
			ret = (FLASH_STATUS_READY & status)?0:(-EIO);
		break;
		case AMD_STD:
			status = _erase_one_block_spansion(offset);
			ret = (0 == status)?0:(-EIO);
		break;
		default:
			ret = -EINVAL;
	}
	return ret;
}
int intel_ce_flash_erase(int offset, int flags, int eflags)
{
	int ret = -EIO;
	if(eflags == 1)
	{
		if(offset >= SECRET_START)
			return -EINVAL; 
	}
	ret = _erase_one_block(offset);

	return ret;
}
/*
	Read and check p30 flash manufacture and device ID
	We can only support following kinds of P30 flash:
	0x8817,0x8818,0x8919,0x881a,0x881b,0x891c
*/
int intel_ce_flash_readinfo_p30(intel_ce_flash_info_t * pInfo)
{
	volatile u16 * flashBase = (u16 *)INTEL_CE_FLASH_BASE_VIRT;
	u16 tmp16;
	u16 device_id;

	if(INTEL_EXT != chip_type || !pInfo)
		return -EINVAL;
	
	*flashBase = FLASH_READ_INFO;
	tmp16 = *((u16 *)flashBase);
	DPRINTK("tmp16=0x%x\n", tmp16);
	
	if(0x89 != tmp16)
	{
		INTEL_CE_PERROR("Manufacturer ID not correct: 0x%x!\n", tmp16);
		intel_ce_flash_reset();
		return -EIO;
	}
	device_id = *((u16 *)(flashBase + 1));
	DPRINTK("device_id=0x%x\n", device_id);
	switch(device_id)
	{
		case 0x8817:
		case 0x8818:
		case 0x8919:
		case 0x881a:
		case 0x881b:
		case 0x891c:
			break;
		default:
			INTEL_CE_PERROR("Invalid device id: 0x%x\n", device_id);
			intel_ce_flash_reset();
			return -EIO;
	}

	intel_ce_flash_reset();
	copy_to_user((char *)(pInfo), (char *)(&flash_info), sizeof(flash_info));
	return 0;
}
/*Read and check spansion flash manufacture device ID*/
int intel_ce_flash_readinfo_spansion(intel_ce_flash_info_t * pInfo)
{
	volatile u16 * flashBase = (u16 *)INTEL_CE_FLASH_BASE_VIRT;
	u16 mfr_id, dev_id, dev_id2, dev_id3;

	if (AMD_STD!= chip_type || !pInfo)
		return -EINVAL;
	FLASH_WR(flashBase, FLASH_UNLOCK_ADDR1, FLASH_UNLOCK_DATA1);
	FLASH_WR(flashBase, FLASH_UNLOCK_ADDR2, FLASH_UNLOCK_DATA2);

	FLASH_WR(flashBase, FLASH_UNLOCK_ADDR1, FLASH_AUTOSELECT_CMD);

	mfr_id = FLASH_RD(flashBase, FLASH_OFFSET(0x0));
	dev_id = FLASH_RD(flashBase, FLASH_OFFSET(0x1));
	dev_id2 = FLASH_RD(flashBase, FLASH_OFFSET(0xE));
	dev_id3 = FLASH_RD(flashBase, FLASH_OFFSET(0xF));

	intel_ce_flash_reset();
	
       DPRINTK(KERN_INFO "mfr ID: %04x, dev ID: %04x %04x %04x\n",
		mfr_id, dev_id, dev_id2, dev_id3);
	/* Add code to check Spansion device id type here
	*/
	copy_to_user((char *)(pInfo), (char *)(&flash_info), sizeof(flash_info));
	return 0;

}
int intel_ce_flash_readinfo(intel_ce_flash_info_t * pInfo)
{
	int ret = -EINVAL;
	if(!pInfo)
		return -EINVAL;
	switch (chip_type)
	{
		case INTEL_EXT:
			ret = intel_ce_flash_readinfo_p30(pInfo);
		break;
		case AMD_STD:
			ret = intel_ce_flash_readinfo_spansion(pInfo);
			break;
		default:
			ret = -EINVAL;
	}

	return ret;
}
// Read some bytes
int intel_ce_flash_read_bytes(intel_ce_flash_cmd_t * pCmd)
{
	int ret = 0;
	unsigned long volatile flashBase = INTEL_CE_FLASH_BASE_VIRT;

	if(!pCmd)
		return -EINVAL;
	if(!(pCmd->buf))
		return -EINVAL;
	if(pCmd->offset < 0 || pCmd->offset >= SECRET_START || (pCmd->offset + pCmd->length) > SECRET_START)
	{
		INTEL_CE_PERROR("read out of flash range: offset=0x%x, size=0x%x\n",
			pCmd->offset, pCmd->length);
		return -EINVAL;
	}

	flashBase += (pCmd->offset);
        DPRINTK("flashBase=0x%lx\n", flashBase);
	copy_to_user((char *)(pCmd->buf), (char *)(flashBase), pCmd->length);
	return ret;
}
int intel_ce_flash_read_secret_bytes_internal(intel_ce_flash_cmd_t * pCmd, int flag)
{
	int ret = 0;
	int start = SECRET_START;
	int end = SECRET_END;
	
	unsigned long volatile flashBase = INTEL_CE_FLASH_BASE_VIRT + start;

	if(!pCmd)
		return -EINVAL;
	if(!(pCmd->buf))
		return -EINVAL;
	if((pCmd->offset < 0) || (pCmd->offset >= (end-start+1)) || ((pCmd->offset + pCmd->length) > (end-start +1)))   
	{
		INTEL_CE_PERROR("read out of flash range: offset=0x%x, size=0x%x\n",
			pCmd->offset, pCmd->length);
		return -EINVAL;
	}

	flashBase += (pCmd->offset);
        DPRINTK("flashBase=0x%lx\n", flashBase);
	if(flag == 1)
		copy_to_user((char *)(pCmd->buf), (char *)(flashBase), pCmd->length);
	else
		memcpy((char *)(pCmd->buf), (char *)(flashBase), pCmd->length);
	return ret;
}
int intel_ce_flash_read_secret_bytes(intel_ce_flash_cmd_t * pCmd)
{
	return intel_ce_flash_read_secret_bytes_internal(pCmd, 0);
}

EXPORT_SYMBOL(intel_ce_flash_read_secret_bytes);
// Read a block
int intel_ce_flash_read_block(intel_ce_flash_cmd_t * pCmd, int flags)
{
	int ret = 0;
	int count;
	unsigned long volatile flashBase = INTEL_CE_FLASH_BASE_VIRT;
	int temp_count;
	if(flags == 1)
		temp_count = memory_map_count - 1;
	else
		temp_count = memory_map_count;
	
	if(!pCmd)
		return -EINVAL;
	if(!(pCmd->buf))
		return -EINVAL;
	if((pCmd->blockno) >= temp_count || (pCmd->blockno) < 0)
	{
		INTEL_CE_PERROR("invalid block number: %d\n", pCmd->blockno);
		return -EINVAL;
	}

	flashBase += memory_map[pCmd->blockno].start;
	count = (memory_map[pCmd->blockno].end) - (memory_map[pCmd->blockno].start) + 1;
        DPRINTK("flashBase=0x%lx\n", flashBase);
	if(flags == 1)
		copy_to_user((char *)(pCmd->buf), (char *)(flashBase), count);
	else
		memcpy((char *)(pCmd->buf), (char *)(flashBase), count);
	return ret;
}


// write a block
int intel_ce_flash_write_block_p30(intel_ce_flash_cmd_t * pCmd)
{
	int ret = 0;
	volatile unsigned long flashBase;
	volatile u16 * src, * dst;
	int i, count;

	if(INTEL_EXT != chip_type || !pCmd)
		return -EINVAL;
	if(!(pCmd->buf))
		return -EINVAL;
	if((pCmd->blockno) < 0 || (pCmd->blockno) >= memory_map_count)
	{
		INTEL_CE_PERROR("invalid block number: %d\n", pCmd->blockno);
		return -EINVAL;
	}


	flashBase = (INTEL_CE_FLASH_BASE_VIRT + (memory_map[pCmd->blockno].start));
	count = (memory_map[pCmd->blockno].end) - (memory_map[pCmd->blockno].start) + 1;
	count = count >> 1;
	dst = (volatile u16 *)flashBase;
		src = (u16 *)pCmd->buf;
	DPRINTK("pBuf=0x%p, blockno=0x%x, count=0x%x\n", pCmd->buf, pCmd->blockno, count);
	DPRINTK("program block...\n");
	for(i=0; i<count; i++) {
		*((u16 *)dst) = (u16)FLASH_PROGRAM;	// address to program
		*((u16 *)dst) = *src;	// address and data
		intel_ce_flash_waitready(INTEL_EXT);
		dst++;
		src++;
    	}
    	DPRINTK("finish\n");
	
	DPRINTK("call reset\n");
	intel_ce_flash_reset();

	return ret;
}
// Write a block in buffered mode
int intel_ce_flash_write_block_buf_p30(intel_ce_flash_cmd_t * pCmd, int flags)
{
	int ret = 0;
	volatile unsigned long flashBase;
	u16 * src;
	volatile u16 * dst;
	int i, j, count;
	int wc = MAX_BUFSIZE;
	register u32 data;
	int temp_count;
	if(flags == 1)
		temp_count = memory_map_count - 1;
	else
		temp_count = memory_map_count;

	if(INTEL_EXT != chip_type || !pCmd)
		return -EINVAL;
	if(!(pCmd->buf))
		return -EINVAL;
	if((pCmd->blockno) < 0 || (pCmd->blockno) >= temp_count)
	{
		INTEL_CE_PERROR("invalid block number: %d\n", pCmd->blockno);
		return -EINVAL;
	}
	DPRINTK("pBuf=0x%p, blockno=0x%x\n", pCmd->buf, pCmd->blockno);
	DPRINTK("program block...\n");


	flashBase = (INTEL_CE_FLASH_BASE_VIRT);
	dst = (volatile u16 *)(flashBase + (memory_map[pCmd->blockno].start));
	src = (u16 *)pCmd->buf;
	count = (memory_map[pCmd->blockno].end) - (memory_map[pCmd->blockno].start) + 1;
	count = count >> 6;
	wc = MAX_BUFSIZE;
	DPRINTK("count=0x%x, wc=0x%x\n", count, wc);
	for(i = 0; i < count; i++)
	{
		// Issue the "Program Buf" command
		do {
			*((volatile u16 *)dst) = FLASH_PROGRAM_BUF;
		} while(!((*((volatile u16 *)dst)) & FLASH_STATUS_READY));
		// Write the word/byte count
		*((volatile u16 *)dst) = wc - 1;
		// Write data
		for(j = 0; j < wc; j+=2)
		{
			data = *((u32 *)(src));
			*((volatile u16 *)dst) = (u16)(data);
			dst++;
			data >>= 16;
			*((volatile u16 *)dst) = (u16)(data);
			dst++;
			src++;
			src++;
		}
		// Issue the "Comfirm" command
		*((volatile u16 *)(dst - 2)) = FLASH_CMD_CONFIRM;
		// Wait for ready
		*((volatile u16 *)flashBase) = FLASH_READ_STATUS;		
		while(!((*((volatile u16 *)(dst - 2))) & FLASH_STATUS_READY))
			yield();
	}

    	DPRINTK("finish\n");
	
	DPRINTK("call reset\n");
    	intel_ce_flash_reset_block_p30(((volatile u16 *)(dst - 2)));

	return ret;
}

// Write a block in word mode
int intel_ce_flash_write_block_spansion(intel_ce_flash_cmd_t * pCmd, int flags)
{
	int ret = 0;
	volatile u16 *flashBase;
	u16 * src;
	int i, count = 0;
	int temp_count;
	if(flags == 1)
		temp_count = memory_map_count - 1;
	else
		temp_count = memory_map_count;

	if(AMD_STD != chip_type || !pCmd)
		return -EINVAL;
	if(!(pCmd->buf))
		return -EINVAL;
	if((pCmd->blockno) < 0 || (pCmd->blockno) >= temp_count)
	{
		INTEL_CE_PERROR("invalid block number: %d\n", pCmd->blockno);
		return -EINVAL;
	}
        DPRINTK("pBuf=0x%p, blockno=0x%x, count=0x%x\n", pCmd->buf, pCmd->blockno, count);
	DPRINTK("program block...\n");


	flashBase = (u16 *)INTEL_CE_FLASH_BASE_VIRT;
	src = (u16 *)pCmd->buf;
	count = (memory_map[pCmd->blockno].end) - (memory_map[pCmd->blockno].start) + 1;
	
	DPRINTK("count=0x%x\n", count);
	for(i = 0; i < count; i += 2)
	{
		int offset = memory_map[pCmd->blockno].start + i;
		
		FLASH_WR(flashBase, FLASH_UNLOCK_ADDR1, FLASH_UNLOCK_DATA1);
		FLASH_WR(flashBase, FLASH_UNLOCK_ADDR2, FLASH_UNLOCK_DATA2);
		FLASH_WR(flashBase, FLASH_UNLOCK_ADDR1, FLASH_PROGRAM_CMD);

		FLASH_WR(flashBase, offset, src[i >> 1]);

		intel_ce_flash_waitready(offset);
		if(FLASH_DEV_NOT_BUSY != intel_ce_flash_readstatus_spansion(offset))
		{
			u16 status = 0;
			
			status = _get_status(offset);
			printk("status=0x%x\n", status);
			return -EIO;
		}
	}

	DPRINTK("finish\n");
	
	DPRINTK("call reset\n");
	intel_ce_flash_reset();

	return ret;
}

// Write a block in buffered mode
int intel_ce_flash_write_block_buf_spansion(intel_ce_flash_cmd_t * pCmd, int flags)
{
		int ret = 0;
		volatile u16 *flashBase;
		u16 * src;
		int i, j, count = 0;
		int temp_count,offset;
		register u32 data;
		int buffer_size;
		if(flags == 1)
			temp_count = memory_map_count - 1;
		else
			temp_count = memory_map_count;
	
		if(AMD_STD != chip_type || !pCmd)
			return -EINVAL;
		if(!(pCmd->buf))
			return -EINVAL;
		if((pCmd->blockno) < 0 || (pCmd->blockno) >= temp_count)
		{
			INTEL_CE_PERROR("invalid block number: %d\n", pCmd->blockno);
			return -EINVAL;
		}
			DPRINTK("pBuf=0x%p, blockno=0x%x, count=0x%x\n", pCmd->buf, pCmd->blockno, count);
		DPRINTK("program block...\n");
	
		flashBase = (u16 *)INTEL_CE_FLASH_BASE_VIRT;
		src = (u16 *)pCmd->buf;	
		count = (memory_map[pCmd->blockno].end) - (memory_map[pCmd->blockno].start) + 1;		
		offset = memory_map[pCmd->blockno].start;

		DPRINTK("count=0x%x, write_buffer_size=0x%x\n", count, write_buffer_size);
		
		for(i = 0; i < count; i += 2*write_buffer_size)
		{
			FLASH_WR(flashBase, FLASH_UNLOCK_ADDR1, FLASH_UNLOCK_DATA1);
			FLASH_WR(flashBase, FLASH_UNLOCK_ADDR2, FLASH_UNLOCK_DATA2);
			FLASH_WR(flashBase, offset, FLASH_WRITE_BUFFER_LOAD_CMD);
			FLASH_WR(flashBase, offset, write_buffer_size-1);
			for(j = 0;j < write_buffer_size;j += 2){
				data = *((u32 *)(src));
				FLASH_WR(flashBase, offset, (u16)(data));
				offset += 2;
				data >>= 16;
				FLASH_WR(flashBase, offset, (u16)(data));
				offset += 2;
				src++;
				src++;
			}
			FLASH_WR(flashBase, (offset-2), FLASH_WRITE_BUFFER_PGM_CONFIRM_CMD);
	
			intel_ce_flash_waitready(offset-2);
			if(FLASH_DEV_NOT_BUSY != intel_ce_flash_readstatus_spansion(offset-2))
			{
				u16 status = 0;
				
				status = _get_status(offset-2);
				printk("status=0x%x\n", status);
				return -EIO;
			}
		}
	
		DPRINTK("finish\n");
		
		DPRINTK("call reset\n");
		intel_ce_flash_reset();
		return ret;
}

int intel_ce_flash_write_block_buf(intel_ce_flash_cmd_t * pCmd, int flags)
{
	int ret = -EINVAL;
	switch (chip_type)
	{
		case INTEL_EXT:
			ret = intel_ce_flash_write_block_buf_p30(pCmd, flags);
			break;
		case AMD_STD:
			if(intel_ce_cfi_ident.MaxBufWriteSize){
				ret = intel_ce_flash_write_block_buf_spansion(pCmd, flags);
			}
			else{
				ret = intel_ce_flash_write_block_spansion(pCmd, flags);
			}
			break;
		default:
			ret = -EINVAL;
	}
	return ret;
}

int intel_ce_flash_write_secret_bytes_internal(intel_ce_flash_cmd_t * pCmd, int flags)
{
	int ret = 0;
	intel_ce_flash_cmd_t rwcmd;
	u8 *temp_buf;
	u8 *p;
	int start = SECRET_START;
	int end = SECRET_END;
	int i = 0;

	if(!pCmd)
		return -EINVAL;
	if(!(pCmd->buf))
		return -EINVAL;
	if((pCmd->offset < 0) || (pCmd->offset >= (end-start+1)) || ((pCmd->offset + pCmd->length) > (end-start +1)))   
	{
		INTEL_CE_PERROR("write out of flash range: offset=0x%x, size=0x%x\n",
			pCmd->offset, pCmd->length);
		return -EINVAL;
	}
	temp_buf = kzalloc(SECRET_BLK_NUM * flash_info.blocksize, GFP_KERNEL);
	if(!temp_buf)
	{
		return -ENOMEM;
	}
	
	rwcmd.buf = temp_buf;
	for (i = (memory_map_count - SECRET_BLK_NUM); i< memory_map_count; i++)
	{
	rwcmd.blockno = i;
	ret=intel_ce_flash_read_block(&rwcmd, 0);
	if(ret < 0)
	{
		printk("error reading block\n");
		goto ioreturn;
	}
	rwcmd.buf += (memory_map[rwcmd.blockno].end) - (memory_map[rwcmd.blockno].start) + 1;
	}
	p = temp_buf + pCmd->offset;
	if(flags == 1)
		copy_from_user(p, pCmd->buf, pCmd->length);
	else
		memcpy(p, pCmd->buf, pCmd->length);

	rwcmd.buf = temp_buf;
	for (i = (memory_map_count - SECRET_BLK_NUM); i< memory_map_count; i++)
	{
	rwcmd.blockno = i;

	ret = intel_ce_flash_erase(memory_map[rwcmd.blockno].start,0,0);
	if(ret < 0)
	{
		printk("erase error\n");
		goto ioreturn;
	}

	ret = intel_ce_flash_write_block_buf(&rwcmd, 0);
	if(ret < 0)
	{
		printk("write error\n");
		goto ioreturn;
	}
	rwcmd.buf += (memory_map[rwcmd.blockno].end) - (memory_map[rwcmd.blockno].start) + 1;
	}
ioreturn:
	kfree(temp_buf);
	return ret;

}

int intel_ce_flash_write_secret_bytes(intel_ce_flash_cmd_t * pCmd, int flags)
{
	return intel_ce_flash_write_secret_bytes_internal(pCmd, 0);
}

EXPORT_SYMBOL(intel_ce_flash_write_secret_bytes);

//Get memory map information
int intel_ce_flash_get_map(memory_map_t * pMap)
{
	memory_map_t *tmpmap;
	tmpmap = kmalloc(sizeof(memory_map_t), GFP_KERNEL);
	if (!tmpmap)
		return -ENOMEM;
	tmpmap->memory_map_size = memory_map_count;
	memcpy(tmpmap->map, memory_map, sizeof(memory_map_item_t) * MAX_MEMORY_MAP);
	if(copy_to_user(pMap, tmpmap, sizeof(memory_map_t))){
		kfree(tmpmap);
		return -EFAULT;
	}
	kfree(tmpmap);
	return 0;
}

static int intel_ce_flash_ioctl(struct file * file, 
		   unsigned int cmd, unsigned long arg)
{
	int status = -EINVAL;
	intel_ce_flash_cmd_t rwcmd;
        DPRINTK("cmd=%d arg=0x%08lx\n", cmd, arg);
	DOWN_RW_SEM;

	switch(cmd)
	{
		case INTEL_CE_FLASH_ERASE:
			status = intel_ce_flash_erase(arg, file->f_flags, 1);
			break;
		case INTEL_CE_FLASH_READINFO:
			status = intel_ce_flash_readinfo((intel_ce_flash_info_t *)arg);
			break;
		case INTEL_CE_FLASH_RESET:
			status = intel_ce_flash_reset();
			break;
		case INTEL_CE_FLASH_READ_BLOCK:
			if(0 == copy_from_user(&rwcmd, (void *)arg, sizeof(rwcmd)))
			{
				status = intel_ce_flash_read_block(&rwcmd, 1);
			}
			break;
		case INTEL_CE_FLASH_READ_BYTES:
			if(0 == copy_from_user(&rwcmd, (void *)arg, sizeof(rwcmd)))
			{
				status = intel_ce_flash_read_bytes(&rwcmd);
			}
			break;
		case INTEL_CE_FLASH_READ_SECRET_BYTES:
			if(0 == copy_from_user(&rwcmd, (void *)arg, sizeof(rwcmd)))
			{
				status = intel_ce_flash_read_secret_bytes_internal(&rwcmd,1);
			}
			break;
		case INTEL_CE_FLASH_WRITE_SECRET_BYTES:
			if(0 == copy_from_user(&rwcmd, (void *)arg, sizeof(rwcmd)))
			{
				status = intel_ce_flash_write_secret_bytes_internal(&rwcmd, 1);
			}
			break;

		case INTEL_CE_FLASH_WRITE_BLOCK:
			if(0 == copy_from_user(&rwcmd, (void *)arg, sizeof(rwcmd)))
				status = intel_ce_flash_write_block_buf(&rwcmd, 1);
			break;
		case INTEL_CE_FLASH_GET_MAP:
			status = intel_ce_flash_get_map((memory_map_t *)arg);
			break;
		default:
			break;
	}
	UP_RW_SEM;
	return status;
}

static int intel_ce_flash_open(struct inode *inode, struct file * file)
{	return 0;
}

static int intel_ce_flash_close(struct inode *inode, struct file * file)
{	return 0;
}

static struct file_operations intel_ce_flash_fops = {
	.open = intel_ce_flash_open,
	.release = intel_ce_flash_close,
	//.read = intel_ce_flash_read,
	//.write = intel_ce_flash_write,
	.unlocked_ioctl = intel_ce_flash_ioctl
};

int intel_ce_flash_read_k(unsigned long flash_pos, char * dst, int length)
{
	volatile u8 * flashBase = (u8 *)INTEL_CE_FLASH_BASE_VIRT;

       DPRINTK("src=%08lx dst=0x%p length=%08x\n", flash_pos, dst, length);

		memcpy(dst, (char *)(flashBase + flash_pos), length);

		return length;
}

int _build_memory_map(memory_map_item_t * pMap, intel_ce_flash_info_t * pInfo)
{
	int i, count = 0;
	unsigned int start = 0;
	
	if(!pMap || !pInfo)
		return 1;
	
	if(pInfo->size_in_bytes > 128*1024*1024)
		return 1;

	for (i = 0; i < intel_ce_cfi_ident.NumEraseRegions; i++) 
	{
		unsigned long ernum, ersize;
		int j;
		
		ersize = ((intel_ce_cfi_ident.EraseRegionInfo[i] >> 8) & ~0xff);
		ernum = (intel_ce_cfi_ident.EraseRegionInfo[i] & 0xffff) + 1;

		for (j = 0; j < ernum; j++)
		{
			if (count + j >= MAX_MEMORY_MAP)
			{
				printk("Exceeds memory map size[%d]!\n", MAX_MEMORY_MAP);
				return 1;
			}
			
			pMap[count + j].start = start;
			pMap[count + j].end = start + ersize - 1;
			start += ersize;

			DPRINTK("[%04d] start: %08x, end: %08x\n",
				count + j, 
				pMap[count + j].start,
				pMap[count + j].end);
		}

		count += ernum;
	}

	memory_map_count = count;
	
#ifdef INTEL_CE_FLASH_DEBUG
	for(i = 0; i < 4; i++)
	{
		printk("%02d: start=0x%08x, end=0x%08x\n", i, pMap[i].start, pMap[i].end);
	}
	for(i = count - 4; i < count; i++)
	{
		printk("%02d: start=0x%08x, end=0x%08x\n", i, pMap[i].start, pMap[i].end);
	}
#endif
	return 0;
}
// Intel CE flash initialize
static int intel_ce_flash_init(void)
{
	int i;
	int rev = 0;
	INTEL_CE_FLASH_BASE_VIRT = (volatile unsigned long)ioremap_nocache(0xc0000000,3000);
	if(!INTEL_CE_FLASH_BASE_VIRT){
		printk(KERN_ERR "intel_ce_flash_init: ioremap_nocache failed!");
		rev = -ENOMEM;
		goto out;
	}
	INTEL_CE_EXP_TIMING_CS0  = (volatile unsigned long)ioremap_nocache(0xdffe8000,256);
	if(!INTEL_CE_EXP_TIMING_CS0){
		printk(KERN_ERR "intel_ce_flash_init: Unable to remap memory control register region");
		rev = -ENOMEM;
		goto remapcs0_err_out;
	}
	//printk("bitmode=%d\n", bitmode);
	sema_init(&rw_sem,1);
	

	i = register_chrdev(INTEL_CE_FLASH_MAJOR, "intel_ce_flash", &intel_ce_flash_fops);
	if (i) {
		printk("flashtool::intel_ce_flash_init: unable to get major %d\n", INTEL_CE_FLASH_MAJOR);
		rev = -EIO;
		goto map_err_out;
	}
	if(0 != (i = _flash_probe()))
	{
		if (i == -ENODEV)
		{
			printk(KERN_ERR "flashtool::No valid NOR flash device detected, skipping attempt to install NOR flash driver\n");
			rev = -ENODEV;
			goto chrdev_err_out;
		}
		else
		{
			printk(KERN_ERR "flashtool::Error probing flash device; Failed to install NOR flash driver\n");
			rev = -EIO;
			goto chrdev_err_out;
			}
	}
	if(intel_ce_flash_readinfo(&flash_info))
	{
		printk("flashtool::Error getting device info! Failed to install NOR flash driver\n");
		rev = -EIO;
		goto chrdev_err_out;
	}

	printk("flashtool::intel_ce_flash driver initialized\n");
	printk("Device size in bytes: 0x%x\n", flash_info.size_in_bytes);
	printk("Block size: 0x%x 0x%x\n", flash_info.blocksize, 
		flash_info.blocksize2);
	iounmap((void *)INTEL_CE_FLASH_BASE_VIRT);
	INTEL_CE_FLASH_BASE_VIRT = (volatile unsigned long)ioremap_nocache(0xc0000000,flash_info.size_in_bytes);
	if(!INTEL_CE_FLASH_BASE_VIRT){
		printk(KERN_ERR "intel_ce_flash_init: ioremap_nocache failed!");
		rev = -ENOMEM;
		goto remapflash_err_out;
	}
	//config CS1 when flash size is lager than 64M
	if(flash_info.size_in_bytes > 64*1024*1024){
		*((int *)(INTEL_CE_EXP_TIMING_CS0+4)) = (*((int *)INTEL_CE_EXP_TIMING_CS0)&0xFFFFFEFF); 
	}
	if(flash_info.size_in_bytes > 128*1024*1024)
	{
		printk("Only supports flash up to 128M bytes now\n");
		rev = -EIO;
		goto chrdev_err_out;
	}
	memset(&memory_map[0], 0, sizeof(memory_map_item_t) * MAX_MEMORY_MAP);
	if(_build_memory_map(&memory_map[0], &flash_info))
	{
		printk("Error building memory map\n");
		rev = -EIO;
		goto chrdev_err_out;
	}
	i = *((int *)INTEL_CE_EXP_TIMING_CS0);
	DPRINTK("Timing: 0x%x\n", i);
	if(i & 0x00000001)
		bitmode = 8;
	else
		bitmode = 16;
	//printk("bitmode=%d\n", bitmode);

#if 0
	printk("Read first 64 bytes...\n");
	intel_ce_flash_read_k(0, data, 64);
	for(i = 0; i < 64; i++)
		printk("%x ", data[i]);
	printk("\n");

	printk("Read first 64 bytes again...\n");
	intel_ce_flash_read_k(0xF60000, data, 64);
	for(i = 0; i < 64; i++)
		printk("%x ", data[i]);
	printk("\n");
#endif

out:
	return rev;
remapflash_err_out:
	unregister_chrdev(INTEL_CE_FLASH_MAJOR, "intel_ce_flash");
	iounmap((void *)INTEL_CE_EXP_TIMING_CS0);
	goto out;
chrdev_err_out:
	unregister_chrdev(INTEL_CE_FLASH_MAJOR, "intel_ce_flash");
map_err_out:
	iounmap((void *)INTEL_CE_EXP_TIMING_CS0);
remapcs0_err_out:
	iounmap((void *)INTEL_CE_FLASH_BASE_VIRT);
	goto out;
}

static void intel_ce_flash_exit(void)
{
	unregister_chrdev(INTEL_CE_FLASH_MAJOR, "intel_ce_flash");
 	iounmap((void *)INTEL_CE_FLASH_BASE_VIRT);
 	iounmap((void *)INTEL_CE_EXP_TIMING_CS0);
	printk(KERN_INFO "intel_ce_flash driver removed\n");
}

module_param(bitmode, int, 8);

module_init(intel_ce_flash_init);
module_exit(intel_ce_flash_exit);

MODULE_DESCRIPTION("Intel(R) Flash Device Driver");
MODULE_AUTHOR("Intel Corporation, (C) 2008-2012 All Rights Reserved");
MODULE_LICENSE("GPL");

