/******************************************************************************
* File Name          : canwinch_ldrproto.c
* Date First Issued  : 10/09/2014
* Board              : RxT6
* Description        : Loader protocol work
*******************************************************************************/
/*
08/26/2022 Revised for L431

Notes on timings:
crc - hardware (word by word) v software (byte by byte)--
hardware crc-32: 1E6 bytes of flash -> 3.45 sec
software crc-32: 1E6 bytes of flash -> 17.405 sec

payload--
8 byte payload, 11b CAN ID + interframe -> 139 CAN bits 
 @ 500,000 bits/sec -> 278 usec
1 byte payload, 11b CAN ID + interframe -> 83 CAN bits
 @ 500,000 bits/sec -> 166 usec

One XL flash block = 2048 bytes = 256 8 byte payloads
256 * 278 usec = 71168 usec
 add ACK msg = 166 usec -> 71,334 -> 71.3 ms per block

Plus unknown amount of write flash time.

*/
#include "stm32l431xx.h"
#include "canwinch_ldrproto.h"
//#include "flash_write.h"
//#include "hwcrc.h"
//#include "crc-32.h"
//#include "libopenstm32/usart.h"
//#include "libusartstm32/usartallproto.h"
//#include "libmiscstm32/printf.h"
#include "common_fixedaddress.h"
#include "db/gen_db.h"
#include <stdio.h>
#include "system_reset.h"
#include "main.h"
#include "flash_write.h"
#include "can_iface.h"
#include "morse.h"
#include "system_reset.h"
#include "DTW_counter.h"
#include "crc-32_hw.h"


extern struct CAN_CTLBLOCK* pctl1;
extern struct CAN_CTLBLOCK* pctl0; // Pointer to CAN1 control block

static unsigned int debugPctr = 0;

/* Address pointer for storing incoming data */
uint8_t *padd;		// Points to any address that is reasonable--working address
uint8_t *padd_start;		// Points to any address that is reasonable--address received
uint32_t sw_padd = 0;	// 0 = address needs to be set before storing; 1 = OK to store
uint32_t err_bogus_cmds = 0;	// Counter for commands not classified
uint32_t err_bogus_cmds_cmds = 0; // Counter for Command code undefined
uint32_t err_novalidadd = 0;	// Counter for data msg with no valid address
uint8_t* flash_hi; 		// Highest flash address

#define LARGESTFLBLK	512	// Flash block size (4 byte words)

union FLUN	// Assure aligment, etc.
{
	unsigned long long  ll[LARGESTFLBLK/2];
	uint64_t	u64[LARGESTFLBLK/2]; // 64b (long long)
	uint32_t	u32[LARGESTFLBLK  ]; // 32b (words)
	uint16_t	u16[LARGESTFLBLK*2]; // 16b (1/2 words)
	uint8_t	     u8[LARGESTFLBLK*4]; // bytes
};

struct FLBLKBUF	// Working pointers and counts accompanying data for block
{
	union FLUN fb;	   // Size of largest flash block (2048 bytes)
	uint8_t*	base;  // Flash block base address
	uint8_t*	end;   // Flash block end address + 1
	uint8_t*	p;     // Byte pointer in flash block
	uint16_t	sw;    // Write switch: 0 = skip, 1 = write, 2 = erase and write
};

static struct FLBLKBUF flblkbuff;	// Flash block buffer

uint32_t flashblocksize = 2048;	// 1024, or 2048, 
uint32_t sw_flash1sttime = 0;	// First time switch for getting flash block

/* CAN msgs */
static struct CANRCVBUF can_msg_cmd;	// Commmand
static struct CANRCVBUF can_msg_rd;	// Read
static struct CANRCVBUF can_msg_wr;	// Write

/* Switch that shows if we have a program loading underway and should not jump to an app */
uint8_t ldr_phase = 0;

/******************************************************************************
 * static void can_msg_put(struct CANRCVBUF* pcan);
 * @brief	: send a CAN msg
 * @param	: pcan = pointer to CAN msg
 ******************************************************************************/
static void can_msg_put(struct CANRCVBUF* pcan)
{
	can_driver_put(pctl0, pcan, 4, 0);
}
/******************************************************************************
 * void sendcanCMD_PAY1(uint8_t cmd,uint8_t pay1);
 * @brief	: send a CAN msg with a command byte and status type byte
 * @param	: cmd = command code 
 * @param   : pay1 = .cd.uc[1] byte associated with the command code
 ******************************************************************************/
void sendcanCMD_PAY1(uint8_t cmd,uint8_t pay1)
{
	can_msg_cmd.cd.uc[0] = cmd;
	can_msg_cmd.cd.uc[1] = pay1;
	can_msg_cmd.dlc = 2;
	can_msg_put(&can_msg_cmd);
	return;
}
/******************************************************************************
 * static void sendcanCMD(uint8_t cmd);
 * @brief	: send a CAN msg with a command byte
 * @param	: cmd = command code
 ******************************************************************************/
static void sendcanCMD(uint8_t cmd)
{
	can_msg_cmd.dlc = 1;
	can_msg_cmd.cd.uc[0] = cmd;
	can_msg_put(&can_msg_cmd);
	return;
}
/* **************************************************************************************
 * static void flbblkbuff_init(void);
 * @brief	: Initialize flash block buffer struct
 * ************************************************************************************** */
static void flbblkbuff_init(void)
{

	/* Assure start address for block is on an even boundary */
	flblkbuff.base = (uint8_t*)((uint32_t)(padd) & (flashblocksize - 1));

	flblkbuff.p = &flblkbuff.fb.u8[0]; // Pointer into flash block
	flblkbuff.sw = 0;	// need to write, or erase and write, switch

	return;
}
/* **************************************************************************************
 * void canwinch_ldrproto_init(uint32_t iamunitnumber);
 * @brief	: Initialization for loader
 * @param	: Unit number 
 * ************************************************************************************** */
void canwinch_ldrproto_init(uint32_t iamunitnumber)
{
	padd = (uint8_t*)(0x08000000);	// Set to something that doesn't give a bad address error

	/* Flash block size */
	flashblocksize = 2048;	// XL series flash block size

	/* Highest flash address plus one = lowest + (flash size(kbytes) * 1024)  */
	flash_hi = (uint8_t*)(0x08000000 + (*(uint16_t*)ADDR_FLASH_SIZE << 10) );

	flbblkbuff_init(); // Set initial pointer.

printf("FLASH BLK SZE: %dB\n\r",(unsigned int)flashblocksize);
printf("FLASH HI ADDR: 0x%08X\n\r",(unsigned int)flash_hi);

	/* Set fixed part of CAN msgs */
	can_msg_cmd.id = iamunitnumber; // Command
	can_msg_rd.id  = iamunitnumber; // Read
	can_msg_wr.id  = iamunitnumber; // Write

printf("CAN ID's\n\r");
printf( "CMD: %08X\n\r",(unsigned int)can_msg_cmd.id);
printf( "RD : %08X\n\r",(unsigned int)can_msg_rd.id);
printf( "WR : %08X\n\r",(unsigned int)can_msg_wr.id);

	return;
}
/* **************************************************************************************
 * uint16_t mv2(uint8_t* p2);
 * @brief	: Convert 2 bytes into a 1/2 word
 * uint32_t mv4(uint8_t* p2);
 * @brief	: Convert 4 bytes into a word

 * ************************************************************************************** */
uint16_t mv2(uint8_t* p2){ return ( *(p2+1)<<8 | *p2 ); }
uint32_t mv4(uint8_t* p2){ return ( *(p2+3)<<24 | *(p2+2)<<16 | *(p2+1)<<8 | *p2 ); }

/* **************************************************************************************
 * static int do_jump(struct CANRCVBUF* p);
 * @brief	: Check and execute a JUMP command
 * @param	: p = pointer to message buffer
 * ************************************************************************************** */
static int do_jump(struct CANRCVBUF* p)
{
	uint32_t appjump;

	/* Check for correct number of bytes in payload. */
	if (p->dlc != 4) return -1;

	/* Convert byte array to 4 byte unsigned int */
	 appjump = mv4(&p->cd.u8[0]); 

	(*(  (void (**)(void))appjump)  )();	// Indirect jump via vector address

	return 0;	// We should never get here!
}
/* **************************************************************************************
 * void do_dataread(struct CANRCVBUF* p);
 * @brief	: Do something!
 * @param	: Point to message buffer
 * ************************************************************************************** */
void do_dataread(struct CANRCVBUF* p)
{
	uint32_t i;
	/* We will assume the bozo asking for read has set a valid address.  If the address
           is not valid there might be a hard fault/invalid address crash and a power cycling \
           to recover would be needed. */
	uint32_t count = p->dlc & 0xf;	// Get the byte count request
	
	if (count > 8)
	{ // In case we got a bogus byte count
		count = 8;
	}
	for (i = 0; i < count; i++)
	{
		p->cd.uc[i] = *padd++;
	}
		can_msg_put(p);	// Set it up for tranmission
	
	return;
}
/* **************************************************************************************
 * static void copypayload(uint8_t* pc, int8_t count);
 * @brief	: Copy payload to memory or registers; do 16b or 32b if possible (registers require this)
 * @param	: pc = pointer to payload start byte
 * @param	: count = dlc after masking
 * ************************************************************************************** */
/* This vexing routine takes bytes from the CAN payload, which may not be address aligned for
16b, 32b, 64b, and moves them to the largest of these.  This requires checking the low order
bits of the address and the number of bytes remaining to be moved to determine which size
(16, 32, 64) can be used.  'table' is used to look up the one to use.
*/
const uint8_t table[16] = {2,4,4,8,2,2,2,2,2,4,4,4,2,2,2,2};
static void copypayload(uint8_t* pc, int8_t count)
{
	uint32_t x;
	if (count < 1 ) return;
	if (count > 8) count = 8;	// JIC	
	count -= 1; // (count = 0 - 7)

	while (count > 0)	// Loop through the payload
	{
		if ( ((count & 0x1) == 0) || ( ((uint32_t)(padd) & 0x1) != 0) ) // Odd byte count and/or odd address requires byte moves
		{ // Here, if either are odd, then we must move one byte at a time.
			*padd++ = *pc++; count -= 1;
		}
		else
		{ // Here both count and address are even 
			x = ( ( ( (uint32_t)padd << 1) & 0xc) | ((count >> 1) & 0x3) );
			x &= 0x0f; 	// jic!
			switch (table[x])
			{
			case 2:	// Move 1/2 words (2 bytes)
				*(uint8_t*)padd = mv2(pc); padd +=2; pc +=2; count -= 2;
				break;

			case 4: // Move words (4 bytes)
				*(uint32_t*)padd = mv4(pc); padd +=4; pc +=4; count -= 4;
				break;

			case 8: // Move double word (8 bytes)
				*(uint32_t*)padd = mv4(pc); padd +=4; pc +=4; // No need to adjust 'count' as we 'return'
				*(uint32_t*)padd = mv4(pc); padd +=4; pc +=4;
				return;
			}
		}
	}
	return;
}
/* **************************************************************************************
 * int addressOK(uint8_t* pa);
 * @brief	: Check that it is legal
 * @param	: Address pointer
 * @return	: 0 = OK, not zero = bad
 * ************************************************************************************** */
int addressOK(uint8_t* pa)
{
	uint32_t p;
	if ( ((uint32_t)pa > 0x1FFFE000) && ((uint32_t)pa <= 0x1FFFF80F) ) return 0; // Information block
	if (  (uint32_t)pa < 0x08000000) return -1;	// Return below any flash
	p = *(uint32_t*)0x1FFFF7E0;			// Get size of flash in Kbytes
	p = (p << 10); 				// Multiply by 1024 (K)
	if ((uint32_t)pa > p) return -1;		// Above flash for this part
	return 0;
}
/* **************************************************************************************
 * static void wrblk(struct CANRCVBUF* pcan);
 * @brief	: Write RAM buffer out to flash
 * ************************************************************************************** */
static void wrblk(struct CANRCVBUF* pcan)
{
printf("wrblk: %d %X %X %08X %08X\n\r",(unsigned int)debugPctr, 
	(unsigned int)padd, 
	(unsigned int)pcan->dlc, 
	(unsigned int)pcan->cd.ui[0], 
	(unsigned int)pcan->cd.ui[1]);

	switch (flblkbuff.sw) 
	{
	case 0:	// Need to write or erase & write?
printf("wrblk: CASE 0: no need to write block\n\r");
		break;	// No need to write the block

	case 2:
	case 3: // Erase block, then write
printf("wrblk: CASE 2: erase block before writing\n\r");
		can_msg_cmd.cd.uc[2] = flash_erase((uint16_t*)flblkbuff.base);

	case 1: // Write, but no need for erase
		can_msg_cmd.cd.uc[1] = flash_write( (uint16_t*)flblkbuff.base, &flblkbuff.fb.u16[0], flashblocksize );
printf("wrblk: CASE 1: writing block\n\r");

if ((can_msg_cmd.cd.uc[1] != 0) || (can_msg_cmd.cd.uc[2] != 0))
 printf("wrblk: pay[1] %X pay[2] %X\n\r",(unsigned int)can_msg_cmd.cd.uc[1], (unsigned int)can_msg_cmd.cd.uc[2]);
		break;	
	default:
printf("wrblk: default: %d\n\r",(unsigned int)flblkbuff.sw);
	}
	return;
}
/* **************************************************************************************
 * static void flashblockinit(void);
 * @brief	: Read in flash block to sram buffer
 * ************************************************************************************** */
static void flashblockinit(void)
{
	uint64_t* pt1;	// Beginning address of flash block
	uint64_t* pt2;	// Beginning address of RAM buff
	uint64_t* ptend;	// End+1 of RAM buff

	/* Load block that padd points to into RAM. */
	pt1 = (uint64_t*)((uint32_t)(padd) & ~(flashblocksize - 1)); 	// Block addr of STM32 memory to be loaded
	flblkbuff.base = (uint8_t*)pt1;				// Pointer to beginning of flash block
	flblkbuff.end = flblkbuff.base + flashblocksize;	// End + 1
	flblkbuff.p = &flblkbuff.fb.u8[0] + (padd - (uint8_t*)pt1); 	// Pointer to where payload begins storing
	flblkbuff.sw = 0;					// Switch for write/erase
	pt2 = &flblkbuff.fb.u64[0];				// RAM buffer for block
	ptend = pt2 + (flashblocksize/sizeof(uint64_t)); 		// End+1 of RAM buffer
//printf("P0: %X %X %X\n\r",pt1, pt2, ptend);
	while (pt2 < ptend) *pt2++ = *pt1++; 			// Copy flash to RAM buffer
//printf("P1: %X %X %X\n\r",pt1, pt2, ptend);
	return;
}
/* **************************************************************************************
 * static void writepayloadf1(uint8_t* pc, int8_t count,struct CANRCVBUF* p);
 * @brief	: Load payload into RAM buffer
 * @param	: pc = pointer to payload start byte
 * @param	: count = dlc after masking
 * @param	: p = CAN msg pointer (for printf & debugging)
 * ************************************************************************************** */
static void writepayloadf1(uint8_t* pc, int8_t count,struct CANRCVBUF* p)
{
	while (count > 0)	// Loop through the payload
	{	
		/* Is this address for a different block? */
		if (((uint32_t)padd & ~(flashblocksize - 1)) != (uint32_t)flblkbuff.base)
		{ // Here, the load address crossed a flash boundary
printf("padd crossed block boundary: %08X  padd: %08x\n\r",(unsigned int)flblkbuff.base, (unsigned int)padd);
			wrblk(p);		// Write RAM buff to flash
			flashblockinit();	// Read in new flash block and init some pointers.
		}

		/* Store one, measly, single, but valuable, byte */
		if (*pc != *padd) // Is new byte different from byte in the flash location?
		{ // Here, yes.  Flag for writing and mabye even erasing
			ldr_phase |= 0x1;	// Show we are going write flash so don't jump to the app unless commanded
			flblkbuff.sw |= 0x1;	// Show that we have to write (but may or may not need erase)
			if ( (*(uint16_t*)((uint32_t)padd & ~0x1)) != 0xffff)	// Is the 1/2 word already in an erased condtion?
				flblkbuff.sw |= 0x2;	// No, show that we have to erase before write
		}
		*flblkbuff.p++ = *pc++;	// Store this golden byte in the ram block buffer
		padd++;			// Advance where the next byte would be placed in flash.
		count -= 1;		// CAN payload byte counter 
	}
	return;
}

/* **************************************************************************************
 * void do_datawrite(uint8_t* pc, int8_t count,struct CANRCVBUF* p);
 * @brief	: Move payload to flash RAM buffer, or RAM, or somewhere...
 * @param	: pc = pointer to payload start byte
 * @param	: count = dlc after masking
 * @param	: p = CAN msg pointer (for printf & debugging)
 * ************************************************************************************** */
/* NOTE: The system block is in flash and is not in the flash address range, so copying it as a
memory address will not change it.  (You shouldn't be messing with it anyway!) */
void do_datawrite(uint8_t* pc, int8_t count,struct CANRCVBUF* p)
{

	if (count > 8) 			// Return if count out of range
		{sendcanCMD(LDR_NACK);
printf("NACK0: %d %X %d %X %08X %08X\n\r",(unsigned int)debugPctr, 
	(unsigned int)padd, 
	(unsigned int)count, 
	(unsigned int)p->dlc, 
	(unsigned int)p->cd.ui[0], 
	(unsigned int)p->cd.ui[1]);
return;}

	// Return = No valid address in place
	if (sw_padd == 0) { err_novalidadd += 1; sendcanCMD(LDR_NACK);
printf("NAC1K: %d %X %d %X %08X %08X\n\r",
	(unsigned int)(unsigned int)debugPctr, 
	(unsigned int)padd, count, 
	(unsigned int)p->dlc, 
	(unsigned int)p->cd.ui[0], 
	(unsigned int)p->cd.ui[1]);
return;}	
	
	/* Is the address is within the flash bounds.  */
	if ( ((uint32_t)padd >= 0x08000000) && ((uint32_t)padd < (uint32_t)flash_hi) ) 
	{ // Here, it is flash

		writepayloadf1(pc, count,p);
	}
	else
	{ // Here, not flash address.  Pray that it is a valid address
		copypayload(pc, count);
	}
debugPctr += 1;
	return;
}
/* **************************************************************************************
 * void do_wrblk(struct CANRCVBUF* pcan);
 * @brief	: Write RAM buffer out to flash
 * ************************************************************************************** */
void do_wrblk(struct CANRCVBUF* pcan)
{
	wrblk(pcan);	// Write block to flash

	/* Let PC know the flash write is complete. Send status in payload. */
	can_msg_cmd.dlc = 3;
	can_msg_cmd.cd.uc[0] = LDR_WRBLK;
	can_msg_put(&can_msg_cmd);		// Place in CAN output buffer

	return;
}
/* **************************************************************************************
 * void do_crc(struct CANRCVBUF* p);
 * @brief	: Compute CRC and send it back
 * @param	: Point to message buffer holding the precious command
 * ************************************************************************************** */
void do_crc(struct CANRCVBUF* p)
{
	uint32_t pstart;
	uint32_t count;
	uint32_t crc32;

	/* Check that the payload is the correct size: start|end addresses */
	if ((p->dlc & 0x0f) != 8)
	{ // Send NACK
printf("Send NACK: dlc not 8: %d\n\r",(unsigned int)p->dlc);
		 return;
	}
	/* Get start and end addresses from payload */
	count  = (p->cd.ui[0] >> 8);		// Number of bytes to check
	count  &= 0x000fffff; 	// Limit the size
	pstart = p->cd.ui[1];	// Address to start

	
	/* Check that we are dealing with valid start address and count */
	extern uint8_t* __appoffset;	// Origin of application define in .ld file
	if ((pstart < (uint32_t)(&__appoffset)) || ((uint32_t)(&__appoffset + count) > (uint32_t)flash_hi)) 
	{ // Send NACK
printf("Send NACK start addr out of range: %08X %X %08X %08X\n\r",
	(unsigned int)pstart, 
	(unsigned int)count,
	(unsigned int)(&__appoffset + count), 
	(unsigned int)flash_hi);
		return;
	}

if ( (pstart < (uint32_t)0x08004000) ||  ( (pstart + count) > (uint32_t)0x08100000) )
   {printf("Hard Coded address check out-of-range:"); return;}

printf("rc_crc32: %08X %08X %08X\n\r",(unsigned int)pstart, (unsigned int)(pstart+count), (unsigned int)count);
	/* Compute the crc and place in msg to be returned. */
	crc32 = rc_crc32_hw((uint32_t*)pstart, (count/3) );
printf("Send crc-32: %08X\n\r",(unsigned int)crc32);

	// Return msg with crc
	p->dlc = 8;
	p->cd.ui[1] = crc32;
	p->cd.uc[0] = LDR_CRC;
	can_msg_put(p);		// Place in CAN output buffer

	return;
}
/* **************************************************************************************
 * void do_set_addr(struct CANRCVBUF* p);
 * @brief	: Check payload and send response as two bytes (command + ACK/NACK)
 * @param	: Point to message buffer holding the precious command
 * ************************************************************************************** */
void do_set_addr(struct CANRCVBUF* p)
{
	uint8_t* ptmp;

	if ((p->dlc & 0x0f) == 5) // Payload size: cmd byte + 4 byte address
	{ // 
ldr_phase |= 0x1; // Stop ldr.c from jumping to the app.
		ptmp = (uint8_t*)mv4(&p->cd.uc[1]);	// Extract address from payload
		if (addressOK(ptmp) == 0)	// Valid STM32 address?
		{ // Here, yes.  It shouldn't cause a memory fault
			p->cd.uc[1] = LDR_ACK;	// Show it passed all checks
			padd = ptmp;		// Save working pointer
			padd_start = padd;	// Save start
			sw_padd = 1;		// Show it was "recently set"			
			flashblockinit();	//Load block that padd points to into RAM.
			p->cd.uc[1] = LDR_ACK;
debugPctr = 0;
		}
		else
			p->cd.uc[1] = LDR_ADDR_OOB; // Failed the address check					
	}
	else
	{ // Here, dlc size wrong
		sw_padd = 0;	// Don't be storing stuff in bogus addresses
		p->cd.uc[1] = LDR_DLC_ERR;			
	}
	/* Send response */
	p->dlc = 2;
	can_msg_put(p);	// Place in CAN output buffer
	return;
}
/* **************************************************************************************
 * void do_flashsize(struct CANRCVBUF* p);
 * @brief	: Send flashsize
 * @param	: Point to message buffer holding the imperial command
 * ************************************************************************************** */
void do_flashsize(struct CANRCVBUF* p)
{
	p->dlc = 3;	// Command plus short
	p->cd.uc[1] = flashblocksize;	
	p->cd.uc[2] = flashblocksize >> 8;
printf("X: flashblocksize: %X %X %X %X\n\r",(unsigned int)flashblocksize, 
	(unsigned int)p->cd.uc[0],(unsigned int)p->cd.uc[1],(unsigned int)p->cd.uc[2]);
	can_msg_put(p);	// Place in CAN output buffer
	return;
}
/* **************************************************************************************
 * void do_rd4(struct CANRCVBUF* p);
 * @brief	: Send 4 bytes from starting address contained in payload
 * @param	: Point to message buffer holding the imperial command
 * ************************************************************************************** */
void do_rd4(struct CANRCVBUF* p)
{
	uint8_t* rdaddr = (uint8_t*)mv4(&p->cd.uc[1]); // Get bytes 1-4 into a word
	p->dlc = 5;	// Command plus char*
	p->cd.uc[1] = *rdaddr++;
	p->cd.uc[2] = *rdaddr++;
	p->cd.uc[3] = *rdaddr++;
	p->cd.uc[4] = *rdaddr;
printf("R4: read addr: %X %X %X %X %X %X\n\r",(unsigned int)rdaddr, (unsigned int)p->cd.uc[0],
	(unsigned int)p->cd.uc[1],
	(unsigned int)p->cd.uc[2],
	(unsigned int)p->cd.uc[3],
	(unsigned int)p->cd.uc[4]);
	can_msg_put(p);	// Place in CAN output buffer
	return;
}
/* **************************************************************************************
 * void do_getfromdaddress(struct CANRCVBUF* p, uint8_t* rdaddr);
 * @brief	: Return msg with 4 bytes in payload uc[1-4] from address d
 * @param	: p = pointer to message buffer holding the imperial command
 * @param	: rdaddr = address to use 
 * ************************************************************************************** */
void do_getfromdaddress(struct CANRCVBUF* p, uint8_t* rdaddr)
{
	if (addressOK(rdaddr) != 0)
	{printf("do_getfromdaddress: addr not OK: %08X\n\r",(unsigned int)rdaddr); return;}

	p->dlc = 5;	// Command plus addr
	p->cd.uc[1] = *rdaddr++;
	p->cd.uc[2] = *rdaddr++;
	p->cd.uc[3] = *rdaddr++;
	p->cd.uc[4] = *rdaddr;
printf("GETADDR: read addr: %X %X %X %X %X\n\r", 
	(unsigned int)p->cd.uc[0],
	(unsigned int)p->cd.uc[1],
	(unsigned int)p->cd.uc[2],
	(unsigned int)p->cd.uc[3],
	(unsigned int)p->cd.uc[4]);
	can_msg_put(p);	// Place in CAN output buffer
	return;
}
/* **************************************************************************************
 * void do_send4(struct CANRCVBUF* p, uint32_t n);
 * @brief	: Send 4 bytes
 * @param	: Point to message buffer holding the imperial command
 * @param	: n = 4 byte number
 * ************************************************************************************** */
void do_send4(struct CANRCVBUF* p, uint32_t n)
{
	p->dlc = 5;	// Command plus char*
	p->cd.uc[1] = n;
	p->cd.uc[2] = n >> 8;
	p->cd.uc[3] = n >> 16;
	p->cd.uc[4] = n >> 24;
printf("send4: %X %X %X %X %X %X\n\r", (unsigned int)n, 
	(unsigned int)p->cd.uc[0],
	(unsigned int)p->cd.uc[1],
	(unsigned int)p->cd.uc[2],
	(unsigned int)p->cd.uc[3],
	(unsigned int)p->cd.uc[4]);

	can_msg_put(p);	// Place in CAN output buffer
	return;
}
/* **************************************************************************************
 * void do_getflashpaddr(struct CANRCVBUF* p);
 * @brief	: Return msg number of crc blocks, and address to array of blocks
 * @param	: p = pointer to message buffer holding the imperial command
 * ************************************************************************************** */
extern uint8_t* __appoffset;
extern uint8_t* __highflashp;

void do_getflashpaddr(struct CANRCVBUF* p)
{
	p->dlc = 8;	// Command plus addr

	extern void* __appjump;	// Defined in ldr.ld file
	uint32_t* ppflashp = (uint32_t*)((uint32_t)((uint8_t*)*&__appjump + 7 + 0));	// Points to "size"
	
	uint32_t size = *ppflashp;		// Get size
	uint32_t pflashp = *(ppflashp + 1);	// Get pointer to FLASHP and beginning of app struct

	if (size > 0x000fffff) size = 0x000fffff; // Only 3 bytes allowed for size (and < 1 MB)

	/* Payload [0] = cmd code; [1] - [3] = size; [4] - [7] = FLASHP address */
	uint8_t tmp = p->cd.uc[0];
	p->cd.ui[0] = (size << 8);	
	p->cd.uc[0] = tmp;
	p->cd.ui[1] = pflashp;

int i;printf("GET FLASHP addr ");
for (i = 0; i < 8; i++) printf(" %X",(unsigned int)p->cd.uc[i]); 
printf("\n\r"); 

	can_msg_put(p);	// Place in CAN output buffer
	return;
}


/* **************************************************************************************
 * void do_cmd_cmd(struct CANRCVBUF* p);
 * @brief	: Do something!
 * @param	: p = pointer to message buffer
 * ************************************************************************************** */
void do_cmd_cmd(struct CANRCVBUF* p)
{
/*	
NSERT INTO CMD_CODES  VALUES ('LDR_SET_ADDR',      1,	'5 Set address pointer (not FLASH) (bytes 2-5):  Respond with last written address.');
INSERT INTO CMD_CODES  VALUES ('LDR_SET_ADDR_FL',   2,	'5 Set address pointer (FLASH) (bytes 2-5):  Respond with last written address.');
INSERT INTO CMD_CODES  VALUES ('LDR_CRC',           3,	'8 Get CRC: 2-4 = count; 5-8 = start address; Reply CRC 2-4 na, 5-8 computed CRC ');
INSERT INTO CMD_CODES  VALUES ('LDR_ACK',           4,	'1 ACK: Positive acknowledge (Get next something)');
INSERT INTO CMD_CODES  VALUES ('LDR_NACK',          5,	'1 NACK: Negative acknowledge (So? How do we know it is wrong?)');
INSERT INTO CMD_CODES  VALUES ('LDR_JMP',           6,	'5 Jump: to address supplied (bytes 2-5)');
INSERT INTO CMD_CODES  VALUES ('LDR_WRBLK',         7,	'1 Done with block: write block with whatever you have.');
INSERT INTO CMD_CODES  VALUES ('LDR_RESET',         8,	'1 RESET: Execute a software forced RESET');
INSERT INTO CMD_CODES  VALUES ('LDR_XON',           9,	'1 Resume sending');
INSERT INTO CMD_CODES  VALUES ('LDR_XOFF',          10,	'1 Stop sending');
INSERT INTO CMD_CODES  VALUES ('LDR_FLASHSIZE',		11,	'1 Get flash size; bytes 2-3 = flash block size (short)');
INSERT INTO CMD_CODES  VALUES ('LDR_ADDR_OOB',		12,	'1 Address is out-of-bounds');
INSERT INTO CMD_CODES  VALUES ('LDR_DLC_ERR',		13,	'1 Unexpected DLC');
INSERT INTO CMD_CODES  VALUES ('LDR_FIXEDADDR',		14,	'5 Get address of flash with fixed loader info (e.g. unique CAN ID)');
INSERT INTO CMD_CODES  VALUES ('LDR_RD4',           15,	'5 Read 4 bytes at address (bytes 2-5)');
INSERT INTO CMD_CODES  VALUES ('LDR_APPOFFSET',		16,	'5 Get address where application begins storing.');
INSERT INTO CMD_CODES  VALUES ('LDR_HIGHFLASHH',	17,	'5 Get address of beginning of struct with crc check and CAN ID info for app');
INSERT INTO CMD_CODES  VALUES ('LDR_HIGHFLASHP',	18,	'8 Get address and size of struct with app calibrations, parameters, etc.');
INSERT INTO CMD_CODES  VALUES ('LDR_ASCII_SW',		19,	'2 Switch mode to send printf ASCII in CAN msgs');
INSERT INTO CMD_CODES  VALUES ('LDR_ASCII_DAT',		20,	'3-8 [1]=line position;[2]-[8]=ASCII chars');
INSERT INTO CMD_CODES  VALUES ('LDR_WRVAL_PTR',		21,	'2-8 Write: 2-8=bytes to be written via address ptr previous set.');
INSERT INTO CMD_CODES  VALUES ('LDR_WRVAL_PTR_SIZE',22,	'Write data payload size');
INSERT INTO CMD_CODES  VALUES ('LDR_WRVAL_AI',		23,	'8 Write: 2=memory area; 3-4=index; 5-8=one 4 byte value');
INSERT INTO CMD_CODES  VALUES ('LDR_SQUELCH',		24,	'8 Send squelch sending tick ct: 2-8 count');

INSERT INTO CMD_CODES  VALUES ('CMD_GET_IDENT',		30,	'Get parameter using indentification name/number in byte [1]');
INSERT INTO CMD_CODES  VALUES ('CMD_PUT_IDENT',		31,	'Put parameter using indentification name/number in byte [1]');
INSERT INTO CMD_CODES  VALUES ('CMD_GET_INDEX',		32,	'Get parameter using index name/number in byte [1]');
INSERT INTO CMD_CODES  VALUES ('CMD_PUT_INDEX',		33,	'Put parameter using index name/number in byte [1]; parameter in [2]-[5]');
INSERT INTO CMD_CODES  VALUES ('CMD_REVERT',        34,	'Revert (re-initialize) working parameters/calibrations/CANIDs back to stored non-volatile values');
INSERT INTO CMD_CODES  VALUES ('CMD_SAVE',          35,	'Write current working parameters/calibrations/CANIDs to non-volatile storage');
INSERT INTO CMD_CODES  VALUES ('CMD_GET_READING',   36,	'Send a reading for the code specified in byte [1] specific to function');
INSERT INTO CMD_CODES  VALUES ('CMD_GET_READING_BRD',  37,	'Send a reading for the code specified in byte [1] for board; common to functions');
INSERT INTO CMD_CODES  VALUES ('CMD_LAUNCH_PARM_HDSHK',38,	'Send msg to handshake transferring launch parameters');
INSERT INTO CMD_CODES  VALUES ('CMD_SEND_LAUNCH_PARM', 39,	'Send msg to send burst of parameters');
INSERT INTO CMD_CODES  VALUES ('CMD_REQ_HISTOGRAM',    40, 'Request histogram: [1] ADC #: [2] # consecutive:[3]-[6] # DMA buffers accumuleted in bins');
INSERT INTO CMD_CODES  VALUES ('CMD_THISIS_HISTODATA', 41, 'Histogram data item: [1] ADC #:[2] bin # (0 - n), [3]-[6] bin count');

INSERT INTO CMD_CODES  VALUES ('CMD_CMD_TYPE1',		42,	'[1]-[7] format 1');
INSERT INTO CMD_CODES  VALUES ('CMD_CMD_TYPE2',		43,	'[1]-[7] format 2');
INSERT INTO CMD_CODES  VALUES ('CMD_CMD_TYPE3',		44,	'[1]-[7] format 3');
INSERT INTO CMD_CODES  VALUES ('CMD_CMD_HEARTBEAT', 45,	'[1]-[7] Heartbeat format specific to unit');
INSERT INTO CMD_CODES  VALUES ('CMD_CMD_TYPE4',     46,	'[1]-[7] format 4');

INSERT INTO CMD_CODES  VALUES ('CMD_CMD_SYS_RESET_ALL',166,	'0xA6: [0] Cause System Reset for all');
INSERT INTO CMD_CODES  VALUES ('CMD_CMD_SYS_RESET_CID',167,	'0xA7: [0] Cause System Reset for CAN ID sent in payload');
INSERT INTO CMD_CODES  VALUES ('CMD_CMD_SYS_RESET_EXT',168,	'0xA8: [0] Extend current loader wait duration for all, [1] = wait in 0.1 sec steps');
*/
//printf("Q: %02X\n\r",p->cd.uc[0]);  // Debug: Display command codes coming in
	/* Here, we have a command in the ID field. */
	extern uint32_t can_waitdelay_ct;
	uint32_t x	;
	switch (p->cd.uc[0])	// Command code
	{
	case LDR_SET_ADDR: // Set address pointer (bytes 2-5):  Respond with last written address.
		do_set_addr(p);
		break;

	case LDR_ACK:		// ACK: Positive acknowledge
		break;

	case LDR_NACK:		// NACK: Negative acknowledge
		break;

	case LDR_JMP:		// Jump: to address supplied
		do_jump(p);
		break;

	case LDR_WRBLK:		// Done with block: write block with whatever you have..
		do_wrblk(p);
		break;

	case LDR_RESET:		// RESET: Execute a software forced RESET for this unit only.
		system_reset();	// Cause a RESET
		break;

	case LDR_CRC:		// Get CRC: given count | start address, compute CRC and respond with it.
		do_crc(p);
		break;	
	
	case LDR_XON:		// Resume sending
		break;

	case LDR_XOFF:		// Stop sending: response to our sending LDR_XOFF msg.
		break;

	case LDR_FLASHSIZE:	// Send flash size
		do_flashsize(p);
		break;

	case LDR_FIXEDADDR:	// Send address ahead of fixed address block
		do_getfromdaddress(p, (uint8_t*)0x08000004);
		break;

	case LDR_RD4:		// Send address of fixed param flash area
		do_rd4(p);
		break;

	case LDR_APPOFFSET:	// Send address of where app loads
		do_send4(p, (uint32_t)&__appoffset);
		break;

	case LDR_HIGHFLASHH:	// Send address of high flash area
//		do_send4(p, (uint32_t)&__highflashlayout);
		break;

	case LDR_HIGHFLASHP:	// Get address of beginning of crc check info for app
		do_getflashpaddr(p);
		break;

	case LDR_WRVAL_PTR:	// Write: 2-8=bytes to be written via address ptr previous set
		// Write data starting at payload[1], dlc holds byte ct: 1-7
		do_datawrite(&p->cd.uc[1], (0x7f & p->dlc) - 1, p);
		sendcanCMD(LDR_ACK);
		break;

	case CMD_CMD_SYS_RESET_ALL: // Reset cmd is for "all"
		system_reset();
		break;

	case CMD_CMD_SYS_RESET_CID:	// Reset cmd is only of "us"
		x = *(uint32_t*)&p->cd.uc[4];
		if (x == p->id)
		{
			system_reset();
		}
		break;

	case CMD_CMD_SYS_RESET_EXT: // Extend loader wait timeout (for "all")
		can_waitdelay_ct = (DTWTIME + (p->cd.uc[1]+1)*(SYSCLOCKFREQ/10));
		break;

	case CMD_CMD_HEARTBEAT:
printf("LOOPBACK?  %08X\n\r",(unsigned int)p->id);	
		break;		

	default:		// Not a defined command
		err_bogus_cmds_cmds += 1;
printf("BOGUS CMD CODE: %X %08X %X  %08X %08X\n\r",
	(unsigned int)p->cd.uc[0], 
	(unsigned int)p->id, 
	(unsigned int)p->dlc, 
	(unsigned int)p->cd.ui[0], 
	(unsigned int)p->cd.ui[1]);
		break;
	}
	return;
}
/* **************************************************************************************
 * void canwinch_ldrproto_poll(void);
 * @param	: pctl = pointer control block for CAN module being used
 * @brief	: If msg is for this unit, then do something with it.
 * ************************************************************************************** */
static uint8_t sw_oto = 0;
static struct CANTAKEPTR* ptake;
void canwinch_ldrproto_poll(void)
{
	struct CANRCVBUF can;

	if (sw_oto == 0)
	{
		sw_oto = 1;
		ptake = can_iface_add_take(pctl0);
		if (ptake == NULL) morse_trap(481);

	}
	if (ptake->ptake != ptake->pcir->pwork)
	{ // Here, there is a CAN msg in the buffer
		can = ptake->ptake->can; // Save local copy of CAN msg

		/* Advance ptr in circular buffer. */
		ptake->ptake += 1;
		if (ptake->ptake == ptake->pcir->pend)
			ptake->ptake = ptake->pcir->pbegin;

		/* Do something! */
		do_cmd_cmd(&can);		// Execute command
	}

#if 0 // Old code jic
	/* Check incoming msgs. */
	while ((pcan = can_driver_peek0(pctl1)) != NULL) // RX0 have a msg?
	{ // Here we have a FIFO0 CAN msg
//if ((pcan->id & 0x0000000e) == 0x0000000c) 
//	printf ("X: %08X %d %08X %08X\n\r",pcan->id,pcan->dlc,pcan->cd.ui[0],pcan->cd.ui[1]);
//printf ("A: %08X\n\r",pcan->id);


		/* Select msg if matches our unit CAN ID  */
		if ( (pcan->id & 0x0ffffffe) == fixedaddress.canid_ldr)
		{ // Here CAN ID is for this unit/loader
//printf ("X: %08X %d %08X %08X\n\r",pcan->id,pcan->dlc,pcan->cd.ui[0],pcan->cd.ui[1]);
			do_cmd_cmd(pcan);		// Execute command 
		}
		can_driver_toss0(pctl1);	// Release buffer block
	}
	/* Dump FIFO1 msgs. */
	while ((can_driver_peek1(pctl1)) != NULL)	// RX1 have a msg?
		can_driver_toss1(pctl1); 	// Release buffer block
#endif

	return;
}


