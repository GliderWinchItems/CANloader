/******************************************************************************
* File Name          : flash_write.h
* Date First Issued  : 07/28/2013
* Board              : ../svn_sensor/hw/trunk/eagle/f103R/RxT6
* Description        : flash write: small bits of code that execute in ram
*******************************************************************************/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FLASH_WRITE
#define __FLASH_WRITE

#define FLASH_SIZE_REG	MMIO16(0x1FFFF7E0)	//This field value indicates the Flash memory size of the device in Kbytes.
						//Example: 0x0080 = 128 Kbytes. (p 1044 ref man)


/* Includes ------------------------------------------------------------------*/
#include "common_can.h"	

/******************************************************************************/
int flash_write_ram(uint16_t *pflash, uint16_t *pfrom, int count);
/*  @brief 	: Write "count" uint16_t (1/2 words) to flash
 *  @param	: pflash = 1/2 word pointer to address in flash
 *  @param	: pfrom = 1/2 word pointer to address with source data
 *  @param	: count = number of 1/2 words to write
 *  @return	: zero = success; not zero = failed
*******************************************************************************/
int flash_write(uint16_t *pflash, uint16_t *pfrom, int count);
/*  @brief 	: Write "count" uint16_t (1/2 words) to flash
 *  @param	: pflash = 1/2 word pointer to address in flash
 *  @param	: pfrom = 1/2 word pointer to address with source data
 *  @param	: count = number of 1/2 words to write
 *  @return	: 
 *           0 = success
 *          -1 = address greater than 1 MB
 *          -2 = unlock sequence failed for upper bank
 *          -3 = address below start of ram.
 *          -4 = unlock sequence failed for lower bank
 *          -5 = error at some point in the writes, flash_err has the bits
*******************************************************************************/
int flash_erase(uint16_t *pflash);
/*  @brief 	: Erase one page
 *  @param	: pflash = 1/2 word pointer to address in flash
 *  @param	: pfrom = 1/2 word pointer to address with source data
 *  @return	: 
 *           0 = success
 *          -1 = address greater than 1 MB
 *          -2 = unlock sequence failed for upper bank
 *          -3 = address below start of ram.
 *          -4 = unlock sequence failed for lower bank
 *          -5 = error at some point in the writes, flash_err has the bits
*******************************************************************************/


#endif

