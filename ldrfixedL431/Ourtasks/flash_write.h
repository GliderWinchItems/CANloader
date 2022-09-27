/******************************************************************************
* File Name          : flash_write.c
* Date First Issued  : 09/27/2022
* Board              : bmsadbms1818
* Description        : flash write: STM32L431 standard mode
*******************************************************************************/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FLASH_WRITE
#define __FLASH_WRITE

#define FLASH_SIZE_REG	MMIO16(0x1FFFF7E0)	//This field value indicates the Flash memory size of the device in Kbytes.
						//Example: 0x0080 = 128 Kbytes. (p 1044 ref man)


/* Includes ------------------------------------------------------------------*/
#include "common_can.h"	

/******************************************************************************/
 int flash_erase(uint64_t* pflash);
/*  @brief 	: Erase one page
 *  @param	: pflash = double word pointer to address in flash
 *  @return	: 
 *           0 = success
 *          -3 = address below start of flash
 *          -4 = unlock sequence failed for lower bank
 *          -5 = error at some point in the writes, flash_err has the bits
*******************************************************************************/
int flash_write(uint64_t *pflash, uint64_t *pfrom, int count);
/*  @brief 	: Write "count" uint64_t double words to flash
 *  @param	: pflash = pointer to double word flash address
 *  @param	: pfrom  = pointer to double word sram address
 *  @param	: count = number of double words
 *  @return	: 
 *           0 = success
 *          -1 = address greater than 1 MB
 *          -2 = unlock sequence failed for upper bank
 *          -3 = address below start of ram.
 *          -4 = unlock sequence failed for lower bank
 *          -5 = error at some point in the writes, flash_err has the bits
*******************************************************************************/


#endif

