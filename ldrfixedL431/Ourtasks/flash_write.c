/******************************************************************************
* File Name          : flash_write.c
* Date First Issued  : 09/27/2022
* Board              : bmsadbms1818
* Description        : flash write: STM32L431 standard mode
*******************************************************************************/
/*
07/28/2013 Original flash_write.c
09/27/2022 Update & test for STM32L431
*/

#include "stm32l431xx.h"
//#include "stm32l4xx_hal_flash.c"
#include "flash_write.h"

#include "DTW_counter.h"

/******************************************************************************
 *  int flash_unlock(uint32_t address);
 *  @brief 	: Perform unlock sequence
 *  @return	: 0 = unlocked; not zero = failed and locked until next reset.
*******************************************************************************/
#define FLASH_RDPRT_KEY 0x00A5	// Protection code
#define FLASH_KEY1  0x45670123	// Unlock 1st
#define FLASH_KEY2  0xCDEF89AB	// Unlock 2nd

#define ERR_FLGS 0xC3FB // Error flags

extern uint32_t __appbegin; // .ld file supplies app loading address

static uint8_t otosw;

int flash_unlock(void)
{
	if (otosw != 0) return 0;
	otosw = 1;
	FLASH->KEYR = FLASH_KEY1;
	FLASH->KEYR = FLASH_KEY2;
	return (FLASH->CR & FLASH_CR_LOCK);
}

/******************************************************************************
 * int flash_write(uint64_t *pflash, uint64_t *pfrom, int count);
 *  @brief 	: Write "count" uint64_t double words to flash
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
/*
1. Check that no Flash main memory operation is ongoing by checking the BSY bit in the
	Flash status register (FLASH_SR).

2. Check and clear all error programming flags due to a previous programming. If not,
	PGSERR is set.

3. Set the PG bit in the Flash control register (FLASH_CR).

4. Perform the data write operation at the desired memory address, inside main memory
	block or OTP area. Only double word can be programmed.
	– Write a first word in an address aligned with double word
	– Write the second word

5. Wait until the BSY bit is cleared in the FLASH_SR register.

6. Check that EOP flag is set in the FLASH_SR register (meaning that the programming
	operation has succeed), and clear it by software.

7. Clear the PG bit in the FLASH_CR register if there no more programming request anymore.

*/
uint32_t dtwfl1;
uint32_t dtwfl2;
#if 1
uint32_t flash_err;
int flash_write(uint64_t *pflash, uint64_t *pfrom, int count)
{
	int i;
	flash_err = 0;

	/* Don't ruin the CANloader! */
	if ((uint32_t*)pflash < &__appbegin)  return -3;

//	if (flash_unlock() != 0) return -4;

	while ((FLASH->SR & 0x1) != 0);	// Wait for busy to go away

uint32_t* pfl = (uint32_t*)pflash;
uint32_t* prm = (uint32_t*)pfrom;	

dtwfl1 = DTWTIME;

	for (i = 0; i < count; i++)
	{
	
		while ((FLASH->SR & 0x1) != 0);	// Wait for busy to go away
		
		/* Clear any existing error flags. */
		FLASH->SR |= ERR_FLGS; 

		/* Set PG (flash program bit) */
		FLASH->CR |= 0x1;  

		/* Send two words to flash */
		*pfl++ = *prm++; 

 		/* Barrier to ensure programming is performed in 2 steps, in right order
    	(independently of compiler optimization behavior) */
  		__ISB();

		*pfl++ = *prm++; 

		/* Wait for busy to go away */
		while ((FLASH->SR & (1 << 16)) != 0);	

		flash_err |= FLASH->SR;			
	}	
dtwfl2 = DTWTIME;	

	/* Clear PG (flash program bit) */
	FLASH->CR &= ~0x1; 

	if ( (flash_err & (FLASH_SR_WRPERR | FLASH_SR_PGAERR)) != 0) return -5;	
	return 0;
}
#endif

#if 0
uint32_t flash_err;
int flash_write(uint64_t *pflash, uint64_t *pfrom, int count)
{
	int i;
	flash_err = 0;

	/* Don't ruin the CANloader! */
	if ((uint32_t*)pflash < &__appbegin)  return -3;

//	if (flash_unlock() != 0) return -4;

	while ((FLASH->SR & 0x1) != 0);	// Wait for busy to go away

	for (i = 0; i < count; i++)
	{
	
		while ((FLASH->SR & 0x1) != 0);	// Wait for busy to go away
		
		/* Clear any existing error flags. */
		FLASH->SR |= ERR_FLGS; 

		/* Set PG (flash program bit) */
		FLASH->CR |= 0x1;  

		/* Send two words to flash */
		*pflash++ = *pfrom++; 

		/* Wait for busy to go away */
		while ((FLASH->SR & (1 << 16)) != 0);	

		flash_err |= FLASH->SR;			
	}	
	/* Clear PG (flash program bit) */
	FLASH->CR &= ~0x1; 

	if ( (flash_err & (FLASH_SR_WRPERR | FLASH_SR_PGAERR)) != 0) return -5;	
	return 0;
}
#endif
/******************************************************************************
 * int flash_erase(uint64_t* pflash);
 *  @brief 	: Erase one page
 *  @param	: pflash = double word pointer to address in flash
 *  @return	: 
 *           0 = success
 *          -3 = address below start of flash
 *          -4 = unlock sequence failed for lower bank
 *          -5 = error at some point in the writes, flash_err has the bits
*******************************************************************************/
/*
To erase a page (2 Kbyte), follow the procedure below:
1. Check that no Flash memory operation is ongoing by checking the BSY bit in the Flash
status register (FLASH_SR).
2. Check and clear all error programming flags due to a previous programming. If not,
PGSERR is set.
3. Set the PER bit and select the page you wish to erase (PNB) in the Flash control
register (FLASH_CR).
4. Set the STRT bit in the FLASH_CR register.
5. Wait for the BSY bit to be cleared in the FLASH_SR register.
*/

int flash_erase(uint64_t *pflash)
{
	flash_err = 0;

	/* Don't erase the CANloader! */
	if ((uint32_t*)pflash < &__appbegin)  return -3;

	while ((FLASH->SR & 0x1) != 0);	// Wait for busy to go away

	if (flash_unlock() != 0) return -4;

	/* Clear any existing error flags. */
	FLASH->SR |= ERR_FLGS; 

	/* Set the PER bit and select the page */
	FLASH->CR  &= ~((0x7FF) << 3); // Clear old page number
	uint16_t tmp = ((uint32_t)pflash >> 11); // New page number
	FLASH->CR  |=  (tmp << 3) | 0x2; // New page | Enable page erase

	/* Start erase. */
	FLASH->CR |= (1<<16); // Start bit

	/* Wait for busy to go away. */
	while ((FLASH->SR & 0x1) != 0);

	FLASH->CR &= ~FLASH_CR_PER; // Remove PER bit
	flash_err |= FLASH->SR;

	if ( (flash_err & (FLASH_SR_WRPERR | FLASH_SR_PGAERR)) != 0) return -5;
	return 0;
}

