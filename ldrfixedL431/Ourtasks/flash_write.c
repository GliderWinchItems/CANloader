/******************************************************************************
* File Name          : flash_write.c
* Date First Issued  : 07/28/2013
* Board              : ../svn_sensor/hw/trunk/eagle/f103R/RxT6
* Description        : flash write: small bits of code that execute in ram
*******************************************************************************/
/*


*/
#include "stm32l431xx.h"
#include "flash_write.h"
/******************************************************************************
 * int flash_write_ram(uint16_t *pflash, uint16_t* pfrom, int count);
 *  @brief 	: Write "count" uint16_t (1/2 words) to flash from ram
 *  @param	: pflash = 1/2 word pointer to address in flash
 *  @param	: pfrom = 1/2 word pointer to address with source data
 *  @param	: count = number of 1/2 words to write
 *  @return	: zero = success; not zero = failed
*******************************************************************************/
/*     The main Flash memory programming sequence in standard mode is as follows:
      ●    Check that no main Flash memory operation is ongoing by checking the BSY bit in the
           FLASH_SR register.
      ●    Set the PG bit in the FLASH_CR register.
      ●    Perform the data write (half-word) at the desired address.
      ●    Wait for the BSY bit to be reset.
      ●    Read the programmed value and verify.
Note: The registers are not accessible in write mode when the BSY bit of the FLASH_SR register
      is set.
*/
int flash_write_ram(uint16_t *pflash, uint16_t *pfrom, int count)
{
	while (count > 0)
	{
		while ((FLASH->SR & 0x1) != 0);
		FLASH->CR |= 1;	// Set program bit
		*pflash++ = *pfrom++;
		while ((FLASH->SR & 0x1) != 0);
		FLASH->CR |= 1;	// Set program bit
		*pflash++ = *pfrom++;
		count--;
	}
	
	return 0;	
}

/******************************************************************************
 *  int flash_unlock(uint32_t address);
 *  @brief 	: Perform unlock sequence
 *  @return	: 0 = unlocked; not zero = failed and locked until next reset.
*******************************************************************************/
#define FLASH_RDPRT_KEY 0x00A5	// Protection code
#define FLASH_KEY1  0x45670123	// Unlock 1st
#define FLASH_KEY2  0xCDEF89AB	// Unlock 2nd

int flash_unlock(void)
{
	FLASH->KEYR = FLASH_KEY1;
	FLASH->KEYR = FLASH_KEY2;
	return (FLASH->CR & FLASH_CR_LOCK);
}
#ifdef TWO_FLASH_BANKS
int flash_unlock2(void)
{
	FLASH->KEYR2 = FLASH_KEY1;
	FLASH->KEYR2 = FLASH_KEY2;
	return (FLASH->CR2 & FLASH_CR_LOCK);
}
#endif

/******************************************************************************
 * int flash_write(uint16_t *pflash, uint16_t *pfrom, int count);
 *  @brief 	: Write "count" uint16_t (1/2 words) to flash
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
uint32_t flash_err;
int flash_write(uint16_t *pflash, uint16_t *pfrom, int count)
{
	int i;
	flash_err = 0;
	for (i = 0; i < count; i++)
	{
#ifdef TWO_FLASH_BANKS

		if (pflash >= (uint16_t*)0x08080000)
		{ // Here maybe 2nd bank
			if (pflash > (uint16_t*)0x080FFFFe)  return -1;
			while ((FLASH->SR2 & 0x1) != 0);	// Wait for busy to go away
			if (flash_unlock2() != 0) return -2;
			FLASH->SR2 = (FLASH_EOP | FLASH_WRPRTERR | FLASH_PGERR); // Clear any error bits
			FLASH->CR2 = FLASH_PG;		// Set program bit
			*pflash++ = *pfrom++;		// Program the 1/2 word
			while ((FLASH->SR2 & 0x1) != 0);	// Wait for busy to go away
			flash_err |= FLASH->SR2;			
		}
		else
#endif			
		{ // Here maybe 1st bank
			if (pflash < (uint16_t*)0x0800000)  return -3;
			while ((FLASH->SR & 0x1) != 0);	// Wait for busy to go away
			if (flash_unlock() != 0) return -4;
			FLASH->SR = (FLASH_SR_EOP | FLASH_SR_WRPERR | FLASH_SR_PROGERR); // Clear any error bits
			FLASH->CR = FLASH_SR_PGAERR;		// Set program bit
			*pflash++ = *pfrom++;		// Program the 1/2 word
			while ((FLASH->SR & 0x1) != 0);	// Wait for busy to go away
			flash_err |= FLASH->SR;			
		}
	}	
	if ( (flash_err & (FLASH_SR_WRPERR | FLASH_SR_PGAERR)) != 0) return -5;	
	return 0;
}
/******************************************************************************
 * int flash_erase(uint16_t* pflash);
 *  @brief 	: Erase one page
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
/*
● Check that no Flash memory operation is ongoing by checking the BSY bit in the
  FLASH_CR register
● Set the PER bit in the FLASH_CR register
● Program the FLASH_AR register to select a page to erase
● Set the STRT bit in the FLASH_CR register
● Wait for the BSY bit to be reset
● Read the erased page and verify
*/
int flash_erase(uint16_t *pflash)
{
	flash_err = 0;
	if (pflash >= (uint16_t*)0x08080000)
	{ // Here maybe 2nd bank
#ifdef TWO_FLASH_BANKS

		if (pflash > (uint16_t*)0x080FFFFe)  return -1;
		while ((FLASH_SR2 & 0x1) != 0);	// Wait for busy to go away
		if (flash_unlock2() != 0) return -2;
		FLASH_CR2 = FLASH_CR_PER;		// Set Page Erase function
		FLASH_ACR2 = (uint32_t)pflash;	// Set page address
		FLASH_CR2 |= FLASH_CR_STRT;	// Start erase
		while ((FLASH_SR2 & 0x1) != 0);	// Wait for busy to go away
		FLASH_CR2 &= ~FLASH_CR_PER;	// Remove PER bit
		flash_err |= FLASH_SR2;
#endif	
	}
	else
	{ // Here maybe 1st bank
		if (pflash < (uint16_t*)0x0800000)  return -3;
		while ((FLASH->SR & 0x1) != 0);	// Wait for busy to go away
		if (flash_unlock() != 0) return -4;
		FLASH->CR  = FLASH_CR_PER;		// Set Page Erase function
		FLASH->ACR = (uint32_t)pflash;		// Set page address
		FLASH->CR |= FLASH_CR_STRT;		// Start erase
		while ((FLASH->SR & 0x1) != 0);	// Wait for busy to go away
		FLASH->CR &= ~FLASH_CR_PER;		// Remove PER bit
		flash_err |= FLASH->SR;
	}
	if ( (flash_err & (FLASH_SR_WRPERR | FLASH_SR_PGAERR)) != 0) return -5;
	return 0;
}

