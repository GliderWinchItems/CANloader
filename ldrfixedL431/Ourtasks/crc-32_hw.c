/******************************************************************************
* File Name          : crc-32_hw.c
* Date First Issued  : 08/27/22
* Board              : ...
* Description        : CRC-32 hardware implementaton L431
*******************************************************************************/
/*
*/
/*
For some testing--
https://crccalc.com/

0x11 0x22 0x33 0x44 0x55 0x66 0x77 0x88  

Algorithm   Result  Check   Poly  Init  RefIn   RefOut  XorOut
CRC-32
  0x9118E1C2  0xCBF43926  0x04C11DB7  0xFFFFFFFF  true  true  0xFFFFFFFF
CRC-32/BZIP2
  0x25404A32  0xFC891918  0x04C11DB7  0xFFFFFFFF  false   false   0xFFFFFFFF
CRC-32C
  0xD441A37D  0xE3069283  0x1EDC6F41  0xFFFFFFFF  true  true  0xFFFFFFFF
CRC-32D
  0xC98043E1  0x87315576  0xA833982B  0xFFFFFFFF  true  true  0xFFFFFFFF
CRC-32/JAMCRC
  0x6EE71E3D  0x340BC6D9  0x04C11DB7  0xFFFFFFFF  true  true  0x00000000
CRC-32/MPEG-2
  0xDABFB5CD  0x0376E6E7  0x04C11DB7  0xFFFFFFFF  false   false   0x00000000
CRC-32/POSIX
  0x4C44F16B  0x765E7680  0x04C11DB7  0x00000000  false   false   0xFFFFFFFF
CRC-32Q
  0x59556055  0x3010BF7F  0x814141AB  0x00000000  false   false   0x00000000
CRC-32/XFER
  0x15EE692A  0xBD0BE338  0x000000AF  0x00000000  false   false   0x00000000
  */
#include "main.h"
/******************************************************************************
 * uint8_t rc_crc32_hw_init(void);
 * @brief 	: Hardware set up for CRC
 * @return	: something
*******************************************************************************/
uint8_t rc_crc32_hw_init(void)
{
	/* Bit 12 CRCEN: CRC clock enable */
    RCC->AHB1ENR |= RCC_AHB1ENR_CRCEN;
	return 0;
}

/******************************************************************************
 * uint32_t rc_crc32_hw(uint32_t* buff, uint32_t len);
 * @brief 	: Compute CRC-32 byte-by-byte
 * @param	: pData = pointer input bytes
 * @param	: len = byte count
 * @return	: crc
*******************************************************************************/
uint32_t rc_crc32_hw(uint32_t* pdata, uint32_t len)
#define THIRTY_TWO // Use the full word implementaton
#ifdef  THIRTY_TWO
{
//	uint32_t* pend  = pdata + len;

 	/* Reset sets polynomial to 0x04C11DB7 and INIT to 0xFFFFFFFF */
	CRC->CR = 0x01; // 32b poly, + reset CRC computation
	do		
	{
//		*(__IO uint32_t*)CRC_BASE = (uint32_t)__REV (*pdata++);
		*(__IO uint32_t*)CRC_BASE = ((uint32_t)*pdata++);
		len -= 1;
	} while (len != 0);

	return ~CRC->DR;
}
#else
// Byte implementation
{
  uint8_t* p8 = (uint8_t*)pdata;
  uint8_t* pend = (uint8_t*)p8 + len*4;
  CRC->CR = 0x1; // Reset 
  do
  {
     *(__IO uint8_t*)CRC_BASE = *p8++;
  } while (p8< pend);

  return ~CRC->DR;//*CRCBASE;
}

#endif
