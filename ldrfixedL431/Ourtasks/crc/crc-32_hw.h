/******************************************************************************
* File Name          : crc-32_hw.c
* Date First Issued  : 08/27/22
* Board              : ...
* Description        : CRC-32 hardware implementaton L431
*******************************************************************************/

#ifndef __CRC_32_HW
#define __CRC_32_HW

#include <stdint.h>
/******************************************************************************/
uint8_t rc_crc32_hw_init(void);
/* @brief 	: Hardware set up for CRC
 * @return	: something
 ******************************************************************************/
uint32_t rc_crc32_hw(uint32_t* buff, uint32_t len);
/* @brief 	: Compute CRC-32 byte-by-byte
 * @param	: pData = pointer input bytes
 * @param	: len = byte count
 * @return	: crc
*******************************************************************************/

#endif 

