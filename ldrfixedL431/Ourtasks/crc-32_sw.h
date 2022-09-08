/******************************************************************************
* File Name          : crc-32_sw.h
* Date First Issued  : 10/03/2014
* Board              : ...
* Description        : CRC-32 (byte-byt-byte) computation
*******************************************************************************/

#ifndef __CRC_32_SW
#define __CRC_32_SW

#include <stdint.h>
//#include "common.h"
/******************************************************************************/
uint32_t rc_crc32_sw(uint8_t *buf, uint32_t len);
/* @brief 	: Compute CRC-32 byte-byt-byte
 * @param	: pData = pointer input bytes
 * @param	: len = byte count
 * @return	: crc
*******************************************************************************/

#endif 

