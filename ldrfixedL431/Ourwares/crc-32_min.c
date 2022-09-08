/******************************************************************************
* File Name          : crc-32_min.c
* Date First Issued  : 08-29-2022
* Board              : 
* Description        : CRC-32 computation with no table lookup (minimal)
*******************************************************************************/
/* See--
https://community.st.com/s/global-search/CRC-32
*/
#include "crc-32_min.h"
/******************************************************************************
 * uint32_t crc_32_min_acc(uint32_t Crc, uint32_t Data);
 * @brief   : Accumulate Crc value for one input word
 * @param   : Crc = previous Crc value
 * @param   : Data = input word
*******************************************************************************/
    uint32_t crc_32_min_acc(uint32_t Crc, uint32_t Data)
     {
     int i;

     Crc = Crc ^ Data;

     for(i=0; i<32; i++)
       if (Crc & 0x80000000)
         Crc = (Crc << 1) ^ 0x04C11DB7; // Polynomial used in STM32
       else
         Crc = (Crc << 1);
     
     return(Crc);
     }
/******************************************************************************
 * uint32_t crc_32_min_calc(uint32_t* pdata, uint32_t len);
 * @brief   : Compute CRC-32
 * @param   : pdata = pointer to input array (words)
 * @param   : len = number of words
 * @param   : CRC-32
*******************************************************************************/
uint32_t crc_32_min_calc(uint32_t* pdata, uint32_t len)
{
    uint32_t crc = ~0L;
    do
    {
        crc = crc_32_min_acc(crc, *pdata++);
        len -= 1;
    } while (len > 0);
    return crc;
}