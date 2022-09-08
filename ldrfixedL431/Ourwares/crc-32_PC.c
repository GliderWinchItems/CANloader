/******************************************************************************
* File Name          : crc-32_PC.c
* Date First Issued  : 08-29-2022
* Board              : 
* Description        : CRC-32 computation on PC to match STM32
*******************************************************************************/
/* See--
https://community.st.com/s/global-search/CRC-32
*/
//--------CRC32.c---------------------------------------
#include <stdint.h>
uint32_t Table[256];
uint32_t CRC;
uint32_t Reflect(uint32_t ref, char ch)
{
    uint32_t value = 0;
    // Swap bit 0 for bit 7
    // bit 1 for bit 6, etc.
    for(int i = 1; i < (ch + 1); i++)
    {
        if (ref & 1)
            value |= 1 << (ch - i);
        ref >>= 1;
    }
    return value;
}
 
static void crc_32_gen_table(void)
{
    // This is the official polynomial used by CRC-32 
    // in PKZip, WinZip and Ethernet. 
    uint32_t ulPolynomial = 0x04C11DB7;

    // 256 values representing ASCII character codes.
    for (int i = 0; i <= 0xFF; i++)
    {
        Table[i] = Reflect(i, 8) << 24;
        for (int j = 0; j < 8; j++)
            Table[i] = (Table[i] << 1) ^ (Table[i] & (1 << 31) ? ulPolynomial : 0);
        Table[i] = Reflect(Table[i], 32);
    }
}
 
void CRC32_Calculate(const LPBYTE buffer, UINT size, uint32_t &CRC)
{   // calculate the CRC
    LPBYTE pbyte = buffer;
 
    while (size--)
        CRC = (CRC >> 8) ^ Table[(CRC & 0xFF) ^ *pbyte++];
}
 
//--------main.c--------------------------------------------
 
crc_32_gen_table();
CRC = 0xFFFFFFFF;
CRC32_Calculate((LPBYTE)Buffer, sz, CRC);
CRC ^= 0xFFFFFFFF;
 
//========== STM32F side===========
/**********************
reverse bits of DWORD
Input:
  u32 data -- the input
Output:
  u32 data -- the output
**********************/
uint32_t revbit(uint32_t data)
 
{
  asm(''rbit r0,r0'');
  return data;
};
/**********************
Calculate CRC32 of DWORD data array.
Input:
  u32 *dworddata -- the array point
  u32 dwordcount -- the data len in DWORD
Output:
  u32 CRC32 -- the result
**********************/
 
uint32_t CalcCRC32(uint32_t *dworddata,uint32_t dwordcount)
{
  uint32_t ui32;
  for(;dwordcount>0;dwordcount--)
  {
    ui32=*dworddata;
    dworddata++;
    ui32=revbit(ui32);
    CRC->DR=ui32;
  }
  ui32=CRC->DR;
  ui32=revbit(ui32);
  ui32^=0xffffffff;
  return ui32;
};
//========================================

 

 

 

 