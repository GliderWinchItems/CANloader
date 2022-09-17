/******************************************************************************
* File Name          : system_reset.c
* Board              : 
* Date First Issued  : 08/24/2022
* Description        : System reset
*******************************************************************************/
/* **************************************************************************************
 * void system_reset(void);
 * @brief	: Software caused RESET
 * ************************************************************************************** */
void system_reset(void)
{
/* PM 214 p 228 (March 2020)
4.4.4 Application interrupt and reset control register (SCB_AIRCR)
      Address offset: 0x0C
      Reset value: 0xFA05 0000
      Required privilege: Privileged
      The AIRCR provides priority grouping control for the exception model, endian status for data
      accesses, and reset control of the system.
      To write to this register, you must write 0x5FA to the VECTKEY field, otherwise the
      processor ignores the write.
*/

/* Bit 2 SYSRESETREQ System reset request
      This is intended to force a large system reset of all major components except for debug.
      This bit reads as 0.
      0: No system reset request
      1: Asserts a signal to the outer system that requests a reset.
*/
//	SCB_AIRCR = (0x5FA << 16) | SCB_AIRCR_SYSRESETREQ;	// Cause a RESET
      #define SCB_AIRCR 0xE000ED0C
	*(volatile unsigned int*)SCB_AIRCR = (0x5FA << 16) | 0x4;	// Cause a RESET
	while (1==1);
}