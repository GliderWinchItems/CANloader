/******************************************************************************
* File Name          : can_iface_bmx.c
* Date First Issued  : 08/27/2022
* Board              : L431
* Description        : Interface HAL CAN: mailbox lower level 
*******************************************************************************/
#define CANBUFSIZE 8 
static struct CANTAKEPTR* ptake;
static struct CANRCVBUF can[CANBUFSIZE];
/* *************************************************************************
 * void can_iface_bmx_init(struct CAN_CTLBLOCK* pctl);
 * @brief	: Initialize lower level interrupt to filter CAN msgs
 * *************************************************************************/
void can_iface_bmx_init(struct CAN_CTLBLOCK* pctl)
{
	/* Get pointer to can_iface incoming CAN circular buffer. */
	ptake = can_iface_add_take(struct CAN_CTLBLOCK*  pctl);
	if (ptake == NULL) morse_trap(480);

	/* Initialize interrupt: SDMMC1_IRQHandler */

	return ptake;
}
/* *************************************************************************
 * uint8_t can_iface_mbx_chk(uint32_t canid);
 * @brief	: Check if CAN ID is in our list
 * @param   : 0 = no; 1 = yes
 * *************************************************************************/
uint8_t can_iface_mbx_chk(uint32_t canid)
{
	if (canid == IAMUNITNUMBER)	return 1;
	if (canid == CANID_ALL_PC)  return 1;
	if (canid == CANID_ALL_OTH) return 1;
	return 0;
}


