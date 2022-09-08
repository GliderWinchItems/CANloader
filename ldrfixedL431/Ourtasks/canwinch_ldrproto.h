/******************************************************************************
* File Name          : canwinch_ldrproto.h
* Date First Issued  : 07/26/2013
* Board              : RxT6
* Description        : Loader protocol work
*******************************************************************************/
/*
08/25/2022 Revise for L431 in GliderWinchItems/CANloader
*/

#ifndef __CANWINCH_LDRPROTO
#define __CANWINCH_LDRPROTO

#include "common_misc.h"
#include "common_can.h"

/******************************************************************************/
void sendcanCMD_PAY1(uint8_t cmd,uint8_t pay1);
/* @brief	: send a CAN msg with a command byte and status type byte
 * @param	: cmd = command code 
 * @param   : pay1 = .cd.uc[1] byte associated with the command code
 * **************************************************************************************/
void canwinch_ldrproto_init(uint32_t iamunitnumber);
/* @brief	: Initialization for loader
 * @param	: Unit number 
 * ************************************************************************************** */
void canwinch_ldrproto_poll(void);
/* @param	: pctl = pointer control block for CAN module being used
 * @brief	: If msg is for this unit, then do something with it.
 * ************************************************************************************** */



#endif 

