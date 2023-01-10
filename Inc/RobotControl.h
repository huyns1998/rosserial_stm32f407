/*******************************************************************************
 *				 _ _                                             _ _
				|   |                                           (_ _)
				|   |        _ _     _ _   _ _ _ _ _ _ _ _ _ _   _ _
				|   |       |   |   |   | |    _ _     _ _    | |   |
				|   |       |   |   |   | |   |   |   |   |   | |   |
				|   |       |   |   |   | |   |   |   |   |   | |   |
				|   |_ _ _  |   |_ _|   | |   |   |   |   |   | |   |
				|_ _ _ _ _| |_ _ _ _ _ _| |_ _|   |_ _|   |_ _| |_ _|
								(C)2022 Lumi
 * Copyright (c) 2022
 * Lumi, JSC.
 * All Rights Reserved
 *
 * File name: RobotControl.h
 *
 * Description:
 *
 * Author: HuyNS
 *
 * Last Changed By:  $Author: huyns $
 * Revision:         $Revision: $
 * Last Changed:     $Date: $April 15, 2022
 *
 * Code sample:
 ******************************************************************************/
// Enclosing macro to prevent multiple inclusion
#ifndef ROBOTCONTROL_H_
#define ROBOTCONTROL_H_


/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/
#include "typedefs.h"
/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/



//@ref direction
 enum
{
	ROBOT_STOP			=		1,
	ROBOT_FORWARD		=		2,
	ROBOT_BACKWARD		=		3,
	ROBOT_ROTATE_LEFT	=		4,
	ROBOT_ROTATE_RIGHT	=		5
} typedef RobotState;

/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/

/******************************************************************************/
/*                              EXPORTED DATA                                 */
/******************************************************************************/


/******************************************************************************/
/*                            PRIVATE FUNCTIONS                               */
/******************************************************************************/

/******************************************************************************/
/*                            EXPORTED FUNCTIONS                              */
/******************************************************************************/
void_t robot_forward(u16_t wSpeed);
void_t robot_backward(u16_t wSpeed);
void robot_stop(void);
void_t robot_rotateright(void_t);
void_t robot_rotateleft(void_t);
RobotState getRobotState(void_t);
u32_t convertDistanceToPulse(u32_t wDistance);
void_t robot_forward_distance(u16_t wSpeed, u32_t wDistance);
void_t robot_backward_distance(u16_t wSpeed, u32_t wDistance);
u32_t getDc1PulseCount(void_t);
u32_t getDc2PulseCount(void_t);
u8_t isRun_distance(void_t);
void_t resetPulse_distance(void_t);
void_t robot_forward_distance(u16_t wSpeed, u32_t wDistance);
u32_t getDc1Despulse(void_t);
u32_t getDc2Despulse(void_t);
void_t robot_rotateleft_angle(float angle);
void_t robot_rotateright_angle(float angle);
float robot_get_current_angle(void_t);
float robot_get_des_angle(void_t);
u8_t isRobotRotate(void_t);
void HardwareSetup_Robot(void);
/******************************************************************************/



#endif /* MID_ROBOTCONTROL_H_ */
