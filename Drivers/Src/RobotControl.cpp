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
 * File name: RobotControl.c
 *
 * Description: This code is used for tranning Lumi IOT member. It is the code form statandard.
 *
 * Author: HuyNS
 *
 * Last Changed By:  $Author: huyns $
 * Revision:         $Revision: $
 * Last Changed:     $Date: $April 14, 2022
 *
 * Code sample:
 ******************************************************************************/
/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/
#include <math.h>
#include "dc-Driver.h"
#include "DcControl.h"
#include "RobotControl.h"
/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/

/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/


/******************************************************************************/
/*                              EXPORTED DATA                                 */
/******************************************************************************/
extern dcState g_DirectionDc1;
extern dcState g_DirectionDc2;



extern u8_t g_run_distance;
extern u32_t g_dc1_pulse;
extern u32_t g_dc2_pulse;
extern u32_t g_dc1_pulse_count;
extern u32_t g_dc2_pulse_count;

extern u8_t g_rotate;
float g_current_angle;
float g_des_angle;
extern RobotState g_robotState;
/******************************************************************************/
/*                            PRIVATE FUNCTIONS                               */
/******************************************************************************/

/******************************************************************************/
/*                            EXPORTED FUNCTIONS                              */
/******************************************************************************/

/******************************************************************************/

/**
  * @brief 	run forward
  *
  * @param 	[byDirection]:	direction @ref direction
  * @param 	[wSpeed]:	speed(cm/s)
  *
  * @return		-	None
  */
void_t robot_forward(u16_t wSpeed)
{
	float temp = (float)wSpeed;

	temp = temp*10/(M_PI*WHEEL_DIAMETER)*60;

	u16_t wRpm = (u16_t)temp;

	dc1_RotateClockWise(wRpm);
	dc2_RotateAntiClockWise(wRpm);

	g_robotState = getRobotState();
}

/**
  * @brief 	run backward
  *
  * @param 	[byDirection]:	direction @ref direction
  * @param 	[wSpeed]:	speed(cm/s)
  *
  * @return		-	None
  */
void_t robot_backward(u16_t wSpeed)
{
	float temp = (float)wSpeed;

	temp = temp*10/(M_PI*WHEEL_DIAMETER)*60;

	u16_t wRpm = (u16_t)temp;

	dc1_RotateAntiClockWise(wRpm);
	dc2_RotateClockWise(wRpm);

	g_robotState = getRobotState();
}

/**
  * @brief 	run backward
  *
  * @param 	[byDirection]:	direction @ref direction
  * @param 	[wSpeed]:	speed(cm/s)
  *
  * @return		-	None
  */
void robot_stop(void)
{
	dc1_Stop();
	dc2_Stop();

	g_robotState = getRobotState();
}

/**
  * @brief 	rotate left
  *
  * @param 		-	None
  *
  * @return		-	None
  */
void_t robot_rotateleft(void_t)
{
	dc1_RotateAntiClockWise(70);
	dc2_RotateAntiClockWise(70);

	g_robotState = getRobotState();
}

/**
  * @brief 	rotate left
  *
  * @param 		-	None
  *
  * @return		-	None
  */
void_t robot_rotateright(void_t)
{
	dc1_RotateClockWise(70);
	dc2_RotateClockWise(70);

	g_robotState = getRobotState();
}

/**
  * @brief 	get robot state
  *
  * @param 		-	None
  *
  * @return		-	RobotState
  */
RobotState getRobotState(void_t)
{
	if(g_DirectionDc1 == DC_STOP && g_DirectionDc2 == DC_STOP)
	{
		return ROBOT_STOP;
	}
	else {
		if(g_DirectionDc1 == DC_CLOCKWISE && g_DirectionDc2 == DC_ANTICLOCKWISE)
		{
			return ROBOT_FORWARD;
		}
		else
		{
			if (g_DirectionDc1 == DC_ANTICLOCKWISE && g_DirectionDc2 == DC_CLOCKWISE)
			{
				return ROBOT_BACKWARD;
			}
			else
			{
				if(g_DirectionDc1 == DC_ANTICLOCKWISE && g_DirectionDc2 == DC_ANTICLOCKWISE)
				{
					return ROBOT_ROTATE_LEFT;
				}
				else
				{
					if(g_DirectionDc1 == DC_CLOCKWISE && g_DirectionDc2 == DC_CLOCKWISE)
					{
						return ROBOT_ROTATE_RIGHT;
					}
				}
			}
		}
	}
}

/**
  * @brief 	run forward in a distance
  *
  * @param 	[wDistance]:	distance(cm)
  * @param 	[wSpeed]:		speed(cm/s)
  *
  * @return		-	None
  */
void_t robot_forward_distance(u16_t wSpeed, u32_t wDistance)
{
	resetPulse_distance();

	g_run_distance = 1;
	g_dc1_pulse = convertDistanceToPulse(wDistance);
	g_dc2_pulse = g_dc1_pulse;
	robot_forward(wSpeed);

//	HAL_TIM_Base_Start_IT(&TIM_CHECK_ROBOT_STOP_UPDATE);
}

/**
  * @brief 	run forward in a distance
  *
  * @param 	[wDistance]:	distance(cm)
  * @param 	[wSpeed]:		speed(cm/s)
  *
  * @return		-	None
  */
void_t robot_backward_distance(u16_t wSpeed, u32_t wDistance)
{
	resetPulse_distance();

	g_run_distance = 1;
	g_dc1_pulse = convertDistanceToPulse(wDistance);
	g_dc2_pulse = g_dc1_pulse;
	robot_backward(wSpeed);

//	HAL_TIM_Base_Start_IT(&TIM_CHECK_ROBOT_STOP_UPDATE);
}





/**
  * @brief 	convert distance to pulse
  *
  * @param 	[wDistance]:	distance(cm)
  *
  * @return		-	None
  */
u32_t convertDistanceToPulse(u32_t wDistance)
{
	float temp = (float)wDistance;
	temp = wDistance*10*11*30/(M_PI*WHEEL_DIAMETER);
	return (u32_t)temp;
}

/**
  * @brief 	check if robot run in a distance
  *
  * @param 	-	None
  *
  * @return		-
  */
u8_t isRun_distance(void_t)
{
	return g_run_distance;
}

/**
  * @brief 	gwet
  *
  * @param 	-	None
  *
  * @return		-	pulsecount
  */
u32_t getDc1PulseCount(void_t)
{
	return g_dc1_pulse_count;
}

/**
  * @brief 	check if robot run in a distance
  *
  * @param 	-	None
  *
  * @return		-	pulsecount
  */
u32_t getDc2PulseCount(void_t)
{
	return g_dc2_pulse_count;
}

/**
  * @brief 	reset robot parameter
  *
  * @param 	-	None
  *
  * @return		-	None
  */
void_t resetPulse_distance(void_t)
{
//	HAL_TIM_Base_Stop(&TIM_CHECK_ROBOT_STOP_UPDATE);
	g_dc1_pulse_count = 0;
	g_dc1_pulse = 0;
	g_dc2_pulse_count = 0;
	g_dc2_pulse = 0;
	g_run_distance = 0;
	g_rotate = 0;
}

/**
  * @brief 	get dc1 des pulse
  *
  * @param 	-	None
  *
  * @return		-	None
  */
u32_t getDc1Despulse(void_t)
{
	return g_dc1_pulse;
}

/**
  * @brief 	get dc2 des pulse
  *
  * @param 	-	None
  *
  * @return		-	None
  */
u32_t getDc2Despulse(void_t)
{
	return g_dc2_pulse;
}

/**
  * @brief 	get dc2 des pulse
  *
  * @param 	-	None
  *
  * @return		-	None
  */
u8_t isRobotRotate(void_t)
{
	return g_rotate;
}

/**
  * @brief 	Conver angle to pulse
  *
  * @param 	-	Angle
  *
  * @return		-	pulse
  */
u32_t convertAngleToPulse(float angle)
{
	float temp = M_PI*WHEEL_BASE*0.1*angle/360;
	u32_t wResult = convertDistanceToPulse((u32_t)temp);

	return wResult;
}

/**
  * @brief 	left rotate in an angle
  *
  * @param [angle]	:	angle want to obtain
  *
  * @return		-	None
  */
void_t robot_rotateleft_angle(float angle)
{
	resetPulse_distance();

//	g_des_angle = g_current_angle - angle;
//
//	if(g_des_angle < -180.0)
//	{
//		g_des_angle = 360.0 + g_des_angle;
//	}

	g_dc1_pulse = convertAngleToPulse(angle);
	g_dc2_pulse = g_dc1_pulse;

	g_rotate = 1;
	robot_rotateleft();
//	HAL_TIM_Base_Start_IT(&TIM_CHECK_ROBOT_STOP_UPDATE);
}

/**
  * @brief 	left right in an angle
  *
  * @param [angle]	:	angle want to obtain
  *
  * @return		-	None
  */
void_t robot_rotateright_angle(float angle)
{
	resetPulse_distance();

//	g_des_angle = g_current_angle + angle;
//
//	if(g_des_angle > 180.0)
//	{
//		g_des_angle = g_des_angle - 360.0;
//	}

	g_dc1_pulse = convertAngleToPulse(angle);
	g_dc2_pulse = g_dc1_pulse;

	g_rotate = 1;
	robot_rotateleft();

	g_rotate = 1;
	robot_rotateright();
//	HAL_TIM_Base_Start_IT(&TIM_CHECK_ROBOT_STOP_UPDATE);
}

/**
  * @brief 	get current angle
  *
  * @param -	None
  *
  * @return		-	current angle
  */
float robot_get_current_angle(void_t)
{
	return g_current_angle;
}

/**
  * @brief 	get destination angle
  *
  * @param -	None
  *
  * @return		-	destination angle
  */
float robot_get_des_angle(void_t)
{
	return g_des_angle;
}

void HardwareSetup_Robot(void)
{
	HardwareSetup();
}





