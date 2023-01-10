/*
 * dc-Driver.h
 *
 *  Created on: Nov 13, 2022
 *      Author: Quang
 */

#ifndef DC_DRIVER_H_
#define DC_DRIVER_H_

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"
#include "typedefs.h"


extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

#define TIM_PERIOD						8400
#define ENCODER_NUM						11
#define DC_RATIO						30
//DC1(left)
#define DC1_C2_PORT						GPIOA
#define DC1_C2_PIN						GPIO_PIN_6
#define DC1_TIM_PWM						htim1
#define DC1_TIM_PWM_CHANNEL				TIM_CHANNEL_4

#define DC1_TIM_PULSE_COUNT				htim2
#define DC1_TIM_PULSE_COUNT_CHANNEL		TIM_CHANNEL_2
#define DC1_TIM_PULSE_COUNT_CHANNEL_ACTIVE		HAL_TIM_ACTIVE_CHANNEL_2

#define DC1_TIM_UPDATE					htim3

#define INT1_PORT						GPIOA
#define INT1_PIN 						GPIO_PIN_0
#define INT2_PORT						GPIOA
#define INT2_PIN						GPIO_PIN_1

//DC2(right)
#define DC2_C2_PORT						GPIOA
#define DC2_C2_PIN						GPIO_PIN_7
#define DC2_TIM_PWM						htim1
#define DC2_TIM_PWM_CHANNEL				TIM_CHANNEL_1

#define DC2_TIM_PULSE_COUNT				htim2
#define DC2_TIM_PULSE_COUNT_CHANNEL		TIM_CHANNEL_3
#define DC2_TIM_PULSE_COUNT_CHANNEL_ACTIVE		HAL_TIM_ACTIVE_CHANNEL_3

#define DC2_TIM_UPDATE					DC1_TIM_UPDATE

#define INT3_PORT						GPIOA
#define INT3_PIN 						GPIO_PIN_4
#define INT4_PORT						GPIOA
#define INT4_PIN						GPIO_PIN_5

#define TIM_CHECK_ROBOT_STOP_UPDATE		htim4

#define WHEEL_DIAMETER	64//mm
#define WHEEL_BASE					279//mm
#define SAMPLING_TIME				5
#define INV_SAMPLING_TIME			200




void DcControlSetup(void);
void MX_I2C1_Init(void);
void dcControl(i32_t wPulse, TIM_HandleTypeDef htim, u8_t tim_Channel);

#endif /* DC_DRIVER_H_ */
