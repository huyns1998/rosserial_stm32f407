/*
 * main.cpp
 */
#include <mainpp.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <stdio.h>
#include "std_msgs/UInt8.h"
#include "std_srvs/Empty.h"
#include "std_msgs/Int32.h"

ros::NodeHandle nh;

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);
char hello[] = "Hello!";

void led0_cb(const std_msgs::UInt8& msg);
ros::Subscriber<std_msgs::UInt8> led0_sub("led0", &led0_cb);

void publish_optical_sensor();
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
  nh.getHardware()->flush();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
  nh.getHardware()->reset_rbuf();
}

void setup(void)
{
  nh.initNode();
  nh.advertise(chatter);
  nh.subscribe(led0_sub);
}

uint32_t count;
static unsigned long pre_time = 0;

void loop(void)
{
	if (millis() - pre_time >= (1000 / 1)) {
		pre_time = millis();
		count++;
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_15);
		char s_temp[30];
		sprintf(s_temp, "%s,  count = %d (s)", hello, count);
		str_msg.data = s_temp;
		chatter.publish(&str_msg);
	}

  nh.spinOnce();

  //HAL_Delay(10);
}

void led0_cb(const std_msgs::UInt8& msg){
	nh.loginfo("inside led0_cb !!\n\r");
}

