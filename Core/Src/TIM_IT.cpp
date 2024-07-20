/*
 * TIM_IT.cpp
 *
 *  Created on: Jul 17, 2024
 *      Author: tu
 */

#include "TIM_IT.h"

bool reset = false;
unsigned int which_point = 0;
int arrived = 1;
int done = 0;
int delay_cnt = 0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM8)
	{
		// Software reset
		if (reset)
			NVIC_SystemReset();
		// Determine goal
		if (arrived && which_point < path.size())
		{
			pointToDist(path[which_point].x, path[which_point].y);
			initParam();
			which_point++;
			arrived = 0;
		}
		// calculate cmd_vel
		arrived = moveTo();
		// cmd_vel to motor_vel
		inverse_kinematics_model();
		kinematics_model();
		Encoder();
		PID_PWM();
	}
}
