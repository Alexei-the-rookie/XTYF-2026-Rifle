/**
  ******************************************************************************
  * @file           : Control_Task.c
  * @brief          : Control task
  * @author         : Yan Yuanbin
  * @date           : 2023/04/27
  * @version        : v1.0
  ******************************************************************************
  * @attention      : None
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CONTROL_TASK_H
#define CONTROL_TASK_H

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "stdbool.h"


/**
 * @brief typedef structure that contains the information of chassis control
*/

typedef struct 
{
	
	struct{
		
		float Chassis_Velocity;
		
	
	}Target;	
 
 struct{
		
	 float	Chassis_Velocity;
		
	
	}Measure;
	
	int16_t SendValue[4];
	
}Control_Info_Typedef;

typedef struct
{
	float theta_ll;
	float dtheta_ll;
	float theta_lr;
	float dtheta_lr;
	float theta_wl;
	float dtheta_wl;
	float theta_wr;
	float dtheta_wr;
	float theta_b;
	float dtheta_b;
}vec_state;

typedef struct
{
	float T_bll;
	float T_blr;
	float T_lwl;
	float T_lwr;
}vec_input;

/* Externs---------------------------------------------------------*/
extern vec_state state, last_state, d_state;
extern vec_input input;
extern Control_Info_Typedef Control_Info;
#endif //CONTROL_TASK_H