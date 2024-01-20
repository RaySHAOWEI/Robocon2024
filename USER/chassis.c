//
// Created by Ray on 2023/11/24.
//

#include "chassis.h"

#define FOUR_OMNI_CHASSIS  1
#define THREE_OMNI_CHASSIS 0
#define MECANUM_CHASSIS 0

ROBOT_CHASSIS ROBOT_CHASSI;

void chassis_init(void)
{
	//速度模式初始化
    ROBOT_CHASSI.Vx_MAX = 1.6f;
    ROBOT_CHASSI.Vy_MAX = 1.6f;
	ROBOT_CHASSI.Vw_MAX = 2.0f;

	ROBOT_CHASSI.Vx = 0;
	ROBOT_CHASSI.Vy = 0;
	ROBOT_CHASSI.Vw = 0;

	ROBOT_CHASSI.world_x = 0;
	ROBOT_CHASSI.world_y = 0;
	ROBOT_CHASSI.world_w = 0;

	ROBOT_CHASSI.plan_x = 0;
	ROBOT_CHASSI.plan_y = 0;
	ROBOT_CHASSI.plan_w = 0;

	ROBOT_CHASSI.World_Move_Flag = 0;
	ROBOT_CHASSI.Path_planning = 0;
	
	ROBOT_CHASSI.Motor_Target_RPM[0] = 0;
	ROBOT_CHASSI.Motor_Target_RPM[1] = 0;
	ROBOT_CHASSI.Motor_Target_RPM[2] = 0;
	ROBOT_CHASSI.Motor_Target_RPM[3] = 0;
}

/**
 * @brief 地盘解算，四全向轮
 * @param NULL
 * @return NULL
*/
void Robot_Wheels_RPM_calculate(void)
{
	// float COS,SIN;
	// COS = cos (ROBOT_CHASSI.world_w * PI /180);
	// SIN = sin (ROBOT_CHASSI.world_w * PI /180);
	//世界坐标系转换
	//转化为转速
    // if(ROBOT_CHASSI.World_Move_Flag == 1)
    // {
	// 	ROBOT_CHASSI.Vx  = (ROBOT_CHASSI.world_x * COS - ROBOT_CHASSI.world_y * SIN);
	// 	ROBOT_CHASSI.Vy  = (ROBOT_CHASSI.world_x * SIN + ROBOT_CHASSI.world_y * COS);
    // }
	if (ROBOT_CHASSI.Path_planning == 1)
	{
		ROBOT_CHASSI.Vx += ROBOT_CHASSI.plan_x;
		ROBOT_CHASSI.Vy += ROBOT_CHASSI.plan_y;
		ROBOT_CHASSI.Vw += ROBOT_CHASSI.plan_w;
	}

#if FOUR_OMNI_CHASSIS 

	ROBOT_CHASSI.Motor_Target_RPM[0] = (-ROBOT_CHASSI.Vy*COS45 + ROBOT_CHASSI.Vx*COS45 + ROBOT_CHASSI.Vw*CHASSIS_R) * MS_transition_RM;
	ROBOT_CHASSI.Motor_Target_RPM[1] = (-ROBOT_CHASSI.Vy*COS45 - ROBOT_CHASSI.Vx*COS45 + ROBOT_CHASSI.Vw*CHASSIS_R) * MS_transition_RM;
	ROBOT_CHASSI.Motor_Target_RPM[2] = ( ROBOT_CHASSI.Vy*COS45 - ROBOT_CHASSI.Vx*COS45 + ROBOT_CHASSI.Vw*CHASSIS_R) * MS_transition_RM;
	ROBOT_CHASSI.Motor_Target_RPM[3] = ( ROBOT_CHASSI.Vy*COS45 + ROBOT_CHASSI.Vx*COS45 + ROBOT_CHASSI.Vw*CHASSIS_R) * MS_transition_RM;

	for (int i = 0; i < 4; i++)
	{
		Speed_Control(&can1motorRealInfo[i],ROBOT_CHASSI.Motor_Target_RPM[i]);
	}
#endif

#if THREE_OMNI_CHASSIS 

	ROBOT_CHASSI.Motor_Target_RPM[0] = ( ROBOT_CHASSI.Vy*COS30 - ROBOT_CHASSI.Vx*SIN30 + ROBOT_CHASSI.Vw*MS_transition_RM) * MS_transition_RM;
	ROBOT_CHASSI.Motor_Target_RPM[1] = (-ROBOT_CHASSI.Vy*SIN30 + ROBOT_CHASSI.Vx*COS30 + ROBOT_CHASSI.Vw*MS_transition_RM) * MS_transition_RM; 
	ROBOT_CHASSI.Motor_Target_RPM[2] = ( ROBOT_CHASSI.Vx + ROBOT_CHASSI.Vw*MS_transition_RM) * MS_transition_RM;

	for (int i = 0; i < 3; i++)
	{
		Speed_Control(&MOTO_REAL_INFO[i],ROBOT_CHASSI.Motor_Target_RPM[i]);
	}

#endif

#if MECANUM_CHASSIS

	ROBOT_CHASSI.Motor_Target_RPM[0] = (-ROBOT_CHASSI.Vx + ROBOT_CHASSI.Vy + (Mecanum_Rx + Mecanum_Ry)*ROBOT_CHASSI.Vw) * MS_transition_RM;
	ROBOT_CHASSI.Motor_Target_RPM[1] = ( ROBOT_CHASSI.Vx + ROBOT_CHASSI.Vy - (Mecanum_Rx + Mecanum_Ry)*ROBOT_CHASSI.Vw) * MS_transition_RM;
	ROBOT_CHASSI.Motor_Target_RPM[2] = (-ROBOT_CHASSI.Vx + ROBOT_CHASSI.Vy - (Mecanum_Rx + Mecanum_Ry)*ROBOT_CHASSI.Vw) * MS_transition_RM;
	ROBOT_CHASSI.Motor_Target_RPM[3] = ( ROBOT_CHASSI.Vx + ROBOT_CHASSI.Vy + (Mecanum_Rx + Mecanum_Ry)*ROBOT_CHASSI.Vw) * MS_transition_RM;

	for (int i = 0; i < 4; i++)
	{
		Speed_Control(&MOTO_REAL_INFO[i],ROBOT_CHASSI.Motor_Target_RPM[i]);
	}
#endif
}

void chassis_stop(void)
{
	ROBOT_CHASSI.Vx = 0;
	ROBOT_CHASSI.Vy = 0;
	ROBOT_CHASSI.Vw = 0;
	ROBOT_CHASSI.Path_planning = 0;
	Robot_Wheels_RPM_calculate();
}

/**
 * @brief 底盘自由控制函数
*/
void Free_Control(void)
{
	if(ABS(YaoGan_LEFT_X-1500) > 20){
		ROBOT_CHASSI.Vx = ((YaoGan_LEFT_X-1500.0f)/500) * ROBOT_CHASSI.Vx_MAX;
	}else if(ABS(YaoGan_LEFT_X-1500) <= 10){
		ROBOT_CHASSI.Vx = 0;
	}
	if(ABS(YaoGan_LEFT_Y-1500) > 20){
		ROBOT_CHASSI.Vy = ((YaoGan_LEFT_Y-1500.0f)/500) * ROBOT_CHASSI.Vy_MAX;
	}else if(ABS(YaoGan_LEFT_Y-1500) <= 10){
		ROBOT_CHASSI.Vy = 0;
	}
	if(ABS(YaoGan_RIGHT_X-1500) > 20){
		ROBOT_CHASSI.Vw = ((YaoGan_RIGHT_X-1500.0f)/500) * ROBOT_CHASSI.Vw_MAX;
	}else if(ABS(YaoGan_RIGHT_X-1500) <= 10){
		ROBOT_CHASSI.Vw = 0;
	}
}

void free_ctrl_change(void)
{
	if(ABS(YaoGan_LEFT_X-1500) > 20){
		ROBOT_CHASSI.Vy = -((YaoGan_LEFT_X-1500.0f)/500) * ROBOT_CHASSI.Vy_MAX;
	}else if(ABS(YaoGan_LEFT_X-1500) <= 10){
		ROBOT_CHASSI.Vy = 0;
	}
	if(ABS(YaoGan_LEFT_Y-1500) > 20){
		ROBOT_CHASSI.Vx = ((YaoGan_LEFT_Y-1500.0f)/500) * ROBOT_CHASSI.Vx_MAX;
	}else if(ABS(YaoGan_LEFT_Y-1500) <= 10){
		ROBOT_CHASSI.Vx = 0;
	}
	if(ABS(YaoGan_RIGHT_X-1500) > 20){
		ROBOT_CHASSI.Vw = ((YaoGan_RIGHT_X-1500.0f)/500) * ROBOT_CHASSI.Vw_MAX;
	}else if(ABS(YaoGan_RIGHT_X-1500) <= 10){
		ROBOT_CHASSI.Vw = 0;
	}
}
