//
// Created by Ray on 2023/11/24.
//

#ifndef INC_2024RC_B_R1_CHASSIS_H
#define INC_2024RC_B_R1_CHASSIS_H

#include "main.h"

#define COS60 					0.500000f
#define COS30 					0.866025f
#define COS45 					0.70710678f
#define SIN45              		0.70710678f
#define PI              		3.14159265358979f
#define WHEEL_R            		0.076f                	//轮子半径(单位：m)
#define CHASSIS_R 				0.40f                   //底盘半径(单位：m)
#define RM_transition_MS     	(PI * WHEEL_R)/570.0f	//转速与速度的转换 转动一圈的路程 / 每秒转动的圈数 算法：2*pi*R / 19（3508减速箱）* 60（分转化秒） rpm 2 m (单位：m/s)  rpm： 转/分钟
#define MS_transition_RM     	570.0f/(PI * WHEEL_R)   //速度与转速的转换 m 2 rpm (单位：m/s)

#define Mecanum_Rx 				0.5
#define Mecanum_Ry 				0.5



typedef struct ROBOT_CHASSIS_T
{
	float Vx;//最终底盘速度
    float Vy;
    float Vw;

    float world_x;//接收action传回来的世界坐标（机器人的真实位置）
    float world_y;
    float world_w;

	float plan_x;//接收路径规划传回的速度期望值
    float plan_y;
    float plan_w;

	float Vy_MAX;//最大速度限制
	float Vx_MAX;
	float Vw_MAX;

	float Motor_Target_RPM[4];           //4个轮子的目标转速

	int8_t Path_planning;      			//路径规划标志位
	int8_t World_Move_Flag;			 	//世界坐标系启用标志位（暂时未用）
}ROBOT_CHASSIS;

extern ROBOT_CHASSIS ROBOT_CHASSI;

void chassis_init(void);
void Robot_Wheels_RPM_calculate(void);
void chassis_stop(void);
void Free_Control(void);
void free_ctrl_change(void);

#endif //INC_2024RC_B_R1_CHASSIS_H
