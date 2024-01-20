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
#define WHEEL_R            		0.076f                	//���Ӱ뾶(��λ��m)
#define CHASSIS_R 				0.40f                   //���̰뾶(��λ��m)
#define RM_transition_MS     	(PI * WHEEL_R)/570.0f	//ת�����ٶȵ�ת�� ת��һȦ��·�� / ÿ��ת����Ȧ�� �㷨��2*pi*R / 19��3508�����䣩* 60����ת���룩 rpm 2 m (��λ��m/s)  rpm�� ת/����
#define MS_transition_RM     	570.0f/(PI * WHEEL_R)   //�ٶ���ת�ٵ�ת�� m 2 rpm (��λ��m/s)

#define Mecanum_Rx 				0.5
#define Mecanum_Ry 				0.5



typedef struct ROBOT_CHASSIS_T
{
	float Vx;//���յ����ٶ�
    float Vy;
    float Vw;

    float world_x;//����action���������������꣨�����˵���ʵλ�ã�
    float world_y;
    float world_w;

	float plan_x;//����·���滮���ص��ٶ�����ֵ
    float plan_y;
    float plan_w;

	float Vy_MAX;//����ٶ�����
	float Vx_MAX;
	float Vw_MAX;

	float Motor_Target_RPM[4];           //4�����ӵ�Ŀ��ת��

	int8_t Path_planning;      			//·���滮��־λ
	int8_t World_Move_Flag;			 	//��������ϵ���ñ�־λ����ʱδ�ã�
}ROBOT_CHASSIS;

extern ROBOT_CHASSIS ROBOT_CHASSI;

void chassis_init(void);
void Robot_Wheels_RPM_calculate(void);
void chassis_stop(void);
void Free_Control(void);
void free_ctrl_change(void);

#endif //INC_2024RC_B_R1_CHASSIS_H
