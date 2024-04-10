//
// Created by Ray on 2023/12/21.
//

#ifndef INC_2024RC_B_R1_MOVE_H
#define INC_2024RC_B_R1_MOVE_H

#include "main.h"

typedef struct
{
	float Distance;
	float Pstart;        //��ʼλ��
	float Pend;          //����λ��
	float Vstart;        //��ʼ���ٶ�           
	float Vmax;          //�����ٶ�
	float Vend;          //ĩβ���ٶ�
	float Rac;           //����·�̵ı���
	float Rde;           //����·�̵ı���
	int flag;            //��ɱ�־λ��ͣ������ʱ����1
}TrapezoidPlaning_TYPEDEF;


extern PID_T point_traker_x_pid;
extern PID_T point_traker_y_pid;
extern PID_T point_traker_yaw_pid;
extern PID_T TRACK_PID;
extern PID_T track_pid;

void AngleLimit(float *angle);
int YawAdjust(float Taget_angle);

uint32_t getSysTickCnt(void);
void movebase(void);
void move_seed(void);
void put_seed(void);
void auto_detection(void);

int chassis_TrapezoidPlaning(float POS_X_start,
	                        float POS_Y_start,
								        float POS_X_end,
										float POS_Y_end,
										float POS_Yaw,
										float V_start,
										float V_end,
										float V_max,
										float R_ac,
										float R_de);


#endif //INC_2024RC_B_R1_MOVE_H
