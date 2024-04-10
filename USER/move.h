//
// Created by Ray on 2023/12/21.
//

#ifndef INC_2024RC_B_R1_MOVE_H
#define INC_2024RC_B_R1_MOVE_H

#include "main.h"

typedef struct
{
	float Distance;
	float Pstart;        //开始位置
	float Pend;          //结束位置
	float Vstart;        //开始的速度           
	float Vmax;          //最大的速度
	float Vend;          //末尾的速度
	float Rac;           //加速路程的比例
	float Rde;           //减速路程的比例
	int flag;            //完成标志位，停下来的时候置1
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
