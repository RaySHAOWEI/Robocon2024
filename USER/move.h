//
// Created by Ray on 2023/12/21.
//

#ifndef INC_2024RC_B_R1_MOVE_H
#define INC_2024RC_B_R1_MOVE_H

#include "main.h"

extern float X[9];
extern float Y[9];
extern float Yaw[9];

extern float X1[9];
extern float Y1[9];
extern float Yaw1[9];

typedef struct
{
    float X;
    float Y;
    float Yaw;
    float V_x;
    float V_y;
    float W;
}PATH_TYPEDEF;

typedef enum//瞎写的之后讨论一下怎么个事。。。
{
    //停止
    MOVE_STOP,

    //取苗点
    MOVE_2_GET_SEED_POINT_1,
    MOVE_2_GET_SEED_POINT_2,
    MOVE_2_GET_SEED_POINT_3,

    // 种苗点
    MOVE_2_SEED_POINT_1,
    MOVE_2_SEED_POINT_2,
    MOVE_2_SEED_POINT_3,
    MOVE_2_SEED_POINT_4,
    MOVE_2_SEED_POINT_5,
    MOVE_2_SEED_POINT_6,

    // 取球点
    MOVE_2_GET_BALL_POINT_1,
    MOVE_2_GET_BALL_POINT_2,
    MOVE_2_GET_BALL_POINT_3,


    //射球点
    MOVE_2_SHOOT_POINT_1,

    // 启动区
    MOVE_2_RESTART

}MOVE_STATE_ITEMS;

extern MOVE_STATE_ITEMS MOVE_STATE;

extern PID_T yaw_pid;
extern PID_T point_X_pid;
extern PID_T point_Y_pid;

void MoveInit(void);

void YawAdjust(float Target_angle);

void LockupPoint(float POS_X, float POS_Y, float POS_YAW);

void PDController(PATH_TYPEDEF target_point, ROBOT_REAL_POS robot_now_pos);

int PathPlan(float t_real, float t_target, int num, float *X , float *Y, float *Yaw);

#endif //INC_2024RC_B_R1_MOVE_H
