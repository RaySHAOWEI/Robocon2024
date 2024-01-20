//
// Created by Ray on 2023/11/26.
//

#include "config.h"

/**
 * @brief 底盘电机类型初始化
 * 
 */
void can1_config(void)
{
    can1motorRealInfo[0].Motor_Type = M_3508;//底盘右后轮
    can1motorRealInfo[1].Motor_Type = M_3508;//底盘右前轮
    can1motorRealInfo[2].Motor_Type = M_3508;//底盘左前轮
    can1motorRealInfo[3].Motor_Type = M_3508;//底盘左后轮

    //速度环
    pid_param_init(&can1MOTOR_PID_RPM[0], PID_Position, 8192, 1024, 0, 0.5f, 16384, 12.0f, 0.1f, 0.0f);
    pid_param_init(&can1MOTOR_PID_RPM[1], PID_Position, 8192, 1024, 0, 0.5f, 16384, 12.0f, 0.1f, 0.0f);
    pid_param_init(&can1MOTOR_PID_RPM[2], PID_Position, 8192, 1024, 0, 0.5f, 16384, 12.0f, 0.1f, 0.0f);
    pid_param_init(&can1MOTOR_PID_RPM[3], PID_Position, 8192, 1024, 0, 0.5f, 16384, 12.0f, 0.1f, 0.0f);
}

/**
 * @brief can2初始化
 * 
 */
void can2_config(void)
{
    can2motorRealInfo[0].Motor_Type = M_3508;//秧苗抬升电机
    can2motorRealInfo[1].Motor_Type = M_3508;//秧苗翻转电机
    can2motorRealInfo[2].Motor_Type = M_3508;//发射电机左
    can2motorRealInfo[3].Motor_Type = M_3508;//发射电机右
    can2motorRealInfo[4].Motor_Type = M_3508;//发射电机上

    //速度环
    pid_param_init(&can2MOTOR_PID_RPM[0], PID_Incremental, 8192, 900, 0, 0.1f, 16384, 15.0f, 0.4f, 0.2f);
    pid_param_init(&can2MOTOR_PID_RPM[1], PID_Incremental, 8192, 900, 0, 0.1f, 16384, 20.0f, 0.5f, 0.2f);
    pid_param_init(&can2MOTOR_PID_RPM[2], PID_Incremental, 16384, 16384, 0, 0.1f, 16384, 13.0f, 0.4f, 0.2f);
    pid_param_init(&can2MOTOR_PID_RPM[3], PID_Incremental, 16384, 16384, 0, 0.1f, 16384, 13.0f, 0.4f, 0.2f);
    pid_param_init(&can2MOTOR_PID_RPM[4], PID_Incremental, 16384, 16384, 0, 0.1f, 16384, 13.0f, 0.4f, 0.2f);

    //位置环
    pid_param_init(&can2MOTOR_PID_POS[0], PID_Position, 1024, 800, 0, 0.1f, 16384, 12.0f, 0.0f, 0.3f);
    pid_param_init(&can2MOTOR_PID_POS[1], PID_Position, 2048, 800, 0, 0.1f, 16384, 15.0f, 0.0f, 0.3f);
}


