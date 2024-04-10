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
    pid_param_init(&can1MOTOR_PID_RPM[0], PID_Position, 8192, 1024, 0, 0.5f, 16384, 12.0f, 0.0f, 0.2f);
    pid_param_init(&can1MOTOR_PID_RPM[1], PID_Position, 8192, 1024, 0, 0.5f, 16384, 12.0f, 0.0f, 0.2f);
    pid_param_init(&can1MOTOR_PID_RPM[2], PID_Position, 8192, 1024, 0, 0.5f, 16384, 12.0f, 0.0f, 0.2f);
    pid_param_init(&can1MOTOR_PID_RPM[3], PID_Position, 8192, 1024, 0, 0.5f, 16384, 12.0f, 0.0f, 0.2f);
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
    pid_param_init(&can2MOTOR_PID_RPM[0], PID_Position, 8192, 900, 0, 0.1f, 16384, 20.0f, 0.0f, 0.0f);
    pid_param_init(&can2MOTOR_PID_RPM[1], PID_Incremental, 8192, 900, 0, 0.1f, 16384, 20.0f, 0.5f, 0.2f);
	
    pid_param_init(&can2MOTOR_PID_RPM[2], PID_Position, 16384, 16384, 0, 0.1f, 16384, 14.0f, 0.05f, 0.0f);
    pid_param_init(&can2MOTOR_PID_RPM[3], PID_Position, 16384, 16384, 0, 0.1f, 16384, 14.0f, 0.05f, 0.0f);
    pid_param_init(&can2MOTOR_PID_RPM[4], PID_Position, 16384, 16384, 0, 0.1f, 16384, 14.0f, 0.05f, 0.0f);

    //位置环
    pid_param_init(&can2MOTOR_PID_POS[0], PID_Position, 1024, 800, 0, 0.1f, 16384, 18.0f, 0.0f, 0.01f);
    pid_param_init(&can2MOTOR_PID_POS[1], PID_Position, 2048, 800, 0, 0.1f, 16384, 15.0f, 0.0f, 0.3f);
}


void Move_Init(void)
{
	// //PD跟踪器
    pid_param_init(&point_traker_x_pid, PID_Position, 2000, 0, 0, 10, 0, 3, 0, 0.5);
    pid_param_init(&point_traker_y_pid, PID_Position, 2000, 0, 0, 10, 0, 3, 0, 0.5);
    pid_param_init(&point_traker_yaw_pid, PID_Position, 1000, 0, 0, 1, 0, 30, 0, 0.1);
    pid_param_init(&TRACK_PID, PID_Position, 1000, 0, 0, 1, 0, 30, 0, 0.1);
    pid_param_init(&track_pid, PID_Position, 1000, 0, 0, 1, 0, 30, 0, 0.1);
	
	// 激光PID
	pid_param_init(&laser_X_pid, PID_Position, 500, 0, 0, 10, 0, 1, 0, 0.5);
	pid_param_init(&laser_Y_pid, PID_Position, 100, 0, 0, 10, 0, 1, 0, 0.5);
	pid_param_init(&laser_K_pid, PID_Position, 300, 0, 0, 10, 0, 3, 0, 0.5);
	pid_param_init(&laser_B_pid, PID_Position, 300, 0, 0, 10, 0, 2, 0, 0.5);
	
//	// 自动路径PID
//	PID_parameter_init(&point_pid ,  1.2f,0, 0.5f, 500, 0, 5);
//	PID_parameter_init(&yaw_pid ,  50.0f,0.1f, 1.0f, 1000.0f, 0.0f, 0.1f);
//	PID_parameter_init(&catch_ring_pid ,  3.0f,0.0f, 0.3f, 100.0f, 0.0f, 1.0f);
//	PID_parameter_init(&catch_ring_pid_b ,  3.1f,0.0f, 0.3f, 100.0f, 0.0f, 1.0f);
}

