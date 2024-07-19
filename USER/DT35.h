//
// Created by 19115 on 2024/4/1.
//

#ifndef INC_2024RC_A_R1_DT35_H
#define INC_2024RC_A_R1_DT35_H

#include "main.h"

extern PID_T laser_X_pid;
extern PID_T laser_Y_pid;
extern PID_T laser_K_pid;
extern PID_T laser_B_pid;
#define START 0X11

typedef struct
{
    int x;
    int y;
    int k;
    int b;
} DT35_TPYE;

// 激光结构体
typedef struct averageFilte_TPYE
{
    float data[5];
    float indata;
    float outdata;
} averageFilter_TPYE;

typedef struct
{
    float LastP; // 上次估算协方差 初始化值为0.05()
    float Now_P; // 当前估算协方差 初始化值为0(估计误差)
    float out;   // 卡尔曼滤波器输出 初始化值为0(输出值)
    float Kg;    // 卡尔曼增益 初始化值为0(增益)
    float Q;     // 过程噪声协方差 初始化值为0.001()
    float R;     // 观测噪声协方差 初始化值为1(测量误差)
} KFP;           // Kalman Filter parameter

typedef struct
{
    int t;
    int data[100];
} average;

extern DT35_TPYE origin_DT35, DT35, DT35_FILTE;
extern KFP KFP_x, KFP_y, KFP_k, KFP_b;
extern average average_x, average_y, average_k, average_b;

void Update_DT35(int theta_angle, int theta_pitch, int theta_yaw, int theta_ras);
float kalmanFilter(KFP *kfp, float input);

extern int tt;
extern float Laser_error_x, Laser_error_k, ERROR_SHOOTING;
int Laser_calibration_ball(float x, float y, float yaw, float v_max);
int Laser_calibration_ball_right(float x, float y, float yaw, float v_max);
int Laser_calibration_seed(float x, float y, float yaw, float v_max);
int Laser_calibration_seed_right(float x, float y, float yaw, float v_max);
int Laser_calibration(float x, float y, float yaw, float v_max);
int Laser_calibration1(float x, float y, float yaw, float v_max);
int Laser_calibration7(float x, float y, float yaw, float v_max);
int Laser2_calibration(float location_x1, float location_x2);
int Laser3_calibration(float k, float v_max);
int Laser1_calibration(float k, float v_max);
int Laser_calibration_right(float x, float y, float yaw, float v_max);
int Laser_calibration_init(float x, float y, float yaw, float v_max);
int Laser_calibration_ball_left(float x, float y, float yaw, float v_max);
int Laser_calibration_right1(float x, float y, float yaw, float v_max);
void VisionReceiveData_1(int *theta_angle, int *theta_pitch, int *theta_yaw, int *theta_ras);
int Laser_calibration_laser_left(float x, float y, float yaw, float v_max);

#endif // INC_2024RC_A_R1_DT35_H
