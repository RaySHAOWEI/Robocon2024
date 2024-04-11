//
// Created by 19115 on 2024/4/1.
//

#include "DT35.h"

DT35_TPYE origin_DT35 = {0}, DT35 = {0}, DT35_FILTE = {0};
KFP KFP_x = {0.05, 0, 0, 0, 0.001, 1}, KFP_y = {0.05, 0, 0, 0, 0.001, 1}, KFP_k = {0.05, 0, 0, 0, 0.001, 1}, KFP_b = {0.05, 0, 0, 0, 0.001, 1};

PID_T laser_X_pid;
PID_T laser_Y_pid;
PID_T laser_K_pid;
PID_T laser_B_pid;

void Update_DT35(int theta_angle, int theta_pitch, int theta_yaw, int theta_ras)
{
    origin_DT35.x = theta_angle;
    origin_DT35.y = theta_pitch;
    origin_DT35.k = theta_yaw;
    origin_DT35.b = theta_ras;

    DT35_FILTE.x = kalmanFilter(&KFP_x, origin_DT35.x);
    DT35_FILTE.y = kalmanFilter(&KFP_y, origin_DT35.y);
    DT35_FILTE.k = kalmanFilter(&KFP_k, origin_DT35.k);
    DT35_FILTE.b = kalmanFilter(&KFP_b, origin_DT35.b);

    DT35.x = 1.88 * DT35_FILTE.x + 83.3;
    DT35.y = 1.78 * DT35_FILTE.y + 11.9;
    DT35.k = DT35_FILTE.k;
    DT35.b = DT35_FILTE.b;
}

float kalmanFilter(KFP *kfp, float input)
{
    // 预测协方差方程：k时刻系统估算协方差 = k-1时刻的系统协方差 + 过程噪声协方差
    kfp->Now_P = kfp->LastP + kfp->Q;
    // 卡尔曼增益方程：卡尔曼增益 = k时刻系统估算协方差 / （k时刻系统估算协方差 + 观测噪声协方差）
    kfp->Kg = kfp->Now_P / (kfp->Now_P + kfp->R);
    // 更新最优值方程：k时刻状态变量的最优值 = 状态变量的预测值 + 卡尔曼增益 * （测量值 - 状态变量的预测值）
    kfp->out = kfp->out + kfp->Kg * (input - kfp->out); // 因为这一次的预测值就是上一次的输出值
    // 更新协方差方程: 本次的系统协方差付给 kfp->LastP 威下一次运算准备。
    kfp->LastP = (1 - kfp->Kg) * kfp->Now_P;
    return kfp->out;
}

// 单激光简单控制
float Laser_error_k;
int Laser1_calibration(float k, float v_max)
{
    Laser_error_k = DT35.k - k;

    if (ABS(Laser_error_k) > 5000)
    {
        ROBOT_CHASSI.plan_x = 0;
        ROBOT_CHASSI.plan_y = 0;
        return 0;
    }
    else
    {
        laser_K_pid.MaxOutput = ABS(v_max);
        pid_calc_by_error(&laser_K_pid, Laser_error_k);
        ROBOT_CHASSI.plan_y = -(laser_K_pid.output);
        if (ABS(Laser_error_k) < 10)
        {
            ROBOT_CHASSI.plan_x = 0;
            ROBOT_CHASSI.plan_y = 0;
            return 1;
        }
        else
        {
            return 0;
        }
    }
}

int8_t tt = 0;
// 双激光使底盘保持正
int Laser2_calibration(float location_x1, float location_x2)
{
    ROBOT_CHASSI.plan_x = 0;
    ROBOT_CHASSI.plan_y = 0;
    if ((location_x1 - location_x2) > 10)
    {
        ROBOT_CHASSI.plan_w = 100;
        tt = -1;
    }
    else if ((location_x1 - location_x2) < -10)
    {
        ROBOT_CHASSI.plan_w = -100;
        tt = 1;
    }
    else
    {
        ROBOT_CHASSI.plan_w = 0;
        return 1;
    }
    return 0;
}

float Laser_error_y, Laser_error_k, ERROR_SHOOTING;
// 修改定位
int Laser_calibration(float x, float y, float yaw, float v_max)
{
    YawAdjust(yaw);
    // 转到指定角度
    Laser_error_y = DT35.y - x;
    Laser_error_k = DT35.k - y;
    ERROR_SHOOTING = sqrt(Laser_error_y * Laser_error_y + Laser_error_k * Laser_error_k);
    // 怎么利用该误差还需填充
    // 判断距离是否合适
    if (Laser_error_y < 8000 && Laser_error_k < 8000)
    {
        if (ABS(Laser_error_y) > 10 || ABS(Laser_error_k) > 10)
        {
            laser_B_pid.MaxOutput = ABS(v_max);
            pid_calc_by_error(&laser_B_pid, ERROR_SHOOTING);
            ROBOT_CHASSI.plan_x = -Max_Value_Limit((laser_B_pid.output * 0.1f * Laser_error_k / ERROR_SHOOTING), v_max);
            ROBOT_CHASSI.plan_y = -Max_Value_Limit(laser_B_pid.output * 0.1f * Laser_error_y / sqrt(ERROR_SHOOTING), v_max);

            Laser_error_y = DT35.y - x;
            Laser_error_k = DT35.k - y;
        }
        else
            return 1;
    }
    return 0;
}
