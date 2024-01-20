//
// Created by Ray on 2023/11/24.
//

#ifndef INC_2024RC_B_R1_PID_H
#define INC_2024RC_B_R1_PID_H

#define ABS(x)  ((x)>0? (x):(-(x)))

/**
 * @brief
 *  初始化PID参数
    位置式有积分限幅 增量式没有积分限幅
    位置式有初始误差功能 增量式没有初始误差功能
    位置式和增量式都有死区功能 死区设为负数则死区功能不开启
 */
typedef enum pid_st
{
    PREV = 0,
    LAST = 1,
    NOW = 2,
    NEXT = 3,

    PID_Position,       //位置式
    PID_Incremental     //增量式
}pidSt;

/**
 * @brief PID结构体
 * 20231008 新增积分分离阈值
 */
typedef struct _PID_T
{
    float kp;                       //比例系数
    float ki;                       //积分系数
    float kd;                       //微分系数

    float target[3];			    //目标值
    float measure[3];				//测量值
    float err[3];					//误差

    float pout;                     //比例项
    float iout;                     //积分项
    float dout;                     //微分项

    float output;					//本次输出
    float last_output;			    //上次输出

    pidSt mode;                     //PID类型：位置式或者增量式
    float MaxOutput;				//输出限幅
    float IntegralLimit;		    //积分限幅
    float IntegralSeparate;         //积分分离阈值，用于避免积分饱和现象
    float DeadBand;			        //死区（绝对值）
    float Max_Err;					//最大误差
}PID_T;

void pid_param_init(
        PID_T *pid,
        pidSt mode,
        float maxOutput,
        float integralLimit,
        float IntegralSeparate,
        float deadband,
        float max_err,
        float kp,
        float ki,
        float kd);

void pid_reset(PID_T* pid, float kp, float ki, float kd);

float pid_calc(PID_T *pid, float target, float measure);

#endif //INC_2024RC_B_R1_PID_H
