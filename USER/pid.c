//
// Created by Ray on 2023/11/24.
//

#include "pid.h"

/**
 * @brief 输入的数据保证在正确的范围内
 * @param input 输入值
 * @param max 限定值
 */
void LimitMax(float *input, float max) {
    if (*input > max) *input = max;
    if (*input < -max) *input = -max;
}


/**
 * @brief PID参数初始化
 * @param pid
 * @param mode
 * @param maxOutput
 * @param integralLimit
 * @param IntegralSeparate
 * @param deadband
 * @param max_err
 * @param kp
 * @param ki
 * @param kd
 */
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
        float kd){
    pid->mode = mode;
    pid->MaxOutput = maxOutput;
    pid->IntegralLimit = integralLimit;
    pid->IntegralSeparate = IntegralSeparate;
    pid->DeadBand = deadband;
    pid->Max_Err = max_err;
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->last_output = 0;
    pid->output = 0;
}

/**
 * @brief 中途修改pid参数
 * @param pid PID结构体
 * @param kp 参数p
 * @param ki 参数i
 * @param kd 参数d
 */
void pid_reset(PID_T *pid, float kp, float ki, float kd) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

/**
 * @brief PID计算
 * @param pid PID结构体
 * @param measure 当前值
 * @param target 目标值
 * @return
 */
float pid_calc(PID_T *pid, float target, float measure){
    pid->measure[NOW] = measure;                                         //更新本次最新测量值
    pid->target[NOW] = target;
    pid->err[NOW] = pid->target[NOW] - pid->measure[NOW];                     //更新误差值

    if (pid->Max_Err != 0 && ABS(pid->err[NOW]) > pid->Max_Err)        //误差超过限制跳出
    {
		pid->output = 0;
        return 0;
    }
        
    if (pid->DeadBand != 0 && ABS(pid->err[NOW]) < pid->DeadBand)       //误差小于死区跳出
	{
		pid->output = 0;
        return 0;
	}

    if (pid->mode == PID_Position) {
        pid->pout = pid->kp * pid->err[NOW];                           //p输出为Kp*误差
        if (pid->IntegralSeparate == 0)
            pid->iout += pid->ki * pid->err[NOW];                       //i输出为i+ki*误差
        else {
            //判断是否积分分离
            if (ABS(pid->err[NOW]) < pid->IntegralSeparate)
                pid->iout += pid->ki * pid->err[NOW];                   //i输出为i+ki*误差
            else
                pid->iout = 0;                                          //i输出为0
        }
        pid->dout = pid->kd * (pid->err[NOW] - pid->err[LAST]);        //d输出为kd*（误差-上次误差）

        LimitMax(&(pid->iout), pid->IntegralLimit);          //积分是否超出限制
        pid->output = pid->pout + pid->iout + pid->dout;                //pid输出和
        //pid->output = pid->output*0.7f + pid->last_output*0.3f;       //滤波
        LimitMax(&(pid->output), pid->MaxOutput);
        pid->last_output = pid->output;                                 //更新数据

    } else if (pid->mode == PID_Incremental) {
        pid->pout = pid->kp * (pid->err[NOW] - pid->err[LAST]);         //p输出为Kp * 误差增量
        pid->iout = pid->ki * pid->err[NOW];                            //i输出为ki * 误差
        pid->dout = pid->kd * (pid->err[NOW] - (2 * pid->err[LAST]) + pid->err[PREV]);
        //d输出为kd *（误差-2*上次误差+上上次误差）

        // LimitMax(&(pid->iout), pid->IntegralLimit);         //积分是否超出限制
        pid->output = pid->last_output + (pid->pout + pid->iout + pid->dout);   //pid输出和
        //pid->output = pid->output*0.7f + pid->last_output*0.3f;   //滤波？
        LimitMax(&(pid->output), pid->MaxOutput);
        pid->last_output = pid->output;

    }

    //  数据更新
    pid->err[PREV] = pid->err[LAST];
    pid->err[LAST] = pid->err[NOW];
    pid->measure[PREV] = pid->measure[LAST];
    pid->measure[LAST] = pid->measure[NOW];
    pid->target[PREV] = pid->target[LAST];
    pid->target[LAST] = pid->target[NOW];

    return pid->output;
}
