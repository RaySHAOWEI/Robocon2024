//
// Created by Ray on 2023/11/24.
//

#include "pid.h"

/**
 * @brief ��������ݱ�֤����ȷ�ķ�Χ��
 * @param input ����ֵ
 * @param max �޶�ֵ
 */
void LimitMax(float *input, float max) {
    if (*input > max) *input = max;
    if (*input < -max) *input = -max;
}


/**
 * @brief PID������ʼ��
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
 * @brief ��;�޸�pid����
 * @param pid PID�ṹ��
 * @param kp ����p
 * @param ki ����i
 * @param kd ����d
 */
void pid_reset(PID_T *pid, float kp, float ki, float kd) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

/**
 * @brief PID����
 * @param pid PID�ṹ��
 * @param measure ��ǰֵ
 * @param target Ŀ��ֵ
 * @return
 */
float pid_calc(PID_T *pid, float target, float measure){
    pid->measure[NOW] = measure;                                         //���±������²���ֵ
    pid->target[NOW] = target;
    pid->err[NOW] = pid->target[NOW] - pid->measure[NOW];                     //�������ֵ

    if (pid->Max_Err != 0 && ABS(pid->err[NOW]) > pid->Max_Err)        //������������
    {
		pid->output = 0;
        return 0;
    }
        
    if (pid->DeadBand != 0 && ABS(pid->err[NOW]) < pid->DeadBand)       //���С����������
	{
		pid->output = 0;
        return 0;
	}

    if (pid->mode == PID_Position) {
        pid->pout = pid->kp * pid->err[NOW];                           //p���ΪKp*���
        if (pid->IntegralSeparate == 0)
            pid->iout += pid->ki * pid->err[NOW];                       //i���Ϊi+ki*���
        else {
            //�ж��Ƿ���ַ���
            if (ABS(pid->err[NOW]) < pid->IntegralSeparate)
                pid->iout += pid->ki * pid->err[NOW];                   //i���Ϊi+ki*���
            else
                pid->iout = 0;                                          //i���Ϊ0
        }
        pid->dout = pid->kd * (pid->err[NOW] - pid->err[LAST]);        //d���Ϊkd*�����-�ϴ���

        LimitMax(&(pid->iout), pid->IntegralLimit);          //�����Ƿ񳬳�����
        pid->output = pid->pout + pid->iout + pid->dout;                //pid�����
        //pid->output = pid->output*0.7f + pid->last_output*0.3f;       //�˲�
        LimitMax(&(pid->output), pid->MaxOutput);
        pid->last_output = pid->output;                                 //��������

    } else if (pid->mode == PID_Incremental) {
        pid->pout = pid->kp * (pid->err[NOW] - pid->err[LAST]);         //p���ΪKp * �������
        pid->iout = pid->ki * pid->err[NOW];                            //i���Ϊki * ���
        pid->dout = pid->kd * (pid->err[NOW] - (2 * pid->err[LAST]) + pid->err[PREV]);
        //d���Ϊkd *�����-2*�ϴ����+���ϴ���

        // LimitMax(&(pid->iout), pid->IntegralLimit);         //�����Ƿ񳬳�����
        pid->output = pid->last_output + (pid->pout + pid->iout + pid->dout);   //pid�����
        //pid->output = pid->output*0.7f + pid->last_output*0.3f;   //�˲���
        LimitMax(&(pid->output), pid->MaxOutput);
        pid->last_output = pid->output;

    }

    //  ���ݸ���
    pid->err[PREV] = pid->err[LAST];
    pid->err[LAST] = pid->err[NOW];
    pid->measure[PREV] = pid->measure[LAST];
    pid->measure[LAST] = pid->measure[NOW];
    pid->target[PREV] = pid->target[LAST];
    pid->target[LAST] = pid->target[NOW];

    return pid->output;
}
