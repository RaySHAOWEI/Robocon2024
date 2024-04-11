//
// Created by Ray on 2023/11/24.
//

#ifndef INC_2024RC_B_R1_PID_H
#define INC_2024RC_B_R1_PID_H

#include "stm32f4xx.h"

#define ABS(x) ((x) > 0 ? (x) : (-(x)))

/**
 * @brief
 *  ��ʼ��PID����
    λ��ʽ�л����޷� ����ʽû�л����޷�
    λ��ʽ�г�ʼ���� ����ʽû�г�ʼ����
    λ��ʽ������ʽ������������ ������Ϊ�������������ܲ�����
 */
typedef enum pid_st
{
    PREV = 0,
    LAST = 1,
    NOW = 2,
    NEXT = 3,

    PID_Position,   // λ��ʽ
    PID_Incremental // ����ʽ
} pidSt;

/**
 * @brief PID�ṹ��
 * 20231008 �������ַ�����ֵ
 */
typedef struct _PID_T
{
    float kp; // ����ϵ��
    float ki; // ����ϵ��
    float kd; // ΢��ϵ��
    float kf; // ǰ��ϵ��

    float target[3];  // Ŀ��ֵ
    float measure[3]; // ����ֵ
    float err[3];     // ���

    float pout; // ������
    float iout; // ������
    float dout; // ΢����
    float fout; // ǰ����

    float output;      // �������
    float last_output; // �ϴ����

    pidSt mode;             // PID���ͣ�λ��ʽ��������ʽ
    float MaxOutput;        // ����޷�
    float IntegralLimit;    // �����޷�
    float IntegralSeparate; // ���ַ�����ֵ�����ڱ�����ֱ�������
    float DeadBand;         // ����������ֵ��
    float Max_Err;          // ������

    uint8_t first_flag; // ��һ�α�־λ
} PID_T;

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

void pid_fast_init(PID_T *pid, float maxOutput, float kp, float ki, float kd);

void pid_reset(PID_T *pid, float kp, float ki, float kd);

float pid_calc(PID_T *pid, float target, float measure);

float pid_calc_by_error(PID_T *pid, float error);

#endif // INC_2024RC_B_R1_PID_H
