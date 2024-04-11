//
// Created by Ray on 2023/11/24.
//

#ifndef INC_2024RC_B_R1_UPPER_H
#define INC_2024RC_B_R1_UPPER_H

#include "main.h"

/**
 * @brief ���ID���� can2
 */
#define Motor_SHOOT_MOTOR_1 0 // ������1
#define Motor_SHOOT_MOTOR_2 1 // ������2
#define Motor_SHOOT_FLIP 2    // ����ת���
#define Motor_SHOOT_JAW 3     // �����צ2006���
#define Motor_SHOOT_GIMBAL 4  // ������̨6020���

/**
 * @brief ��ת���ת���Ƕ�
 */
#define flip_down 0 // ��ʼλ��
#define flip_up 115 // �ϵ�����
/**
 * @brief ��צ״̬
 */
#define jaw_close -1500 // ��ȡʱ���ٶ�(�Ƕȴ�ԼΪ-160)
#define jaw_open 100    // �ɿ�ʱ�ĽǶ�

#define shootball HAL_GPIO_WritePin(GPIOF, GPIO_PIN_12, GPIO_PIN_SET)
#define back2init HAL_GPIO_WritePin(GPIOF, GPIO_PIN_12, GPIO_PIN_RESET)
#define belt_stop 0     // Ħ����ֹͣ
#define belt_speed 5000 // Ħ�����ٶ�

/**
 * @brief ��̨���Ʋ���
 *
 */
#define gimbal_up 2000   // ��̨����
#define gimbal_mid 3300  // ��̨�м�
#define gimbal_down 4600 // ��̨����
#define gimbal_speed 800 // ��̨����ٶ�

#define jaw_homeing_flag 1 // �����־λ
#define shoot_homeing_flag 1
#define catch_ball 1
#define jaw_up 2

// typedef struct GIMBAL
// {
//     int16_t target_point[2];    //Ŀ���
//     int16_t rpm;         //��̨ת��
//     float TAN;
//     double Theta;
//     float Descripe;             //������   ������R1����Ҫ
//     float gimbal_angle;         //��̨�Ƕ�
//     uint8_t centor_flag;        //�Ƿ������־λ
//     uint8_t aim_flag;           //��׼�ɹ���־λ
// }GIMBAL;

// extern GIMBAL Gimbal;
// extern int16_t Theta;  //���������Ŀ�����н�
// extern int16_t Alpha;
// #define locked_rotor_time 10   //��תʱ�䣬������ֵ��������ĵ����ת����ֵ��������ʱ�����ж�Ϊ�����ת

// uint8_t test_rise_time_up(int current,int boundary_current,int boundary_time);

// void Shooting_ball(void);
// void Shooting_Init(void);
// void Gimbal_controller(void);

void lim(float *input, float max, float min);

int flip_motor(float target_angle);
int jaw_motor(int jaw_state);
void belt_ctrl(int vel);
void gimbal_ctrl(float target_pos);
void gimbal_free_ctrl(void);

void Load_Ball(int step);
void shoot_jaw_homeing(int flag);
int flag_judgment(void);
#endif // INC_2024RC_B_R1_UPPER_H
