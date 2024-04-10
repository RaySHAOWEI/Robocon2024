//
// Created by Ray on 2023/11/24.
//

#ifndef INC_2024RC_B_R1_UPPER_H
#define INC_2024RC_B_R1_UPPER_H

#include "main.h"


extern TIM_HandleTypeDef htim13;
extern TIM_HandleTypeDef htim14;

/**
 * @brief ���ID����
 */
#define Motor_SEED_MOTOR_1      0   //����1
#define Motor_SEED_MOTOR_2      1   //����2
#define Motor_LIFT              2   //̧��
#define Motor_BASE	            3   //��צ��̨���


#define Motor_SHOOT_MOTOR_1     5   //������1
#define Motor_SHOOT_MOTOR_2     6   //������2
#define Motor_SHOOT_GIMBAL      4   //������̨���
#define Motor_SHOOT_JAW         5   //�����צ���
#define Motor_SHOOT_lift        6   //�����צ��ת
/**
 * @brief ̧������
 */
#define lift2ground 0.0f
#define lift2half 75.0f
#define lift2top 130.0f

/**
 * @brief ������̨����
 */
#define base1min 0.0f    
#define base1half 75.0f
#define base1top 160.0f     

/**
 * @brief ��צ����
 */
#define jaw1min -45.0f    
#define jaw1top 45.0f     
#define jaw_angle 30.0f
#define jaw_homeing_flag 1     //�����־λ
#define shoot_homeing_flag 1
#define catch_ball 1
#define jaw_up 2
typedef struct GIMBAL
{
    int16_t target_point[2];    //Ŀ���
    int16_t rpm;         //��̨ת��
    float TAN;           
    double Theta;               
    float Descripe;             //������   ������R1����Ҫ
    float gimbal_angle;         //��̨�Ƕ�
    uint8_t centor_flag;        //�Ƿ������־λ 
    uint8_t aim_flag;           //��׼�ɹ���־λ
}GIMBAL;

extern GIMBAL Gimbal; 
extern int16_t Theta;  //���������Ŀ�����н�
extern int16_t Alpha;
#define locked_rotor_time 10   //��תʱ�䣬������ֵ��������ĵ����ת����ֵ��������ʱ�����ж�Ϊ�����ת
void base_motor(float target_pos);
void lift_motor(float target_angle);
void jaw_motor(float target_angle);
void lim(float *input, float max, float min);
uint8_t test_rise_time_up(int current,int boundary_current,int boundary_time);
void Load_Ball(int step);
void Shooting_ball(void);
void Shooting_Init(void);
void Gimbal_controller(void);
void shoot_jaw_homeing(int flag);
int flag_judgment(void);
#endif //INC_2024RC_B_R1_UPPER_H
