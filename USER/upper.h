//
// Created by Ray on 2023/11/24.
//

#ifndef INC_2024RC_B_R1_UPPER_H
#define INC_2024RC_B_R1_UPPER_H

#include "main.h"

extern TIM_HandleTypeDef htim13;
extern TIM_HandleTypeDef htim14;

#define Motor_UPLIFT            0   //����̧�����
#define Motor_FLIP              1   //���緭ת���
#define Motor_BELT_MOTOR_1      2   //��������
#define Motor_BELT_MOTOR_2      3   //��������
#define Motor_BELT_MOTOR_3      4   //��������

typedef enum {
    finger1_3,  //�������1��3��
    finger2_4,  //�������2��4��
    push1,      //��������1
    push2,      //��������2
    open,       //����������
    test        //������
}Cylinder;

/**
 * @brief ��ת���ת����λ
 */
#define Flip2ground 0.0f
#define Flip2half -180.0f
#define Flip2top -270.0f

/**
 * @brief ̧�����ת����λ
 */
#define lift2ground 0.0f
#define lift2half -50.0f
#define lift2top -250.0f

void cylinder_control(Cylinder cylinder, uint8_t state);
void lift_motor(float target_pos);
void lift_hold(void);
void flip_motor(float target_angle);
void servos_start(void);
void servos_control(float duty);
void servos_stop(void);
void belt_ctrl(float target_spd);
void belt_logs(void);

#endif //INC_2024RC_B_R1_UPPER_H
