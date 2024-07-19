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
 * @brief ��ת���
 */
#define flip_down -360.0f  // �µ����
#define flip_up 0.0f       // ��ʼλ��
#define flip_init 1000.0f  // ������ʼ��
#define flip_speed 2000.0f // ��ת����ٶ�
/**
 * @brief ��צ״̬
 */
// #define jaw_close -1500 // ��ȡʱ���ٶ�-1500(�Ƕȴ�ԼΪ-160)
// #define jaw_open 1500   // �ɿ�ʱ�Ļ����ٶ�

#define jaw_close HAL_GPIO_WritePin(GPIOF, GPIO_PIN_12, GPIO_PIN_SET)
#define jaw_open HAL_GPIO_WritePin(GPIOF, GPIO_PIN_12, GPIO_PIN_RESET)
#define push_ball HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, GPIO_PIN_SET)
#define back_ball HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, GPIO_PIN_RESET)
#define belt_stop 0 // Ħ����ֹͣ
// #define belt_speed 5000 // Ħ�����ٶ�

/**
 * @brief ��̨���Ʋ���
 *
 */
#define gimbal_up 2000    // ��̨����
#define gimbal_mid 3300   // ��̨�м�
#define gimbal_down 4600  // ��̨����
#define gimbal_speed 2048 // ��̨����ٶ�

extern int shooting_point; // �����λ
extern int ball;           // �����λ
extern int ball_loaded;    // ����װ��
extern int shooting_enable;
extern int belt_enable;

typedef struct GIMBAL
{
    int16_t target_point[2]; // Ŀ���
    int16_t rpm;             // ��̨ת��
    float TAN;
    double Theta;
    float Descripe;      // ������   ������R1����Ҫ
    float gimbal_angle;  // ��̨�Ƕ�
    uint8_t centor_flag; // �Ƿ������־λ
    uint8_t aim_flag;    // ��׼�ɹ���־λ
} GIMBAL;

extern GIMBAL Gimbal;
extern float Theta; // ���������Ŀ�����н�
extern float Alpha;
extern int action_ball;
extern int flip_state;
extern int belt_speed;

extern int shooting_point_x[7];
extern int shooting_point_y[7];

// void Shooting_ball(void);
// void Shooting_Init(void);
// void Gimbal_controller(void);

void Auto_shooting_Init(void);
void shooting_Init(int X, int Y);
void Auto_shooting_Shooting(void);
void Gimbal_controller(void);
void lim(float *input, float max, float min);
void belt_base(void);

int flip_motor(float target_angle);
void belt_ctrl(float vel);
void gimbal_ctrl(float target_pos);
void gimbal_free_ctrl(void);
void ball_auto(int field);
void belt_calc(int f, int point);

#endif // INC_2024RC_B_R1_UPPER_H
