//
// Created by Ray on 2023/11/24.
//

#ifndef INC_2024RC_B_R1_UPPER_H
#define INC_2024RC_B_R1_UPPER_H

#include "main.h"

/**
 * @brief 电机ID定义 can2
 */
#define Motor_SHOOT_MOTOR_1 0 // 发射电机1
#define Motor_SHOOT_MOTOR_2 1 // 发射电机2
#define Motor_SHOOT_FLIP 2    // 射球翻转电机
#define Motor_SHOOT_JAW 3     // 夹球夹爪2006电机
#define Motor_SHOOT_GIMBAL 4  // 发射云台6020电机

/**
 * @brief 翻转电机
 */
#define flip_down -360.0f  // 下到最低
#define flip_up 0.0f       // 初始位置
#define flip_init 1000.0f  // 收缩初始化
#define flip_speed 2000.0f // 翻转电机速度
/**
 * @brief 夹爪状态
 */
// #define jaw_close -1500 // 夹取时的速度-1500(角度大约为-160)
// #define jaw_open 1500   // 松开时的回零速度

#define jaw_close HAL_GPIO_WritePin(GPIOF, GPIO_PIN_12, GPIO_PIN_SET)
#define jaw_open HAL_GPIO_WritePin(GPIOF, GPIO_PIN_12, GPIO_PIN_RESET)
#define push_ball HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, GPIO_PIN_SET)
#define back_ball HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, GPIO_PIN_RESET)
#define belt_stop 0 // 摩擦带停止
// #define belt_speed 5000 // 摩擦带速度

/**
 * @brief 云台限制参数
 *
 */
#define gimbal_up 2000    // 云台下限
#define gimbal_mid 3300   // 云台中间
#define gimbal_down 4600  // 云台上限
#define gimbal_speed 2048 // 云台最高速度

extern int shooting_point; // 射球点位
extern int ball;           // 夹球点位
extern int ball_loaded;    // 球已装填
extern int shooting_enable;
extern int belt_enable;

typedef struct GIMBAL
{
    int16_t target_point[2]; // 目标点
    int16_t rpm;             // 云台转速
    float TAN;
    double Theta;
    float Descripe;      // 减数比   对现在R1不需要
    float gimbal_angle;  // 云台角度
    uint8_t centor_flag; // 是否回正标志位
    uint8_t aim_flag;    // 瞄准成功标志位
} GIMBAL;

extern GIMBAL Gimbal;
extern float Theta; // 车体相对于目标点的切角
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
