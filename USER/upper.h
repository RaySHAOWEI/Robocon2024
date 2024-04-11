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
 * @brief 翻转电机转动角度
 */
#define flip_down 0 // 初始位置
#define flip_up 115 // 上到顶端
/**
 * @brief 夹爪状态
 */
#define jaw_close -1500 // 夹取时的速度(角度大约为-160)
#define jaw_open 100    // 松开时的角度

#define shootball HAL_GPIO_WritePin(GPIOF, GPIO_PIN_12, GPIO_PIN_SET)
#define back2init HAL_GPIO_WritePin(GPIOF, GPIO_PIN_12, GPIO_PIN_RESET)
#define belt_stop 0     // 摩擦带停止
#define belt_speed 5000 // 摩擦带速度

/**
 * @brief 云台限制参数
 *
 */
#define gimbal_up 2000   // 云台下限
#define gimbal_mid 3300  // 云台中间
#define gimbal_down 4600 // 云台上限
#define gimbal_speed 800 // 云台最高速度

#define jaw_homeing_flag 1 // 回零标志位
#define shoot_homeing_flag 1
#define catch_ball 1
#define jaw_up 2

// typedef struct GIMBAL
// {
//     int16_t target_point[2];    //目标点
//     int16_t rpm;         //云台转速
//     float TAN;
//     double Theta;
//     float Descripe;             //减数比   对现在R1不需要
//     float gimbal_angle;         //云台角度
//     uint8_t centor_flag;        //是否回正标志位
//     uint8_t aim_flag;           //瞄准成功标志位
// }GIMBAL;

// extern GIMBAL Gimbal;
// extern int16_t Theta;  //车体相对于目标点的切角
// extern int16_t Alpha;
// #define locked_rotor_time 10   //堵转时间，若电流值大于所测的电机堵转电流值，超过此时间则判定为电机堵转

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
