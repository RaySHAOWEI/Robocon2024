//
// Created by Ray on 2023/11/24.
//

#ifndef INC_2024RC_B_R1_UPPER_H
#define INC_2024RC_B_R1_UPPER_H

#include "main.h"


extern TIM_HandleTypeDef htim13;
extern TIM_HandleTypeDef htim14;

/**
 * @brief 电机ID定义
 */
#define Motor_SEED_MOTOR_1      0   //夹苗1
#define Motor_SEED_MOTOR_2      1   //夹苗2
#define Motor_LIFT              2   //抬升
#define Motor_BASE	            3   //夹爪云台电机


#define Motor_SHOOT_MOTOR_1     5   //发射电机1
#define Motor_SHOOT_MOTOR_2     6   //发射电机2
#define Motor_SHOOT_GIMBAL      4   //发射云台电机
#define Motor_SHOOT_JAW         5   //夹球夹爪电机
#define Motor_SHOOT_lift        6   //射球夹爪翻转
/**
 * @brief 抬升参数
 */
#define lift2ground 0.0f
#define lift2half 75.0f
#define lift2top 130.0f

/**
 * @brief 夹苗云台参数
 */
#define base1min 0.0f    
#define base1half 75.0f
#define base1top 160.0f     

/**
 * @brief 夹爪参数
 */
#define jaw1min -45.0f    
#define jaw1top 45.0f     
#define jaw_angle 30.0f
#define jaw_homeing_flag 1     //回零标志位
#define shoot_homeing_flag 1
#define catch_ball 1
#define jaw_up 2
typedef struct GIMBAL
{
    int16_t target_point[2];    //目标点
    int16_t rpm;         //云台转速
    float TAN;           
    double Theta;               
    float Descripe;             //减数比   对现在R1不需要
    float gimbal_angle;         //云台角度
    uint8_t centor_flag;        //是否回正标志位 
    uint8_t aim_flag;           //瞄准成功标志位
}GIMBAL;

extern GIMBAL Gimbal; 
extern int16_t Theta;  //车体相对于目标点的切角
extern int16_t Alpha;
#define locked_rotor_time 10   //堵转时间，若电流值大于所测的电机堵转电流值，超过此时间则判定为电机堵转
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
