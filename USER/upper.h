//
// Created by Ray on 2023/11/24.
//

#ifndef INC_2024RC_B_R1_UPPER_H
#define INC_2024RC_B_R1_UPPER_H

#include "main.h"

extern TIM_HandleTypeDef htim13;
extern TIM_HandleTypeDef htim14;

#define Motor_UPLIFT            0   //秧苗抬升电机
#define Motor_FLIP              1   //秧苗翻转电机
#define Motor_BELT_MOTOR_1      2   //发射电机左
#define Motor_BELT_MOTOR_2      3   //发射电机右
#define Motor_BELT_MOTOR_3      4   //发射电机上

typedef enum {
    finger1_3,  //夹苗机构1、3号
    finger2_4,  //夹苗机构2、4号
    push1,      //推球气缸1
    push2,      //推球气缸2
    open,       //伸缩气缸组
    test        //测试用
}Cylinder;

/**
 * @brief 翻转电机转动档位
 */
#define Flip2ground 0.0f
#define Flip2half -180.0f
#define Flip2top -270.0f

/**
 * @brief 抬升电机转动档位
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
