//
// Created by Ray on 2023/11/24.
//

#include "upper.h"

void lim(float *input, float max, float min) {
    if (max > min){
        if (*input > max) *input = max;
        if (*input < min) *input = min;
    }
    else if (max < min){
        if (*input > min) *input = min;
        if (*input < max) *input = max;
    }
}

void cylinder_control(Cylinder cylinder, uint8_t state){
    switch (cylinder)
    {
        case finger1_3:
            if(state == 0){
                HAL_GPIO_WritePin(finger1_GPIO_Port, finger1_Pin, GPIO_PIN_RESET);
            }
            else if(state == 1){
                HAL_GPIO_WritePin(finger1_GPIO_Port, finger1_Pin, GPIO_PIN_SET);
            }
            break;
        case finger2_4:
            if(state == 0){
                HAL_GPIO_WritePin(finger2_GPIO_Port, finger2_Pin, GPIO_PIN_RESET);
            }
            else if(state == 1){
                HAL_GPIO_WritePin(finger2_GPIO_Port, finger2_Pin, GPIO_PIN_SET);
            }
            break;
        case push:
            if(state == 0){
                HAL_GPIO_WritePin(load_GPIO_Port, load_Pin, GPIO_PIN_RESET);
            }
            else if(state == 1){
                HAL_GPIO_WritePin(load_GPIO_Port, load_Pin, GPIO_PIN_SET);
            }
            break;
        case open:
            if(state == 0){
                HAL_GPIO_WritePin(stretch_GPIO_Port, stretch_Pin, GPIO_PIN_RESET);
            }
            else if(state == 1){
                HAL_GPIO_WritePin(stretch_GPIO_Port, stretch_Pin, GPIO_PIN_SET);
            }
            break;
    }
}

void lift_motor(float target_pos){
    lim(&target_pos,lift2ground,lift2top);
    if (target_pos == lift2ground)
    {
        if(can2motorRealInfo[Motor_UPLIFT].HomingMode.done_flag == 0){
            Homeing_Mode(&can2motorRealInfo[Motor_UPLIFT], 1000, 4096);
        }else{  
            Position_Control(&can2motorRealInfo[Motor_UPLIFT], target_pos);
        }
        Motor_Control();
    }
    else if (target_pos < lift2ground && target_pos >= lift2top)//软限位
    {
        can2motorRealInfo[Motor_UPLIFT].HomingMode.done_flag = 0;
        Position_Control(&can2motorRealInfo[Motor_UPLIFT], target_pos);
        Motor_Control();
	}
}

void lift_hold(void){
    Position_Control(&can2motorRealInfo[Motor_UPLIFT], can2motorRealInfo[Motor_UPLIFT].REAL_ANGLE);
    Motor_Control();
}

//0为收起, 1为放下, 2为半放
void flip_motor(float target_angle){
    lim(&target_angle,Flip2ground,Flip2top);
    if (target_angle == Flip2ground)
    {
        if(can2motorRealInfo[Motor_FLIP].HomingMode.done_flag == 0){
            Homeing_Mode(&can2motorRealInfo[Motor_FLIP], 1000, 4096);
        }else{
            Position_Control(&can2motorRealInfo[Motor_FLIP], target_angle);
        }
        Motor_Control();
    }
    else if (target_angle < Flip2ground && target_angle >= Flip2top)//软限位
    {
        can2motorRealInfo[Motor_FLIP].HomingMode.done_flag = 0;
        Position_Control(&can2motorRealInfo[Motor_FLIP], target_angle);
        Motor_Control();
    }
}

void servos_start(void){
    HAL_TIM_PWM_Start(&htim13, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim14, TIM_CHANNEL_1);
}

void servos_control(float duty){
    __HAL_TIM_SET_COMPARE(&htim13, TIM_CHANNEL_1, (-1 * duty + 230) * 2000 / 180 + 500);//50 75 130
    __HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, duty * 2000 / 180 + 500);//180 155 100
}

void servos_stop(void){
    HAL_TIM_PWM_Stop(&htim13, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim14, TIM_CHANNEL_1);
}

void belt_ctrl(float target_spd){
    float belt_spd[3] = {target_spd * 0.025f,target_spd * 4.0f,target_spd * 0.025f};//上一次：{target_spd * 0.025f,target_spd * 4.0f,target_spd * 0.025f}
    Speed_Control(&can2motorRealInfo[Motor_BELT_MOTOR_1], belt_spd[0]);//左
    Speed_Control(&can2motorRealInfo[Motor_BELT_MOTOR_2], belt_spd[1]);//右
    Speed_Control(&can2motorRealInfo[Motor_BELT_MOTOR_3], belt_spd[2]);//上
    Motor_Control();
}
