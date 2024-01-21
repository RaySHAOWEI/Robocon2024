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
        case push1:
            if(state == 0){
                HAL_GPIO_WritePin(load1_GPIO_Port, load1_Pin, GPIO_PIN_RESET);
            }
            else if(state == 1){
                HAL_GPIO_WritePin(load1_GPIO_Port, load1_Pin, GPIO_PIN_SET);
            }
            break;
        case push2:
            if(state == 0){
                HAL_GPIO_WritePin(load2_GPIO_Port, load2_Pin, GPIO_PIN_RESET);
            }
            else if(state == 1){
                HAL_GPIO_WritePin(load2_GPIO_Port, load2_Pin, GPIO_PIN_SET);
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
        case test:
            if(state == 0){
                HAL_GPIO_WritePin(test_GPIO_Port, test_Pin, GPIO_PIN_RESET);
            }
            else if(state == 1){
                HAL_GPIO_WritePin(test_GPIO_Port, test_Pin, GPIO_PIN_SET);
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

int duty_logs = 0;
void servos_control(float duty){
    if(duty_logs != duty){
        duty_logs = duty;
        __HAL_TIM_SET_COMPARE(&htim13, TIM_CHANNEL_1, (-1 * duty + 230) * 100 / 9 + 500);//50 75 130
        __HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, duty *  100 / 9 + 500);//180 155 100
    }
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

void belt_logs(void){
    sent_data(can2motorRealInfo[Motor_BELT_MOTOR_1].RPM,can2motorRealInfo[Motor_BELT_MOTOR_2].RPM,can2motorRealInfo[Motor_BELT_MOTOR_1].CURRENT,can2motorRealInfo[Motor_BELT_MOTOR_2].CURRENT);
}
