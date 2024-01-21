//
// Created by Ray on 2023/11/24.
//

#ifndef INC_2024RC_B_R1_FSM_H
#define INC_2024RC_B_R1_FSM_H

#include "main.h"

typedef enum {
    ROBOT_STATE_INIT,           //整车初始化
    ROBOT_STATE_CALIBRATION,    //校准
    ROBOT_STATE_AUTO_CTRL,      //自适应状态
    ROBOT_STATE_SEED_CTRL,      //秧苗控制
    ROBOT_STATE_SHOOT_CTRL      //发射控制
}ROBOT_STATE_ITEMS;//整车状态

typedef enum {
    CHASSIS_DISABLE,            //底盘停止
    CHASSIS_STATE_HIGH_SPEED,   //高速模式
    CHASSIS_STATE_LOW_SPEED     //低速模式
}CHASSIS_STATE_ITEMS;//底盘状态

typedef enum {
    SEED_STATE_DISABLE,         //收缩状态
    SEED_STATE_INIT,            //夹苗机构初始化
    SEED_STATE_PEEK,       //夹苗机构夹取
    SEED_STATE_PUT              //夹苗机构放苗（只放一边）
}SEED_STATE_ITEMS;//夹苗机构状态

typedef enum {
    SHOOT_STATE_INIT,           //射球机构初始化
    SHOOT_STATE_LOAD,           //射球机构装填
    SHOOT_STATE_SHOOTING        //射球机构发射
}SHOOT_STATE_ITEMS;//射球机构状态

extern ROBOT_STATE_ITEMS robot_state;
extern CHASSIS_STATE_ITEMS chassis_state;
extern SEED_STATE_ITEMS seed_state;
extern SHOOT_STATE_ITEMS shoot_state;

//建立场地区域结构体
typedef struct AREA
{
    float x_min;
    float x_max;
    float y_min;
    float y_max;
}AREA;

void robot_fsm(void);
void SWA_judge(void);
void auto_switch(void);
void SWB_judge(void);
void SWC_judge(void);
void SWD_judge(void);

#endif //INC_2024RC_B_R1_FSM_H
