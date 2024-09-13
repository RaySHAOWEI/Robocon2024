//
// Created by Ray on 2023/11/24.
//

#ifndef INC_2024RC_B_R1_FSM_H
#define INC_2024RC_B_R1_FSM_H

#include "main.h"
#define Left 0
#define Right 1
extern int field;
extern int catch_ball_laser;
typedef enum
{
    AUTO_DISABLE,
    AUTO_ENABLE
} AUTO_STATE; // 自动模式开关（第一层总状态机）
typedef enum
{
    ROBOT_STATE_INIT,      // 整车初始化（上层底盘都失能）
    ROBOT_UPPER_DISABLE,   // 上层失能
    ROBOT_STATE_SEED_CTRL, // 秧苗控制
    ROBOT_STATE_SHOOT_CTRL // 发射控制
} ROBOT_STATE_ITEMS;       // 整车状态

typedef enum
{
    CHASSIS_DISABLE,          // 底盘停止
    CHASSIS_STATE_HIGH_SPEED, // 高速模式
    CHASSIS_STATE_LOW_SPEED   // 低速模式（底盘自动模式）
} CHASSIS_STATE_ITEMS;        // 底盘状态

typedef enum
{
    SEED_STATE_DISABLE,     // 收缩状态
    SEED_STATE_INIT,        // 夹苗机构初始化(夹苗机构两边放苗)
    SEED_STATE_PEEK_DOWN,   // 夹苗机构夹住苗不上升
    SEED_STATE_PEEK_UP,     // 夹苗机构上升（离地7cm）
    SEED_STATE_PREPUT,      // 夹苗机构预放苗(下降至离地1cm)
    SEED_STATE_PUT,         // 夹苗机构放苗(只放一边)
    SEED_STATE_CLOSE,       // 夹苗机构收缩
    SEED_STATE_HALF,        // 夹苗机构预放苗(不切状态)
    SEED_STATE_JAW_INIT,    // 夹爪初始化
    SEED_STATE_PREPUT_BLUE, // 夹苗机构预放苗(下降至离地1cm)蓝方
    SEED_STATE_PUT_BLUE,    // 夹苗机构放苗(只放一边)蓝方
} SEED_STATE_ITEMS;         // 夹苗机构状态

typedef enum
{
    SHOOT_STATE_DISABLE, // 射球机构失能
    SHOOT_STATE_INIT,    // 射球机构初始化
    SHOOT_STATE_LOAD,    // 射球机构装填
    SHOOT_STATE_SHOOTING // 射球机构发射
} SHOOT_STATE_ITEMS;     // 射球机构状态

typedef enum
{
    GIMBAL_STATE_DISABLE, // 云台回正
    GIMBAL_AIM_1,         // 云台自瞄点1
    GIMBAL_AIM_2,         // 云台自瞄点2
    GIMBAL_AIM_3,         // 云台自瞄点3
} GIMBAL_STATE_ITEMS;     // 云台状态

typedef enum
{
    MOVE_STATE_INIT,        // 自动路径初始化
    MOVE_STATE_WAIT_SEED,   // 等待夹苗
    MOVE_STATE_WAIT_PUT,    // 等待放苗
    MOVE_STATE_WAIT_CATCH,  // 等待放球
    MOVE_STATE_WAIT_ACTION, // 重置action
    MOVE_INIT_ACTION,
    MOVE_STATE_SEED, // 夹苗函数
    MOVE_STATE_PUT,  // 放苗函数
    MOVE_SEED_LASER, // 激光取苗
    MOVE_PUT_LASER,
    MOVE_STATE_LASER, // 激光取球
    MOVE_STATE_ACTION_NEAR,
    CALIBRATION_BY_ACTION, // 激光取球前校准 要定位到取球点前先进此状态，然后跳到MOVE_STATE_LASER
    MOVE_BALL_BY_ACTION,
    MOVE_BY_ACTION
} MOVE_STATE_ITEMS;

extern AUTO_STATE auto_state;
extern ROBOT_STATE_ITEMS robot_state;
extern CHASSIS_STATE_ITEMS chassis_state;
extern SEED_STATE_ITEMS last_seed_state, seed_state;
extern SHOOT_STATE_ITEMS shoot_state;
extern GIMBAL_STATE_ITEMS gimbal_state;
extern MOVE_STATE_ITEMS move_state;
extern float action_yaw;
extern int update_action_seed;
extern int update_action_put;
extern float yaw_disable;
extern float tt3;
extern int caculation_ok;

void robot_fsm(void);
void robot_state_judge(void);
void auto_fsm(int f);
void SWA_judge(void);
void SWB_judge(void);
void SWC_judge(void);
// void SWD_judge(void);
void action_reset(void);

#endif // INC_2024RC_B_R1_FSM_H
