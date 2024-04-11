//
// Created by 19115 on 2024/3/13.
//

#ifndef LOWER_BOARD_TEST_COMMAND_H
#define LOWER_BOARD_TEST_COMMAND_H

#include "main.h"

// typedef enum {
//     SEED_STATE_DISABLE,         //收缩状态
//     SEED_STATE_INIT,            //夹苗机构初始化(夹苗机构两边放苗)
//     SEED_STATE_PEEK_DOWN,       //夹苗机构夹住苗不上升
//     SEED_STATE_PEEK_UP,         //夹苗机构上升（离地7cm）
//     SEED_STATE_PREPUT,          //夹苗机构预放苗(下降至离地1cm)
//     SEED_STATE_PUT,             //夹苗机构放苗(只放一边)
// }SEED_STATE_ITEMS;//夹苗机构状态

typedef struct
{
    uint8_t motor_id; // 1
    uint8_t mode;     // 1
    union kp
    {
        uint8_t kp_array[4]; // 4
        float kp;
    } kp;
    union ki
    {
        uint8_t ki_array[4]; // 4
        float ki;
    } ki;
    union kd
    {
        uint8_t kd_array[4]; // 4
        float kd;
    } kd;
} pid_command;

typedef struct
{
    uint8_t command_cnt; // 命令计数
    uint8_t command;     // 命令
} ctrl_command;

typedef struct
{
    uint8_t command_cnt;    // 命令计数
    uint8_t receive_cnt;    // 接收计数
    uint8_t lost_cnt;       // 丢包数（收到的数据包-有效数据包）
    uint8_t state;          // 当前秧苗总状态
    uint8_t motor_state[4]; // 电机状态 左 右 抬升 云台 0-初始 1-使能
} feedback_command;

// extern pid_command pidsend;
extern ctrl_command ctrlsend;
extern feedback_command feedback; // 状态监视器
// extern SEED_STATE_ITEMS seed_state;

// void pid_sent_test(uint8_t motor_id, uint8_t mode, float kp, float ki, float kd);
void ctrl_sent_test(uint8_t command);
// void Feedback_sends(void);

// void PID_process(uint8_t *data);
// void ctrl_process(uint8_t *data);
void feedback_process(uint8_t *data);

#endif // LOWER_BOARD_TEST_COMMAND_H
