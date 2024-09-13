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
} AUTO_STATE; // �Զ�ģʽ���أ���һ����״̬����
typedef enum
{
    ROBOT_STATE_INIT,      // ������ʼ�����ϲ���̶�ʧ�ܣ�
    ROBOT_UPPER_DISABLE,   // �ϲ�ʧ��
    ROBOT_STATE_SEED_CTRL, // �������
    ROBOT_STATE_SHOOT_CTRL // �������
} ROBOT_STATE_ITEMS;       // ����״̬

typedef enum
{
    CHASSIS_DISABLE,          // ����ֹͣ
    CHASSIS_STATE_HIGH_SPEED, // ����ģʽ
    CHASSIS_STATE_LOW_SPEED   // ����ģʽ�������Զ�ģʽ��
} CHASSIS_STATE_ITEMS;        // ����״̬

typedef enum
{
    SEED_STATE_DISABLE,     // ����״̬
    SEED_STATE_INIT,        // ���������ʼ��(����������߷���)
    SEED_STATE_PEEK_DOWN,   // ���������ס�粻����
    SEED_STATE_PEEK_UP,     // ����������������7cm��
    SEED_STATE_PREPUT,      // �������Ԥ����(�½������1cm)
    SEED_STATE_PUT,         // �����������(ֻ��һ��)
    SEED_STATE_CLOSE,       // �����������
    SEED_STATE_HALF,        // �������Ԥ����(����״̬)
    SEED_STATE_JAW_INIT,    // ��צ��ʼ��
    SEED_STATE_PREPUT_BLUE, // �������Ԥ����(�½������1cm)����
    SEED_STATE_PUT_BLUE,    // �����������(ֻ��һ��)����
} SEED_STATE_ITEMS;         // �������״̬

typedef enum
{
    SHOOT_STATE_DISABLE, // �������ʧ��
    SHOOT_STATE_INIT,    // ���������ʼ��
    SHOOT_STATE_LOAD,    // �������װ��
    SHOOT_STATE_SHOOTING // �����������
} SHOOT_STATE_ITEMS;     // �������״̬

typedef enum
{
    GIMBAL_STATE_DISABLE, // ��̨����
    GIMBAL_AIM_1,         // ��̨�����1
    GIMBAL_AIM_2,         // ��̨�����2
    GIMBAL_AIM_3,         // ��̨�����3
} GIMBAL_STATE_ITEMS;     // ��̨״̬

typedef enum
{
    MOVE_STATE_INIT,        // �Զ�·����ʼ��
    MOVE_STATE_WAIT_SEED,   // �ȴ�����
    MOVE_STATE_WAIT_PUT,    // �ȴ�����
    MOVE_STATE_WAIT_CATCH,  // �ȴ�����
    MOVE_STATE_WAIT_ACTION, // ����action
    MOVE_INIT_ACTION,
    MOVE_STATE_SEED, // ���纯��
    MOVE_STATE_PUT,  // ���纯��
    MOVE_SEED_LASER, // ����ȡ��
    MOVE_PUT_LASER,
    MOVE_STATE_LASER, // ����ȡ��
    MOVE_STATE_ACTION_NEAR,
    CALIBRATION_BY_ACTION, // ����ȡ��ǰУ׼ Ҫ��λ��ȡ���ǰ�Ƚ���״̬��Ȼ������MOVE_STATE_LASER
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
