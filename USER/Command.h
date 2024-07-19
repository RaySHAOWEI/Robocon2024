//
// Created by 19115 on 2024/3/13.
//

#ifndef LOWER_BOARD_TEST_COMMAND_H
#define LOWER_BOARD_TEST_COMMAND_H

#include "main.h"

// typedef enum {
//     SEED_STATE_DISABLE,         //����״̬
//     SEED_STATE_INIT,            //���������ʼ��(����������߷���)
//     SEED_STATE_PEEK_DOWN,       //���������ס�粻����
//     SEED_STATE_PEEK_UP,         //����������������7cm��
//     SEED_STATE_PREPUT,          //�������Ԥ����(�½������1cm)
//     SEED_STATE_PUT,             //�����������(ֻ��һ��)
// }SEED_STATE_ITEMS;//�������״̬

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
    uint8_t command_cnt; // �������
    uint8_t command;     // ����
} ctrl_command;

typedef struct
{
    uint8_t command_cnt;    // �������
    uint8_t receive_cnt;    // ���ռ���
    uint8_t lost_cnt;       // ���������յ������ݰ�-��Ч���ݰ���
    uint8_t state;          // ��ǰ������״̬
    uint8_t motor_state[4]; // ���״̬ �� �� ̧�� ��̨ 0-��ʼ 1-ʹ��
} feedback_command;

typedef struct
{
    uint8_t aim_state; // ��׼����״̬(��̨����)
    uint8_t key_state; // ��λ����״̬����λ��
    int8_t switch_x;   // ҡ��x��
    int8_t switch_y;   // ҡ��y��
} Keyboard_command;

typedef struct
{
    uint8_t command_cnt; // �������
    uint8_t receive_cnt; // ���ռ���
    uint8_t lost_cnt;    // ���������յ������ݰ�-��Ч���ݰ���

    uint8_t auto_state;   // �Զ�״̬
    uint8_t move_state;   // �ƶ�״̬ ȡ��1 �ȴ�ȡ��2 ����3 �ȴ�����4
    uint8_t direction;    // ȡ��
    uint8_t seed;         // ����
    uint8_t put;          // ����
    uint8_t gimbal_state; // ��̨״̬
} keyboard_feedback;

// extern pid_command pidsend;
extern ctrl_command ctrlsend;
extern feedback_command feedback1, feedback2; // ״̬������ 1 �� 2 ��
extern Keyboard_command last_keyboard_commands, keyboard_commands;
extern keyboard_feedback last_keyboard_feedbacks, keyboard_feedbacks;
// extern SEED_STATE_ITEMS seed_state;

// void pid_sent_test(uint8_t motor_id, uint8_t mode, float kp, float ki, float kd);
void ctrl_sent_test(uint8_t command);
// void Feedback_sends(void);

// void PID_process(uint8_t *data);
// void ctrl_process(uint8_t *data);
void feedback_process(uint8_t *data);
void keyboard_process(uint8_t *data);
void keyboard_feedback_send(void);
uint8_t move_state_switch(uint8_t state);


#endif // LOWER_BOARD_TEST_COMMAND_H
