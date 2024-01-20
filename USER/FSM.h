//
// Created by Ray on 2023/11/24.
//

#ifndef INC_2024RC_B_R1_FSM_H
#define INC_2024RC_B_R1_FSM_H

#include "main.h"

typedef enum {
    ROBOT_STATE_INIT,           //������ʼ��
    ROBOT_STATE_CALIBRATION,    //У׼
    ROBOT_STATE_AUTO_CTRL,      //����Ӧ״̬
    ROBOT_STATE_SEED_CTRL,      //�������
    ROBOT_STATE_SHOOT_CTRL      //�������
}ROBOT_STATE_ITEMS;//����״̬

typedef enum {
    CHASSIS_DISABLE,            //����ֹͣ
    CHASSIS_STATE_HIGH_SPEED,   //����ģʽ
    CHASSIS_STATE_LOW_SPEED     //����ģʽ
}CHASSIS_STATE_ITEMS;//����״̬

typedef enum {
    SEED_STATE_DISABLE,         //����״̬
    SEED_STATE_INIT,            //���������ʼ��
    SEED_STATE_PEEK,       //���������ȡ
    SEED_STATE_PUT              //����������磨ֻ��һ�ߣ�
}SEED_STATE_ITEMS;//�������״̬

typedef enum {
    SHOOT_STATE_INIT,           //���������ʼ��
    SHOOT_STATE_LOAD,           //�������װ��
    SHOOT_STATE_SHOOTING        //�����������
}SHOOT_STATE_ITEMS;//�������״̬

extern ROBOT_STATE_ITEMS robot_state;
extern CHASSIS_STATE_ITEMS chassis_state;
extern SEED_STATE_ITEMS seed_state;
extern SHOOT_STATE_ITEMS shoot_state;

//������������ṹ��
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
