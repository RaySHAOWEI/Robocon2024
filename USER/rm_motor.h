//
// Created by Ray on 2023/11/24.
//

#ifndef INC_2024RC_B_R1_RM_MOTOR_H
#define INC_2024RC_B_R1_RM_MOTOR_H

#include "main.h"

// ����������ģʽ
typedef enum
{
    MOTO_OFF,
    CURRENT_MODE,
    SPEED_CONTROL_MODE,
    POSITION_CONTROL_MODE,
    SPEED_TARQUE_CONTROL_MODE,
    POSITION_TORQUE_MODE,
    POSITION_SPEED_LIMIT_MODE,
    HOMEING_MODE
} Motor_Mode_e;

typedef enum
{
    CAN_ALL_ID = 0x200,
    M1_ID = 0x201,
    M2_ID = 0x202,
    M3_ID = 0x203,
    M4_ID = 0x204,
    CAN_OTHER_ID = 0x1FF,
    M5_ID = 0x205,
    M6_ID = 0x206,
    M7_ID = 0x207
} Can_Msg_Id_e;

/**
 * @brief ����ģʽ�ṹ�塣˵ʵ�ڣ����ģʽ���ٶ�ת��ûʲô���ֱ�����ٶ�ת�ؼ���
 * 20231109���������ĳ�У׼ģʽ����������һ�¡�
 */
typedef struct
{
    float Vel;             // ������ٶȣ�����������ת��ת������ʵ��������á�
    int16_t TARGET_TORQUE; // ����Ŀ��ת�أ��õ�����ʾ
    int done_flag;         // ����ɹ���־λ
    int32_t cnt;
} HOMING_MODE_TYPE;

/**
 * @brief  �������  M3508��M2006��M6020
 */
typedef enum
{
    NONE,
    M_3508,
    M_2006,
    M_6020,
} MotorType_TypeDef;

typedef struct
{
    int16_t MotorMaxCurrent; // ���������
    Motor_Mode_e Motor_Mode; // ���ģʽ
    MotorType_TypeDef Motor_Type;

    uint16_t ANGLE;         // ����ת�ӽǶ�
    int16_t RPM;            // ʵ��ת��ת��
    int16_t CURRENT;        // ʵ��ת�ص���
    int16_t TARGET_CURRENT; // Ŀ��ת�ص���

    int16_t TARGET_POS;    // Ŀ��Ƕ�(λ��)
    int16_t TARGET_TORQUE; // Ŀ��ת�أ��õ�����ʾ����ʱ����Ϊ����ֵ��
    float TARGET_RPM;      // Ŀ��ת��

    HOMING_MODE_TYPE HomingMode; // �������ģʽ

    // �ǶȻ���ʱ�õ��������
    float REAL_ANGLE;                  // ���������ʵ�Ƕȣ�������float��
    uint8_t FIRST_ANGLE_INTEGRAL_FLAG; // ��һ�λ��ֱ�־λ
    uint16_t LAST_ANGLE;               // ��һ�εĽǶ�

    int once_flag;     // ֻת��һȦ��־λ
    int current_limit; // ������־λ 0 Ϊ���� 1 Ϊ��������Ĭ��������
} MOTOR_REAL_INFO;

float Max_Value_Limit(float Value, float Limit);
void get_motor_measure(CAN_RxHeaderTypeDef *msg, uint8_t Data[8], CAN_HandleTypeDef *hcan);
void RM_MOTOR_Angle_Integral(MOTOR_REAL_INFO *RM_MOTOR);
void M3508_Send_Currents(void);
void Motor_Control(void);

int Speed_Control(MOTOR_REAL_INFO *RM_MOTOR, float Target_RPM);
int Vel_Torque_Control(MOTOR_REAL_INFO *MOTO_REAL_INFO, uint16_t Target_Torque, float Target_Vel);
int Position_Control(MOTOR_REAL_INFO *MOTOR_REAL_INFO, float Target_Pos);
int Pos_Torque_Control(MOTOR_REAL_INFO *MOTO_REAL_INFO, uint16_t Target_Torque, float Target_Pos);
int Pos_Velimit_Control(MOTOR_REAL_INFO *MOTO_REAL_INFO, float Target_Vel, float Target_Pos);
void Homeing_Mode(MOTOR_REAL_INFO *RM_MOTOR, float homeing_vel, int16_t homeing_torque);

// M3508���صĵ����ʵ��Ϣ
extern MOTOR_REAL_INFO can1motorRealInfo[7]; // can1
extern MOTOR_REAL_INFO can2motorRealInfo[7]; // can2

extern PID_T can1MOTOR_PID_RPM[7]; // �ٶ�pid��Ϣ
extern PID_T can1MOTOR_PID_POS[7]; // λ��pid��Ϣ

extern PID_T can2MOTOR_PID_RPM[7]; // �ٶ�pid��Ϣ
extern PID_T can2MOTOR_PID_POS[7]; // λ��pid��Ϣ

#endif // INC_2024RC_B_R1_RM_MOTOR_H
