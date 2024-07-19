//
// Created by Ray on 2023/11/24.
//

#include "rm_motor.h"

/**
 * @brief ��ֵ���ƺ���
 * @param ����ֵ
 * @param ����ֵ(��ֵ)
 * @return ���ֵ
 */
float Max_Value_Limit(float Value, float Limit)
{
    if (Value > Limit)
        Value = Limit;
    if (Value < -Limit)
        Value = -Limit;
    return Value;
}

MOTOR_REAL_INFO can1motorRealInfo[7] = {0};
MOTOR_REAL_INFO can2motorRealInfo[7] = {0};

PID_T can1MOTOR_PID_RPM[7] = {0}; // �ٶ�pid��Ϣ
PID_T can1MOTOR_PID_POS[7] = {0}; // λ��pid��Ϣ

PID_T can2MOTOR_PID_RPM[7] = {0}; // �ٶ�pid��Ϣ
PID_T can2MOTOR_PID_POS[7] = {0}; // λ��pid��Ϣ

/**
 * @brief Get the motor measure object
 *
 * @param msg CAN����ͷ
 * @param Data ����
 * @param hcan can1����can2
 */
void get_motor_measure(CAN_RxHeaderTypeDef *msg, uint8_t Data[8], CAN_HandleTypeDef *hcan)
{
    if (hcan == &hcan1) // ���̵��can
    {
        switch (msg->StdId) // ����׼ID
        {
        case M1_ID:
        {
            can1motorRealInfo[0].ANGLE = (uint16_t)((Data[0] << 8) | Data[1]);   // ת�ӻ�е�Ƕ�
            can1motorRealInfo[0].RPM = (uint16_t)((Data[2] << 8) | Data[3]);     // ʵ��ת��ת��
            can1motorRealInfo[0].CURRENT = (uint16_t)((Data[4] << 8) | Data[5]); // ʵ��ת�ص���
        };
        break;

        case M2_ID:
        {
            can1motorRealInfo[1].ANGLE = (uint16_t)((Data[0] << 8) | Data[1]);   // ת�ӻ�е�Ƕ�
            can1motorRealInfo[1].RPM = (uint16_t)((Data[2] << 8) | Data[3]);     // ʵ��ת��ת��
            can1motorRealInfo[1].CURRENT = (uint16_t)((Data[4] << 8) | Data[5]); // ʵ��ת�ص���
        };
        break;

        case M3_ID:
        {
            can1motorRealInfo[2].ANGLE = (uint16_t)((Data[0] << 8) | Data[1]);   // ת�ӻ�е�Ƕ�
            can1motorRealInfo[2].RPM = (uint16_t)((Data[2] << 8) | Data[3]);     // ʵ��ת��ת��
            can1motorRealInfo[2].CURRENT = (uint16_t)((Data[4] << 8) | Data[5]); // ʵ��ת�ص���
        };
        break;

        case M4_ID:
        {
            can1motorRealInfo[3].ANGLE = (uint16_t)((Data[0] << 8) | Data[1]);   // ת�ӻ�е�Ƕ�
            can1motorRealInfo[3].RPM = (uint16_t)((Data[2] << 8) | Data[3]);     // ʵ��ת��ת��
            can1motorRealInfo[3].CURRENT = (uint16_t)((Data[4] << 8) | Data[5]); // ʵ��ת�ص���
        };
        break;

        case M5_ID:
        {
            can1motorRealInfo[4].ANGLE = (uint16_t)((Data[0] << 8) | Data[1]);   // ת�ӻ�е�Ƕ�
            can1motorRealInfo[4].RPM = (uint16_t)((Data[2] << 8) | Data[3]);     // ʵ��ת��ת��
            can1motorRealInfo[4].CURRENT = (uint16_t)((Data[4] << 8) | Data[5]); // ʵ��ת�ص���
        };
        break;

        case M6_ID:
        {
            can1motorRealInfo[5].ANGLE = (uint16_t)((Data[0] << 8) | Data[1]);   // ת�ӻ�е�Ƕ�
            can1motorRealInfo[5].RPM = (uint16_t)((Data[2] << 8) | Data[3]);     // ʵ��ת��ת��
            can1motorRealInfo[5].CURRENT = (uint16_t)((Data[4] << 8) | Data[5]); // ʵ��ת�ص���
        };
        break;

        case M7_ID:
        {
            can1motorRealInfo[6].ANGLE = (uint16_t)((Data[0] << 8) | Data[1]);   // ת�ӻ�е�Ƕ�
            can1motorRealInfo[6].RPM = (uint16_t)((Data[2] << 8) | Data[3]);     // ʵ��ת��ת��
            can1motorRealInfo[6].CURRENT = (uint16_t)((Data[4] << 8) | Data[5]); // ʵ��ת�ص���
        };
        break;

        default:
            break;
        }
    }
    if (hcan == &hcan2) // ���̵��can2
    {
        switch (msg->StdId) // ����׼ID
        {
        case M1_ID:
        {
            can2motorRealInfo[0].ANGLE = (uint16_t)((Data[0] << 8) | Data[1]);   // ת�ӻ�е�Ƕ�
            can2motorRealInfo[0].RPM = (uint16_t)((Data[2] << 8) | Data[3]);     // ʵ��ת��ת��
            can2motorRealInfo[0].CURRENT = (uint16_t)((Data[4] << 8) | Data[5]); // ʵ��ת�ص���
        };
        break;

        case M2_ID:
        {
            can2motorRealInfo[1].ANGLE = (uint16_t)((Data[0] << 8) | Data[1]);   // ת�ӻ�е�Ƕ�
            can2motorRealInfo[1].RPM = (uint16_t)((Data[2] << 8) | Data[3]);     // ʵ��ת��ת��
            can2motorRealInfo[1].CURRENT = (uint16_t)((Data[4] << 8) | Data[5]); // ʵ��ת�ص���
        };
        break;

        case M3_ID:
        {
            can2motorRealInfo[2].ANGLE = (uint16_t)((Data[0] << 8) | Data[1]);   // ת�ӻ�е�Ƕ�
            can2motorRealInfo[2].RPM = (uint16_t)((Data[2] << 8) | Data[3]);     // ʵ��ת��ת��
            can2motorRealInfo[2].CURRENT = (uint16_t)((Data[4] << 8) | Data[5]); // ʵ��ת�ص���
        };
        break;

        case M4_ID:
        {
            can2motorRealInfo[3].ANGLE = (uint16_t)((Data[0] << 8) | Data[1]);   // ת�ӻ�е�Ƕ�
            can2motorRealInfo[3].RPM = (uint16_t)((Data[2] << 8) | Data[3]);     // ʵ��ת��ת��
            can2motorRealInfo[3].CURRENT = (uint16_t)((Data[4] << 8) | Data[5]); // ʵ��ת�ص���
        };
        break;

        case M5_ID:
        {
            can2motorRealInfo[4].ANGLE = (uint16_t)((Data[0] << 8) | Data[1]);   // ת�ӻ�е�Ƕ�
            can2motorRealInfo[4].RPM = (uint16_t)((Data[2] << 8) | Data[3]);     // ʵ��ת��ת��
            can2motorRealInfo[4].CURRENT = (uint16_t)((Data[4] << 8) | Data[5]); // ʵ��ת�ص���
        };
        break;

        case M6_ID:
        {
            can2motorRealInfo[5].ANGLE = (uint16_t)((Data[0] << 8) | Data[1]);   // ת�ӻ�е�Ƕ�
            can2motorRealInfo[5].RPM = (uint16_t)((Data[2] << 8) | Data[3]);     // ʵ��ת��ת��
            can2motorRealInfo[5].CURRENT = (uint16_t)((Data[4] << 8) | Data[5]); // ʵ��ת�ص���
        };
        break;

        case M7_ID:
        {
            can2motorRealInfo[6].ANGLE = (uint16_t)((Data[0] << 8) | Data[1]);   // ת�ӻ�е�Ƕ�
            can2motorRealInfo[6].RPM = (uint16_t)((Data[2] << 8) | Data[3]);     // ʵ��ת��ת��
            can2motorRealInfo[6].CURRENT = (uint16_t)((Data[4] << 8) | Data[5]); // ʵ��ת�ص���
        };
        break;

        default:
            break;
        }
    }
}

/**
 * @brief M3508�ǶȻ���
 * @param ����ṹ��
 * @return NULL
 */
void RM_MOTOR_Angle_Integral(MOTOR_REAL_INFO *RM_MOTOR)
{
    float Delta_Pos = 0;
    float Deceleration_P = 0;

    // ��¼��һ�ν���ʱ������
    if (!RM_MOTOR->FIRST_ANGLE_INTEGRAL_FLAG)
    {
        RM_MOTOR->LAST_ANGLE = RM_MOTOR->ANGLE;
        RM_MOTOR->FIRST_ANGLE_INTEGRAL_FLAG = 1;
        return;
    }

    switch (RM_MOTOR->Motor_Type)
    {
    case M_3508:
        Deceleration_P = 19.0f;
        break;

    case M_2006:
        Deceleration_P = 36.0f;
        break;

    case M_6020:
        Deceleration_P = 1.0f;
        break;

    default:
        break;
    }

    // ����Ƕȱ仯
    if (RM_MOTOR->RPM > 0)
    {
        if (RM_MOTOR->ANGLE < RM_MOTOR->LAST_ANGLE)
        {
            if (ABS(8191 + RM_MOTOR->ANGLE - RM_MOTOR->LAST_ANGLE < 1250))
            {
                Delta_Pos = ((float)(8191 + RM_MOTOR->ANGLE - RM_MOTOR->LAST_ANGLE) / 8192.0f) * 360.0f;
                Delta_Pos = Delta_Pos / Deceleration_P; // ���ٱ�
            }
        }
        else if (RM_MOTOR->ANGLE > RM_MOTOR->LAST_ANGLE)
        {
            Delta_Pos = ((float)(RM_MOTOR->ANGLE - RM_MOTOR->LAST_ANGLE) / 8192.0f) * 360.0f;
            Delta_Pos = Delta_Pos / Deceleration_P; // ���ٱ�
        }
        if (Delta_Pos >= 0)
        {
            RM_MOTOR->REAL_ANGLE += Delta_Pos; // ����
        }
    }
    else if (RM_MOTOR->RPM < 0)
    {
        if (RM_MOTOR->ANGLE > RM_MOTOR->LAST_ANGLE)
        {
            if (ABS(8191 - RM_MOTOR->ANGLE + RM_MOTOR->LAST_ANGLE) < 1250) // ��������CAN����ʱ�������ת���ǶȽ����˲�
            {
                Delta_Pos = ((float)(8191 - RM_MOTOR->ANGLE + RM_MOTOR->LAST_ANGLE) / 8192.0f) * 360.0f;
                Delta_Pos = Delta_Pos / Deceleration_P; // ���ٱ�
            }
        }
        else if (RM_MOTOR->ANGLE < RM_MOTOR->LAST_ANGLE)
        {
            Delta_Pos = ((float)(RM_MOTOR->ANGLE - RM_MOTOR->LAST_ANGLE) / 8192.0f) * 360.0f;
            Delta_Pos = Delta_Pos / Deceleration_P; // ���ٱ�
        }
        if (Delta_Pos <= 0)
        {
            RM_MOTOR->REAL_ANGLE += Delta_Pos; // ����
        }
    }
    else
    {
        if (RM_MOTOR->ANGLE != RM_MOTOR->LAST_ANGLE)
        {
            Delta_Pos = ((float)(RM_MOTOR->ANGLE - RM_MOTOR->LAST_ANGLE) / 8192.0f) * 360.0f;
            Delta_Pos = Delta_Pos / Deceleration_P; // ���ٱ�
        }
        else
        {
            Delta_Pos = 0;
        }
        RM_MOTOR->REAL_ANGLE += Delta_Pos; // ����
    }

    // �洢�Ƕ�ֵ
    RM_MOTOR->LAST_ANGLE = RM_MOTOR->ANGLE;
}

/**
 * @brief ���͵������
 * ͬʱ��can1��can2���͵������/��ѹֵ
 * @param NULL
 * @return NULL
 */
void M3508_Send_Currents(void)
{
    CAN_TxHeaderTypeDef TxHeader1;
    CAN_TxHeaderTypeDef TxHeader2;
    uint8_t TxData[8];
    uint8_t TxData2[8];
    uint8_t can2TxData[8];
    uint8_t can2TxData2[8];
    uint32_t Send_Mail_Box;
    uint32_t Send_Mail_Box2;
    uint32_t can2Send_Mail_Box;
    uint32_t can2Send_Mail_Box2;

    // ���ÿ��ƶ�
    TxHeader1.IDE = CAN_ID_STD;
    TxHeader1.RTR = CAN_RTR_DATA;
    TxHeader1.DLC = 0x08;
    // �����ٲöκ����ݶ�
    TxHeader1.StdId = CAN_ALL_ID; // 0x200

    // //���ÿ��ƶ�
    TxHeader2.IDE = CAN_ID_STD;
    TxHeader2.RTR = CAN_RTR_DATA;
    TxHeader2.DLC = 0x08;
    // �����ٲöκ����ݶ�
    TxHeader2.StdId = CAN_OTHER_ID; // 0x1FF

    // can1
    TxData[0] = (uint8_t)(can1motorRealInfo[0].TARGET_CURRENT >> 8); // 0x201
    TxData[1] = (uint8_t)can1motorRealInfo[0].TARGET_CURRENT;

    TxData[2] = (uint8_t)(can1motorRealInfo[1].TARGET_CURRENT >> 8); // 0x202
    TxData[3] = (uint8_t)can1motorRealInfo[1].TARGET_CURRENT;

    TxData[4] = (uint8_t)(can1motorRealInfo[2].TARGET_CURRENT >> 8); // 0x203
    TxData[5] = (uint8_t)can1motorRealInfo[2].TARGET_CURRENT;

    TxData[6] = (uint8_t)(can1motorRealInfo[3].TARGET_CURRENT >> 8); // 0x204
    TxData[7] = (uint8_t)can1motorRealInfo[3].TARGET_CURRENT;

    TxData2[0] = (uint8_t)(can1motorRealInfo[4].TARGET_CURRENT >> 8); // 0x205
    TxData2[1] = (uint8_t)can1motorRealInfo[4].TARGET_CURRENT;

    TxData2[2] = (uint8_t)(can1motorRealInfo[5].TARGET_CURRENT >> 8); // 0x206
    TxData2[3] = (uint8_t)can1motorRealInfo[5].TARGET_CURRENT;

    TxData2[4] = (uint8_t)(can1motorRealInfo[6].TARGET_CURRENT >> 8); // 0x207
    TxData2[5] = (uint8_t)can1motorRealInfo[6].TARGET_CURRENT;

    HAL_CAN_AddTxMessage(&hcan1, &TxHeader1, TxData, &Send_Mail_Box);
    HAL_CAN_AddTxMessage(&hcan1, &TxHeader2, TxData2, &Send_Mail_Box2);

    // can2
    can2TxData[0] = (uint8_t)(can2motorRealInfo[0].TARGET_CURRENT >> 8); // 0x201
    can2TxData[1] = (uint8_t)can2motorRealInfo[0].TARGET_CURRENT;

    can2TxData[2] = (uint8_t)(can2motorRealInfo[1].TARGET_CURRENT >> 8); // 0x202
    can2TxData[3] = (uint8_t)can2motorRealInfo[1].TARGET_CURRENT;

    can2TxData[4] = (uint8_t)(can2motorRealInfo[2].TARGET_CURRENT >> 8); // 0x203
    can2TxData[5] = (uint8_t)can2motorRealInfo[2].TARGET_CURRENT;

    can2TxData[6] = (uint8_t)(can2motorRealInfo[3].TARGET_CURRENT >> 8); // 0x204
    can2TxData[7] = (uint8_t)can2motorRealInfo[3].TARGET_CURRENT;

    can2TxData2[0] = (uint8_t)(can2motorRealInfo[4].TARGET_CURRENT >> 8); // 0x205
    can2TxData2[1] = (uint8_t)can2motorRealInfo[4].TARGET_CURRENT;

    can2TxData2[2] = (uint8_t)(can2motorRealInfo[5].TARGET_CURRENT >> 8); // 0x206
    can2TxData2[3] = (uint8_t)can2motorRealInfo[5].TARGET_CURRENT;

    can2TxData2[4] = (uint8_t)(can2motorRealInfo[6].TARGET_CURRENT >> 8); // 0x207
    can2TxData2[5] = (uint8_t)can2motorRealInfo[6].TARGET_CURRENT;

    // can2TxData2[0] = (uint8_t)(5000 >> 8);//0x205
    // can2TxData2[1] = (uint8_t)5000;

    HAL_CAN_AddTxMessage(&hcan2, &TxHeader1, can2TxData, &can2Send_Mail_Box);
    HAL_CAN_AddTxMessage(&hcan2, &TxHeader2, can2TxData2, &can2Send_Mail_Box2);
}

/**
 * @brief �������ģʽ
 * @param NULL
 * @return NULL
 *
 *
 */
void Motor_Control(void)
{
    for (int i = 0; i < 7; i++)
    {
        // can1���ģʽѡ��
        switch (can1motorRealInfo[i].Motor_Mode)
        {
        case MOTO_OFF: // ����ر�
        {
            can1motorRealInfo[i].TARGET_CURRENT = 0.0f; // ������ֵ
            break;
        }

        case CURRENT_MODE: // ����ģʽ(ֱ�Ӹ�����ֵ)
        {
            break;
        }

        case SPEED_CONTROL_MODE: // �ٶ�ģʽ
        {
            pid_calc(&can1MOTOR_PID_RPM[i], can1motorRealInfo[i].TARGET_RPM, can1motorRealInfo[i].RPM); // �ٶȻ�
            break;
        }

        case POSITION_CONTROL_MODE: // λ��ģʽ
        {
            if (can1motorRealInfo[i].once_flag == 1)
            {                                                                                                 // ��Ȧģʽλ�û�����е��װʱע��Ƕȣ�����8192-0�����䣩
                pid_calc(&can1MOTOR_PID_POS[i], can1motorRealInfo[i].TARGET_POS, can1motorRealInfo[i].ANGLE); // λ�û�
                can1motorRealInfo[i].TARGET_RPM = can1MOTOR_PID_POS[i].output;
                pid_calc(&can1MOTOR_PID_RPM[i], can1motorRealInfo[i].TARGET_RPM, can1motorRealInfo[i].RPM); // �ٶȻ�
            }
            else
            {
                pid_calc(&can1MOTOR_PID_POS[i], can1motorRealInfo[i].TARGET_POS, can1motorRealInfo[i].REAL_ANGLE); // λ�û�
                can1motorRealInfo[i].TARGET_RPM = can1MOTOR_PID_POS[i].output;
                pid_calc(&can1MOTOR_PID_RPM[i], can1motorRealInfo[i].TARGET_RPM, can1motorRealInfo[i].RPM); // �ٶȻ�
            }
            break;
        }

        case SPEED_TARQUE_CONTROL_MODE: // �ٶ�ת��ģʽ
        {
            if (ABS(can1motorRealInfo[i].CURRENT) < can1motorRealInfo[i].TARGET_TORQUE)
                can1motorRealInfo[i].stop_cnt = 0;
            if (ABS(can1motorRealInfo[i].CURRENT) >= can1motorRealInfo[i].TARGET_TORQUE || ABS(can1motorRealInfo[i].CURRENT) >= can1motorRealInfo[i].MotorMaxCurrent)
                can1motorRealInfo[i].stop_cnt++;
            if (can1motorRealInfo[i].stop_cnt >= 30) // ����30��
            {
                can1motorRealInfo[i].stop_flag = 1; // ��־λ��1
            }
            else if (can1motorRealInfo[i].stop_cnt < 30)
            {
                can1motorRealInfo[i].stop_flag = 0; // ��־λ��0
            }
            pid_calc(&can1MOTOR_PID_RPM[i], can1motorRealInfo[i].TARGET_RPM, can1motorRealInfo[i].RPM);                     // �ٶȻ�
            can1MOTOR_PID_RPM[i].output = Max_Value_Limit(can1MOTOR_PID_RPM[i].output, can1motorRealInfo[i].TARGET_TORQUE); // ����ת��ģʽʱ����ֵ
            break;
        }

        case POSITION_TORQUE_MODE: // λ��ת��ģʽ
        {
            pid_calc(&can1MOTOR_PID_POS[i], can1motorRealInfo[i].TARGET_POS, can1motorRealInfo[i].REAL_ANGLE); // λ�û�
            can1motorRealInfo[i].TARGET_RPM = can1MOTOR_PID_POS[i].output;
            pid_calc(&can1MOTOR_PID_RPM[i], can1motorRealInfo[i].TARGET_RPM, can1motorRealInfo[i].RPM);                     // �ٶȻ�
            can1MOTOR_PID_RPM[i].output = Max_Value_Limit(can1MOTOR_PID_RPM[i].output, can1motorRealInfo[i].TARGET_TORQUE); // ����ת��ģʽʱ����ֵ
            break;
        }

        case POSITION_SPEED_LIMIT_MODE: // λ������ģʽ
        {
            pid_calc(&can1MOTOR_PID_POS[i], can1motorRealInfo[i].TARGET_POS, can1motorRealInfo[i].REAL_ANGLE); // λ�û�
            can1motorRealInfo[i].TARGET_RPM = Max_Value_Limit(can1MOTOR_PID_POS[i].output, can1motorRealInfo[i].TARGET_TORQUE);
            pid_calc(&can1MOTOR_PID_RPM[i], can1motorRealInfo[i].TARGET_RPM, can1motorRealInfo[i].RPM); // �ٶȻ�
            break;
        }

        case HOMEING_MODE: // �����ٶ�ģʽ���л��㣬���ͣתһ��ʱ���ǶȻ������㡣
        {
            if (ABS(can1motorRealInfo[i].CURRENT) <= 1.5 * can1motorRealInfo[i].HomingMode.TARGET_TORQUE)
                can1motorRealInfo[i].HomingMode.cnt = 0;
            if (ABS(can1motorRealInfo[i].CURRENT) > 1.5 * can1motorRealInfo[i].HomingMode.TARGET_TORQUE || ABS(can1motorRealInfo[i].CURRENT) >= can1motorRealInfo[i].MotorMaxCurrent)
                can1motorRealInfo[i].HomingMode.cnt++;
            if (can1motorRealInfo[i].HomingMode.cnt >= 30) // ����30��
            {
                can1motorRealInfo[i].HomingMode.done_flag = 1; // ��־λ��һ������ʹ�����ģʽ��ʱ��Ӹ��жϣ��жϸñ�־λ��1��ʱ���л���������ģʽ��
                can1motorRealInfo[i].REAL_ANGLE = 0.0f;
                can1motorRealInfo[i].TARGET_RPM = 0.0f;
            }
            pid_calc(&can1MOTOR_PID_RPM[i], can1motorRealInfo[i].TARGET_RPM, can1motorRealInfo[i].RPM); // �ٶȻ�
            break;
        }

        case SETTING_MODE: // ����ģʽ
        {
            if (ABS(can1motorRealInfo[i].CURRENT) <= 1.5 * can1motorRealInfo[i].SettingMode.TARGET_TORQUE)
                can1motorRealInfo[i].SettingMode.cnt = 0;
            if (ABS(can1motorRealInfo[i].CURRENT) > 1.5 * can1motorRealInfo[i].SettingMode.TARGET_TORQUE || ABS(can1motorRealInfo[i].CURRENT) >= can1motorRealInfo[i].MotorMaxCurrent)
                can1motorRealInfo[i].SettingMode.cnt++;
            if (can1motorRealInfo[i].SettingMode.cnt >= 30) // ����30��
            {
                can1motorRealInfo[i].SettingMode.done_flag = 1; // ��־λ��һ������ʹ�����ģʽ��ʱ��Ӹ��жϣ��жϸñ�־λ��1��ʱ���л���������ģʽ��
                can1motorRealInfo[i].REAL_ANGLE = can1motorRealInfo[i].SettingMode.SETTING_ANGLE;
                can1motorRealInfo[i].TARGET_RPM = 0.0f;
            }
            pid_calc(&can1MOTOR_PID_RPM[i], can1motorRealInfo[i].TARGET_RPM, can1motorRealInfo[i].RPM); // �ٶȻ�
            break;
        }

        case VELOCITY_PLANNING_MODE: // ����ģʽ
        {
            Velocity_Planning(&can1motorRealInfo[i]);
            pid_calc(&can1MOTOR_PID_RPM[i], can1motorRealInfo[i].TARGET_RPM, can1motorRealInfo[i].RPM);
            break;
        }

        default:
            break;
        }

        // can2���ģʽѡ��
        switch (can2motorRealInfo[i].Motor_Mode)
        {
        case MOTO_OFF: // ����ر�
        {
            can2motorRealInfo[i].TARGET_CURRENT = 0.0f; // ������ֵ
            break;
        }

        case CURRENT_MODE: // ����ģʽ(ֱ�Ӹ�����ֵ)
        {
            break;
        }

        case SPEED_CONTROL_MODE: // �ٶ�ģʽ
        {
            pid_calc(&can2MOTOR_PID_RPM[i], can2motorRealInfo[i].TARGET_RPM, can2motorRealInfo[i].RPM); // �ٶȻ�
            break;
        }

        case POSITION_CONTROL_MODE: // λ��ģʽ
        {
            if (can2motorRealInfo[i].once_flag == 1)
            {                                                                                                 // ��Ȧģʽλ�û�����е��װʱע��Ƕȣ�����8192-0�����䣩
                pid_calc(&can2MOTOR_PID_POS[i], can2motorRealInfo[i].TARGET_POS, can2motorRealInfo[i].ANGLE); // λ�û�
                can2motorRealInfo[i].TARGET_RPM = can2MOTOR_PID_POS[i].output;
                pid_calc(&can2MOTOR_PID_RPM[i], can2motorRealInfo[i].TARGET_RPM, can2motorRealInfo[i].RPM); // �ٶȻ�
            }
            else
            {
                pid_calc(&can2MOTOR_PID_POS[i], can2motorRealInfo[i].TARGET_POS, can2motorRealInfo[i].REAL_ANGLE); // λ�û�
                can2motorRealInfo[i].TARGET_RPM = can2MOTOR_PID_POS[i].output;
                pid_calc(&can2MOTOR_PID_RPM[i], can2motorRealInfo[i].TARGET_RPM, can2motorRealInfo[i].RPM); // �ٶȻ�
            }
            break;
        }

        case SPEED_TARQUE_CONTROL_MODE: // �ٶ�ת��ģʽ
        {
            if (ABS(can2motorRealInfo[i].CURRENT) < can2motorRealInfo[i].TARGET_TORQUE)
                can2motorRealInfo[i].stop_cnt = 0;
            if (ABS(can2motorRealInfo[i].CURRENT) >= can2motorRealInfo[i].TARGET_TORQUE || ABS(can2motorRealInfo[i].CURRENT) >= can2motorRealInfo[i].MotorMaxCurrent)
                can2motorRealInfo[i].stop_cnt++;
            if (can2motorRealInfo[i].stop_cnt >= 30) // ����30��
            {
                can2motorRealInfo[i].stop_flag = 1; // ��־λ��1
            }
            else if (can2motorRealInfo[i].stop_cnt < 30)
            {
                can2motorRealInfo[i].stop_flag = 0; // ��־λ��0
            }
            pid_calc(&can2MOTOR_PID_RPM[i], can2motorRealInfo[i].TARGET_RPM, can2motorRealInfo[i].RPM);                     // �ٶȻ�
            can2MOTOR_PID_RPM[i].output = Max_Value_Limit(can2MOTOR_PID_RPM[i].output, can2motorRealInfo[i].TARGET_TORQUE); // ����ת��ģʽʱ����ֵ
            break;
        }

        case POSITION_TORQUE_MODE: // λ��ת��ģʽ
        {
            pid_calc(&can2MOTOR_PID_POS[i], can2motorRealInfo[i].TARGET_POS, can2motorRealInfo[i].REAL_ANGLE); // λ�û�
            can2motorRealInfo[i].TARGET_RPM = can2MOTOR_PID_POS[i].output;
            pid_calc(&can2MOTOR_PID_RPM[i], can2motorRealInfo[i].TARGET_RPM, can2motorRealInfo[i].RPM);                     // �ٶȻ�
            can2MOTOR_PID_RPM[i].output = Max_Value_Limit(can2MOTOR_PID_RPM[i].output, can2motorRealInfo[i].TARGET_TORQUE); // ����ת��ģʽʱ����ֵ
            break;
        }

        case POSITION_SPEED_LIMIT_MODE: // λ������ģʽ
        {
            pid_calc(&can2MOTOR_PID_POS[i], can2motorRealInfo[i].TARGET_POS, can2motorRealInfo[i].REAL_ANGLE); // λ�û�
            can2motorRealInfo[i].TARGET_RPM = Max_Value_Limit(can2MOTOR_PID_POS[i].output, can2motorRealInfo[i].TARGET_TORQUE);
            pid_calc(&can2MOTOR_PID_RPM[i], can2motorRealInfo[i].TARGET_RPM, can2motorRealInfo[i].RPM); // �ٶȻ�
            break;
        }

        case HOMEING_MODE: // �����ٶ�ת��ģʽ��Сת�ؽ��л��㣬���ͣתһ��ʱ���ǶȻ������㡣
        {
            if (ABS(can2motorRealInfo[i].CURRENT) <= 1.5 * can2motorRealInfo[i].HomingMode.TARGET_TORQUE)
                can2motorRealInfo[i].HomingMode.cnt = 0;
            if (ABS(can2motorRealInfo[i].CURRENT) > 1.5 * can2motorRealInfo[i].HomingMode.TARGET_TORQUE || ABS(can2motorRealInfo[i].CURRENT) >= can2motorRealInfo[i].MotorMaxCurrent)
                can2motorRealInfo[i].HomingMode.cnt++;
            if (can2motorRealInfo[i].HomingMode.cnt >= 30) // ����30��
            {
                can2motorRealInfo[i].HomingMode.done_flag = 1; // ��־λ��һ������ʹ�����ģʽ��ʱ��Ӹ��жϣ��жϸñ�־λ��1��ʱ���л���������ģʽ��
                can2motorRealInfo[i].REAL_ANGLE = 0.0f;
                can2motorRealInfo[i].TARGET_RPM = 0.0f;
            }
            pid_calc(&can2MOTOR_PID_RPM[i], can2motorRealInfo[i].TARGET_RPM, can2motorRealInfo[i].RPM); // �ٶȻ�
            break;
        }

        case SETTING_MODE: // ����ģʽ
        {
            if (ABS(can2motorRealInfo[i].CURRENT) <= 1.5 * can2motorRealInfo[i].SettingMode.TARGET_TORQUE)
                can2motorRealInfo[i].SettingMode.cnt = 0;
            if (ABS(can2motorRealInfo[i].CURRENT) > 1.5 * can2motorRealInfo[i].SettingMode.TARGET_TORQUE || ABS(can2motorRealInfo[i].CURRENT) >= can2motorRealInfo[i].MotorMaxCurrent)
                can2motorRealInfo[i].SettingMode.cnt++;
            if (can2motorRealInfo[i].SettingMode.cnt >= 30) // ����30��
            {
                can2motorRealInfo[i].SettingMode.done_flag = 1; // ��־λ��һ������ʹ�����ģʽ��ʱ��Ӹ��жϣ��жϸñ�־λ��1��ʱ���л���������ģʽ��
                can2motorRealInfo[i].REAL_ANGLE = can2motorRealInfo[i].SettingMode.SETTING_ANGLE;
                can2motorRealInfo[i].TARGET_RPM = 0.0f;
            }
            pid_calc(&can2MOTOR_PID_RPM[i], can2motorRealInfo[i].TARGET_RPM, can2motorRealInfo[i].RPM); // �ٶȻ�
            break;
        }

        case VELOCITY_PLANNING_MODE: // ����ģʽ
        {
            Velocity_Planning(&can2motorRealInfo[i]);
            pid_calc(&can2MOTOR_PID_RPM[i], can2motorRealInfo[i].TARGET_RPM, can2motorRealInfo[i].RPM);
            break;
        }

        default:
            break;
        }
    }

    // can1���ת������
    for (int i = 0; i < 7; i++)
    {
        if (can1motorRealInfo[i].Motor_Mode == CURRENT_MODE || can1motorRealInfo[i].Motor_Mode == MOTO_OFF) // ��ֹѡ���ģʽȴ�޷��ж�
        {
        } // ����ģʽ�µ��������
        else
        {
            if (can1motorRealInfo[i].Motor_Type == M_3508)
            {
                can1motorRealInfo[i].MotorMaxCurrent = 10650; // 10A * 1.3 / 20 * 16384
                if (can1motorRealInfo[i].current_limit == 1)  // ������
                {
                    can1motorRealInfo[i].TARGET_CURRENT = can1MOTOR_PID_RPM[i].output;
                }
                else
                {
                    can1motorRealInfo[i].TARGET_CURRENT = Max_Value_Limit(can1MOTOR_PID_RPM[i].output, can1motorRealInfo[i].MotorMaxCurrent);
                }
            }
            else if (can1motorRealInfo[i].Motor_Type == M_2006)
            {
                can1motorRealInfo[i].MotorMaxCurrent = 3195; // 3A * 1.3 / 20 * 16384
                if (can1motorRealInfo[i].current_limit == 1) // ������
                {
                    can1motorRealInfo[i].TARGET_CURRENT = can1MOTOR_PID_RPM[i].output;
                }
                else
                {
                    can1motorRealInfo[i].TARGET_CURRENT = Max_Value_Limit(can1MOTOR_PID_RPM[i].output, can1motorRealInfo[i].MotorMaxCurrent);
                }
            }
            else if (can1motorRealInfo[i].Motor_Type == M_6020)
            {
                can1motorRealInfo[i].MotorMaxCurrent = 25000; // ��ֵΪ����ֵ�����25000���Դ�����
                if (can1motorRealInfo[i].current_limit == 1)  // ������
                {
                    can1motorRealInfo[i].TARGET_CURRENT = can1MOTOR_PID_RPM[i].output;
                }
                else
                {
                    can1motorRealInfo[i].TARGET_CURRENT = Max_Value_Limit(can1MOTOR_PID_RPM[i].output, can1motorRealInfo[i].MotorMaxCurrent);
                }
            }
            else
            {
                can1motorRealInfo[i].TARGET_CURRENT = 0;
            }
        }
    }

    // can2���ת������
    for (int i = 0; i < 7; i++)
    {
        if (can2motorRealInfo[i].Motor_Mode == CURRENT_MODE || can2motorRealInfo[i].Motor_Mode == MOTO_OFF) // ��ֹѡ���ģʽȴ�޷��ж�
        {

        } // ����ģʽ�µ��������
        else
        {
            if (can2motorRealInfo[i].Motor_Type == M_3508)
            {
                can2motorRealInfo[i].MotorMaxCurrent = 10650; // 10A * 1.3 / 20 * 16384
                if (can2motorRealInfo[i].current_limit == 1)  // ������
                {
                    can2motorRealInfo[i].TARGET_CURRENT = can2MOTOR_PID_RPM[i].output;
                }
                else
                {
                    can2motorRealInfo[i].TARGET_CURRENT = Max_Value_Limit(can2MOTOR_PID_RPM[i].output, can2motorRealInfo[i].MotorMaxCurrent);
                }
            }
            else if (can2motorRealInfo[i].Motor_Type == M_2006)
            {
                can2motorRealInfo[i].MotorMaxCurrent = 3195; // 3A * 1.3 / 20 * 16384
                if (can2motorRealInfo[i].current_limit == 1) // ������
                {
                    can2motorRealInfo[i].TARGET_CURRENT = can2MOTOR_PID_RPM[i].output;
                }
                else
                {
                    can2motorRealInfo[i].TARGET_CURRENT = Max_Value_Limit(can2MOTOR_PID_RPM[i].output, can2motorRealInfo[i].MotorMaxCurrent);
                }
            }
            else if (can2motorRealInfo[i].Motor_Type == M_6020)
            {
                can2motorRealInfo[i].MotorMaxCurrent = 25000; // ��ֵΪ��ѹֵ�����25000���Դ�����
                if (can2motorRealInfo[i].current_limit == 1)  // ������
                {
                    can2motorRealInfo[i].TARGET_CURRENT = can2MOTOR_PID_RPM[i].output;
                }
                else
                {
                    can2motorRealInfo[i].TARGET_CURRENT = Max_Value_Limit(can2MOTOR_PID_RPM[i].output, can2motorRealInfo[i].MotorMaxCurrent);
                }
            }
            else
            {
                can2motorRealInfo[i].TARGET_CURRENT = 0;
            }
        }
    }

    // ʹ�ܵ�������͵�������
    M3508_Send_Currents();
}

/**
 * @brief �ٶ�ģʽ
 * @param Ŀ��ת��
 * @return ����Ŀ��ת�ٷ���1�����򷵻�0
 */
int Speed_Control(MOTOR_REAL_INFO *RM_MOTOR, float Target_RPM)
{
    RM_MOTOR->Motor_Mode = SPEED_CONTROL_MODE;
    RM_MOTOR->TARGET_RPM = Target_RPM;
    if (ABS(RM_MOTOR->RPM - Target_RPM) <= 130)
        return 1;
    else
        return 0;
}

/**
 * @brief  �ٶ�ת�ؿ��ƺ���,������Ҫ�ı�����ת����ôֱ�Ӹı�Target_Vel��ֵ����
 * @param  target_torqueĿ��ת��,�õ�����ʾ���������ͣ�
 * @param  target_velĿ��λ�ã�������������ת��
 * @retval ��������1����ת����0
 */
int Vel_Torque_Control(MOTOR_REAL_INFO *MOTO_REAL_INFO, uint16_t Target_Torque, float Target_Vel)
{
    MOTO_REAL_INFO->Motor_Mode = SPEED_TARQUE_CONTROL_MODE;
    MOTO_REAL_INFO->TARGET_RPM = Target_Vel;
    MOTO_REAL_INFO->TARGET_TORQUE = Target_Torque;
    return MOTO_REAL_INFO->stop_flag;
}

/**
 * @brief  λ�ÿ���(��λ�û�����)
 * @param  target_posĿ��λ��
 * @return ����Ŀ��λ�÷���1�����򷵻�0
 */
int Position_Control(MOTOR_REAL_INFO *MOTO_REAL_INFO, float target_pos)
{
    MOTO_REAL_INFO->Motor_Mode = POSITION_CONTROL_MODE;
    MOTO_REAL_INFO->TARGET_POS = target_pos;
    if (ABS(MOTO_REAL_INFO->REAL_ANGLE - target_pos) < 5)
        return 1;
    else
        return 0;
}

/**
 * @brief  λ��ת��ģʽ
 * @param  ����ṹ��
 * @param  target_torque Ŀ��ת�أ��õ�����ʾ
 * @param  target_pos Ŀ��λ��
 * @retval ����Ŀ��λ�÷���1�����򷵻�0
 */
int Pos_Torque_Control(MOTOR_REAL_INFO *MOTO_REAL_INFO, uint16_t Target_Torque, float Target_Pos)
{
    MOTO_REAL_INFO->Motor_Mode = POSITION_TORQUE_MODE;
    MOTO_REAL_INFO->TARGET_POS = Target_Pos;
    MOTO_REAL_INFO->TARGET_TORQUE = Target_Torque;
    if (ABS(MOTO_REAL_INFO->REAL_ANGLE - Target_Pos) < 5)
        return 1;
    else
        return 0;
}

/**
 * @brief λ������ģʽ
 * ���޶����ٶȵ���ָ��λ��
 * @param ����ṹ��
 * @param Target_Vel Ŀ���ٶ�
 * @param Target_Pos Ŀ��λ��
 * @return int
 */
int Pos_Velimit_Control(MOTOR_REAL_INFO *MOTO_REAL_INFO, float Target_Vel, float Target_Pos)
{
    MOTO_REAL_INFO->Motor_Mode = POSITION_SPEED_LIMIT_MODE;
    MOTO_REAL_INFO->TARGET_POS = Target_Pos;
    MOTO_REAL_INFO->TARGET_TORQUE = Target_Vel;
    if (ABS(MOTO_REAL_INFO->REAL_ANGLE - Target_Pos) < 5)
        return 1;
    else
        return 0;
}

/**
 * @brief ����У׼ģʽ
 *
 * @param RM_MOTOR
 * @param homeing_vel
 */
void Homeing_Mode(MOTOR_REAL_INFO *RM_MOTOR, float homeing_vel, int16_t homeing_torque)
{
    RM_MOTOR->Motor_Mode = HOMEING_MODE;
    // �ڲ���ֵ
    RM_MOTOR->HomingMode.Vel = homeing_vel;
    RM_MOTOR->HomingMode.TARGET_TORQUE = homeing_torque;
    // ��ֵ���ⲿ
    RM_MOTOR->TARGET_RPM = RM_MOTOR->HomingMode.Vel;
}
/**
 * @brief ����ģʽ
 *
 * @param RM_MOTOR
 * @param homeing_vel
 */
void Setting_Mode(MOTOR_REAL_INFO *RM_MOTOR, float vel, int16_t torque, float angle)
{
    RM_MOTOR->Motor_Mode = SETTING_MODE;
    // �ڲ���ֵ
    RM_MOTOR->SettingMode.SETTING_ANGLE = angle;
    RM_MOTOR->SettingMode.TARGET_TORQUE = torque;
    RM_MOTOR->SettingMode.Vel = vel;
    // ��ֵ���ⲿ
    RM_MOTOR->TARGET_RPM = RM_MOTOR->SettingMode.Vel;
}

/**
 * @brief  �����ٶȹ滮�Ĳ����������ٶȹ滮����
 * @param
 * @param float Pstart;        //��ʼλ��
 * @param float Pend;          //����λ��
 * @param float Vstart;        //��ʼ���ٶ�  ��λ��RPM ����ֵ
 * @param float Vmax;          //�����ٶ�
 * @param float Vend;          //ĩβ���ٶ�
 * @param float Rac;           //����·�̵ı���
 * @param float Rde;           //����·�̵ı���
 * @retval NULL
 */
void Planning_Mode(MOTOR_REAL_INFO *M3508_MOTOR, float Pstart, float Pend, float Vstart, float Vmax, float Vend, float Rac, float Rde)
{
    M3508_MOTOR->Motor_Mode = VELOCITY_PLANNING_MODE; // ����ģʽ
    M3508_MOTOR->Velocity_Planning.Pstart = Pstart;
    M3508_MOTOR->Velocity_Planning.Pend = Pend;
    M3508_MOTOR->Velocity_Planning.Vstart = Vstart;
    M3508_MOTOR->Velocity_Planning.Vmax = Vmax;
    M3508_MOTOR->Velocity_Planning.Vend = Vend;
    M3508_MOTOR->Velocity_Planning.Rac = Rac;
    M3508_MOTOR->Velocity_Planning.Rde = Rde;
}

/**
 * @brief �ݶ��ٶȹ滮
 * @param M����ṹ��
 * @return NULL
 */
void Velocity_Planning(MOTOR_REAL_INFO *M3508_MOTOR)
{
    // ��ʽ��(V1^2 - V2^2) = 2ax
    // static int cnt;//��ʱ��
    float Ssu; // ��·��
    float Sac; // ����·��
    float Sde; // ����·��
    float Sco; // ����·��
    float Aac; // ���ټ��ٶ�
    float Ade; // ���ټ��ٶ�
    float S;   // ��ǰ·��

    // �����������������ִ���ٶȹ滮
    if ((M3508_MOTOR->Velocity_Planning.Rac > 1) || (M3508_MOTOR->Velocity_Planning.Rac < 0) || // ����·�̵ı���
        (M3508_MOTOR->Velocity_Planning.Rde > 1) || (M3508_MOTOR->Velocity_Planning.Rde < 0) || // ����·�̵ı���
        (M3508_MOTOR->Velocity_Planning.Vmax < M3508_MOTOR->Velocity_Planning.Vstart))          // �����ٶ�<��ʼ���ٶ�
    {
        M3508_MOTOR->TARGET_RPM = 0; // ���צ���˶�
        return;
    }
    // ����ģʽ
    if (M3508_MOTOR->Velocity_Planning.Pstart == M3508_MOTOR->Velocity_Planning.Pend) // ��ʼλ��=����λ��
    {
        M3508_MOTOR->TARGET_RPM = M3508_MOTOR->Velocity_Planning.Vmax; // �����ٶ�
        return;
    }

    // ����һЩ����
    Ssu = ABS(M3508_MOTOR->Velocity_Planning.Pend - M3508_MOTOR->Velocity_Planning.Pstart);                                                                                           // ��·��
    Sac = Ssu * M3508_MOTOR->Velocity_Planning.Rac;                                                                                                                                   // ����·�� =	��·�� * ����·�̵ı���
    Sde = Ssu * M3508_MOTOR->Velocity_Planning.Rde;                                                                                                                                   // ����·�� =	��·�� * ����·�̵ı���
    Sco = Ssu - Sac - Sde;                                                                                                                                                            // ����·�� = ��·�� - ����·�� - ����·��
    Aac = (M3508_MOTOR->Velocity_Planning.Vmax * M3508_MOTOR->Velocity_Planning.Vmax - M3508_MOTOR->Velocity_Planning.Vstart * M3508_MOTOR->Velocity_Planning.Vstart) / (2.0f * Sac); // ���ټ��ٶ� (�����ٶ�*�����ٶ� - ��ʼ���ٶ� *��ʼ���ٶ� ) / (2.0f * ����·��)
    Ade = (M3508_MOTOR->Velocity_Planning.Vend * M3508_MOTOR->Velocity_Planning.Vend - M3508_MOTOR->Velocity_Planning.Vmax * M3508_MOTOR->Velocity_Planning.Vmax) / (2.0f * Sde);     // ���ټ��ٶ� (ĩ�ٶ�*ĩ�ٶ� - ����ٶ� * ����ٶ� ) / (2.0f * ����·��)

    // �����쳣���
    if (((M3508_MOTOR->Velocity_Planning.Pend > M3508_MOTOR->Velocity_Planning.Pstart) && (M3508_MOTOR->REAL_ANGLE < M3508_MOTOR->Velocity_Planning.Pstart)) || //[(����λ�� > ��ʼλ��) && (���������ʵ�Ƕ�pos <��ʼλ��)]	||
        ((M3508_MOTOR->Velocity_Planning.Pend < M3508_MOTOR->Velocity_Planning.Pstart) && (M3508_MOTOR->REAL_ANGLE > M3508_MOTOR->Velocity_Planning.Pstart)))   //	[(����λ�� < ��ʼλ��) && (���������ʵ�Ƕ�pos >��ʼλ��)]
    {
        M3508_MOTOR->TARGET_RPM = M3508_MOTOR->Velocity_Planning.Vstart; // TARGET_RPM = ��ʼ���ٶ�
    }
    else if (((M3508_MOTOR->Velocity_Planning.Pend > M3508_MOTOR->Velocity_Planning.Pstart) && (M3508_MOTOR->REAL_ANGLE > M3508_MOTOR->Velocity_Planning.Pend)) || //[(����λ�� > ��ʼλ��) && (���������ʵ�Ƕ�pos > ����λ��)]	||
             ((M3508_MOTOR->Velocity_Planning.Pend < M3508_MOTOR->Velocity_Planning.Pstart) && (M3508_MOTOR->REAL_ANGLE < M3508_MOTOR->Velocity_Planning.Pend)))   //	[(����λ�� < ��ʼλ��) && (���������ʵ�Ƕ�pos < ����λ��)]
    {
        M3508_MOTOR->TARGET_RPM = M3508_MOTOR->Velocity_Planning.Vstart; // TARGET_RPM = ĩβ���ٶ�
    }
    else
    {
        S = ABS(M3508_MOTOR->REAL_ANGLE - M3508_MOTOR->Velocity_Planning.Pstart); // ��ʼλ��

        // �滮RPM
        if (S < Sac)
            M3508_MOTOR->TARGET_RPM = sqrt(2.0f * Aac * S + M3508_MOTOR->Velocity_Planning.Vstart * M3508_MOTOR->Velocity_Planning.Vstart); // ���ٽ׶�
        else if (S < (Sac + Sco))
            M3508_MOTOR->TARGET_RPM = M3508_MOTOR->Velocity_Planning.Vmax; // ���ٽ׶�
        else
            M3508_MOTOR->TARGET_RPM = sqrt(M3508_MOTOR->Velocity_Planning.Vend * M3508_MOTOR->Velocity_Planning.Vend - 2.0f * Ade * ABS(Ssu - S)); // ���ٽ׶�
    }

    // ������ʵ�������
    if (M3508_MOTOR->Velocity_Planning.Pend < M3508_MOTOR->Velocity_Planning.Pstart)
        M3508_MOTOR->TARGET_RPM = -M3508_MOTOR->TARGET_RPM;
    // �ж��Ƿ����
    if ((fabsf(M3508_MOTOR->REAL_ANGLE - M3508_MOTOR->Velocity_Planning.Pend)) < 5)
    {
        M3508_MOTOR->Velocity_Planning.done_flag = 1; // ���ñ�־λ
        M3508_MOTOR->TARGET_RPM = 0;
    }

    if ((fabsf(M3508_MOTOR->REAL_ANGLE - M3508_MOTOR->Velocity_Planning.Pend)) > 5)
    {
        M3508_MOTOR->Velocity_Planning.done_flag = 0;
    }
}
