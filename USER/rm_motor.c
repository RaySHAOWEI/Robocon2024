//
// Created by Ray on 2023/11/24.
//

#include "rm_motor.h"

/**
 * @brief 最值限制函数
 * @param 传入值
 * @param 限制值(最值)
 * @return 输出值
*/
float Max_Value_Limit(float Value, float Limit)
{
    if(Value > Limit) Value = Limit;
    if(Value < -Limit) Value = -Limit;
    return Value;
}

MOTOR_REAL_INFO can1motorRealInfo[7] = {0};
MOTOR_REAL_INFO can2motorRealInfo[7] = {0};

PID_T can1MOTOR_PID_RPM[7] = {0}; //速度pid信息
PID_T can1MOTOR_PID_POS[7] = {0};	//位置pid信息

PID_T can2MOTOR_PID_RPM[7] = {0}; //速度pid信息
PID_T can2MOTOR_PID_POS[7] = {0};	//位置pid信息

/**
 * @brief Get the motor measure object
 *
 * @param msg CAN配置头
 * @param Data 数据
 * @param hcan can1还是can2
 */
void get_motor_measure(CAN_RxHeaderTypeDef *msg, uint8_t Data[8], CAN_HandleTypeDef *hcan)
{
    if(hcan == &hcan1)//底盘电机can
    {
        switch(msg -> StdId)  // 检测标准ID
        {
            case M1_ID:
            {
                can1motorRealInfo[0].ANGLE   = (uint16_t)((Data[0] << 8) |  Data[1]);  // 转子机械角度
                can1motorRealInfo[0].RPM     = (uint16_t)((Data[2] << 8) |  Data[3]);  // 实际转子转速
                can1motorRealInfo[0].CURRENT = (uint16_t)((Data[4] << 8) |  Data[5]);  // 实际转矩电流
            }; break;

            case M2_ID:
            {
                can1motorRealInfo[1].ANGLE   = (uint16_t)((Data[0] << 8) |  Data[1]);  // 转子机械角度
                can1motorRealInfo[1].RPM     = (uint16_t)((Data[2] << 8) |  Data[3]);  // 实际转子转速
                can1motorRealInfo[1].CURRENT = (uint16_t)((Data[4] << 8) |  Data[5]);  // 实际转矩电流
            }; break;

            case M3_ID:
            {
                can1motorRealInfo[2].ANGLE   = (uint16_t)((Data[0] << 8) |  Data[1]);  // 转子机械角度
                can1motorRealInfo[2].RPM     = (uint16_t)((Data[2] << 8) |  Data[3]);  // 实际转子转速
                can1motorRealInfo[2].CURRENT = (uint16_t)((Data[4] << 8) |  Data[5]);  // 实际转矩电流
            }; break;

            case M4_ID:
            {
                can1motorRealInfo[3].ANGLE   = (uint16_t)((Data[0] << 8) |  Data[1]);  // 转子机械角度
                can1motorRealInfo[3].RPM     = (uint16_t)((Data[2] << 8) |  Data[3]);  // 实际转子转速
                can1motorRealInfo[3].CURRENT = (uint16_t)((Data[4] << 8) |  Data[5]);  // 实际转矩电流
            }; break;

            case M5_ID:
            {
                can1motorRealInfo[4].ANGLE   = (uint16_t)((Data[0] << 8) |  Data[1]);  // 转子机械角度
                can1motorRealInfo[4].RPM     = (uint16_t)((Data[2] << 8) |  Data[3]);  // 实际转子转速
                can1motorRealInfo[4].CURRENT = (uint16_t)((Data[4] << 8) |  Data[5]);  // 实际转矩电流
            }; break;

            case M6_ID:
            {
                can1motorRealInfo[5].ANGLE   = (uint16_t)((Data[0] << 8) |  Data[1]);  // 转子机械角度
                can1motorRealInfo[5].RPM     = (uint16_t)((Data[2] << 8) |  Data[3]);  // 实际转子转速
                can1motorRealInfo[5].CURRENT = (uint16_t)((Data[4] << 8) |  Data[5]);  // 实际转矩电流
            }; break;

            case M7_ID:
            {
                can1motorRealInfo[6].ANGLE   = (uint16_t)((Data[0] << 8) |  Data[1]);  // 转子机械角度
                can1motorRealInfo[6].RPM     = (uint16_t)((Data[2] << 8) |  Data[3]);  // 实际转子转速
                can1motorRealInfo[6].CURRENT = (uint16_t)((Data[4] << 8) |  Data[5]);  // 实际转矩电流
            }; break;

            default: break;
        }
    }
    if(hcan == &hcan2)//底盘电机can2
    {
        switch(msg -> StdId)  // 检测标准ID
        {
            case M1_ID:
            {
                can2motorRealInfo[0].ANGLE   = (uint16_t)((Data[0] << 8) |  Data[1]);  // 转子机械角度
                can2motorRealInfo[0].RPM     = (uint16_t)((Data[2] << 8) |  Data[3]);  // 实际转子转速
                can2motorRealInfo[0].CURRENT = (uint16_t)((Data[4] << 8) |  Data[5]);  // 实际转矩电流
            }; break;

            case M2_ID:
            {
                can2motorRealInfo[1].ANGLE   = (uint16_t)((Data[0] << 8) |  Data[1]);  // 转子机械角度
                can2motorRealInfo[1].RPM     = (uint16_t)((Data[2] << 8) |  Data[3]);  // 实际转子转速
                can2motorRealInfo[1].CURRENT = (uint16_t)((Data[4] << 8) |  Data[5]);  // 实际转矩电流
            }; break;

            case M3_ID:
            {
                can2motorRealInfo[2].ANGLE   = (uint16_t)((Data[0] << 8) |  Data[1]);  // 转子机械角度
                can2motorRealInfo[2].RPM     = (uint16_t)((Data[2] << 8) |  Data[3]);  // 实际转子转速
                can2motorRealInfo[2].CURRENT = (uint16_t)((Data[4] << 8) |  Data[5]);  // 实际转矩电流
            }; break;

            case M4_ID:
            {
                can2motorRealInfo[3].ANGLE   = (uint16_t)((Data[0] << 8) |  Data[1]);  // 转子机械角度
                can2motorRealInfo[3].RPM     = (uint16_t)((Data[2] << 8) |  Data[3]);  // 实际转子转速
                can2motorRealInfo[3].CURRENT = (uint16_t)((Data[4] << 8) |  Data[5]);  // 实际转矩电流
            }; break;

            case M5_ID:
            {
                can2motorRealInfo[4].ANGLE   = (uint16_t)((Data[0] << 8) |  Data[1]);  // 转子机械角度
                can2motorRealInfo[4].RPM     = (uint16_t)((Data[2] << 8) |  Data[3]);  // 实际转子转速
                can2motorRealInfo[4].CURRENT = (uint16_t)((Data[4] << 8) |  Data[5]);  // 实际转矩电流
            }; break;

            case M6_ID:
            {
                can2motorRealInfo[5].ANGLE   = (uint16_t)((Data[0] << 8) |  Data[1]);  // 转子机械角度
                can2motorRealInfo[5].RPM     = (uint16_t)((Data[2] << 8) |  Data[3]);  // 实际转子转速
                can2motorRealInfo[5].CURRENT = (uint16_t)((Data[4] << 8) |  Data[5]);  // 实际转矩电流
            }; break;

            case M7_ID:
            {
                can2motorRealInfo[6].ANGLE   = (uint16_t)((Data[0] << 8) |  Data[1]);  // 转子机械角度
                can2motorRealInfo[6].RPM     = (uint16_t)((Data[2] << 8) |  Data[3]);  // 实际转子转速
                can2motorRealInfo[6].CURRENT = (uint16_t)((Data[4] << 8) |  Data[5]);  // 实际转矩电流
            }; break;

            default: break;
        }
    }
}


/**
 * @brief M3508角度积分
 * @param 电机结构体
 * @return NULL
*/
void RM_MOTOR_Angle_Integral(MOTOR_REAL_INFO* RM_MOTOR)
{
    float Delta_Pos = 0;
    float Deceleration_P = 0;

    //记录第一次进入时的数据
    if(!RM_MOTOR->FIRST_ANGLE_INTEGRAL_FLAG)
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

    //计算角度变化
    if (RM_MOTOR->RPM > 0)
    {
        if(RM_MOTOR->ANGLE < RM_MOTOR->LAST_ANGLE)
        {
            if (ABS(8191 + RM_MOTOR->ANGLE - RM_MOTOR->LAST_ANGLE < 1250))
            {
                Delta_Pos = ((float)(8191 + RM_MOTOR->ANGLE - RM_MOTOR->LAST_ANGLE)/8192.0f) * 360.0f;
                Delta_Pos = Delta_Pos / Deceleration_P;	//减速比
            }
        }
        else if(RM_MOTOR->ANGLE > RM_MOTOR->LAST_ANGLE)
        {
            Delta_Pos = ((float)(RM_MOTOR->ANGLE - RM_MOTOR->LAST_ANGLE) / 8192.0f) * 360.0f;
            Delta_Pos = Delta_Pos / Deceleration_P;	//减速比
        }
        if(Delta_Pos >= 0)
        {
            RM_MOTOR->REAL_ANGLE += Delta_Pos;  // 积分
        }
    }
    else if (RM_MOTOR->RPM < 0)
    {
        if(RM_MOTOR->ANGLE > RM_MOTOR->LAST_ANGLE)
        {
            if(ABS(8191 - RM_MOTOR->ANGLE + RM_MOTOR->LAST_ANGLE) < 1250)  // 利用两次CAN接收时间电机最大转动角度进行滤波
            {
                Delta_Pos = ((float)(8191 - RM_MOTOR->ANGLE + RM_MOTOR->LAST_ANGLE) / 8192.0f) * 360.0f;
                Delta_Pos = Delta_Pos /Deceleration_P;	//减速比
            }
        }
        else if(RM_MOTOR->ANGLE < RM_MOTOR->LAST_ANGLE)
        {
            Delta_Pos = ((float)(RM_MOTOR->ANGLE - RM_MOTOR->LAST_ANGLE) / 8192.0f) * 360.0f;
            Delta_Pos = Delta_Pos / Deceleration_P;	//减速比
        }
        if(Delta_Pos <= 0)
		{
            RM_MOTOR->REAL_ANGLE += Delta_Pos;  // 积分
		}
    }
	else 
	{
        if(RM_MOTOR->ANGLE != RM_MOTOR->LAST_ANGLE)
        {
            Delta_Pos = ((float)(RM_MOTOR->ANGLE - RM_MOTOR->LAST_ANGLE) / 8192.0f) * 360.0f;
            Delta_Pos = Delta_Pos / Deceleration_P;	//减速比
        }
        else
        {
            Delta_Pos = 0;
        }
        RM_MOTOR->REAL_ANGLE += Delta_Pos;  // 积分
	}

    // 存储角度值
    RM_MOTOR->LAST_ANGLE = RM_MOTOR->ANGLE;
}

/**
 * @brief 发送电机数据
 * 同时向can1和can2发送电机电流/电压值
 * @param NULL
 * @return NULL
*/
void M3508_Send_Currents(void)
{
    CAN_TxHeaderTypeDef	TxHeader1;
    CAN_TxHeaderTypeDef	TxHeader2;
    uint8_t TxData[8];
    uint8_t TxData2[8];
    uint8_t can2TxData[8];
    uint8_t can2TxData2[8];
    uint32_t Send_Mail_Box;
    uint32_t Send_Mail_Box2;
    uint32_t can2Send_Mail_Box;
    uint32_t can2Send_Mail_Box2;

    //配置控制端
    TxHeader1.IDE = CAN_ID_STD;
    TxHeader1.RTR = CAN_RTR_DATA;
    TxHeader1.DLC = 0x08;
    //配置仲裁段和数据段
    TxHeader1.StdId = CAN_ALL_ID;//0x200

    // //配置控制端
    TxHeader2.IDE = CAN_ID_STD;
    TxHeader2.RTR = CAN_RTR_DATA;
    TxHeader2.DLC = 0x08;
    //配置仲裁段和数据段
    TxHeader2.StdId = CAN_OTHER_ID;//0x1FF

    //can1
    TxData[0] = (uint8_t)(can1motorRealInfo[0].TARGET_CURRENT >> 8);//0x201
    TxData[1] = (uint8_t) can1motorRealInfo[0].TARGET_CURRENT;

    TxData[2] = (uint8_t)(can1motorRealInfo[1].TARGET_CURRENT >> 8);//0x202
    TxData[3] = (uint8_t) can1motorRealInfo[1].TARGET_CURRENT;

    TxData[4] = (uint8_t)(can1motorRealInfo[2].TARGET_CURRENT >> 8);//0x203
    TxData[5] = (uint8_t) can1motorRealInfo[2].TARGET_CURRENT;

    TxData[6] = (uint8_t)(can1motorRealInfo[3].TARGET_CURRENT >> 8);//0x204
    TxData[7] = (uint8_t) can1motorRealInfo[3].TARGET_CURRENT;

    TxData2[0] = (uint8_t)(can1motorRealInfo[4].TARGET_CURRENT >> 8);//0x205
    TxData2[1] = (uint8_t) can1motorRealInfo[4].TARGET_CURRENT;

    TxData2[2] = (uint8_t)(can1motorRealInfo[5].TARGET_CURRENT >> 8);//0x206
    TxData2[3] = (uint8_t) can1motorRealInfo[5].TARGET_CURRENT;

    TxData2[4] = (uint8_t)(can1motorRealInfo[6].TARGET_CURRENT >> 8);//0x207
    TxData2[5] = (uint8_t) can1motorRealInfo[6].TARGET_CURRENT;

    HAL_CAN_AddTxMessage(&hcan1, &TxHeader1, TxData, &Send_Mail_Box);
    HAL_CAN_AddTxMessage(&hcan1, &TxHeader2, TxData2, &Send_Mail_Box2);


    //can2
    can2TxData[0] = (uint8_t)(can2motorRealInfo[0].TARGET_CURRENT >> 8);//0x201
    can2TxData[1] = (uint8_t) can2motorRealInfo[0].TARGET_CURRENT;

    can2TxData[2] = (uint8_t)(can2motorRealInfo[1].TARGET_CURRENT >> 8);//0x202
    can2TxData[3] = (uint8_t) can2motorRealInfo[1].TARGET_CURRENT;

    can2TxData[4] = (uint8_t)(can2motorRealInfo[2].TARGET_CURRENT >> 8);//0x203
    can2TxData[5] = (uint8_t) can2motorRealInfo[2].TARGET_CURRENT;

    can2TxData[6] = (uint8_t)(can2motorRealInfo[3].TARGET_CURRENT >> 8);//0x204
    can2TxData[7] = (uint8_t) can2motorRealInfo[3].TARGET_CURRENT;

    can2TxData2[0] = (uint8_t)(can2motorRealInfo[4].TARGET_CURRENT >> 8);//0x205
    can2TxData2[1] = (uint8_t) can2motorRealInfo[4].TARGET_CURRENT;

    can2TxData2[2] = (uint8_t)(can2motorRealInfo[5].TARGET_CURRENT >> 8);//0x206
    can2TxData2[3] = (uint8_t) can2motorRealInfo[5].TARGET_CURRENT;

    can2TxData2[4] = (uint8_t)(can2motorRealInfo[6].TARGET_CURRENT >> 8);//0x207
    can2TxData2[5] = (uint8_t) can2motorRealInfo[6].TARGET_CURRENT;

    // can2TxData2[0] = (uint8_t)(5000 >> 8);//0x205
    // can2TxData2[1] = (uint8_t)5000;

    HAL_CAN_AddTxMessage(&hcan2, &TxHeader1, can2TxData, &can2Send_Mail_Box);
    HAL_CAN_AddTxMessage(&hcan2, &TxHeader2, can2TxData2, &can2Send_Mail_Box2);
}

/**
 * @brief 电机控制模式
 * @param NULL
 * @return NULL
 *
 * 
*/
void Motor_Control(void)
{
    for(int i = 0; i < 7; i++)
    {
        //can1电机模式选择
        switch (can1motorRealInfo[i].Motor_Mode)
        {
            case MOTO_OFF://电机关闭
            {
                can1motorRealInfo[i].TARGET_CURRENT = 0.0f;//电流赋值
                break;
            }

            case CURRENT_MODE: //电流模式(直接赋电流值)
            {
                break;
            }

            case SPEED_CONTROL_MODE: //速度模式
            {
                pid_calc(&can1MOTOR_PID_RPM[i],can1motorRealInfo[i].TARGET_RPM,can1motorRealInfo[i].RPM);//速度环
                break;
            }

            case POSITION_CONTROL_MODE://位置模式
            {
                if (can1motorRealInfo[i].once_flag == 1){//单圈模式位置环（机械安装时注意角度，避免8192-0的跳变）
                    pid_calc(&can1MOTOR_PID_POS[i],can1motorRealInfo[i].TARGET_POS, can1motorRealInfo[i].ANGLE);//位置环
                    can1motorRealInfo[i].TARGET_RPM = can1MOTOR_PID_POS[i].output;
                    pid_calc(&can1MOTOR_PID_RPM[i], can1motorRealInfo[i].TARGET_RPM, can1motorRealInfo[i].RPM);//速度环
                }
                pid_calc(&can1MOTOR_PID_POS[i],can1motorRealInfo[i].TARGET_POS, can1motorRealInfo[i].REAL_ANGLE);//位置环
                can1motorRealInfo[i].TARGET_RPM = can1MOTOR_PID_POS[i].output;
                pid_calc(&can1MOTOR_PID_RPM[i], can1motorRealInfo[i].TARGET_RPM, can1motorRealInfo[i].RPM);//速度环
                break;
            }

            case SPEED_TARQUE_CONTROL_MODE://速度转矩模式
            {
                pid_calc(&can1MOTOR_PID_RPM[i],can1motorRealInfo[i].TARGET_RPM,can1motorRealInfo[i].RPM);	//速度环
                can1MOTOR_PID_RPM[i].output = Max_Value_Limit(can1MOTOR_PID_RPM[i].output,can1motorRealInfo[i].TARGET_TORQUE);	//限制转矩模式时电流值
                break;
            }

            case POSITION_TORQUE_MODE://位置转矩模式
            {
                pid_calc(&can1MOTOR_PID_POS[i],can1motorRealInfo[i].TARGET_POS, can1motorRealInfo[i].REAL_ANGLE);//位置环
                can1motorRealInfo[i].TARGET_RPM = can1MOTOR_PID_POS[i].output;
                pid_calc(&can1MOTOR_PID_RPM[i], can1motorRealInfo[i].TARGET_RPM, can1motorRealInfo[i].RPM);//速度环
                can1MOTOR_PID_RPM[i].output = Max_Value_Limit(can1MOTOR_PID_RPM[i].output,can1motorRealInfo[i].TARGET_TORQUE);//限制转矩模式时电流值
                break;
            }

            case HOMEING_MODE://调用速度模式进行回零，检测停转一段时间后角度积分置零。
            {
                if(ABS(can1motorRealInfo[i].CURRENT) <= 1.5*can1motorRealInfo[i].HomingMode.TARGET_TORQUE)   can1motorRealInfo[i].HomingMode.cnt = 0;
                if(ABS(can1motorRealInfo[i].CURRENT) > 1.5*can1motorRealInfo[i].HomingMode.TARGET_TORQUE || ABS(can1motorRealInfo[i].CURRENT) >= can1motorRealInfo[i].MotorMaxCurrent)   can1motorRealInfo[i].HomingMode.cnt++;
                if(can1motorRealInfo[i].HomingMode.cnt >= 30) //计数30次
                {
                    can1motorRealInfo[i].HomingMode.done_flag=1;//标志位置一，建议使用这个模式的时候加个判断，判断该标志位是1的时候切换其他控制模式。
                    can1motorRealInfo[i].REAL_ANGLE=0.0f;
                    can1motorRealInfo[i].TARGET_RPM=0.0f;
                }
                pid_calc(&can1MOTOR_PID_RPM[i],can1motorRealInfo[i].TARGET_RPM,can1motorRealInfo[i].RPM);//速度环
                can1MOTOR_PID_RPM[i].output = Max_Value_Limit(can1MOTOR_PID_RPM[i].output,can1motorRealInfo[i].HomingMode.TARGET_TORQUE);	//限制电流值
                break;
            }

            default: break;
        }

        //can2电机模式选择
        switch (can2motorRealInfo[i].Motor_Mode)
        {
            case MOTO_OFF://电机关闭
            {
                can2motorRealInfo[i].TARGET_CURRENT = 0.0f;//电流赋值
                break;
            }

            case CURRENT_MODE: //电流模式(直接赋电流值)
            {
                break;
            }

            case SPEED_CONTROL_MODE: //速度模式
            {
                pid_calc(&can2MOTOR_PID_RPM[i],can2motorRealInfo[i].TARGET_RPM,can2motorRealInfo[i].RPM);//速度环
                break;
            }

            case POSITION_CONTROL_MODE://位置模式
            {
                if (can2motorRealInfo[i].once_flag == 1){//单圈模式位置环
                    pid_calc(&can2MOTOR_PID_POS[i],can2motorRealInfo[i].TARGET_POS, can2motorRealInfo[i].ANGLE );//位置环
                    can2motorRealInfo[i].TARGET_RPM = can2MOTOR_PID_POS[i].output;
                    pid_calc(&can2MOTOR_PID_RPM[i], can2motorRealInfo[i].TARGET_RPM, can2motorRealInfo[i].RPM);//速度环
                }
                pid_calc(&can2MOTOR_PID_POS[i],can2motorRealInfo[i].TARGET_POS, can2motorRealInfo[i].REAL_ANGLE);//位置环
                can2motorRealInfo[i].TARGET_RPM = can2MOTOR_PID_POS[i].output;
                pid_calc(&can2MOTOR_PID_RPM[i], can2motorRealInfo[i].TARGET_RPM, can2motorRealInfo[i].RPM);//速度环
                break;
            }

            case SPEED_TARQUE_CONTROL_MODE://速度转矩模式
            {
                pid_calc(&can2MOTOR_PID_RPM[i],can2motorRealInfo[i].TARGET_RPM,can2motorRealInfo[i].RPM);	//速度环
                can2MOTOR_PID_RPM[i].output = Max_Value_Limit(can2MOTOR_PID_RPM[i].output,can2motorRealInfo[i].TARGET_TORQUE);	//限制转矩模式时电流值
                break;
            }

            case POSITION_TORQUE_MODE://位置转矩模式
            {
                pid_calc(&can2MOTOR_PID_POS[i],can2motorRealInfo[i].TARGET_POS, can2motorRealInfo[i].REAL_ANGLE);//位置环
                can2motorRealInfo[i].TARGET_RPM = can2MOTOR_PID_POS[i].output;
                pid_calc(&can2MOTOR_PID_RPM[i], can2motorRealInfo[i].TARGET_RPM, can2motorRealInfo[i].RPM);//速度环
                can2MOTOR_PID_RPM[i].output = Max_Value_Limit(can2MOTOR_PID_RPM[i].output,can2motorRealInfo[i].TARGET_TORQUE);//限制转矩模式时电流值
                break;
            }

            case HOMEING_MODE://调用速度转矩模式，小转矩进行回零，检测停转一段时间后角度积分置零。
            {
                if(ABS(can2motorRealInfo[i].CURRENT) <= 1.5*can2motorRealInfo[i].HomingMode.TARGET_TORQUE)   can2motorRealInfo[i].HomingMode.cnt = 0;
                if(ABS(can2motorRealInfo[i].CURRENT) > 1.5*can2motorRealInfo[i].HomingMode.TARGET_TORQUE || ABS(can2motorRealInfo[i].CURRENT) >= can2motorRealInfo[i].MotorMaxCurrent)   can2motorRealInfo[i].HomingMode.cnt++;
                if(can2motorRealInfo[i].HomingMode.cnt >= 30) //计数30次
                {
                    can2motorRealInfo[i].HomingMode.done_flag=1;//标志位置一，建议使用这个模式的时候加个判断，判断该标志位是1的时候切换其他控制模式。
                    can2motorRealInfo[i].REAL_ANGLE=0.0f;
                    can2motorRealInfo[i].TARGET_RPM=0.0f;
                }
                pid_calc(&can2MOTOR_PID_RPM[i],can2motorRealInfo[i].TARGET_RPM,can2motorRealInfo[i].RPM);//速度环
                break;
            }

            default: break;
        }
    }

    //can1电机转动参数
    for(int i = 0; i < 7; i++)
    {
        if(can1motorRealInfo[i].Motor_Mode == CURRENT_MODE)//防止选择该模式却无法判断
        {
        }//电流模式下的特殊情况
        else
        {
            if (can1motorRealInfo[i].Motor_Type == M_3508)
            {
                can1motorRealInfo[i].MotorMaxCurrent = 10650;
                if(can1motorRealInfo[i].CURRENT > 13000){//10000mA * 1.3 = 13000
                    can1motorRealInfo[i].Motor_Mode = MOTO_OFF;
                }
                else {
                    can1motorRealInfo[i].TARGET_CURRENT = Max_Value_Limit(can1MOTOR_PID_RPM[i].output, 10650);	//10A * 1.3 / 20 * 16384
                }
            }
            else if (can1motorRealInfo[i].Motor_Type == M_2006)
            {
                can1motorRealInfo[i].MotorMaxCurrent = 3195;
                if(can1motorRealInfo[i].CURRENT > 3900){//3000mA * 1.3 = 3900
                    can1motorRealInfo[i].Motor_Mode = MOTO_OFF;
                }
                can1motorRealInfo[i].TARGET_CURRENT = Max_Value_Limit(can1MOTOR_PID_RPM[i].output, 3195);  	//3A * 1.3 / 20 * 16384
            }
            else if (can1motorRealInfo[i].Motor_Type == M_6020)                                     
            {
                can1motorRealInfo[i].MotorMaxCurrent = 30000;
                can1motorRealInfo[i].TARGET_CURRENT = Max_Value_Limit(can1MOTOR_PID_RPM[i].output, 30000);  	//赋值为电压值，最高30000，自带限流
            }
            else
            {
                can1motorRealInfo[i].TARGET_CURRENT = 0;
            }
        }
    }

    //can2电机转动参数
    for(int i = 0; i < 7; i++)
    {
        if(can2motorRealInfo[i].Motor_Mode == CURRENT_MODE)//防止选择该模式却无法判断
        {

        }//电流模式下的特殊情况
        else
        {
            if (can2motorRealInfo[i].Motor_Type == M_3508)
            {
                can2motorRealInfo[i].MotorMaxCurrent = 10650;
                if(can2motorRealInfo[i].CURRENT > 13000){//10000mA * 1.3 = 13000
                    can2motorRealInfo[i].Motor_Mode = MOTO_OFF;
                }
                else {
                    can2motorRealInfo[i].TARGET_CURRENT = Max_Value_Limit(can2MOTOR_PID_RPM[i].output, 10650);	//10A * 1.3 / 20 * 16384
                }
            }
            else if (can2motorRealInfo[i].Motor_Type == M_2006)
            {
                can2motorRealInfo[i].MotorMaxCurrent = 3195;
                if(can2motorRealInfo[i].CURRENT > 3900){//3000mA * 1.3 = 3900
                    can2motorRealInfo[i].Motor_Mode = MOTO_OFF;
                }
                can2motorRealInfo[i].TARGET_CURRENT = Max_Value_Limit(can2MOTOR_PID_RPM[i].output, 3195);  	//3A * 1.3 / 20 * 16384
            }
            else if (can2motorRealInfo[i].Motor_Type == M_6020)                                     
            {
                can2motorRealInfo[i].MotorMaxCurrent = 30000;
                can2motorRealInfo[i].TARGET_CURRENT = Max_Value_Limit(can2MOTOR_PID_RPM[i].output, 30000);  	//赋值为电压值，最高30000，自带限流
            }
            else
            {
                can2motorRealInfo[i].TARGET_CURRENT = 0;
            }
        }
    }

    //使能电机，发送电流数据
    M3508_Send_Currents();
}

/**
 * @brief 回零校准模式
 *
 * @param RM_MOTOR
 * @param homeing_vel
 */
void Homeing_Mode(MOTOR_REAL_INFO* RM_MOTOR, float homeing_vel,int16_t homeing_torque)
{
    RM_MOTOR->Motor_Mode = HOMEING_MODE;
    //内部存值
    RM_MOTOR->HomingMode.Vel = homeing_vel;
    RM_MOTOR->HomingMode.TARGET_TORQUE = homeing_torque;
    //赋值给外部
    RM_MOTOR->TARGET_RPM = RM_MOTOR->HomingMode.Vel;
}


/**
  * @brief  位置控制(新位置环程序)
  * @param  target_pos目标位置
  * @return
*/
int Position_Control(MOTOR_REAL_INFO *MOTO_REAL_INFO,float target_pos)
{
    MOTO_REAL_INFO->Motor_Mode = POSITION_CONTROL_MODE;
    MOTO_REAL_INFO->TARGET_POS = target_pos;
    if(ABS(MOTO_REAL_INFO->REAL_ANGLE-target_pos)<5)
        return 1;
    else
        return 0;
}

/**
  * @brief
	* @param 电机结构体
	* @param  target_torque目标转矩，用电流表示
	* @param target_pos目标位置
	* @retval none
  */
int Pos_Torque_Control(MOTOR_REAL_INFO *MOTO_REAL_INFO, uint16_t Target_Torque, float Target_Pos)
{
    MOTO_REAL_INFO->Motor_Mode = POSITION_TORQUE_MODE;
    MOTO_REAL_INFO->TARGET_POS = Target_Pos;
    MOTO_REAL_INFO->TARGET_TORQUE = Target_Torque;
    if(ABS(MOTO_REAL_INFO->REAL_ANGLE-Target_Pos)<5)
        return 1;
    else
        return 0;
}

/**
 * @brief 速度模式
 * @param 目标转速
 * @return NULL
*/
void Speed_Control(MOTOR_REAL_INFO *RM_MOTOR, float Target_RPM)
{
    RM_MOTOR->Motor_Mode = SPEED_CONTROL_MODE;
    RM_MOTOR->TARGET_RPM = Target_RPM;
}

/**
  * @brief  速度转矩控制函数,假如你要改变电机的转向，那么直接改变Target_Vel的值即可
  * @param  target_torque目标转矩,用电流表示（正数类型）
  * @param target_vel目标位置（有正负，代表转向）
  * @retval none
*/
void Vel_Torque_Control(MOTOR_REAL_INFO *MOTO_REAL_INFO, uint16_t Target_Torque, float Target_Vel)
{
    MOTO_REAL_INFO->Motor_Mode = SPEED_TARQUE_CONTROL_MODE;
    MOTO_REAL_INFO->TARGET_RPM = Target_Vel;
    MOTO_REAL_INFO->TARGET_TORQUE = Target_Torque;
}
