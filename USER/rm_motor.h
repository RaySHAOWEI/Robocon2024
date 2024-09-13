//
// Created by Ray on 2023/11/24.
//

#ifndef INC_2024RC_B_R1_RM_MOTOR_H
#define INC_2024RC_B_R1_RM_MOTOR_H

#include "main.h"

// 驱动器工作模式
typedef enum
{
    MOTO_OFF,                  // 电机关闭
    CURRENT_MODE,              // 电流模式(直接赋电流值)
    SPEED_CONTROL_MODE,        // 速度模式
    POSITION_CONTROL_MODE,     // 位置模式
    SPEED_TARQUE_CONTROL_MODE, // 速度转矩模式
    POSITION_TORQUE_MODE,      // 位置转矩模式
    POSITION_SPEED_LIMIT_MODE, // 位置限速模式
    HOMEING_MODE,              // 回零模式
    SETTING_MODE,              // 限位设置模式（反回零模式）
    VELOCITY_PLANNING_MODE,    // 速度规划模式
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
    M7_ID = 0x207,
} Can_Msg_Id_e;

/**
 * @brief 回零模式结构体。说实在，这个模式和速度转矩没什么差别。直接用速度转矩即可
 * 20231109许少威：改成校准模式。废物利用一下。
 */
typedef struct
{
    float Vel;             // 回零的速度，正负代表正转反转。根据实际情况设置。
    int16_t TARGET_TORQUE; // 回零目标转矩，用电流表示
    int done_flag;         // 回零成功标志位
    int32_t cnt;
} HOMING_MODE_TYPE;

/**
 * @brief 设置模式结构体
 * 20240518许少威：复用回零模式，加个设置位置，可以让机械限位设置成任意角度
 */
typedef struct
{
    float Vel;             // 回零的速度，正负代表正转反转。根据实际情况设置。
    int16_t TARGET_TORQUE; // 回零目标转矩，用电流表示
    int done_flag;         // 回零成功标志位
    int32_t cnt;
    float SETTING_ANGLE;
} SETTING_MODE_TYPE;

/**
 * @brief T型速度规划结构体
 * @note
 */
typedef struct VELOCITY_PLANNING // 速度规划
{
    float Distance;
    float Pstart;  // 开始位置
    float Pend;    // 结束位置
    float Vstart;  // 开始的速度           // 单位：RPM 绝对值
    float Vmax;    // 最大的速度
    float Vend;    // 末尾的速度
    float Rac;     // 加速路程的比例
    float Rde;     // 减速路程的比例
    int done_flag; // 完成标志位，电机停下来的时候置1
} VELOCITY_PLANNING;

/**
 * @brief  电机种类  M3508、M2006和M6020
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
    int16_t MotorMaxCurrent; // 电机最大电流
    Motor_Mode_e Motor_Mode; // 电机模式
    MotorType_TypeDef Motor_Type;

    uint16_t ANGLE;         // 采样转子角度
    int16_t RPM;            // 实际转子转速
    int16_t CURRENT;        // 实际转矩电流
    int16_t TARGET_CURRENT; // 目标转矩电流

    int16_t TARGET_POS;    // 目标角度(位置)
    int16_t TARGET_TORQUE; // 目标转矩，用电流表示（分时复用为限速值）
    float TARGET_RPM;      // 目标转速

    int stop_cnt;  // 堵转计数器 堵转置1
    int stop_flag; // 堵转标志位

    VELOCITY_PLANNING Velocity_Planning; // 速度规划
    HOMING_MODE_TYPE HomingMode;         // 电机回零模式
    SETTING_MODE_TYPE SettingMode;       // 电机设置模式

    // 角度积分时用到下面变量
    float REAL_ANGLE;                  // 处理过的真实角度（必须用float）
    uint8_t FIRST_ANGLE_INTEGRAL_FLAG; // 第一次积分标志位
    uint16_t LAST_ANGLE;               // 上一次的角度

    int once_flag;     // 只转动一圈标志位
    int current_limit; // 限流标志位 0 为限流 1 为不限流（默认限流）
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
void Setting_Mode(MOTOR_REAL_INFO *RM_MOTOR, float vel, int16_t torque, float angle);
void Planning_Mode(MOTOR_REAL_INFO *M3508_MOTOR, float Pstart, float Pend, float Vstart, float Vmax, float Vend, float Rac, float Rde);
void Velocity_Planning(MOTOR_REAL_INFO *M3508_MOTOR);

// M3508返回的电机真实信息
extern MOTOR_REAL_INFO can1motorRealInfo[7]; // can1
extern MOTOR_REAL_INFO can2motorRealInfo[7]; // can2

extern PID_T can1MOTOR_PID_RPM[7]; // 速度pid信息
extern PID_T can1MOTOR_PID_POS[7]; // 位置pid信息

extern PID_T can2MOTOR_PID_RPM[7]; // 速度pid信息
extern PID_T can2MOTOR_PID_POS[7]; // 位置pid信息

#endif // INC_2024RC_B_R1_RM_MOTOR_H
