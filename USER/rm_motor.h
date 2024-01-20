//
// Created by Ray on 2023/11/24.
//

#ifndef INC_2024RC_B_R1_RM_MOTOR_H
#define INC_2024RC_B_R1_RM_MOTOR_H

#include "main.h"

//驱动器工作模式
#define  SPEED_CONTROL_MODE		    2
#define  VELOCITY_PLANNING_MODE     3
#define  CURRENT_MODE               4
#define  POSITION_CONTROL_MODE		5
#define  SPEED_TARQUE_CONTROL_MODE  6
#define  POSITION_TORQUE_MODE		7
#define  HOMEING_MODE		        9
#define  MOTO_OFF		            0

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
}Can_Msg_Id_e;

/**
  * @brief T型速度规划结构体
  * @note
*/
typedef struct VELOCITY_PLANNING //速度规划
{
    float Distance;
    float Pstart;        //开始位置
    float Pend;          //结束位置
    float Vstart;        //开始的速度           // 单位：RPM 绝对值
    float Vmax;          //最大的速度
    float Vend;          //末尾的速度
    float Rac;           //加速路程的比例
    float Rde;           //减速路程的比例
    int flag;            //完成标志位，电机停下来的时候置1
}VELOCITY_PLANNING;

/**
 * @brief 回零模式结构体。说实在，这个模式和速度转矩没什么差别。直接用速度转矩即可
 * 20231109许少威：改成校准模式。废物利用一下。
*/
typedef struct
{
    float current;    //没用，之后都删了。
    float Vel;				//回零的速度，正负代表正转反转。根据实际情况设置。
    float output;
    int16_t  TARGET_TORQUE;//回零目标转矩，用电流表示
    int done_flag;  //回零成功标志位
    int32_t cnt;
}HOMING_MODE_TYPE;

/**
  * @brief  电机种类  M3508、M2006和M6020
*/
typedef enum
{
    M_3508 = 1,
    M_2006 = 2,
    M_6020 = 3,
    NONE = 4  //表示没有接入电机
}MotorType_TypeDef;

typedef struct
{
    uint32_t Motor_Mode;//电机模式
    //POSITION_CONTROL_MODE位置模式
    //POSITION_TARQUE_CONTROL_MODE位置_力度模式
    //SPEED_TARQUE_CONTROL_MODE位置_力度模式
    //SPEED_CONTROL_MODE速度模式
    //MOTO_OFF电机关闭-->电流不发送
    //VELOCITY_PLANNING_MODE梯形规划模式

    MotorType_TypeDef Motor_Type;

    uint16_t  	ANGLE;            	// 采样转子角度
    int16_t  	RPM;				// 实际转子转速
    int16_t  	CURRENT;			// 实际转矩电流
    int16_t  	TARGET_CURRENT;		// 目标转矩电流

    int16_t  TARGET_POS;		//目标角度(位置)
    int16_t  TARGET_TORQUE;//目标转矩，用电流表示
    float    TARGET_RPM;		//目标转速
    int      Velflag;			//速度度为零时，置1
    int Stalled;          //堵转标志位
    int32_t Cnt;          //计数标志位

    VELOCITY_PLANNING 		Velocity_Planning;	//速度规划
    HOMING_MODE_TYPE 		HomingMode;			//电机回零模式

    // 角度积分时用到下面变量
    float		REAL_ANGLE;         //处理过的真实角度（必须用float）
    uint8_t	 	FIRST_ANGLE_INTEGRAL_FLAG;  //第一次积分标志位
    uint16_t 	LAST_ANGLE;   //上一次的角度
    int16_t 	Filter_RPM;  //滤波后的转速（未用）
    int once_flag; //只转动一圈标志位
}MOTOR_REAL_INFO;

float Max_Value_Limit(float Value, float Limit);
void get_motor_measure(CAN_RxHeaderTypeDef *msg, uint8_t Data[8], CAN_HandleTypeDef *hcan);
void RM_MOTOR_Angle_Integral(MOTOR_REAL_INFO* RM_MOTOR);
void M3508_Send_Currents(void);
void Motor_Control(void);
void Homeing_Mode(MOTOR_REAL_INFO* RM_MOTOR, float homeing_vel,int16_t homeing_torque);
void Velocity_Planning_MODE(MOTOR_REAL_INFO *M3508_MOTOR);
float Position_Control(MOTOR_REAL_INFO *MOTOR_REAL_INFO,float Target_Pos);
void Pos_Torque_Control(MOTOR_REAL_INFO *MOTO_REAL_INFO, uint16_t Target_Torque, float Target_Pos);
void Speed_Control(MOTOR_REAL_INFO* RM_MOTOR, float Target_RPM);
void Vel_Torque_Control(MOTOR_REAL_INFO *MOTO_REAL_INFO, uint16_t Target_Torque, float Target_Vel);
void Velocity_Planning_setpos(MOTOR_REAL_INFO *M3508_MOTOR,float Pstart,float Pend,float Vstart,float Vmax,float Vend,float Rac,float Rde);
// M3508返回的电机真实信息

extern MOTOR_REAL_INFO can1motorRealInfo[7];//底盘4个3508电机m = 0 1 2 3; 云台2个6020电机m = 4 5（6020驱动器id应当设置为1 2）
extern MOTOR_REAL_INFO can2motorRealInfo[7];//履带2个3508电机m = 0 1; 夹爪翻转1个3508电机m = 2; 凸轮2个3508电机m = 3 4; 炮塔云台1个3508电机m = 5

extern PID_T can1MOTOR_PID_RPM[7]; //速度pid信息
extern PID_T can1MOTOR_PID_POS[7];	//位置pid信息

extern PID_T can2MOTOR_PID_RPM[7]; //速度pid信息
extern PID_T can2MOTOR_PID_POS[7];	//位置pid信息

#endif //INC_2024RC_B_R1_RM_MOTOR_H
