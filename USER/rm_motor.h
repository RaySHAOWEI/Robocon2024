//
// Created by Ray on 2023/11/24.
//

#ifndef INC_2024RC_B_R1_RM_MOTOR_H
#define INC_2024RC_B_R1_RM_MOTOR_H

#include "main.h"

//����������ģʽ
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
  * @brief T���ٶȹ滮�ṹ��
  * @note
*/
typedef struct VELOCITY_PLANNING //�ٶȹ滮
{
    float Distance;
    float Pstart;        //��ʼλ��
    float Pend;          //����λ��
    float Vstart;        //��ʼ���ٶ�           // ��λ��RPM ����ֵ
    float Vmax;          //�����ٶ�
    float Vend;          //ĩβ���ٶ�
    float Rac;           //����·�̵ı���
    float Rde;           //����·�̵ı���
    int flag;            //��ɱ�־λ�����ͣ������ʱ����1
}VELOCITY_PLANNING;

/**
 * @brief ����ģʽ�ṹ�塣˵ʵ�ڣ����ģʽ���ٶ�ת��ûʲô���ֱ�����ٶ�ת�ؼ���
 * 20231109���������ĳ�У׼ģʽ����������һ�¡�
*/
typedef struct
{
    float current;    //û�ã�֮��ɾ�ˡ�
    float Vel;				//������ٶȣ�����������ת��ת������ʵ��������á�
    float output;
    int16_t  TARGET_TORQUE;//����Ŀ��ת�أ��õ�����ʾ
    int done_flag;  //����ɹ���־λ
    int32_t cnt;
}HOMING_MODE_TYPE;

/**
  * @brief  �������  M3508��M2006��M6020
*/
typedef enum
{
    M_3508 = 1,
    M_2006 = 2,
    M_6020 = 3,
    NONE = 4  //��ʾû�н�����
}MotorType_TypeDef;

typedef struct
{
    uint32_t Motor_Mode;//���ģʽ
    //POSITION_CONTROL_MODEλ��ģʽ
    //POSITION_TARQUE_CONTROL_MODEλ��_����ģʽ
    //SPEED_TARQUE_CONTROL_MODEλ��_����ģʽ
    //SPEED_CONTROL_MODE�ٶ�ģʽ
    //MOTO_OFF����ر�-->����������
    //VELOCITY_PLANNING_MODE���ι滮ģʽ

    MotorType_TypeDef Motor_Type;

    uint16_t  	ANGLE;            	// ����ת�ӽǶ�
    int16_t  	RPM;				// ʵ��ת��ת��
    int16_t  	CURRENT;			// ʵ��ת�ص���
    int16_t  	TARGET_CURRENT;		// Ŀ��ת�ص���

    int16_t  TARGET_POS;		//Ŀ��Ƕ�(λ��)
    int16_t  TARGET_TORQUE;//Ŀ��ת�أ��õ�����ʾ
    float    TARGET_RPM;		//Ŀ��ת��
    int      Velflag;			//�ٶȶ�Ϊ��ʱ����1
    int Stalled;          //��ת��־λ
    int32_t Cnt;          //������־λ

    VELOCITY_PLANNING 		Velocity_Planning;	//�ٶȹ滮
    HOMING_MODE_TYPE 		HomingMode;			//�������ģʽ

    // �ǶȻ���ʱ�õ��������
    float		REAL_ANGLE;         //���������ʵ�Ƕȣ�������float��
    uint8_t	 	FIRST_ANGLE_INTEGRAL_FLAG;  //��һ�λ��ֱ�־λ
    uint16_t 	LAST_ANGLE;   //��һ�εĽǶ�
    int16_t 	Filter_RPM;  //�˲����ת�٣�δ�ã�
    int once_flag; //ֻת��һȦ��־λ
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
// M3508���صĵ����ʵ��Ϣ

extern MOTOR_REAL_INFO can1motorRealInfo[7];//����4��3508���m = 0 1 2 3; ��̨2��6020���m = 4 5��6020������idӦ������Ϊ1 2��
extern MOTOR_REAL_INFO can2motorRealInfo[7];//�Ĵ�2��3508���m = 0 1; ��צ��ת1��3508���m = 2; ͹��2��3508���m = 3 4; ������̨1��3508���m = 5

extern PID_T can1MOTOR_PID_RPM[7]; //�ٶ�pid��Ϣ
extern PID_T can1MOTOR_PID_POS[7];	//λ��pid��Ϣ

extern PID_T can2MOTOR_PID_RPM[7]; //�ٶ�pid��Ϣ
extern PID_T can2MOTOR_PID_POS[7];	//λ��pid��Ϣ

#endif //INC_2024RC_B_R1_RM_MOTOR_H
