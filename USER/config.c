//
// Created by Ray on 2023/11/26.
//

#include "config.h"

/**
 * @brief ���̵�����ͳ�ʼ��
 *
 */
void can1_config(void)
{
    can1motorRealInfo[0].Motor_Type = M_3508; // �����Һ���
    can1motorRealInfo[1].Motor_Type = M_3508; // ������ǰ��
    can1motorRealInfo[2].Motor_Type = M_3508; // ������ǰ��
    can1motorRealInfo[3].Motor_Type = M_3508; // ���������

    // �ٶȻ�
    pid_param_init(&can1MOTOR_PID_RPM[0], PID_Position, 16384, 1024, 0, 0, 0, 12.0f, 0.1f, 0.2f);
    pid_param_init(&can1MOTOR_PID_RPM[1], PID_Position, 16384, 1024, 0, 0, 0, 12.0f, 0.1f, 0.2f);
    pid_param_init(&can1MOTOR_PID_RPM[2], PID_Position, 16384, 1024, 0, 0, 0, 12.0f, 0.1f, 0.2f);
    pid_param_init(&can1MOTOR_PID_RPM[3], PID_Position, 16384, 1024, 0, 0, 0, 12.0f, 0.1f, 0.2f);
}

/**
 * @brief can2��ʼ��
 *
 */
void can2_config(void)
{
    can2motorRealInfo[Motor_SHOOT_MOTOR_1].Motor_Type = M_3508; // ������1
    can2motorRealInfo[Motor_SHOOT_MOTOR_2].Motor_Type = M_3508; // ������2
    can2motorRealInfo[Motor_SHOOT_FLIP].Motor_Type = M_3508;    // ����ת���
    can2motorRealInfo[Motor_SHOOT_JAW].Motor_Type = M_2006;     // �����צ2006���
    can2motorRealInfo[Motor_SHOOT_GIMBAL].Motor_Type = M_6020;  // ������̨6020���

    can2motorRealInfo[Motor_SHOOT_MOTOR_1].current_limit = 1; // ������
    can2motorRealInfo[Motor_SHOOT_MOTOR_2].current_limit = 1; // ������
    can2motorRealInfo[Motor_SHOOT_FLIP].current_limit = 1;    // ������
    can2motorRealInfo[Motor_SHOOT_JAW].current_limit = 1;     // ������

    can2motorRealInfo[Motor_SHOOT_GIMBAL].once_flag = 1; // ��̨ʹ�õ�Ȧģʽλ�û�

    // �ٶȻ�
    pid_param_init(&can2MOTOR_PID_RPM[Motor_SHOOT_MOTOR_1], PID_Position, 16384, 1024, 0, 0, 0, 14.0f, 0.05f, 0.0f);
    pid_param_init(&can2MOTOR_PID_RPM[Motor_SHOOT_MOTOR_2], PID_Position, 16384, 1024, 0, 0, 0, 14.0f, 0.05f, 0.0f);
    pid_param_init(&can2MOTOR_PID_RPM[Motor_SHOOT_FLIP], PID_Position, 16384, 1024, 0, 0, 0, 12.0f, 0.1f, 0.0f);
    pid_param_init(&can1MOTOR_PID_RPM[Motor_SHOOT_JAW], PID_Position, 8192, 1024, 0, 0, 0, 12.0f, 0.1f, 0.0f);
    pid_param_init(&can2MOTOR_PID_RPM[Motor_SHOOT_GIMBAL], PID_Position, 30000, 1024, 0, 0, 0, 2.85f, 0.0f, 0.0f); // ������̨

    // λ�û�
    pid_param_init(&can2MOTOR_PID_POS[Motor_SHOOT_FLIP], PID_Position, 3000, 512, 0, 0, 0, 12.0f, 0.0f, 0.3f);
    can2MOTOR_PID_POS[Motor_SHOOT_FLIP].kf = 0.3f; // ǰ��ϵ��
    pid_param_init(&can2MOTOR_PID_POS[Motor_SHOOT_JAW], PID_Position, 3000, 512, 0, 0, 0, 12.0f, 0.0f, 0.3f);
    pid_param_init(&can2MOTOR_PID_POS[Motor_SHOOT_GIMBAL], PID_Position, gimbal_speed, 800, 0, 0, 0, 12.0f, 0.0f, 0.3f);
}

void Move_Init(void)
{
    // //PD������
    pid_param_init(&point_traker_x_pid, PID_Position, 2000, 0, 0, 10, 0, 3, 0, 0.5);
    pid_param_init(&point_traker_y_pid, PID_Position, 2000, 0, 0, 10, 0, 3, 0, 0.5);
    pid_param_init(&point_traker_yaw_pid, PID_Position, 1000, 0, 0, 1, 0, 30, 0, 0.1);
    pid_param_init(&TRACK_PID, PID_Position, 1000, 0, 0, 1, 0, 30, 0, 0.1);
    pid_param_init(&track_pid, PID_Position, 1000, 0, 0, 1, 0, 30, 0, 0.1);

    // ����PID
    pid_param_init(&laser_X_pid, PID_Position, 500, 0, 0, 10, 0, 1, 0, 0.5);
    pid_param_init(&laser_Y_pid, PID_Position, 100, 0, 0, 10, 0, 1, 0, 0.5);
    pid_param_init(&laser_K_pid, PID_Position, 300, 0, 0, 10, 0, 3, 0, 0.5);
    pid_param_init(&laser_B_pid, PID_Position, 300, 0, 0, 10, 0, 2, 0, 0.5);

    //	// �Զ�·��PID
    //	PID_parameter_init(&point_pid ,  1.2f,0, 0.5f, 500, 0, 5);
    //	PID_parameter_init(&yaw_pid ,  50.0f,0.1f, 1.0f, 1000.0f, 0.0f, 0.1f);
    //	PID_parameter_init(&catch_ring_pid ,  3.0f,0.0f, 0.3f, 100.0f, 0.0f, 1.0f);
    //	PID_parameter_init(&catch_ring_pid_b ,  3.1f,0.0f, 0.3f, 100.0f, 0.0f, 1.0f);
}
