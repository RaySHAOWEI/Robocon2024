//
// Created by Ray on 2023/11/24.
//

#include "upper.h"

void lim(float *input, float max, float min)
{
    if (max > min)
    {
        if (*input > max)
            *input = max;
        if (*input < min)
            *input = min;
    }
    else if (max < min)
    {
        if (*input > min)
            *input = min;
        if (*input < max)
            *input = max;
    }
}

// //----------��̨���----------------------------
// GIMBAL Gimbal;   //��̨�ṹ��
// int16_t Theta;  //���������Ŀ�����н�
// int16_t Alpha;  //��ת����
// //---------------------------------------------
// void Gimbal_controller(void)
// {
//     Gimbal.TAN = (Gimbal.target_point[1] - ROBOT_CHASSI.world_y)/(Gimbal.target_point[0] - ROBOT_CHASSI.world_x);   //���нǼ���
//     Gimbal.Theta = atan(Gimbal.TAN);                        //������
//     Gimbal.Theta = (Gimbal.Theta*180/3.14);
//     Gimbal.Descripe = (float)1; //���ٱȣ���š���Ҫ�ʻ�е���д�����
//     Gimbal.gimbal_angle = can1motorRealInfo[Motor_SHOOT_GIMBAL].REAL_ANGLE;           //Gimbal.gimbal_angle = can2motorRealInfo[3].REAL_ANGLE

//     Theta = (int16_t)Gimbal.Theta;
//     Alpha = (int16_t)ROBOT_CHASSI.world_w;
// }

// uint8_t claw_state = 1;     //��צ״̬����תʱΪ0��Ϊ��תΪ1
// void Shooting_Init(void)
// {
//      //��̨�ٶȼ���׼λ�ø�ֵ
//       Gimbal.target_point[0] = -100;      //X(�����趨)
//       Gimbal.target_point[1] = 100;     //Y
//       Gimbal.rpm = 350;
//     //�������ֹͣ
//     Speed_Control(&can1motorRealInfo[Motor_SHOOT_MOTOR_1],0);
//     Speed_Control(&can1motorRealInfo[Motor_SHOOT_MOTOR_2],0);
//     //��צ��λ
//     if(test_rise_time_up(ABS(can1motorRealInfo[Motor_SHOOT_lift].CURRENT),5500,locked_rotor_time)) claw_state = 0;   //��תʱ��צֹͣ�½�
//    //��צ�½�
//     if (claw_state)             //��û��ת
//     {
//         Speed_Control(&can1motorRealInfo[Motor_SHOOT_lift],-200);     //��צ����
//     }
//     else    //��⵽��ת
//     {
//         Speed_Control(&can1motorRealInfo[Motor_SHOOT_lift],0); //�ٶȵ���
//         can1motorRealInfo[Motor_SHOOT_lift].REAL_ANGLE = 0;    //λ������
//     }
//     Gimbal_controller();

//     if(Gimbal.gimbal_angle < 120 && Gimbal.gimbal_angle > -120)
//             {
//                 if(YaoGan_RIGHT_Y !=0)
//                 {
//                  if(YaoGan_RIGHT_Y-1500>100)
//                     Speed_Control(&can1motorRealInfo[Motor_SHOOT_GIMBAL],Gimbal.rpm);
//                 else if(YaoGan_RIGHT_Y-1500<-100)
//                     Speed_Control(&can1motorRealInfo[Motor_SHOOT_GIMBAL],-Gimbal.rpm);
//                 else
//                     Speed_Control(&can1motorRealInfo[Motor_SHOOT_GIMBAL],0);
//                 }
//             }
//             else
//             {
//                if(YaoGan_RIGHT_Y !=0)
//                {
//                  Speed_Control(&can1motorRealInfo[Motor_SHOOT_GIMBAL],0);
//                 if(can1motorRealInfo[5].REAL_ANGLE > 120)
//                 {
//                     if(YaoGan_RIGHT_Y-1500<-100)
//                         Speed_Control(&can1motorRealInfo[Motor_SHOOT_GIMBAL],-Gimbal.rpm);
//                 }
//                 else
//                 {
//                     if(YaoGan_RIGHT_Y-1500>100)
//                         Speed_Control(&can1motorRealInfo[Motor_SHOOT_GIMBAL],Gimbal.rpm);
//                 }
//                }
//             }
// }

// /**
//  * @brief ������
//  * @param  NULL
//  * @return NULL5
// */
// void Shooting_ball(void)
// {    //��̨��׼
//     Gimbal_controller();
//     Gimbal.centor_flag = 0;
//     if(Gimbal.gimbal_angle <= 120 && Gimbal.gimbal_angle >= -120)
//     {   //�о�������λ�û�
//         if(Alpha+Theta > Gimbal.gimbal_angle+1)
//         {
//             Speed_Control(&can1motorRealInfo[Motor_SHOOT_GIMBAL],Gimbal.rpm);
//         }
//         else if(Alpha+Theta < Gimbal.gimbal_angle-1)
//         {
//             Speed_Control(&can1motorRealInfo[Motor_SHOOT_GIMBAL],-Gimbal.rpm);
//         }
//         else
//         {
//             Speed_Control(&can1motorRealInfo[Motor_SHOOT_GIMBAL],0);     //�ɹ���׼
//             Gimbal.aim_flag = 1;
//         }
//     }
//     if (Gimbal.aim_flag)    //�ɹ���׼
//     {
//         //����
//         Speed_Control(&can1motorRealInfo[Motor_SHOOT_MOTOR_1],-7000);
//         Speed_Control(&can1motorRealInfo[Motor_SHOOT_MOTOR_2],7000);    //�Ĵ�ת��

//     }
//     if(test_rise_time_up(ABS(can1motorRealInfo[2].CURRENT),5500,locked_rotor_time)) claw_state = 0;   //��תʱ��צֹͣ�½�
//    //��צ�½�
//     if (claw_state)             //��û��ת
//     {
//         Speed_Control(&can1motorRealInfo[Motor_SHOOT_lift],-200);     //��צ����
//     }
//     else    //��⵽��ת
//     {
//         Speed_Control(&can1motorRealInfo[Motor_SHOOT_lift],0); //�ٶȵ���
//         can1motorRealInfo[Motor_SHOOT_lift].REAL_ANGLE = 0;    //λ������
//     }
// }

////////////////////////////////////////////////////////////////////////////////

/**
 * @brief ��ת�������
 *
 * @param target_angle
 * @return 0:��ʼ�� 1:���ﶥ�˻���ʧ��
 */
int flip_motor(float target_angle)
{
    int feedback;
    static int Enable_flag = 0; // ʹ�ܱ�־λ
    if (target_angle == flip_down)
    {
        if (can2motorRealInfo[Motor_SHOOT_FLIP].HomingMode.done_flag == 0)
        {
            Homeing_Mode(&can2motorRealInfo[Motor_SHOOT_FLIP], -500, 3000); // �ص���ʼλ��
        }
        else
        {
            feedback = !(Position_Control(&can2motorRealInfo[Motor_SHOOT_FLIP], target_angle));
            Enable_flag = 1;
        }
    }
    else if (target_angle == flip_up)
    {
        if (Enable_flag == 1)
        {
            can2motorRealInfo[Motor_SHOOT_FLIP].HomingMode.done_flag = 0;
            feedback = Position_Control(&can2motorRealInfo[Motor_SHOOT_FLIP], target_angle); // ̧��
        }
        else
        {
            feedback = 1;
        }
    }
    return feedback;
}

/**
 * @brief ��צ�������
 *
 * @param jaw_state
 * @return int ��סʱ����1 �ɿ�ʱ����0
 */
int jaw_motor(int jaw_state)
{
    int feedback;
    if (jaw_state == jaw_close)
    {
        feedback = !(Vel_Torque_Control(&can2motorRealInfo[Motor_SHOOT_JAW], 3195, jaw_state)); // �н�
    }
    else if (jaw_state == jaw_open)
    {
        Pos_Torque_Control(&can2motorRealInfo[Motor_SHOOT_JAW], 3195, jaw_state);
        feedback = 0;
    }
    return feedback;
}

/**
 * @brief ����Ħ�������ƺ���
 * @param vel Ħ�����ٶ�
 */
void belt_ctrl(int vel)
{
    Speed_Control(&can2motorRealInfo[Motor_SHOOT_MOTOR_1], -1 * vel);
    Speed_Control(&can2motorRealInfo[Motor_SHOOT_MOTOR_2], vel);
}

/**
 * @brief ��̨ת�����ƺ���
 *
 * @param target_angle Ŀ��Ƕȣ�2000 - 4600
 */
void gimbal_ctrl(float target_pos)
{
    lim(&target_pos, gimbal_down, gimbal_up);
    if (target_pos >= gimbal_up && target_pos <= gimbal_down) // ����λ
        Pos_Velimit_Control(&can2motorRealInfo[Motor_SHOOT_GIMBAL], gimbal_speed, target_pos);
}

/**
 * @brief ��̨ҡ�����ɿ��ƺ���
 *
 */
void gimbal_free_ctrl(void)
{
    float speed;
    if (2000 <= can2motorRealInfo[Motor_SHOOT_GIMBAL].ANGLE && can2motorRealInfo[Motor_SHOOT_GIMBAL].ANGLE <= 4600)
    {
        if (ABS(YaoGan_RIGHT_Y - 1500) > 100)
        {
            speed = ((YaoGan_RIGHT_Y - 1500.0f) / 500) * gimbal_speed;
        }
        else if (ABS(YaoGan_RIGHT_Y - 1500) <= 100)
        {
            speed = 0;
        }
    }
    else
    {
        speed = 0;
    }
    Speed_Control(&can2motorRealInfo[Motor_SHOOT_GIMBAL], speed);
}

/*
��ʼʱ����û�򣬵�λ���ʼ������ʱ�����ڵ�һ��

���˲����ڶ�����
��û���� ��צû�� �ȴ���ת�£�����
��û���� ��צ���� �ȴ���ת�ϣ�װ��
�������� ��צû�� ���ַ�ת�ϣ��ȴ�����
�������� ��צ���� �쳣��������Ӧ�ó���

���ַ�ת�£���צ�н�����ʱ���ж���צ����
���ַ�ת�ϣ���צ�ɿ�����ʱ���ж���צû�� ��������
���ַ��䶯��ʱ���ж���û����


��Ϊ������û�����Ի�ִ�м�����ǰ������Ҳ���Լ�����ʼ������
������ж��Ƿ�е����е���ִ��װ��裬
��ʱ�������򣬲��˻��ڵڶ�������ת������ڸߴ��ȴ�����

���˲���������


*/

/////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * @brief װ����
 * @param  NULL
 * @return NULL
 */
void Load_Ball(int step)
{
    if (step == 1)
    {
        Pos_Torque_Control(&can1motorRealInfo[Motor_SHOOT_JAW], 4500, -160); // �н�

        //  Pos_Torque_Control(&can1motorRealInfo[Motor_SHOOT_JAW],4000,100);//�ɿ�
    }

    if (step == 2)
    {
        Position_Control(&can1motorRealInfo[6], 115);
        can1motorRealInfo[6].HomingMode.done_flag = 0;
    }
}

void shoot_jaw_homeing(int flag)
{
    if (flag == shoot_homeing_flag)
    {
        if (can1motorRealInfo[6].HomingMode.done_flag == 0)
        {
            Homeing_Mode(&can1motorRealInfo[6], -500, 3000);
        }
        else
        {
            Pos_Torque_Control(&can1motorRealInfo[6], 1000, 0);
        }
    }
}

// �ж����Ƿ�������ˣ�����������ˣ�flag=1 ��צ̧��
int flag_judgment(void)
{
    if (ABS(can1motorRealInfo[Motor_SHOOT_JAW].REAL_ANGLE) >= 160) // �����õ����жϣ��ĳ��ٶȻ�
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

//  uint16_t time_up = 0;
// /**
//   * @brief  ����Ƿ��ת����ת�򷵻�1�����򷵻�0
//   * @param  ��������ת��������תʱ��
//   * @retval
//   */
// uint8_t test_rise_time_up(int current,int boundary_current,int boundary_time)
// {
//   if(current<boundary_current) time_up=0;
//   if(current>boundary_current) time_up++;
//   if(time_up>=boundary_time)return 1;
//   return 0;
// }
