//
// Created by Ray on 2023/11/24.
//

#include "upper.h"

void lim(float *input, float max, float min) {
    if (max > min){
        if (*input > max) *input = max;
        if (*input < min) *input = min;
    }
    else if (max < min){
        if (*input > min) *input = min;
        if (*input < max) *input = max;
    }
}


//----------��̨���----------------------------
GIMBAL Gimbal;   //��̨�ṹ��
int16_t Theta;  //���������Ŀ�����н�
int16_t Alpha;  //��ת����
//---------------------------------------------
void Gimbal_controller(void)
{
    Gimbal.TAN = (Gimbal.target_point[1] - ROBOT_CHASSI.world_y)/(Gimbal.target_point[0] - ROBOT_CHASSI.world_x);   //���нǼ���
    Gimbal.Theta = atan(Gimbal.TAN);                        //������
    Gimbal.Theta = (Gimbal.Theta*180/3.14);     
    Gimbal.Descripe = (float)1; //���ٱȣ���š���Ҫ�ʻ�е���д�����
    Gimbal.gimbal_angle = can1motorRealInfo[Motor_SHOOT_GIMBAL].REAL_ANGLE;           //Gimbal.gimbal_angle = can2motorRealInfo[3].REAL_ANGLE

    Theta = (int16_t)Gimbal.Theta;
    Alpha = (int16_t)ROBOT_CHASSI.world_w;
}

uint8_t claw_state = 1;     //��צ״̬����תʱΪ0��Ϊ��תΪ1
void Shooting_Init(void)
{  
     //��̨�ٶȼ���׼λ�ø�ֵ
      Gimbal.target_point[0] = -100;      //X(�����趨)
      Gimbal.target_point[1] = 100;     //Y
      Gimbal.rpm = 350;
    //�������ֹͣ
    Speed_Control(&can1motorRealInfo[Motor_SHOOT_MOTOR_1],0);
    Speed_Control(&can1motorRealInfo[Motor_SHOOT_MOTOR_2],0);
    //��צ��λ
    if(test_rise_time_up(ABS(can1motorRealInfo[Motor_SHOOT_lift].CURRENT),5500,locked_rotor_time)) claw_state = 0;   //��תʱ��צֹͣ�½�       
   //��צ�½�
    if (claw_state)             //��û��ת
    {
        Speed_Control(&can1motorRealInfo[Motor_SHOOT_lift],-200);     //��צ����
    }
    else    //��⵽��ת
    {
        Speed_Control(&can1motorRealInfo[Motor_SHOOT_lift],0); //�ٶȵ���
        can1motorRealInfo[Motor_SHOOT_lift].REAL_ANGLE = 0;    //λ������
    }
    Gimbal_controller();

    if(Gimbal.gimbal_angle < 120 && Gimbal.gimbal_angle > -120)
            {
                if(YaoGan_RIGHT_Y !=0)
                {
                 if(YaoGan_RIGHT_Y-1500>100)
                    Speed_Control(&can1motorRealInfo[Motor_SHOOT_GIMBAL],Gimbal.rpm);
                else if(YaoGan_RIGHT_Y-1500<-100)
                    Speed_Control(&can1motorRealInfo[Motor_SHOOT_GIMBAL],-Gimbal.rpm);
                else
                    Speed_Control(&can1motorRealInfo[Motor_SHOOT_GIMBAL],0);
                }      
            }
            else
            {
               if(YaoGan_RIGHT_Y !=0)
               {
                 Speed_Control(&can1motorRealInfo[Motor_SHOOT_GIMBAL],0);
                if(can1motorRealInfo[5].REAL_ANGLE > 120)
                {
                    if(YaoGan_RIGHT_Y-1500<-100)
                        Speed_Control(&can1motorRealInfo[Motor_SHOOT_GIMBAL],-Gimbal.rpm);
                }
                else
                {   
                    if(YaoGan_RIGHT_Y-1500>100)
                        Speed_Control(&can1motorRealInfo[Motor_SHOOT_GIMBAL],Gimbal.rpm);
                }
               }                    
            }
}

/**
 * @brief ������   
 * @param  NULL
 * @return NULL5
*/
void Shooting_ball(void)
{    //��̨��׼
    Gimbal_controller();
    Gimbal.centor_flag = 0;
    if(Gimbal.gimbal_angle <= 120 && Gimbal.gimbal_angle >= -120)   
    {   //�о�������λ�û�
        if(Alpha+Theta > Gimbal.gimbal_angle+1)
        {  
            Speed_Control(&can1motorRealInfo[Motor_SHOOT_GIMBAL],Gimbal.rpm);
        }
        else if(Alpha+Theta < Gimbal.gimbal_angle-1)
        {
            Speed_Control(&can1motorRealInfo[Motor_SHOOT_GIMBAL],-Gimbal.rpm);
        }
        else
        {
            Speed_Control(&can1motorRealInfo[Motor_SHOOT_GIMBAL],0);     //�ɹ���׼
            Gimbal.aim_flag = 1;       
        }
    }
    if (Gimbal.aim_flag)    //�ɹ���׼
    {
        //����       
        Speed_Control(&can1motorRealInfo[Motor_SHOOT_MOTOR_1],-7000);
        Speed_Control(&can1motorRealInfo[Motor_SHOOT_MOTOR_2],7000);    //�Ĵ�ת��

    }
    if(test_rise_time_up(ABS(can1motorRealInfo[2].CURRENT),5500,locked_rotor_time)) claw_state = 0;   //��תʱ��צֹͣ�½�       
   //��צ�½�
    if (claw_state)             //��û��ת
    {
        Speed_Control(&can1motorRealInfo[Motor_SHOOT_lift],-200);     //��צ����
    }
    else    //��⵽��ת
    {
        Speed_Control(&can1motorRealInfo[Motor_SHOOT_lift],0); //�ٶȵ���
        can1motorRealInfo[Motor_SHOOT_lift].REAL_ANGLE = 0;    //λ������
    }    
}

/**
 * @brief װ����    
 * @param  NULL
 * @return NULL
*/
void Load_Ball(int step)
{   
     if(step == 1){
     Pos_Torque_Control(&can1motorRealInfo[Motor_SHOOT_JAW],4500,-160);
               }

     if(step == 2){
     Position_Control(&can1motorRealInfo[6],115);
     can1motorRealInfo[6].HomingMode.done_flag = 0;
               }
}


void shoot_jaw_homeing(int flag)
{
        if(flag == shoot_homeing_flag) {
         if(can1motorRealInfo[6].HomingMode.done_flag == 0 ){
          Homeing_Mode(&can1motorRealInfo[6],-500,3000);   
     }       
            else{
            Pos_Torque_Control(&can1motorRealInfo[6],1000,0); 
        }
          }
}

//�ж����Ƿ�������ˣ�����������ˣ�flag=1 ��צ̧��
int flag_judgment(void){
    if(ABS(can1motorRealInfo[Motor_SHOOT_JAW].REAL_ANGLE)>=160)    //�����õ����жϣ��ĳ��ٶȻ�
		{ 
        return 1;
		}
		else{
			  return 0;
		} 

}
 uint16_t time_up = 0;
/**
  * @brief  ����Ƿ��ת����ת�򷵻�1�����򷵻�0
  * @param  ��������ת��������תʱ��
  * @retval 
  */
uint8_t test_rise_time_up(int current,int boundary_current,int boundary_time)
{ 
  if(current<boundary_current) time_up=0;
  if(current>boundary_current) time_up++;
  if(time_up>=boundary_time)return 1;
  return 0; 
}
