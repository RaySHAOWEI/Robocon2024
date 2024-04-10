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


//----------云台相关----------------------------
GIMBAL Gimbal;   //云台结构体
int16_t Theta;  //车体相对于目标点的切角
int16_t Alpha;  //自转补偿
//---------------------------------------------
void Gimbal_controller(void)
{
    Gimbal.TAN = (Gimbal.target_point[1] - ROBOT_CHASSI.world_y)/(Gimbal.target_point[0] - ROBOT_CHASSI.world_x);   //正切角计算
    Gimbal.Theta = atan(Gimbal.TAN);                        //弧度制
    Gimbal.Theta = (Gimbal.Theta*180/3.14);     
    Gimbal.Descripe = (float)1; //减速比，大概。需要问机械，有待更改
    Gimbal.gimbal_angle = can1motorRealInfo[Motor_SHOOT_GIMBAL].REAL_ANGLE;           //Gimbal.gimbal_angle = can2motorRealInfo[3].REAL_ANGLE

    Theta = (int16_t)Gimbal.Theta;
    Alpha = (int16_t)ROBOT_CHASSI.world_w;
}

uint8_t claw_state = 1;     //夹爪状态，堵转时为0，为堵转为1
void Shooting_Init(void)
{  
     //云台速度及瞄准位置赋值
      Gimbal.target_point[0] = -100;      //X(后期设定)
      Gimbal.target_point[1] = 100;     //Y
      Gimbal.rpm = 350;
    //发射机构停止
    Speed_Control(&can1motorRealInfo[Motor_SHOOT_MOTOR_1],0);
    Speed_Control(&can1motorRealInfo[Motor_SHOOT_MOTOR_2],0);
    //夹爪复位
    if(test_rise_time_up(ABS(can1motorRealInfo[Motor_SHOOT_lift].CURRENT),5500,locked_rotor_time)) claw_state = 0;   //堵转时夹爪停止下降       
   //夹爪下降
    if (claw_state)             //还没堵转
    {
        Speed_Control(&can1motorRealInfo[Motor_SHOOT_lift],-200);     //夹爪下来
    }
    else    //检测到堵转
    {
        Speed_Control(&can1motorRealInfo[Motor_SHOOT_lift],0); //速度调零
        can1motorRealInfo[Motor_SHOOT_lift].REAL_ANGLE = 0;    //位置置零
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
 * @brief 射球函数   
 * @param  NULL
 * @return NULL5
*/
void Shooting_ball(void)
{    //云台瞄准
    Gimbal_controller();
    Gimbal.centor_flag = 0;
    if(Gimbal.gimbal_angle <= 120 && Gimbal.gimbal_angle >= -120)   
    {   //感觉可以用位置环
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
            Speed_Control(&can1motorRealInfo[Motor_SHOOT_GIMBAL],0);     //成功瞄准
            Gimbal.aim_flag = 1;       
        }
    }
    if (Gimbal.aim_flag)    //成功瞄准
    {
        //发射       
        Speed_Control(&can1motorRealInfo[Motor_SHOOT_MOTOR_1],-7000);
        Speed_Control(&can1motorRealInfo[Motor_SHOOT_MOTOR_2],7000);    //履带转动

    }
    if(test_rise_time_up(ABS(can1motorRealInfo[2].CURRENT),5500,locked_rotor_time)) claw_state = 0;   //堵转时夹爪停止下降       
   //夹爪下降
    if (claw_state)             //还没堵转
    {
        Speed_Control(&can1motorRealInfo[Motor_SHOOT_lift],-200);     //夹爪下来
    }
    else    //检测到堵转
    {
        Speed_Control(&can1motorRealInfo[Motor_SHOOT_lift],0); //速度调零
        can1motorRealInfo[Motor_SHOOT_lift].REAL_ANGLE = 0;    //位置置零
    }    
}

/**
 * @brief 装球函数    
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

//判断球是否夹起来了，如果夹起来了，flag=1 夹爪抬升
int flag_judgment(void){
    if(ABS(can1motorRealInfo[Motor_SHOOT_JAW].REAL_ANGLE)>=160)    //后面用电流判断，改成速度环
		{ 
        return 1;
		}
		else{
			  return 0;
		} 

}
 uint16_t time_up = 0;
/**
  * @brief  检测是否堵转，堵转则返回1，否则返回0
  * @param  电流，堵转电流，堵转时间
  * @retval 
  */
uint8_t test_rise_time_up(int current,int boundary_current,int boundary_time)
{ 
  if(current<boundary_current) time_up=0;
  if(current>boundary_current) time_up++;
  if(time_up>=boundary_time)return 1;
  return 0; 
}
