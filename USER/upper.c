//
// Created by Ray on 2023/11/24.
//

#include "upper.h"
#include "FSM.h"

int shooting_point;
int ball;
int ball_loaded;
int shooting_enable;
int flip_state = 0;
int action_ball = 0;
int belt_speed = 0;
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

//----------云台相关----------------------------
GIMBAL Gimbal;           // 云台结构体
float Theta;             // 车体相对于目标点的切角
float Alpha;             // 自转补偿
float init_angle = 3300; // 6020电机正对时码盘值
//---------------------------------------------
// 左边场地
void Gimbal_controller(void)
{
    Gimbal.TAN = (Gimbal.target_point[0] - ROBOT_CHASSI.world_x) / (Gimbal.target_point[1] - ROBOT_CHASSI.world_y); // 正切角计算
    Gimbal.Theta = atan(Gimbal.TAN);                                                                                // 弧度制
    Gimbal.Theta = (Gimbal.Theta * 180 / 3.14);
    Gimbal.Descripe = (float)1;                                        // 减速比，大概。需要问机械，有待更改
    Gimbal.gimbal_angle = can2motorRealInfo[Motor_SHOOT_GIMBAL].ANGLE; // Gimbal.gimbal_angle = can2motorRealInfo[3].ANGLE

    Theta = Gimbal.Theta;
    Alpha = (-((Theta / 360) * 8192) + init_angle);
    //	 Alpha = (int16_t)(((Gimbal.gimbal_angle - init_angle)/8192)*360);
    //     Alpha = (int16_t)ROBOT_CHASSI.world_w;  //舍弃掉action返回的偏航角，直接双激光矫正
}

/**********************************************************以下为自动射球的函数*************************************************************/

int shooting_point_x[7] = {0, 300, 1400, 1700, -300, -1400, -2000};
int shooting_point_y[7] = {0, 3000, 3000, 5000, 3000, 3000, 6000};
// int shooting_point_y[7] = {0, 2600, 2600, 3800,2400,2400,3800};

/**
 * @brief 自动射球初始化
 * @param 射球目标点（x，y)
 */
void Auto_shooting_Init(void)
{
    // 云台赋值
    if (field == Left)
    {
        if (shooting_point == 1)
        {
            Gimbal.target_point[0] = shooting_point_x[1];
            Gimbal.target_point[1] = shooting_point_y[1];
        }
        if (shooting_point == 2)
        {
            Gimbal.target_point[0] = shooting_point_x[2];
            Gimbal.target_point[1] = shooting_point_y[2];
        }
        if (shooting_point == 3)
        {
            Gimbal.target_point[0] = shooting_point_x[3];
            Gimbal.target_point[1] = shooting_point_y[3];
        }
    }
    if (field == Right)
    {
        if (shooting_point == 1)
        {
            Gimbal.target_point[0] = shooting_point_x[4];
            Gimbal.target_point[1] = shooting_point_y[4];
        }
        if (shooting_point == 2)
        {
            Gimbal.target_point[0] = shooting_point_x[5];
            Gimbal.target_point[1] = shooting_point_y[5];
        }
        if (shooting_point == 3)
        {
            Gimbal.target_point[0] = shooting_point_x[6];
            Gimbal.target_point[1] = shooting_point_y[6];
        }
    }
    Gimbal_controller();
    if (Theta <= 180 && Theta >= -180)
    {
        if (Position_Control(&can2motorRealInfo[Motor_SHOOT_GIMBAL], Alpha))
        {
            Gimbal.aim_flag = Position_Control(&can2motorRealInfo[Motor_SHOOT_GIMBAL], Alpha);
        }
    }
}

/**
 * @brief 自动射球初始化
 * @param 射球目标点（x，y)
 */
void shooting_Init(int X, int Y)
{
    // 云台赋值
    Gimbal.target_point[0] = X;
    Gimbal.target_point[1] = Y;
}
/**
 * @brief 自动射球发射
 * @param NULL
 */
void Auto_shooting_Shooting(void)
{
    Gimbal_controller();
    if (Theta <= 180 && Theta >= -180)
    {
        if (Position_Control(&can2motorRealInfo[Motor_SHOOT_GIMBAL], Alpha))
        {
            Gimbal.aim_flag = 1;
        }
    }
    if (Gimbal.aim_flag) // 成功瞄准
    {
        belt_ctrl(6000);
    }
}

/**
 * @brief 翻转电机控制
 *
 * @param target_angle
 * @return 0:顶端 1:到达底部
 */
// int init_cnt = 0;
int flip_motor(float target_angle)
{
    static int init_cnt = 0;
    static int up_cnt = 0;
    static int feedback = 2;
    if (target_angle == flip_down)
    {
        init_cnt = 0;
        up_cnt = 0;
        can2motorRealInfo[Motor_SHOOT_FLIP].HomingMode.done_flag = 0;
        can2motorRealInfo[Motor_SHOOT_FLIP].Velocity_Planning.done_flag = 0;
        if (can2motorRealInfo[Motor_SHOOT_FLIP].SettingMode.done_flag == 0)
        {
            Setting_Mode(&can2motorRealInfo[Motor_SHOOT_FLIP], -0.8f * flip_speed, 5000, target_angle); // 下放
        }
        else
        {
            Pos_Torque_Control(&can2motorRealInfo[Motor_SHOOT_FLIP], 1500, flip_down + 3.0f);
            feedback = 1;
        }
    }
    else if (target_angle == flip_up)
    {
        init_cnt = 0;
        can2motorRealInfo[Motor_SHOOT_FLIP].SettingMode.done_flag = 0;
        if (can2motorRealInfo[Motor_SHOOT_FLIP].Velocity_Planning.done_flag == 0)
        {
            Planning_Mode(&can2motorRealInfo[Motor_SHOOT_FLIP], flip_down + 5.0f, flip_up - 65.0f, 0.1f * flip_speed, 2.5f * flip_speed, 0.05f * flip_speed, 0.2f, 0.5f); // 上翻

            //            Planning_Mode(&can2motorRealInfo[Motor_SHOOT_FLIP], flip_down , flip_up , 0.1f * flip_speed, 2.5f * flip_speed, 0.05f * flip_speed, 0.2f, 0.5f); // 上翻
        }
        else
        {
            // Homeing_Mode(&can2motorRealInfo[Motor_SHOOT_FLIP], 0.1f * flip_speed, 2000); // 回到初始位置
            if (can2motorRealInfo[Motor_SHOOT_FLIP].HomingMode.done_flag == 0)
            {
                Homeing_Mode(&can2motorRealInfo[Motor_SHOOT_FLIP], 0.1f * flip_speed, 2000); // 回到初始位置
            }
            else
            {
                // Vel_Torque_Control(&can2motorRealInfo[Motor_SHOOT_FLIP], 500, 0.01f * flip_speed);
                if (up_cnt < 250)
                {
                    Pos_Torque_Control(&can2motorRealInfo[Motor_SHOOT_FLIP], 1500, flip_up - 1.0f);
                    up_cnt++;
                }
                else
                {
                    can2motorRealInfo[Motor_SHOOT_FLIP].Motor_Mode = MOTO_OFF;
                    feedback = 0;
                }
            }
            // feedback = 0;
        }
    }
    else if (target_angle == flip_init)
    {
        up_cnt = 0;
        can2motorRealInfo[Motor_SHOOT_FLIP].SettingMode.done_flag = 0;
        if (can2motorRealInfo[Motor_SHOOT_FLIP].Velocity_Planning.done_flag == 0 && init_cnt < 1000)
        {
            Planning_Mode(&can2motorRealInfo[Motor_SHOOT_FLIP], flip_down + 5.0f, flip_up - 30.0f, 0.1f * flip_speed, 1.5f * flip_init, 0.1f * flip_speed, 0.2f, 0.3f); // 上翻
            init_cnt++;
        }
        else
        {
            if (can2motorRealInfo[Motor_SHOOT_FLIP].HomingMode.done_flag == 0)
            {
                Homeing_Mode(&can2motorRealInfo[Motor_SHOOT_FLIP], 0.2f * flip_init, 2000); // 回到初始位置
            }
            else
            {
                can2motorRealInfo[Motor_SHOOT_FLIP].Motor_Mode = MOTO_OFF;
                feedback = 0;
            }
        }
    }
    flip_state = feedback;
    return feedback;
}

/**
 * @brief 发射摩擦带控制函数
 * @param vel 摩擦带速度
 */
void belt_ctrl(float vel)
{
    Speed_Control(&can2motorRealInfo[Motor_SHOOT_MOTOR_1], -1 * vel);
    Speed_Control(&can2motorRealInfo[Motor_SHOOT_MOTOR_2], vel);
    if (vel != 0 && Speed_Control(&can2motorRealInfo[Motor_SHOOT_MOTOR_1], -1 * vel) && Speed_Control(&can2motorRealInfo[Motor_SHOOT_MOTOR_2], vel))
    {
        shooting_enable = 1;
    }
    else
    {
        shooting_enable = 0;
    }
}

void belt_base(void)
{
    if (shoot_state == SHOOT_STATE_SHOOTING)
    {
        Speed_Control(&can2motorRealInfo[Motor_SHOOT_MOTOR_1], -1 * 5000);
        Speed_Control(&can2motorRealInfo[Motor_SHOOT_MOTOR_2], 5000);
        if (Speed_Control(&can2motorRealInfo[Motor_SHOOT_MOTOR_1], -1 * 5000) && Speed_Control(&can2motorRealInfo[Motor_SHOOT_MOTOR_2], 5000))
        {
            shooting_enable = 1;
        }
    }
}

/**
 * @brief 利用action自动判断ball
 *
 * @param
 */
void ball_auto(int field)
{
    if (field == Left) // 2030
    {
        if (1880.0f < ROBOT_CHASSI.world_x && ROBOT_CHASSI.world_x < 2180.0f)
        {
            action_ball = ROBOT_CHASSI.world_y > 500.0f ? 1 : 7;
        }
        else if (2380.0f < ROBOT_CHASSI.world_x && ROBOT_CHASSI.world_x < 2680.0f)
        {
            action_ball = ROBOT_CHASSI.world_y > 500.0f ? 2 : 8;
        }
        else if (2880.0f < ROBOT_CHASSI.world_x && ROBOT_CHASSI.world_x < 3180.0f)
        {
            action_ball = ROBOT_CHASSI.world_y > 500.0f ? 3 : 9;
        }
        else if (3380.0f < ROBOT_CHASSI.world_x && ROBOT_CHASSI.world_x < 3680.0f)
        {
            action_ball = ROBOT_CHASSI.world_y > 500.0f ? 4 : 10;
        }
        else if (3880.0f < ROBOT_CHASSI.world_x && ROBOT_CHASSI.world_x < 4180.0f)
        {
            action_ball = ROBOT_CHASSI.world_y > 500.0f ? 5 : 11;
        }
        else if (4380.0f < ROBOT_CHASSI.world_x && ROBOT_CHASSI.world_x < 4680.0f)
        {
            action_ball = ROBOT_CHASSI.world_y > 500.0f ? 6 : 12;
        }
        else
        {
            action_ball = 0;
        }
    }
    else if (field == Right) //-2010
    {
        if (-2160.0f < ROBOT_CHASSI.world_x && ROBOT_CHASSI.world_x < -1860.0f)
        {
            action_ball = ROBOT_CHASSI.world_y > 500.0f ? 6 : 12;
        }
        else if (-2660.0f < ROBOT_CHASSI.world_x && ROBOT_CHASSI.world_x < -2360.0f)
        {
            action_ball = ROBOT_CHASSI.world_y > 500.0f ? 5 : 11;
        }
        else if (-3160.0f < ROBOT_CHASSI.world_x && ROBOT_CHASSI.world_x < -2860.0f)
        {
            action_ball = ROBOT_CHASSI.world_y > 500.0f ? 4 : 10;
        }
        else if (-3660.0f < ROBOT_CHASSI.world_x && ROBOT_CHASSI.world_x < -3360.0f)
        {
            action_ball = ROBOT_CHASSI.world_y > 500.0f ? 3 : 9;
        }
        else if (-4160.0f < ROBOT_CHASSI.world_x && ROBOT_CHASSI.world_x < -3860.0f)
        {
            action_ball = ROBOT_CHASSI.world_y > 500.0f ? 2 : 8;
        }
        else if (-4660.0f < ROBOT_CHASSI.world_x && ROBOT_CHASSI.world_x < -4360.0f)
        {
            action_ball = ROBOT_CHASSI.world_y > 500.0f ? 1 : 7;
        }
        else
        {
            action_ball = 0;
        }
    }
}

/**
 * @brief 云台转动控制函数
 *
 * @param target_angle 目标角度：2000 - 4600
 */
void gimbal_ctrl(float target_pos)
{
    //    lim(&target_pos, gimbal_down, gimbal_up);
    //    if (target_pos >= gimbal_up && target_pos <= gimbal_down)
    //        Pos_Velimit_Control(&can2motorRealInfo[Motor_SHOOT_GIMBAL], gimbal_speed, target_pos);
    Position_Control(&can2motorRealInfo[Motor_SHOOT_GIMBAL], target_pos);
}

/**
 * @brief 云台摇杆自由控制函数
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

int belt_Left[4][13] = {{5000},
                        {5000,
                         4000, 4300, 4600, 4900, 5200, 5500,
                         4600, 4900, 5200, 5500, 5900, 6200},
                        {5000,
                         4200, 4400, 4600, 4800, 5100, 5600,
                         4500, 4750, 5100, 5300, 5500, 5800},
                        {5000,
                         4300, 4300, 4600, 4700, 5000, 5200,
                         5000, 5200, 5400, 5600, 5800, 6000}};

int belt_Right[4][13] = {{5000},
                         {5000,
                          5500, 5200, 4900, 4600, 4300, 4000,
                          6200, 5900, 5500, 5200, 4900, 4600},
                         {5000,
                          5600, 5100, 4800, 4600, 4400, 4200,
                          5800, 5500, 5300, 5100, 4750, 4500},
                         {5000,
                          5200, 5000, 4700, 4600, 4300, 4300,
                          6000, 5800, 5600, 5400, 5200, 5000}};

void belt_calc(int f, int point)
{
    if (f == Left)
    {
        belt_speed = belt_Left[point][action_ball];
    }
    else if (f == Right)
    {
        belt_speed = belt_Right[point][action_ball];
    }
}
