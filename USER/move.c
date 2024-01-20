//
// Created by Ray on 2023/12/21.
//

#include "move.h"

// 取箭
float X[5+4] = {0.0, 0.0, 0.0, -2650.0, -5250.0, -7875.0, -10600.0, -10600.0, -10600.0};
float Y[5+4] = {0.0, 0.0, 0.0,   175.0,   350.0,   525.0,  800.0,  800.0,  800.0};
float Yaw[5+4] = {0};

// 取箭架到射箭地点
float X1[5+4] = {-10600.0, -10600.0, -10600.0, -9300.0, -8000.0, -6700.0, -5400.0, -5400.0, -5400.0};
float Y1[5+4] = {800.0, 800.0, 800.0,   775.0,  750.0,  725.0,  700.0, 700.0, 700.0};
float Yaw1[5+4] = {0};


MOVE_STATE_ITEMS MOVE_STATE = MOVE_STOP;
PID_T yaw_pid = {0};
PID_T point_X_pid = {0};
PID_T point_Y_pid = {0};


/**
* @brief  Move_Init初始化
* @note		移动相关PID初始化
* @param
* @retval
*/
void MoveInit(void)
{
    pid_param_init(&yaw_pid, PID_Position, 0.3, 0.05, 0, 0.1f, 180, 0.025f, 0.001f, 0.33f);
    pid_param_init(&point_X_pid, PID_Position, 0.3, 0.05, 0, 0.1f, 0.5, 0.025f, 0.001f, 0.33f);
    pid_param_init(&point_Y_pid, PID_Position, 0.3, 0.05, 0, 0.1f, 0.5, 0.03f, 0.001f, 0.015f);
}


/**
* @brief  AngleLimit角度限幅
* @note		将角度限制在-180°到180°
* @param  angle:要限制的值
* @retval
*/
void AngleLimit(float *angle)
{
    static uint8_t recursiveTimes = 0;

    recursiveTimes++;

    if(recursiveTimes<100)
    {
        if(*angle>180.0f)
        {
            *angle-=360.0f;
            AngleLimit(angle);
        }
        else if(*angle<-180.0f)
        {
            *angle+=360.0f;
            AngleLimit(angle);
        }
    }

    recursiveTimes--;
}


/**
* @brief  YawAdjust偏航角控制
* @note		将偏航角控制在目标角度
* @param  Target_angle:要限制的值
* @retval
*/
void YawAdjust(float Target_angle)
{
    float error;

    // 计算误差
    if(ROBOT_CHASSI.world_w * Target_angle >= 0)
    {
        error = Target_angle - ROBOT_CHASSI.world_w;
    }
    else
    {
        if(ABS(ROBOT_CHASSI.world_w)+ABS(Target_angle) <= 180) error = Target_angle - ROBOT_CHASSI.world_w;
        else
        {
            AngleLimit(&error);
        }
    }

    // 直接利用PID输出角速度
    ROBOT_CHASSI.plan_w = -1 * pid_calc(&yaw_pid, error, 0);	// 底盘角速度 单位：rad/s
}




/**
* @brief  LockupPoint锁定车
* @note		将车锁定在某一点上
* @param  POS_X:要限制的X值，POS_Y:要限制的Y值，POS_YAW:要限制的偏航角
* @retval
*/
void LockupPoint(float POS_X, float POS_Y, float POS_YAW)
{
    YawAdjust(POS_YAW);

    ROBOT_CHASSI.plan_x = pid_calc(&point_X_pid, POS_X, ROBOT_CHASSI.world_x);
    ROBOT_CHASSI.plan_x = pid_calc(&point_Y_pid, POS_Y, ROBOT_CHASSI.world_y);
}




float kp_x = 0.008;
float kd_x = 0;	//0.00011
float kp_y = 0.008;
float kd_y = 0;	//0.00011
float kp_yaw = 0.01;
float kd_yaw = 0;
float error_X;float error_Y;	// 世界X、Y偏差
float error_x;float error_y;	// 本体x、y偏差
float error_Yaw;							// 偏航角偏差
float now_yaw;								// 当前弧度制偏航角
float u_output;								// 本体坐标x方向速度输出
float v_output;								// 本体坐标y方向速度输出
float w_ouput;								// 角速度输出
/**
* @brief    PDController跟踪器
* @note	    跟踪规划好的路径
* @param    target_point:单位时间要跟踪的点（需先规划好速度），robot_now_pos:机器人当前世界坐标下的位置
* @retval
*/
void PDController(PATH_TYPEDEF target_point, ROBOT_REAL_POS robot_now_pos)
{
    // 计算误差
    error_X = target_point.X - robot_now_pos.world_x;
    error_Y = target_point.Y - robot_now_pos.world_y;
    error_Yaw = target_point.Yaw - robot_now_pos.world_w;
    //角度制转换为弧度制
    now_yaw = robot_now_pos.world_w * PI / 180.0f;
    // 换算到本体坐标
    error_x =  cos(now_yaw) * error_X + sin(now_yaw) * error_Y;
    error_y = -sin(now_yaw) * error_X + cos(now_yaw) * error_Y;

    // 计算速度
    w_ouput  = (kp_yaw * error_Yaw + kd_yaw * target_point.W) / (1 + kd_yaw);
    u_output = (kp_x*error_x + kd_x*( target_point.V_x  * cos(now_yaw) + \
																		target_point.V_y  * sin(now_yaw) + \
																		w_ouput * error_y * cos(now_yaw) - \
																		w_ouput * error_x * sin(now_yaw)))/(1 + kd_x);
    v_output = (kp_y*error_y + kd_y*(-target_point.V_x  * sin(now_yaw) + \
																		target_point.V_y  * cos(now_yaw) - \
																		w_ouput * error_y * sin(now_yaw) - \
																		w_ouput * error_x * cos(now_yaw)))/(1+kd_y);

    // 换算为世界坐标系下的速度
    ROBOT_CHASSI.plan_y = u_output * cos(now_yaw) - v_output * sin(now_yaw);
    ROBOT_CHASSI.plan_x = u_output * sin(now_yaw) + v_output * cos(now_yaw);
    ROBOT_CHASSI.plan_w  = -w_ouput;
}




int k;
float t;
float f1s;float f2s;float f3s;float f4s;
float last_X;float last_Y;float last_Yaw;
float Sx_error;float Sy_error;
float Hz;
int first_time_flag = 1;
PATH_TYPEDEF now_path_point;
/**
* @brief  PathPlan规划+跟踪
* @note		三次B样条规划，误差直接赋值，到达终点返回1，否则返回0
* @param  t_real:真实经过的时间，t_target:目标总时间，num:控制点数目+1，X、Y:控制点数组
* @retval
*/
int PathPlan(float t_real, float t_target, int num, float *X , float *Y, float *Yaw)
{
    k = (int)(t_real * num / t_target);	// 第k段
    t = t_real - k * t_target / num;		// 第k段时间
    t = t * num / t_target;							// 归一化

    // 位置样条函数
    f1s = (1 - t) * (1 - t) * (1 - t) / 6;
    f2s = (3 * t * t * t - 6 * t * t + 4) / 6;
    f3s = (-3 * t * t * t + 3 * t * t + 3 * t + 1) / 6;
    f4s = (t * t * t) / 6;

    // 计算目标跟踪点
    now_path_point.X = X[k] * f1s + X[k+1] * f2s + X[k+2] * f3s + X[k+3] * f4s;
    now_path_point.Y = Y[k] * f1s + Y[k+1] * f2s + Y[k+2] * f3s + Y[k+3] * f4s;
    now_path_point.Yaw = Yaw[k] * f1s + Yaw[k+1] * f2s + Yaw[k+2] * f3s + Yaw[k+3] * f4s;
    if(first_time_flag)
    {
        now_path_point.V_x = 0;
        now_path_point.V_y = 0;
        now_path_point.W = 0;
        first_time_flag = 0;
        Hz = 1 / t_real;
    }
    else
    {
        now_path_point.V_x = (now_path_point.X - last_X) * Hz;
        now_path_point.V_y = (now_path_point.Y - last_Y) * Hz;
        now_path_point.W = (now_path_point.Yaw - last_Yaw) * Hz;
    }

    // PD跟踪器
    PDController(now_path_point, ROBOT_REAL_POS_DATA);

    // 保留本次值
    last_X = now_path_point.X;
    last_Y = now_path_point.Y;
    last_Yaw = now_path_point.Yaw;

    // 到达终点
    if(t_real > t_target)
    {
        ROBOT_CHASSI.plan_x = 0;
        ROBOT_CHASSI.plan_y = 0;
        first_time_flag = 1;
        return 1;
    }
    return 0;
}
