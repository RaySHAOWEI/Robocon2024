//
// Created by Ray on 2023/12/21.
//

#include "move.h"

// ȡ��
float X[5+4] = {0.0, 0.0, 0.0, -2650.0, -5250.0, -7875.0, -10600.0, -10600.0, -10600.0};
float Y[5+4] = {0.0, 0.0, 0.0,   175.0,   350.0,   525.0,  800.0,  800.0,  800.0};
float Yaw[5+4] = {0};

// ȡ���ܵ�����ص�
float X1[5+4] = {-10600.0, -10600.0, -10600.0, -9300.0, -8000.0, -6700.0, -5400.0, -5400.0, -5400.0};
float Y1[5+4] = {800.0, 800.0, 800.0,   775.0,  750.0,  725.0,  700.0, 700.0, 700.0};
float Yaw1[5+4] = {0};


MOVE_STATE_ITEMS MOVE_STATE = MOVE_STOP;
PID_T yaw_pid = {0};
PID_T point_X_pid = {0};
PID_T point_Y_pid = {0};


/**
* @brief  Move_Init��ʼ��
* @note		�ƶ����PID��ʼ��
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
* @brief  AngleLimit�Ƕ��޷�
* @note		���Ƕ�������-180�㵽180��
* @param  angle:Ҫ���Ƶ�ֵ
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
* @brief  YawAdjustƫ���ǿ���
* @note		��ƫ���ǿ�����Ŀ��Ƕ�
* @param  Target_angle:Ҫ���Ƶ�ֵ
* @retval
*/
void YawAdjust(float Target_angle)
{
    float error;

    // �������
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

    // ֱ������PID������ٶ�
    ROBOT_CHASSI.plan_w = -1 * pid_calc(&yaw_pid, error, 0);	// ���̽��ٶ� ��λ��rad/s
}




/**
* @brief  LockupPoint������
* @note		����������ĳһ����
* @param  POS_X:Ҫ���Ƶ�Xֵ��POS_Y:Ҫ���Ƶ�Yֵ��POS_YAW:Ҫ���Ƶ�ƫ����
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
float error_X;float error_Y;	// ����X��Yƫ��
float error_x;float error_y;	// ����x��yƫ��
float error_Yaw;							// ƫ����ƫ��
float now_yaw;								// ��ǰ������ƫ����
float u_output;								// ��������x�����ٶ����
float v_output;								// ��������y�����ٶ����
float w_ouput;								// ���ٶ����
/**
* @brief    PDController������
* @note	    ���ٹ滮�õ�·��
* @param    target_point:��λʱ��Ҫ���ٵĵ㣨���ȹ滮���ٶȣ���robot_now_pos:�����˵�ǰ���������µ�λ��
* @retval
*/
void PDController(PATH_TYPEDEF target_point, ROBOT_REAL_POS robot_now_pos)
{
    // �������
    error_X = target_point.X - robot_now_pos.world_x;
    error_Y = target_point.Y - robot_now_pos.world_y;
    error_Yaw = target_point.Yaw - robot_now_pos.world_w;
    //�Ƕ���ת��Ϊ������
    now_yaw = robot_now_pos.world_w * PI / 180.0f;
    // ���㵽��������
    error_x =  cos(now_yaw) * error_X + sin(now_yaw) * error_Y;
    error_y = -sin(now_yaw) * error_X + cos(now_yaw) * error_Y;

    // �����ٶ�
    w_ouput  = (kp_yaw * error_Yaw + kd_yaw * target_point.W) / (1 + kd_yaw);
    u_output = (kp_x*error_x + kd_x*( target_point.V_x  * cos(now_yaw) + \
																		target_point.V_y  * sin(now_yaw) + \
																		w_ouput * error_y * cos(now_yaw) - \
																		w_ouput * error_x * sin(now_yaw)))/(1 + kd_x);
    v_output = (kp_y*error_y + kd_y*(-target_point.V_x  * sin(now_yaw) + \
																		target_point.V_y  * cos(now_yaw) - \
																		w_ouput * error_y * sin(now_yaw) - \
																		w_ouput * error_x * cos(now_yaw)))/(1+kd_y);

    // ����Ϊ��������ϵ�µ��ٶ�
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
* @brief  PathPlan�滮+����
* @note		����B�����滮�����ֱ�Ӹ�ֵ�������յ㷵��1�����򷵻�0
* @param  t_real:��ʵ������ʱ�䣬t_target:Ŀ����ʱ�䣬num:���Ƶ���Ŀ+1��X��Y:���Ƶ�����
* @retval
*/
int PathPlan(float t_real, float t_target, int num, float *X , float *Y, float *Yaw)
{
    k = (int)(t_real * num / t_target);	// ��k��
    t = t_real - k * t_target / num;		// ��k��ʱ��
    t = t * num / t_target;							// ��һ��

    // λ����������
    f1s = (1 - t) * (1 - t) * (1 - t) / 6;
    f2s = (3 * t * t * t - 6 * t * t + 4) / 6;
    f3s = (-3 * t * t * t + 3 * t * t + 3 * t + 1) / 6;
    f4s = (t * t * t) / 6;

    // ����Ŀ����ٵ�
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

    // PD������
    PDController(now_path_point, ROBOT_REAL_POS_DATA);

    // ��������ֵ
    last_X = now_path_point.X;
    last_Y = now_path_point.Y;
    last_Yaw = now_path_point.Yaw;

    // �����յ�
    if(t_real > t_target)
    {
        ROBOT_CHASSI.plan_x = 0;
        ROBOT_CHASSI.plan_y = 0;
        first_time_flag = 1;
        return 1;
    }
    return 0;
}
