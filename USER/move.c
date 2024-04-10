//
// Created by Ray on 2023/12/21.
//

#include "move.h"

float kp_x = 6;
float kd_x = 0;
float kp_y = 6;
float kd_y = 0;
float kp_yaw = 1;
float kd_yaw = 0;
float error_X;
float error_Y; // 世界X,Y偏差
float error_x;
float error_y;	 // 本体x，y偏差
float error_Yaw; // 偏航角偏差
float now_yaw;	 // 当前弧度制偏航角
float u_output;	 // 本体x方向速度输出
float v_output;	 // 本体坐标y方向速度输出
float w_output;	 // 角速度输出
float Error = 0;

/*pid相关结构体--------------------------------------*/
PID_T point_traker_x_pid;
PID_T point_traker_y_pid;
PID_T point_traker_yaw_pid;
PID_T TRACK_PID;
PID_T track_pid;

int tim;
// 下面是多点切换走直线，修正误差
uint32_t getSysTickCnt()
{
	return HAL_GetTick();
}

void movebase()
{
	// 初始化梯形规划的初始位置

	// float POSX_TrapezoidPlaning = 0;
	// float POSY_TrapezoidPlaning = 0;

	//	calculation();
	//	if(chassis_TrapezoidPlaning(POSX_TrapezoidPlaning,POSY_TrapezoidPlaning,-800.0,3000.0,0.0,100.0,100.0,2000,0.2,0.55))
	//	if(Laser_calibration(2000,1000,0,500))
	//		if(Laser2_calibration(location_y,location_x))
	//            //     if(PathPlan(move_time_counter,2.8,7, X0 , Y0, Yaw0))
	//			{
	//		      //使用激光更新T型规划初始值  并进入第二段，DT35数据进行T型规划  第三第四个激光
	//				POSX_TrapezoidPlaning = ROBOT_CHASSI.world_x;
	//				POSY_TrapezoidPlaning = ROBOT_CHASSI.world_y;
	//			 auto_state = auto_init;
	//			}
}

float move_flag = 1; // 用于更新T型规划初始值的标志位
float move_1flag = 1;
// 移动到取苗端 俩段T型规划抵达（后续如果另一个算法搞好，就换）
void move_seed()
{
	// 初始化梯形规划的初始位置
	float POSX_TrapezoidPlaning = 0;
	float POSY_TrapezoidPlaning = 0;
	uint16_t tt = 0;
	//	if(PATH_THACKING(point,2))
	if (chassis_TrapezoidPlaning(POSX_TrapezoidPlaning, POSY_TrapezoidPlaning, 0.0, 50.0, 0.0, 100.0, 100.0, 2000, 0.2, 0.55))
	{
		if (move_flag)
		{
			// 更新T型规划位置初始值  直接赋值也可以的
			POSX_TrapezoidPlaning = ROBOT_CHASSI.world_x;
			POSY_TrapezoidPlaning = ROBOT_CHASSI.world_y;
			move_flag = 0;
		}
		tt++;
		// 加个延迟切换点
		if (tt > 300)
		{

			// 抵达取苗点
			if (chassis_TrapezoidPlaning(POSX_TrapezoidPlaning, POSY_TrapezoidPlaning, -800.0, 50.0, 0.0, 100.0, 100.0, 2000, 0.2, 0.55))
			{
				if (move_1flag)
				{
					// 更新T型规划位置初始值
					POSX_TrapezoidPlaning = ROBOT_CHASSI.world_x;
					POSY_TrapezoidPlaning = ROBOT_CHASSI.world_y;
					move_1flag = 0;
				}
				// 双激光使车体对正 X Y
				if (Laser2_calibration(DT35.y, DT35.x))
				{
					// 激光矫正 Y K
					if (Laser_calibration(2000, 1000, 0, 500))
					{
						// 不确定要不要加上倒车，看看测试 这里加上夹爪夹苗之类的即可
					}
				}
			}
		}
	}
}

// 移动到放苗点
void put_seed()
{
	// 更新T型规划位置初始值
	float POSX_TrapezoidPlaning = ROBOT_CHASSI.world_x;
	float POSY_TrapezoidPlaning = ROBOT_CHASSI.world_y;
	// 车体转180度
	if (chassis_TrapezoidPlaning(POSX_TrapezoidPlaning, POSY_TrapezoidPlaning, -800.0, 500.0, 180.0, 100.0, 100.0, 2000, 0.2, 0.55))
	{
		// 双激光使车体对正 X Y
		if (Laser2_calibration(DT35.y, DT35.x))
		{
			// 激光矫正 Y K
			if (Laser_calibration(2000, 1000, 0, 500))
			{
				// 不确定要不要加上倒车，看看测试 这里加上夹爪放苗之类的即可
			}
		}
	}
}

// 自动检测是否到位，并且取夹球
void auto_detection()
{
	if ((500 < DT35.y && DT35.y < 700) && (1000 < DT35.k && DT35.k < 1200) && (ROBOT_CHASSI.world_x < 700 && ROBOT_CHASSI.world_y > 660) &&
		(ROBOT_CHASSI.world_y < 600 && ROBOT_CHASSI.world_y > 560))
	{
		Laser_calibration(600, 1100, 0, 500);
	}
}

/**
 * @brief  AngleLimit角度限幅
 * @note		将角度限制在-180°到180°
 * @param  angle:要限制的值
 * @retval
 */
void AngleLimit(float *angle)
{
	static uint8_t recursiveTimes = 0; // 避免无限递归导致程序异常
	recursiveTimes++;

	if (recursiveTimes < 100)
	{
		if (*angle > 180.0f)
		{
			*angle -= 360.0f;
			AngleLimit(angle);
		}
		else if (*angle < -180.0f)
		{
			*angle += 360.0f;
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
int YawAdjust(float Target_angle)
{
	float YawAdjust_error;
	// 计算误差
	if (ROBOT_CHASSI.world_w * Target_angle >= 0)
	{
		YawAdjust_error = Target_angle - ROBOT_CHASSI.world_w;
	}
	else
	{
		if (ABS(ROBOT_CHASSI.world_w) + ABS(Target_angle) <= 180)
			YawAdjust_error = Target_angle - ROBOT_CHASSI.world_w;
		else
		{
			AngleLimit(&YawAdjust_error);
		}
	}
	// 直接利用PID输出角速度
	pid_calc_by_error(&point_traker_yaw_pid, YawAdjust_error);
	ROBOT_CHASSI.plan_w = -point_traker_yaw_pid.output; // 底盘角速度 单位：rad/s

	if (ABS(YawAdjust_error) < 0.2f)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

// 定义变量名称
float Ssu_chassis; // 总路程
float Sac_chassis; // 加速路程
float Sde_chassis; // 减速路程
float Sco_chassis; // 匀速路程
float Aac_chassis; // 加速加速度
float Ade_chassis; // 减速加速度
float S_chassis;   // 当前路程
float output_V;	   // 输出的速度
float real_error;  // 真实误差
// 下面看需不需要
float x_error_end_Trapezoid; // 到结束点的误差
float y_error_end_Trapezoid;
float x_error_start_Trapezoid; // 到开始点的误差
float y_error_start_Trapezoid;
// 路径规划，A*算法(使用欧几里距离)
int chassis_TrapezoidPlaning(float POS_X_start,
							 float POS_Y_start,
							 float POS_X_end,
							 float POS_Y_end,
							 float POS_Yaw,
							 float V_start,
							 float V_end,
							 float V_max,
							 float R_ac,
							 float R_de)
{

	x_error_end_Trapezoid = POS_X_end - ROBOT_CHASSI.world_x; // 到结束点的误差
	y_error_end_Trapezoid = POS_Y_end - ROBOT_CHASSI.world_y;
	x_error_start_Trapezoid = POS_X_start - ROBOT_CHASSI.world_x; // 到开始点的误差
	y_error_start_Trapezoid = POS_Y_start - ROBOT_CHASSI.world_y;
	real_error = sqrt((x_error_end_Trapezoid) * (x_error_end_Trapezoid) + (y_error_end_Trapezoid) * (y_error_end_Trapezoid));

	Ssu_chassis = sqrt((POS_X_start - POS_X_end) * (POS_X_start - POS_X_end) + (POS_Y_start - POS_Y_end) * (POS_Y_start - POS_Y_end));

	Sac_chassis = Ssu_chassis * R_ac;
	Sde_chassis = Ssu_chassis * R_de;
	Sco_chassis = Ssu_chassis - Sac_chassis - Sde_chassis;
	Aac_chassis = (V_max * V_max - V_start * V_start) / (2.0f * Sac_chassis);														 // 加速加速度 (最大的速度*最大的速度 - 开始的速度 *开始的速度 ) / (2.0f * 加速路程)
	Ade_chassis = (V_end * V_end - V_max * V_max) / (2.0f * Sde_chassis);															 // 减速加速度
	S_chassis = sqrt((x_error_start_Trapezoid) * (x_error_start_Trapezoid) + (y_error_start_Trapezoid) * (y_error_start_Trapezoid)); // 开始位置

	// 规划RPM
	if (S_chassis < Sac_chassis)
	{
		output_V = sqrt(2.0f * Aac_chassis * S_chassis + V_start * V_start);
		// YawAdjust(POS_YAW);//加速阶段不调整角度

	} // 加速阶段
	else if (S_chassis < (Sac_chassis + Sco_chassis))
	{
		//			output_V = sqrt(2.0f * Aac_chassis * Sac_chassis + V_start * V_start);
		output_V = V_max;
	} // 匀速阶段
	else if (S_chassis < Ssu_chassis)
	{
		output_V = sqrt(V_end * V_end - 2.0f * Ade_chassis * ABS(Ssu_chassis - S_chassis));
		YawAdjust(POS_Yaw); // 锁死角度
	}						// 减速阶段

	//		else
	//			output_V=V_end;
	//
	//	}
	// 分解速度，并分配合适得正负号
	ROBOT_CHASSI.plan_x = -(output_V * 1.0f * (x_error_end_Trapezoid) / real_error); // x轴
	ROBOT_CHASSI.plan_y = (output_V * 1.0f * (y_error_end_Trapezoid) / real_error);	 // y轴
	// ABS(ROBOT_CHASSI.world_x - POS_X_end)<100&&ABS(ROBOT_CHASSI.Vy - POS_Y_end)
	if ((Ssu_chassis - S_chassis) < 10) // 提前跳出
	{

		output_V = 0;
		ROBOT_CHASSI.plan_x = 0;
		ROBOT_CHASSI.plan_y = 0;
		return 1;
	}
	else
		return 0;
}
