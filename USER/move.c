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
float error_Y; // ����X,Yƫ��
float error_x;
float error_y;	 // ����x��yƫ��
float error_Yaw; // ƫ����ƫ��
float now_yaw;	 // ��ǰ������ƫ����
float u_output;	 // ����x�����ٶ����
float v_output;	 // ��������y�����ٶ����
float w_output;	 // ���ٶ����
float Error = 0;

/*pid��ؽṹ��--------------------------------------*/
PID_T point_traker_x_pid;
PID_T point_traker_y_pid;
PID_T point_traker_yaw_pid;
PID_T TRACK_PID;
PID_T track_pid;

int tim;
// �����Ƕ���л���ֱ�ߣ��������
uint32_t getSysTickCnt()
{
	return HAL_GetTick();
}

void movebase()
{
	// ��ʼ�����ι滮�ĳ�ʼλ��

	// float POSX_TrapezoidPlaning = 0;
	// float POSY_TrapezoidPlaning = 0;

	//	calculation();
	//	if(chassis_TrapezoidPlaning(POSX_TrapezoidPlaning,POSY_TrapezoidPlaning,-800.0,3000.0,0.0,100.0,100.0,2000,0.2,0.55))
	//	if(Laser_calibration(2000,1000,0,500))
	//		if(Laser2_calibration(location_y,location_x))
	//            //     if(PathPlan(move_time_counter,2.8,7, X0 , Y0, Yaw0))
	//			{
	//		      //ʹ�ü������T�͹滮��ʼֵ  ������ڶ��Σ�DT35���ݽ���T�͹滮  �������ĸ�����
	//				POSX_TrapezoidPlaning = ROBOT_CHASSI.world_x;
	//				POSY_TrapezoidPlaning = ROBOT_CHASSI.world_y;
	//			 auto_state = auto_init;
	//			}
}

float move_flag = 1; // ���ڸ���T�͹滮��ʼֵ�ı�־λ
float move_1flag = 1;
// �ƶ���ȡ��� ����T�͹滮�ִ���������һ���㷨��ã��ͻ���
void move_seed()
{
	// ��ʼ�����ι滮�ĳ�ʼλ��
	float POSX_TrapezoidPlaning = 0;
	float POSY_TrapezoidPlaning = 0;
	uint16_t tt = 0;
	//	if(PATH_THACKING(point,2))
	if (chassis_TrapezoidPlaning(POSX_TrapezoidPlaning, POSY_TrapezoidPlaning, 0.0, 50.0, 0.0, 100.0, 100.0, 2000, 0.2, 0.55))
	{
		if (move_flag)
		{
			// ����T�͹滮λ�ó�ʼֵ  ֱ�Ӹ�ֵҲ���Ե�
			POSX_TrapezoidPlaning = ROBOT_CHASSI.world_x;
			POSY_TrapezoidPlaning = ROBOT_CHASSI.world_y;
			move_flag = 0;
		}
		tt++;
		// �Ӹ��ӳ��л���
		if (tt > 300)
		{

			// �ִ�ȡ���
			if (chassis_TrapezoidPlaning(POSX_TrapezoidPlaning, POSY_TrapezoidPlaning, -800.0, 50.0, 0.0, 100.0, 100.0, 2000, 0.2, 0.55))
			{
				if (move_1flag)
				{
					// ����T�͹滮λ�ó�ʼֵ
					POSX_TrapezoidPlaning = ROBOT_CHASSI.world_x;
					POSY_TrapezoidPlaning = ROBOT_CHASSI.world_y;
					move_1flag = 0;
				}
				// ˫����ʹ������� X Y
				if (Laser2_calibration(DT35.y, DT35.x))
				{
					// ������� Y K
					if (Laser_calibration(2000, 1000, 0, 500))
					{
						// ��ȷ��Ҫ��Ҫ���ϵ������������� ������ϼ�צ����֮��ļ���
					}
				}
			}
		}
	}
}

// �ƶ��������
void put_seed()
{
	// ����T�͹滮λ�ó�ʼֵ
	float POSX_TrapezoidPlaning = ROBOT_CHASSI.world_x;
	float POSY_TrapezoidPlaning = ROBOT_CHASSI.world_y;
	// ����ת180��
	if (chassis_TrapezoidPlaning(POSX_TrapezoidPlaning, POSY_TrapezoidPlaning, -800.0, 500.0, 180.0, 100.0, 100.0, 2000, 0.2, 0.55))
	{
		// ˫����ʹ������� X Y
		if (Laser2_calibration(DT35.y, DT35.x))
		{
			// ������� Y K
			if (Laser_calibration(2000, 1000, 0, 500))
			{
				// ��ȷ��Ҫ��Ҫ���ϵ������������� ������ϼ�צ����֮��ļ���
			}
		}
	}
}

// �Զ�����Ƿ�λ������ȡ����
void auto_detection()
{
	if ((500 < DT35.y && DT35.y < 700) && (1000 < DT35.k && DT35.k < 1200) && (ROBOT_CHASSI.world_x < 700 && ROBOT_CHASSI.world_y > 660) &&
		(ROBOT_CHASSI.world_y < 600 && ROBOT_CHASSI.world_y > 560))
	{
		Laser_calibration(600, 1100, 0, 500);
	}
}

/**
 * @brief  AngleLimit�Ƕ��޷�
 * @note		���Ƕ�������-180�㵽180��
 * @param  angle:Ҫ���Ƶ�ֵ
 * @retval
 */
void AngleLimit(float *angle)
{
	static uint8_t recursiveTimes = 0; // �������޵ݹ鵼�³����쳣
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
 * @brief  YawAdjustƫ���ǿ���
 * @note		��ƫ���ǿ�����Ŀ��Ƕ�
 * @param  Target_angle:Ҫ���Ƶ�ֵ
 * @retval
 */
int YawAdjust(float Target_angle)
{
	float YawAdjust_error;
	// �������
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
	// ֱ������PID������ٶ�
	pid_calc_by_error(&point_traker_yaw_pid, YawAdjust_error);
	ROBOT_CHASSI.plan_w = -point_traker_yaw_pid.output; // ���̽��ٶ� ��λ��rad/s

	if (ABS(YawAdjust_error) < 0.2f)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

// �����������
float Ssu_chassis; // ��·��
float Sac_chassis; // ����·��
float Sde_chassis; // ����·��
float Sco_chassis; // ����·��
float Aac_chassis; // ���ټ��ٶ�
float Ade_chassis; // ���ټ��ٶ�
float S_chassis;   // ��ǰ·��
float output_V;	   // ������ٶ�
float real_error;  // ��ʵ���
// ���濴�費��Ҫ
float x_error_end_Trapezoid; // ������������
float y_error_end_Trapezoid;
float x_error_start_Trapezoid; // ����ʼ������
float y_error_start_Trapezoid;
// ·���滮��A*�㷨(ʹ��ŷ�������)
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

	x_error_end_Trapezoid = POS_X_end - ROBOT_CHASSI.world_x; // ������������
	y_error_end_Trapezoid = POS_Y_end - ROBOT_CHASSI.world_y;
	x_error_start_Trapezoid = POS_X_start - ROBOT_CHASSI.world_x; // ����ʼ������
	y_error_start_Trapezoid = POS_Y_start - ROBOT_CHASSI.world_y;
	real_error = sqrt((x_error_end_Trapezoid) * (x_error_end_Trapezoid) + (y_error_end_Trapezoid) * (y_error_end_Trapezoid));

	Ssu_chassis = sqrt((POS_X_start - POS_X_end) * (POS_X_start - POS_X_end) + (POS_Y_start - POS_Y_end) * (POS_Y_start - POS_Y_end));

	Sac_chassis = Ssu_chassis * R_ac;
	Sde_chassis = Ssu_chassis * R_de;
	Sco_chassis = Ssu_chassis - Sac_chassis - Sde_chassis;
	Aac_chassis = (V_max * V_max - V_start * V_start) / (2.0f * Sac_chassis);														 // ���ټ��ٶ� (�����ٶ�*�����ٶ� - ��ʼ���ٶ� *��ʼ���ٶ� ) / (2.0f * ����·��)
	Ade_chassis = (V_end * V_end - V_max * V_max) / (2.0f * Sde_chassis);															 // ���ټ��ٶ�
	S_chassis = sqrt((x_error_start_Trapezoid) * (x_error_start_Trapezoid) + (y_error_start_Trapezoid) * (y_error_start_Trapezoid)); // ��ʼλ��

	// �滮RPM
	if (S_chassis < Sac_chassis)
	{
		output_V = sqrt(2.0f * Aac_chassis * S_chassis + V_start * V_start);
		// YawAdjust(POS_YAW);//���ٽ׶β������Ƕ�

	} // ���ٽ׶�
	else if (S_chassis < (Sac_chassis + Sco_chassis))
	{
		//			output_V = sqrt(2.0f * Aac_chassis * Sac_chassis + V_start * V_start);
		output_V = V_max;
	} // ���ٽ׶�
	else if (S_chassis < Ssu_chassis)
	{
		output_V = sqrt(V_end * V_end - 2.0f * Ade_chassis * ABS(Ssu_chassis - S_chassis));
		YawAdjust(POS_Yaw); // �����Ƕ�
	}						// ���ٽ׶�

	//		else
	//			output_V=V_end;
	//
	//	}
	// �ֽ��ٶȣ���������ʵ�������
	ROBOT_CHASSI.plan_x = -(output_V * 1.0f * (x_error_end_Trapezoid) / real_error); // x��
	ROBOT_CHASSI.plan_y = (output_V * 1.0f * (y_error_end_Trapezoid) / real_error);	 // y��
	// ABS(ROBOT_CHASSI.world_x - POS_X_end)<100&&ABS(ROBOT_CHASSI.Vy - POS_Y_end)
	if ((Ssu_chassis - S_chassis) < 10) // ��ǰ����
	{

		output_V = 0;
		ROBOT_CHASSI.plan_x = 0;
		ROBOT_CHASSI.plan_y = 0;
		return 1;
	}
	else
		return 0;
}
