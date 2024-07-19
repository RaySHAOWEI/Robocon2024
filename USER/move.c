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
float tr1 = 0;

/*pid��ؽṹ��--------------------------------------*/
PID_T point_traker_x_pid;
PID_T point_traker_y_pid;
PID_T point_traker_yaw_pid;
PID_T TRACK_PID;
PID_T track_pid;

int tim;
float tr = 0;
float current_laser[2];
float target_action[2];
// �����Ƕ���л���ֱ�ߣ��������
uint32_t getSysTickCnt()
{
	return HAL_GetTick();
}
float POSX_TrapezoidPlaning = 0;
float POSY_TrapezoidPlaning = 0;
float tt2 = 0;
void movebase()
{
	tt2 = 1;
	// ��ʼ�����ι滮�ĳ�ʼλ��

	// float POSX_TrapezoidPlaning = 0;
	// float POSY_TrapezoidPlaning = 0;

	//	calculation();
	//	if(chassis_TrapezoidPlaning(POSX_TrapezoidPlaning,POSY_TrapezoidPlaning,-800.0,3000.0,0.0,100.0,100.0,2000,0.2,0.55))
	//	if(Laser_calibration(2000,1000,0,500))
	//			if(Laser2_calibration(DT35.y,DT35.x))
	//				if(action_calibration())
	if (YawAdjust(90.0))
	//            //     if(PathPlan(move_time_counter,2.8,7, X0 , Y0, Yaw0))
	{
		tt2 = 2;
		ROBOT_CHASSI.plan_x = 0;
		ROBOT_CHASSI.plan_y = 0;
		ROBOT_CHASSI.plan_w = 0;
		move_state = MOVE_STATE_INIT;
		//		      //ʹ�ü������T�͹滮��ʼֵ  ������ڶ��Σ�DT35���ݽ���T�͹滮  �������ĸ�����
		//				POSX_TrapezoidPlaning = ROBOT_CHASSI.world_x;
		//				POSY_TrapezoidPlaning = ROBOT_CHASSI.world_y;
		//			 auto_state = auto_init;
	}
}

float action_Init = 0; // ���ڵ�R1������
// ������������ʼ��action
void action_init()
{
	ACTION_GL_POS_DATA.ANGLE_X = 0.0f;
	ACTION_GL_POS_DATA.ANGLE_X = 0.0f;
	action_Init = 1;
}

// �ƶ���ȡ��� ����T�͹滮�ִ�
float flag = 0;
float flag1 = 0;
float tr3 = 0;
void move_seed()
{
	// ��ʼ�Ļ�ȡactionֵ�ľͲ�����
	if (seed == 1)
	{
		// ��Щ��־λ��һ���Եģ��������費��Ҫ�ĳɶ��
		if (flag == 0)
		{ // ������Ǳ���  X Y
			tr3 = 1;
			seed_state = SEED_STATE_INIT;
			if (chassis_TrapezoidPlaning(0.0, 0.0, 0.0, 300.0, 0.0, 200.0, 100.0, 2000, 0.3, 0.55, 1.0, 1.3))
			{
				flag = 1;
				//				ROBOT_CHASSI.plan_x = 0;
				//				ROBOT_CHASSI.plan_y = 0;
				//				ROBOT_CHASSI.plan_w = 0;
				//			   move_state = MOVE_STATE_INIT;
			}
		}

		else
		{
			//			if(!flag1)
			//			{
			if (update_action_seed)
			{
				POSX_TrapezoidPlaning = ROBOT_CHASSI.world_x;
				POSY_TrapezoidPlaning = ROBOT_CHASSI.world_y;
				update_action_seed = 0;
			}
			// �ִ�ȡ���
			if (chassis_TrapezoidPlaning(POSX_TrapezoidPlaning, POSY_TrapezoidPlaning, 3281, 290, 0.0, 200.0, 100.0, 3500, 0.2, 0.60, 1.3, 1.0))
			{
				ROBOT_CHASSI.plan_x = 0;
				ROBOT_CHASSI.plan_y = 0;
				ROBOT_CHASSI.plan_w = 0;
				flag1 = 1;
				tr3 = 1;
				move_state = MOVE_SEED_LASER;
				//				move_state = MOVE_STATE_WAIT_SEED;
			}
		}
	}
	// ȡ�ڶ�����
	else if (seed == 2)
	{   
				// ��Щ��־λ��һ���Եģ��������費��Ҫ�ĳɶ��
		if (flag == 0)
		{ // ������Ǳ���  X Y
			tr3 = 1;
			seed_state = SEED_STATE_INIT;
			if (chassis_TrapezoidPlaning(0.0, 0.0, 0.0, 300.0, 0.0, 200.0, 100.0, 2000, 0.3, 0.55, 1.0, 1.3))
			{
				flag = 1;
				//				ROBOT_CHASSI.plan_x = 0;
				//				ROBOT_CHASSI.plan_y = 0;
				//				ROBOT_CHASSI.plan_w = 0;
				//			   move_state = MOVE_STATE_INIT;
			}
		}
		else {
		if (update_action_seed)
		{
			POSX_TrapezoidPlaning = ROBOT_CHASSI.world_x;
			POSY_TrapezoidPlaning = ROBOT_CHASSI.world_y;
			update_action_seed = 0;
		}
		// �ִ�ȡ���
		if (chassis_TrapezoidPlaning(POSX_TrapezoidPlaning, POSY_TrapezoidPlaning, 2371.0, 320.0, 0.0, 200.0, 100.0, 3000, 0.2, 0.65, 1.0, 1.3))
		{
			ROBOT_CHASSI.plan_x = 0;
			ROBOT_CHASSI.plan_y = 0;
			ROBOT_CHASSI.plan_w = 0;
			move_state = MOVE_SEED_LASER;
			//			move_state = MOVE_SEED_LASER;
		}
	   }
	}
	// ȡ��������
	else if (seed == 3)
	{   
				// ��Щ��־λ��һ���Եģ��������費��Ҫ�ĳɶ��
		if (flag == 0)
		{ // ������Ǳ���  X Y
			tr3 = 1;
			seed_state = SEED_STATE_INIT;
			if (chassis_TrapezoidPlaning(0.0, 0.0, 0.0, 300.0, 0.0, 200.0, 100.0, 2000, 0.3, 0.55, 1.0, 1.3))
			{
				flag = 1;
				//				ROBOT_CHASSI.plan_x = 0;
				//				ROBOT_CHASSI.plan_y = 0;
				//				ROBOT_CHASSI.plan_w = 0;
				//			   move_state = MOVE_STATE_INIT;
			}
		}
		else 
		{
		if (update_action_seed)
		{
			POSX_TrapezoidPlaning = ROBOT_CHASSI.world_x;
			POSY_TrapezoidPlaning = ROBOT_CHASSI.world_y;
			update_action_seed = 0;
		}
		// �ִ�ȡ���
		if (chassis_TrapezoidPlaning(POSX_TrapezoidPlaning, POSY_TrapezoidPlaning, 1371.0, 300.0, 0.0, 200.0, 100.0, 3000, 0.2, 0.65, 1.0, 1.3))
		{
			ROBOT_CHASSI.plan_x = 0;
			ROBOT_CHASSI.plan_y = 0;
			ROBOT_CHASSI.plan_w = 0;
			//			move_state = MOVE_STATE_WAIT_SEED;
			//			     ROBOT_CHASSI.wVx=0;
			//		         ROBOT_CHASSI.wVy=0;
			move_state = MOVE_SEED_LASER;
			//			move_state = MOVE_STATE_WAIT_SEED;
		}
	}
		//		ROBOT_CHASSI.plan_x = 0;
		//		ROBOT_CHASSI.plan_y = 500;
	}
	else
	{
	}
}

float tt11 = 0;
void laser_seed()
{
	if (seed == 1)
	{
		tt11 = 1;
		// ����zhuizong Y K
		if (Laser_calibration_laser_left(3098, 4305, 0.0, 1300) || feedback1.motor_state[3] == 1 || feedback2.motor_state[3] == 1)
		//		if(Laser1_calibration(1100,500))
		{
			tt11 = 2;
			ROBOT_CHASSI.plan_x = 0;
			ROBOT_CHASSI.plan_y = 0;
			ROBOT_CHASSI.plan_w = 0;
			move_state = MOVE_STATE_WAIT_SEED;
			// ��ȷ��Ҫ��Ҫ���ϵ������������� ������ϼ�צ����֮��ļ���
			//			move_state = MOVE_STATE_LASER_NEAR;
		}
	}
	else if (seed == 2)
	{
		// ����zhuizong Y K
		if (Laser_calibration_laser_left(3098, 3305, 0.0, 1300) || feedback1.motor_state[3] == 1 || feedback2.motor_state[3] == 1)
		//		if(Laser1_calibration(1100,500))
		{
			ROBOT_CHASSI.plan_x = 0;
			ROBOT_CHASSI.plan_y = 0;
			ROBOT_CHASSI.plan_w = 0;
			move_state = MOVE_STATE_WAIT_SEED;
			// ��ȷ��Ҫ��Ҫ���ϵ������������� ������ϼ�צ����֮��ļ���
			//			move_state = MOVE_STATE_LASER_NEAR;
		}
	}
	else if (seed == 3)
	{
		// ����zhuizong Y K
		if (Laser_calibration_laser_left(3098, 2305, 0.0, 1300) || feedback1.motor_state[3] == 1 || feedback2.motor_state[3] == 1)
		//		if(Laser1_calibration(1100,500))
		{
			ROBOT_CHASSI.plan_x = 0;
			ROBOT_CHASSI.plan_y = 0;
			ROBOT_CHASSI.plan_w = 0;
			move_state = MOVE_STATE_WAIT_SEED;
			// ��ȷ��Ҫ��Ҫ���ϵ������������� ������ϼ�צ����֮��ļ���
			//			move_state = MOVE_STATE_LASER_NEAR;
		}
	}
	else
	{
	}
}

float error_x, error_y;
int num_ptp = 0;
int point_to_point(float x, float y, float yaw, float v_max)
{
	//	YawAdjust(yaw);
	// ת��ָ���Ƕ�
	error_x = ROBOT_CHASSI.world_x - x;
	error_y = ROBOT_CHASSI.world_y - y;
	ERROR_SHOOTING = sqrt(error_x * error_x + error_y * error_y);
	// ��ô���ø��������
	// �жϾ����Ƿ����
	if (error_y < 8000 && error_x < 8000)
	{
		//		if(ROBOT_CHASSI.Vx<50 && ROBOT_CHASSI.Vy<50&&DT35.b<4330)
		//		{
		//			ROBOT_CHASSI.plan_x = 0;
		//			ROBOT_CHASSI.plan_y = 0;
		//			ROBOT_CHASSI.plan_w = 0;
		//			return 1;
		//		}
		if (ABS(error_x) < 10)
		{
			num_ptp++;
		}
		if (num_ptp < 100)
		{
			TRACK_PID.MaxOutput = ABS(v_max);
			pid_calc_by_error(&TRACK_PID, ERROR_SHOOTING);
			// 1.1 1.3
			ROBOT_CHASSI.plan_x = -1.0f * (Max_Value_Limit((TRACK_PID.output * 0.1f * error_x / ERROR_SHOOTING), v_max));
			ROBOT_CHASSI.plan_y = -1.0f * (Max_Value_Limit((TRACK_PID.output * 0.1f * error_y / ERROR_SHOOTING), v_max));

			error_y = ROBOT_CHASSI.world_y - y;
			error_x = ROBOT_CHASSI.world_x - x;
		}
		else
		{
			tt = ROBOT_CHASSI.world_x;
			ROBOT_CHASSI.plan_x = 0;
			ROBOT_CHASSI.plan_y = 0;
			ROBOT_CHASSI.plan_w = 0;
			num_ptp = 0;
			return 1;
		}
	}
	return 0;
}

int out_x = 0;
int out_y = 0;

// �ƶ��������  ��������
void put_seed()
{
	//	YawAdjust(-90.0);
	// ��Щ��־λ��һ���Եģ��������費��Ҫ�ĳɶ��
	if (put == 1)
	{
		if (update_action_put)
		{
			POSX_TrapezoidPlaning = ROBOT_CHASSI.world_x;
			POSY_TrapezoidPlaning = ROBOT_CHASSI.world_y;
			update_action_put = 0;
		}
		// ����wu����ת(90)
		if (chassis_TrapezoidPlaning(POSX_TrapezoidPlaning, POSY_TrapezoidPlaning, 3475, 2000, 0.0, 200.0, 200.0, 3000, 0.2, 0.65, 1.0, 1.0))
		{
			ROBOT_CHASSI.plan_x = 0;
			ROBOT_CHASSI.plan_y = 0;
			ROBOT_CHASSI.plan_w = 0;
			out_x = ROBOT_CHASSI.world_x;
			out_y = ROBOT_CHASSI.world_y;
			move_state = MOVE_PUT_LASER;
			//			move_state = MOVE_STATE_WAIT_PUT;
		}
	}
	else if (put == 2)
	{
		//		//		Skew_laser();
		if (update_action_put)
		{
			POSX_TrapezoidPlaning = ROBOT_CHASSI.world_x;
			POSY_TrapezoidPlaning = ROBOT_CHASSI.world_y;
			update_action_put = 0;
		}
		target_action[0] = POSX_TrapezoidPlaning - 228.0f;
		target_action[1] = POSY_TrapezoidPlaning + 503.0f;
		// ����wu����ת(90)
		if (chassis_TrapezoidPlaning(POSX_TrapezoidPlaning, POSY_TrapezoidPlaning, target_action[0], target_action[1], 0.0, 200.0, 100.0, 1500, 0.2, 0.55, 1.0, 1.0))
		{
			ROBOT_CHASSI.plan_x = 0;
			ROBOT_CHASSI.plan_y = 0;
			ROBOT_CHASSI.plan_w = 0;
			////			move_state = MOVE_PUT_LASER;
			move_state = MOVE_STATE_WAIT_PUT;
		}
	}
	else if (put == 3)
	{
		if (update_action_put)
		{
			POSX_TrapezoidPlaning = ROBOT_CHASSI.world_x;
			POSY_TrapezoidPlaning = ROBOT_CHASSI.world_y;
			update_action_put = 0;
		}
		// ����wu����ת(90)
		if (chassis_TrapezoidPlaning(POSX_TrapezoidPlaning, POSY_TrapezoidPlaning, 2475, 2000, 0.0, 200.0, 200.0, 3000, 0.2, 0.65, 1.0, 1.3))
		{
			//			ROBOT_CHASSI.plan_x = 0;
			//			ROBOT_CHASSI.plan_y = 0;
			//			ROBOT_CHASSI.plan_w = 0;
			out_x = ROBOT_CHASSI.world_x;
			out_y = ROBOT_CHASSI.world_y;
			move_state = MOVE_PUT_LASER;
			//			move_state = MOVE_STATE_WAIT_PUT;
		}
	}
	else if (put == 4)
	{
		//		Skew_laser();
		if (update_action_put)
		{
			POSX_TrapezoidPlaning = ROBOT_CHASSI.world_x;
			POSY_TrapezoidPlaning = ROBOT_CHASSI.world_y;
			update_action_put = 0;
		}
		target_action[0] = POSX_TrapezoidPlaning - 238.0f;
		target_action[1] = POSY_TrapezoidPlaning + 503.0f;
		// ����wu����ת(90)
		if (chassis_TrapezoidPlaning(POSX_TrapezoidPlaning, POSY_TrapezoidPlaning, target_action[0], target_action[1], 0.0, 200.0, 100.0, 1500, 0.2, 0.55, 1.0, 1.0))
		{
			ROBOT_CHASSI.plan_x = 0;
			ROBOT_CHASSI.plan_y = 0;
			ROBOT_CHASSI.plan_w = 0;
			//			move_state = MOVE_PUT_LASER;
			move_state = MOVE_STATE_WAIT_PUT;
		}
	}
	else if (put == 5)
	{
		if (update_action_put)
		{
			POSX_TrapezoidPlaning = ROBOT_CHASSI.world_x;
			POSY_TrapezoidPlaning = ROBOT_CHASSI.world_y;
			update_action_put = 0;
		}
		// ����wu����ת(90)
		if (chassis_TrapezoidPlaning(POSX_TrapezoidPlaning, POSY_TrapezoidPlaning, 1475, 2000, 0.0, 200.0, 200.0, 3000, 0.2, 0.65, 1.0, 1.0))
		{
			ROBOT_CHASSI.plan_x = 0;
			ROBOT_CHASSI.plan_y = 0;
			ROBOT_CHASSI.plan_w = 0;
			out_x = ROBOT_CHASSI.world_x;
			out_y = ROBOT_CHASSI.world_y;
			move_state = MOVE_PUT_LASER;
			//			move_state = MOVE_STATE_WAIT_PUT;
		}
	}
	else if (put == 6)
	{
		//		caculation_ok = 1;
		//		Skew_laser();
		if (update_action_put)
		{
			POSX_TrapezoidPlaning = ROBOT_CHASSI.world_x;
			POSY_TrapezoidPlaning = ROBOT_CHASSI.world_y;
			update_action_put = 0;
		}
		target_action[0] = POSX_TrapezoidPlaning - 248.0f;
		target_action[1] = POSY_TrapezoidPlaning + 503.0f;
		// ����wu����ת(90)
		if (chassis_TrapezoidPlaning(POSX_TrapezoidPlaning, POSY_TrapezoidPlaning, target_action[0], target_action[1], 0.0, 200.0, 100.0, 1500, 0.2, 0.55, 1.0, 1.0))
		{
			ROBOT_CHASSI.plan_x = 0;
			ROBOT_CHASSI.plan_y = 0;
			ROBOT_CHASSI.plan_w = 0;
			////			move_state = MOVE_PUT_LASER;
			move_state = MOVE_STATE_WAIT_PUT;
		}
	}
	else
	{
	}
}

float tt20 = 0;
// ���ڼ������
void laser_put()
{
	if (put == 1)
	{
		tt20 = 1;
		if (Laser_calibration_seed(1109, 4425, 0.0, 1300))
		//		if(Laser1_calibration(1100,500))
		{

			ROBOT_CHASSI.plan_x = 0;
			ROBOT_CHASSI.plan_y = 0;
			ROBOT_CHASSI.plan_w = 0;
			move_state = MOVE_STATE_WAIT_PUT;
			tt20 = 2;
			// ��ȷ��Ҫ��Ҫ���ϵ������������� ������ϼ�צ����֮��ļ���
			//			move_state = MOVE_STATE_LASER_NEAR;
		}
	}
	else if (put == 3)
	{
		if (Laser_calibration_seed(1109, 3415, 0.0, 1300))
		//		if(Laser1_calibration(1100,500))
		{
			ROBOT_CHASSI.plan_x = 0;
			ROBOT_CHASSI.plan_y = 0;
			ROBOT_CHASSI.plan_w = 0;
			move_state = MOVE_STATE_WAIT_PUT;
			// ��ȷ��Ҫ��Ҫ���ϵ������������� ������ϼ�צ����֮��ļ���
			//			move_state = MOVE_STATE_LASER_NEAR;
		}
	}
	else if (put == 5)
	{
		if (Laser_calibration_seed(1109, 2415, 0.0, 1300))
		//		if(Laser1_calibration(1100,500))
		{
			ROBOT_CHASSI.plan_x = 0;
			ROBOT_CHASSI.plan_y = 0;
			ROBOT_CHASSI.plan_w = 0;
			move_state = MOVE_STATE_WAIT_PUT;
			// ��ȷ��Ҫ��Ҫ���ϵ������������� ������ϼ�צ����֮��ļ���
			//			move_state = MOVE_STATE_LASER_NEAR;
		}
	}
	else
	{
	}
}

// �ü���ʵ��б�ƹ̶�����
void Skew_laser()
{
	if (update_action_put)
	{
		current_laser[0] = DT35.y; // Y
		current_laser[1] = DT35.b; // X
		update_action_put = 0;
	}
	if (Laser_calibration7(current_laser[0] - 492.0f, current_laser[1] + 98.0f, 0, 1000))
	//		if(Laser1_calibration(1100,500))
	{
		ROBOT_CHASSI.plan_x = 0;
		ROBOT_CHASSI.plan_y = 0;
		ROBOT_CHASSI.plan_w = 0;
		move_state = MOVE_STATE_WAIT_PUT;
		// ��ȷ��Ҫ��Ҫ���ϵ������������� ������ϼ�צ����֮��ļ���
		//			move_state = MOVE_STATE_LASER_NEAR;
	}
}

// ACTIONȡ��
int update_action_ball = 0;
int POSX_TrapezoidPlaning_ball = 0;
int POSY_TrapezoidPlaning_ball = 0;
int tt_b1 = 0;
int tt_b3 = 0;
int tt_b = 0;
int aaaa = 0;
void action_catch_ball()
{
	//---------------------------------��7��û��--------------------------------------------------------------
	if (direction == 1)
	{
		if (update_action_ball)
		{
			POSX_TrapezoidPlaning_ball = ROBOT_CHASSI.world_x;
			POSY_TrapezoidPlaning_ball = ROBOT_CHASSI.world_y;
			tt_b3++;
			tt_b++;
			update_action_ball = 0;
		}
		if (chassis_TrapezoidPlaning(POSX_TrapezoidPlaning_ball, POSY_TrapezoidPlaning_ball, 1964, 825.6, 0.0, 100.0, 100.0, 500.0, 0.3, 0.55, 1.0, 1.0) || catch_ball_laser == 0)
		{
			ROBOT_CHASSI.plan_x = 0;
			ROBOT_CHASSI.plan_y = 0;
			ROBOT_CHASSI.plan_w = 0;
			tt_b1++;
			move_state = MOVE_STATE_LASER;
		}
	}
	else if (direction == 2)
	{
		if (update_action_ball)
		{
			POSX_TrapezoidPlaning_ball = ROBOT_CHASSI.world_x;
			POSY_TrapezoidPlaning_ball = ROBOT_CHASSI.world_y;
			tt_b++;
			update_action_ball = 0;
		}
		if (chassis_TrapezoidPlaning(POSX_TrapezoidPlaning_ball, POSY_TrapezoidPlaning_ball, 2425, 825, 0.0, 100.0, 100.0, 500, 0.3, 0.55, 1.0, 1.0) || catch_ball_laser == 0)
		{
			ROBOT_CHASSI.plan_x = 0;
			ROBOT_CHASSI.plan_y = 0;
			ROBOT_CHASSI.plan_w = 0;
			move_state = MOVE_STATE_LASER;
			aaaa = 1;
		}
	}
	else if (direction == 3)
	{
		if (update_action_ball)
		{
			POSX_TrapezoidPlaning_ball = ROBOT_CHASSI.world_x;
			POSY_TrapezoidPlaning_ball = ROBOT_CHASSI.world_y;
			tt_b++;
			update_action_ball = 0;
		}
		if (chassis_TrapezoidPlaning(POSX_TrapezoidPlaning_ball, POSY_TrapezoidPlaning_ball, 2946, 825, 0.0, 100.0, 100.0, 500, 0.3, 0.55, 1.0, 1.0) || catch_ball_laser == 0)
		{
			ROBOT_CHASSI.plan_x = 0;
			ROBOT_CHASSI.plan_y = 0;
			ROBOT_CHASSI.plan_w = 0;
			move_state = MOVE_STATE_LASER;
		}
	}
	else if (direction == 4)
	{
		if (update_action_ball)
		{
			POSX_TrapezoidPlaning_ball = ROBOT_CHASSI.world_x;
			POSY_TrapezoidPlaning_ball = ROBOT_CHASSI.world_y;
			tt_b++;
			update_action_ball = 0;
		}
		if (chassis_TrapezoidPlaning(POSX_TrapezoidPlaning_ball, POSY_TrapezoidPlaning_ball, 3444, 825, 0.0, 100.0, 100.0, 500, 0.3, 0.55, 1.0, 1.0) || (catch_ball_laser == 0))
		{
			ROBOT_CHASSI.plan_x = 0;
			ROBOT_CHASSI.plan_y = 0;
			ROBOT_CHASSI.plan_w = 0;
			move_state = MOVE_STATE_LASER;
		}
	}
	else if (direction == 5)
	{
		if (update_action_ball)
		{
			POSX_TrapezoidPlaning_ball = ROBOT_CHASSI.world_x;
			POSY_TrapezoidPlaning_ball = ROBOT_CHASSI.world_y;
			tt_b++;
			update_action_ball = 0;
		}
		if (chassis_TrapezoidPlaning(POSX_TrapezoidPlaning_ball, POSY_TrapezoidPlaning_ball, 3954, 825, 0.0, 100.0, 100.0, 500, 0.3, 0.55, 1.0, 1.0) || catch_ball_laser == 0)
		{
			ROBOT_CHASSI.plan_x = 0;
			ROBOT_CHASSI.plan_y = 0;
			ROBOT_CHASSI.plan_w = 0;
			move_state = MOVE_STATE_LASER;
		}
	}
	else if (direction == 6)
	{
		if (update_action_ball)
		{
			POSX_TrapezoidPlaning_ball = ROBOT_CHASSI.world_x;
			POSY_TrapezoidPlaning_ball = ROBOT_CHASSI.world_y;
			tt_b++;
			update_action_ball = 0;
		}
		if (chassis_TrapezoidPlaning(POSX_TrapezoidPlaning_ball, POSY_TrapezoidPlaning_ball, 4386, 825, 0.0, 100.0, 100.0, 500, 0.3, 0.55, 1.0, 1.0) || catch_ball_laser == 0)
		{
			ROBOT_CHASSI.plan_x = 0;
			ROBOT_CHASSI.plan_y = 0;
			ROBOT_CHASSI.plan_w = 0;
			//		move_state = MOVE_STATE_INIT;
			move_state = MOVE_STATE_LASER;
		}
	}
	//-----------------------------------------------------------------------------------------------
	// ǰ��6����û��
	else if (direction == 7)
	{
		if (update_action_ball)
		{
			POSX_TrapezoidPlaning_ball = ROBOT_CHASSI.world_x;
			POSY_TrapezoidPlaning_ball = ROBOT_CHASSI.world_y;
			tt_b3++;
			tt_b++;
			update_action_ball = 0;
		}
		if (chassis_TrapezoidPlaning(POSX_TrapezoidPlaning_ball, POSY_TrapezoidPlaning_ball, 1995, 331, 0.0, 200.0, 100.0, 1500.0, 0.3, 0.55, 1.0, 1.0))
		{
			ROBOT_CHASSI.plan_x = 0;
			ROBOT_CHASSI.plan_y = 0;
			ROBOT_CHASSI.plan_w = 0;
			tt_b1++;
			move_state = MOVE_STATE_LASER;
		}
	}
	else if (direction == 8)
	{
		if (update_action_ball)
		{
			POSX_TrapezoidPlaning_ball = ROBOT_CHASSI.world_x;
			POSY_TrapezoidPlaning_ball = ROBOT_CHASSI.world_y;
			tt_b++;
			update_action_ball = 0;
		}
		if (chassis_TrapezoidPlaning(POSX_TrapezoidPlaning_ball, POSY_TrapezoidPlaning_ball, 2497, 331, 0.0, 200.0, 100.0, 1500, 0.3, 0.55, 1.0, 1.0))
		{
			ROBOT_CHASSI.plan_x = 0;
			ROBOT_CHASSI.plan_y = 0;
			ROBOT_CHASSI.plan_w = 0;
			move_state = MOVE_STATE_LASER;
		}
	}
	else if (direction == 9)
	{
		if (update_action_ball)
		{
			POSX_TrapezoidPlaning_ball = ROBOT_CHASSI.world_x;
			POSY_TrapezoidPlaning_ball = ROBOT_CHASSI.world_y;
			tt_b++;
			update_action_ball = 0;
		}
		if (chassis_TrapezoidPlaning(POSX_TrapezoidPlaning_ball, POSY_TrapezoidPlaning_ball, 3003, 331, 0.0, 200.0, 100.0, 1500, 0.3, 0.55, 1.0, 1.0))
		{
			ROBOT_CHASSI.plan_x = 0;
			ROBOT_CHASSI.plan_y = 0;
			ROBOT_CHASSI.plan_w = 0;
			move_state = MOVE_STATE_LASER;
		}
	}
	else if (direction == 10)
	{
		if (update_action_ball)
		{
			POSX_TrapezoidPlaning_ball = ROBOT_CHASSI.world_x;
			POSY_TrapezoidPlaning_ball = ROBOT_CHASSI.world_y;
			tt_b++;
			update_action_ball = 0;
		}
		if (chassis_TrapezoidPlaning(POSX_TrapezoidPlaning_ball, POSY_TrapezoidPlaning_ball, 3524, 331, 0.0, 200.0, 100.0, 1500, 0.3, 0.55, 1.0, 1.0))
		{
			ROBOT_CHASSI.plan_x = 0;
			ROBOT_CHASSI.plan_y = 0;
			ROBOT_CHASSI.plan_w = 0;
			move_state = MOVE_STATE_LASER;
		}
	}
	else if (direction == 11)
	{
		if (update_action_ball)
		{
			POSX_TrapezoidPlaning_ball = ROBOT_CHASSI.world_x;
			POSY_TrapezoidPlaning_ball = ROBOT_CHASSI.world_y;
			tt_b++;
			update_action_ball = 0;
		}
		if (chassis_TrapezoidPlaning(POSX_TrapezoidPlaning_ball, POSY_TrapezoidPlaning_ball, 4005, 331, 0.0, 200.0, 100.0, 1500, 0.3, 0.55, 1.0, 1.0))
		{
			ROBOT_CHASSI.plan_x = 0;
			ROBOT_CHASSI.plan_y = 0;
			ROBOT_CHASSI.plan_w = 0;
			move_state = MOVE_STATE_LASER;
		}
	}
	else if (direction == 12)
	{
		if (update_action_ball)
		{
			POSX_TrapezoidPlaning_ball = ROBOT_CHASSI.world_x;
			POSY_TrapezoidPlaning_ball = ROBOT_CHASSI.world_y;
			tt_b++;
			update_action_ball = 0;
		}
		if (chassis_TrapezoidPlaning(POSX_TrapezoidPlaning_ball, POSY_TrapezoidPlaning_ball, 4500, 331, 0.0, 200.0, 100.0, 1500, 0.3, 0.55, 1.0, 1.0))
		{
			ROBOT_CHASSI.plan_x = 0;
			ROBOT_CHASSI.plan_y = 0;
			ROBOT_CHASSI.plan_w = 0;
			//			move_state = MOVE_STATE_INIT;
			move_state = MOVE_STATE_LASER;
		}
	}
	else
	{
	}
}

uint8_t direction = 0; // ����ȡ�ĸ���
uint8_t seed = 0;	   // ���ڼ���
uint8_t put = 0;	   // ���ڷ���
float tr5 = 0;
// ��������Լ�׷�ٺ���  ��Ӧ��������ʽ��������Լ�ֱ�Ӽ���
void laser_catch()
{
	// ���²��������޸�
	//-----------------------ǰ6����-------------------------------------------
	if (direction == 1)
	{
		if (Laser_calibration_ball_left(514, 2047, 0.0, 2500) || catch_ball_laser == 1)
		{
			//					if(Laser3_calibration(449,100))
			//					{
			ROBOT_CHASSI.plan_x = 0;
			ROBOT_CHASSI.plan_y = 0;
			ROBOT_CHASSI.plan_w = 0;
			aaaa = 2;
			move_state = MOVE_STATE_WAIT_CATCH;
		}
	}
	else if (direction == 2)
	{
		// ˫����ʹ������� X Y
		//			if(Laser2_calibration(DT35.y,DT35.x))
		//			if(action_calibration())
		//			{
		// ����zhuizong X K
		// if (Laser_calibration_ball(507, 2585, 0.0, 1500) || feedback1.motor_state[3] == 1 || feedback2.motor_state[3] == 1)
		if (Laser_calibration_ball_left(492, 2547, 0.0, 2500) || catch_ball_laser == 1)
		{
			// ��ȷ��Ҫ��Ҫ���ϵ������������� ������ϼ�צ����֮��ļ���
			ROBOT_CHASSI.plan_x = 0;
			ROBOT_CHASSI.plan_y = 0;
			ROBOT_CHASSI.plan_w = 0;
			move_state = MOVE_STATE_WAIT_CATCH;
		}
		//		     }
	}
	else if (direction == 3)
	{
		// ˫����ʹ������� X Y
		//			if(Laser2_calibration(DT35.y,DT35.x))
		//			{
		// ����zhuizong Y K
		// if (Laser_calibration_ball(515, 3065, 0.0, 1500) || feedback1.motor_state[3] == 1 || feedback2.motor_state[3] == 1)
		if (Laser_calibration_ball_left(495, 3047, 0.0, 2500) || catch_ball_laser == 1)
		{
			// ��ȷ��Ҫ��Ҫ���ϵ������������� ������ϼ�צ����֮��ļ���
			ROBOT_CHASSI.plan_x = 0;
			ROBOT_CHASSI.plan_y = 0;
			ROBOT_CHASSI.plan_w = 0;
			move_state = MOVE_STATE_WAIT_CATCH;
		}
		//		     }
	}
	else if (direction == 4)
	{
		if (Laser_calibration_ball_left(505, 3547, 0.0, 2500) || catch_ball_laser == 1)
		{
			// ��ȷ��Ҫ��Ҫ���ϵ������������� ������ϼ�צ����֮��ļ���
			ROBOT_CHASSI.plan_x = 0;
			ROBOT_CHASSI.plan_y = 0;
			ROBOT_CHASSI.plan_w = 0;
			move_state = MOVE_STATE_WAIT_CATCH;
		}
	}
	else if (direction == 5)
	{
		if (Laser_calibration_ball_left(1021, 4047, 0.0, 2500) || catch_ball_laser == 1)
		{
			// ��ȷ��Ҫ��Ҫ���ϵ������������� ������ϼ�צ����֮��ļ���
			ROBOT_CHASSI.plan_x = 0;
			ROBOT_CHASSI.plan_y = 0;
			ROBOT_CHASSI.plan_w = 0;
			move_state = MOVE_STATE_WAIT_CATCH;
		}
		//		     }
	}
	else if (direction == 6)
	{
		//		      //˫����ʹ������� X Y
		//			if(Laser2_calibration(DT35.y,DT35.x))
		//			{
		// ����zhuizong X K
		// if (Laser_calibration_ball(933, 4560, 0.0, 1500) || feedback1.motor_state[3] == 1 || feedback2.motor_state[3] == 1)
		if (Laser_calibration_ball_left(1000, 4547, 0.0, 2500) || catch_ball_laser == 1)
		{
			// ��ȷ��Ҫ��Ҫ���ϵ������������� ������ϼ�צ����֮��ļ���
			ROBOT_CHASSI.plan_x = 0;
			ROBOT_CHASSI.plan_y = 0;
			ROBOT_CHASSI.plan_w = 0;
			move_state = MOVE_STATE_WAIT_CATCH;
		}
		//		     }
	}
	//---------------------------------------------------------------------
	//-----------------------��5������ϵ�7��������ȡ-----------------------
	// ��7�����Ǻ���ȡ��
	else if (direction == 7)
	{
		//		     //˫����ʹ������� X Y
		//			if(Laser2_calibration(DT35.y,DT35.x))
		//			{
		// ����zhuizong Y B
		if (Laser_calibration(1016, 2047, 0.0, 1500) || catch_ball_laser == 1)
		{
			// ��ȷ��Ҫ��Ҫ���ϵ������������� ������ϼ�צ����֮��ļ���
			ROBOT_CHASSI.plan_x = 0;
			ROBOT_CHASSI.plan_y = 0;
			ROBOT_CHASSI.plan_w = 0;
			move_state = MOVE_STATE_WAIT_CATCH;
		}
		//		     }
	}
	// �����Ǽ�����ȫ�Ǽ������action  ǰ���Ѿ����ù�action��
	else if (direction == 8)
	{
		if (Laser_calibration(1007, 2547, 0.0, 1500) || catch_ball_laser == 1 || DT35.k < 540)
		{
			tr = 1;
			// ��ȷ��Ҫ��Ҫ���ϵ������������� ������ϼ�צ����֮��ļ���
			//					move_state = MOVE_STATE_INIT;
			ROBOT_CHASSI.plan_x = 0;
			ROBOT_CHASSI.plan_y = 0;
			ROBOT_CHASSI.plan_w = 0;
			move_state = MOVE_STATE_WAIT_CATCH;
		}
		//		     }
	}
	else if (direction == 9)
	{
		if (Laser_calibration(997, 3047, 0.0, 1500) || catch_ball_laser == 1 || DT35.k < 1090)
		{
			tr = 1;
			// ��ȷ��Ҫ��Ҫ���ϵ������������� ������ϼ�צ����֮��ļ���
			ROBOT_CHASSI.plan_x = 0;
			ROBOT_CHASSI.plan_y = 0;
			ROBOT_CHASSI.plan_w = 0;
			move_state = MOVE_STATE_WAIT_CATCH;
		}
		//		     }
	}
	else if (direction == 10)
	{
		if (Laser_calibration(1000, 3547, 0.0, 1500) || catch_ball_laser == 1 || DT35.k < 1640)
		{
			tr = 1;
			// ��ȷ��Ҫ��Ҫ���ϵ������������� ������ϼ�צ����֮��ļ���
			ROBOT_CHASSI.plan_x = 0;
			ROBOT_CHASSI.plan_y = 0;
			ROBOT_CHASSI.plan_w = 0;
			move_state = MOVE_STATE_WAIT_CATCH;
		}
		//		     }
	}
	else if (direction == 11)
	{
		if (Laser_calibration(1496, 4047, 0.0, 1500) || catch_ball_laser == 1 || DT35.k < 2120)
		{
			tr = 1;
			// ��ȷ��Ҫ��Ҫ���ϵ������������� ������ϼ�צ����֮��ļ���
			ROBOT_CHASSI.plan_x = 0;
			ROBOT_CHASSI.plan_y = 0;
			ROBOT_CHASSI.plan_w = 0;
			move_state = MOVE_STATE_WAIT_CATCH;
		}
		//		     }
	}
	else if (direction == 12)
	{
		if (Laser_calibration(1538, 4547, 0.0, 1500) || catch_ball_laser == 1 || DT35.k < 2670)
		{
			tr = 1;
			// ��ȷ��Ҫ��Ҫ���ϵ������������� ������ϼ�צ����֮��ļ���
			ROBOT_CHASSI.plan_x = 0;
			ROBOT_CHASSI.plan_y = 0;
			ROBOT_CHASSI.plan_w = 0;
			move_state = MOVE_STATE_WAIT_CATCH;
		}
		//		     }
	}
}
float tr4 = 0;
void laser_catch_near()
{
	if (Laser3_calibration(450, 100))
	{

		ROBOT_CHASSI.plan_y = 0;
		ROBOT_CHASSI.plan_w = 0;
		ROBOT_CHASSI.plan_x = 0;
		move_state = MOVE_STATE_INIT;
		tr5 = 2;
		// ��ȷ��Ҫ��Ҫ���ϵ������������� ������ϼ�צ����֮��ļ���
		//					   move_state = MOVE_STATE_LASER_NEAR;
	}
}

// ���溯�����ڵ���  �����̶�����
int action_catch()
{
	if (update_action_ball)
	{
		POSX_TrapezoidPlaning = ROBOT_CHASSI.world_x;
		POSY_TrapezoidPlaning = ROBOT_CHASSI.world_y;
		update_action_ball = 0;
	}
//	ROBOT_CHASSI.plan_w = 0;
//	tr1 = 1;
	// ��������ø�
	float AIM_Y = (POSY_TrapezoidPlaning - 500.0f);
	float AIM_X = (POSX_TrapezoidPlaning + 0.0f);
	if (chassis_TrapezoidPlaning(POSX_TrapezoidPlaning, POSY_TrapezoidPlaning, AIM_X, AIM_Y, 0.0, 100.0, 100.0, 500, 0.2, 0.55, 1.0, 1.0) || catch_ball_laser == 1)
	{
		ROBOT_CHASSI.plan_x = 0;
		ROBOT_CHASSI.plan_y = 0;
		move_state = MOVE_STATE_WAIT_CATCH;
		return 1;
	}
	else 
	{
		return 0;
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
float YawAdjust_error;
float tt10 = 0;
int YawAdjust(float Target_angle)
{
	// �������
	if (ROBOT_CHASSI.world_w * Target_angle >= 0)
	{
		tt10 = 1;
		YawAdjust_error = Target_angle - ROBOT_CHASSI.world_w;
	}
	else
	{
		if (ABS(ROBOT_CHASSI.world_w) + ABS(Target_angle) <= 180)
		{
			tt10 = -1;
			YawAdjust_error = Target_angle - ROBOT_CHASSI.world_w;
		}
		else
		{

			AngleLimit(&YawAdjust_error);
		}
	}

	if (ABS(YawAdjust_error) < 0.2f)
	{
		//		ROBOT_CHASSI.plan_x = 0;
		//		ROBOT_CHASSI.plan_y = 0;
		ROBOT_CHASSI.plan_w = 0;
		tt10 = 2;
		return 1;
	}
	else
	{
		tt10 = 3;
		// ֱ������PID������ٶ�
		pid_calc_by_error(&point_traker_yaw_pid, YawAdjust_error);
		ROBOT_CHASSI.plan_w = point_traker_yaw_pid.output; // ���̽��ٶ� ��λ��rad/
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
int tt_b2 = 0;
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
							 float R_de,
							 float P_X,
							 float P_Y)
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
	//	   YawAdjust(POS_Yaw);
	if (S_chassis < Sac_chassis)
	{
		output_V = sqrt(2.0f * Aac_chassis * S_chassis + V_start * V_start);
		// YawAdjust(POS_YAW);//���ٽ׶β������Ƕ�

	} // ���ٽ׶�
	else if (S_chassis < (Sac_chassis + Sco_chassis))
	{
		output_V = sqrt(2.0f * Aac_chassis * Sac_chassis + V_start * V_start);
		//		output_V = V_max;
	} // ���ٽ׶�
	else if (S_chassis < Ssu_chassis)
	{
		output_V = sqrt(V_end * V_end - 2.0f * Ade_chassis * ABS(Ssu_chassis - S_chassis));

		//		YawAdjust(POS_Yaw); // �����Ƕ�

	} // ���ٽ׶�

	//		else
	//			output_V=V_end;
	//
	//	}
	// �ֽ��ٶȣ���������ʵ�������
	ROBOT_CHASSI.plan_x = -(output_V * 1.0f * (x_error_end_Trapezoid) / real_error); // x��
	ROBOT_CHASSI.plan_y = (output_V * 1.0f * (y_error_end_Trapezoid) / real_error);	 // y��
	// ABS(ROBOT_CHASSI.world_x - POS_X_end)<100&&ABS(ROBOT_CHASSI.Vy - POS_Y_end)
	if (ABS(Ssu_chassis - S_chassis) < 5) // ��ǰ����
										  //	if(ABS(x_error_end_Trapezoid)<5 && ABS(y_error_end_Trapezoid)<5)
	{
		tt_b2 = 1;
		output_V = 0;
		ROBOT_CHASSI.plan_x = 0;
		ROBOT_CHASSI.plan_y = 0;
		return 1;
	}
	else
		tt_b2 = x_error_end_Trapezoid;
	return 0;
}

int chassis_TrapezoidPlaning_ball(float POS_X_start,
								  float POS_Y_start,
								  float POS_X_end,
								  float POS_Y_end,
								  float POS_Yaw,
								  float V_start,
								  float V_end,
								  float V_max,
								  float R_ac,
								  float R_de,
								  float P_X,
								  float P_Y)
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
	//	YawAdjust(POS_Yaw);
	if (S_chassis < Sac_chassis)
	{
		output_V = sqrt(2.0f * Aac_chassis * S_chassis + V_start * V_start);
		// YawAdjust(POS_YAW);//���ٽ׶β������Ƕ�

	} // ���ٽ׶�
	else if (S_chassis < (Sac_chassis + Sco_chassis))
	{
		//					output_V = sqrt(2.0f * Aac_chassis * Sac_chassis + V_start * V_start);
		output_V = V_max;
	} // ���ٽ׶�
	else if (S_chassis < Ssu_chassis)
	{
		output_V = sqrt(V_end * V_end - 2.0f * Ade_chassis * ABS(Ssu_chassis - S_chassis));

		//		YawAdjust(POS_Yaw); // �����Ƕ�

	} // ���ٽ׶�

	//		else
	//			output_V=V_end;
	// s
	//	}
	// �ֽ��ٶȣ���������ʵ�������
	ROBOT_CHASSI.plan_x = -(output_V * 1.0f * (x_error_end_Trapezoid) / real_error); // x��
	ROBOT_CHASSI.plan_y = (output_V * 1.0f * (y_error_end_Trapezoid) / real_error);	 // y��
	// ABS(ROBOT_CHASSI.world_x - POS_X_end)<100&&ABS(ROBOT_CHASSI.Vy - POS_Y_end)
	if (ABS(Ssu_chassis - S_chassis) < 5) // ��ǰ����
										  //	if(ABS(x_error_end_Trapezoid)<5 && ABS(y_error_end_Trapezoid)<5)
	{
		tt_b2 = 1;
		output_V = 0;
		ROBOT_CHASSI.plan_x = 0;
		ROBOT_CHASSI.plan_y = 0;
		return 1;
	}
	else
		tt_b2 = x_error_end_Trapezoid;
	return 0;
}
