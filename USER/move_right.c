#include "move_right.h"
int num = 0;
float dianjix,dianjiy;
void move_seed_right()
{
//	caculation_ok = 1;
	// 初始的获取action值的就不改了
	if (seed == 3)
	{
		// 这些标志位是一次性的，看后面需不需要改成多次
		if (flag == 0)
		{ // 后面的是比例  X Y
			robot_state = ROBOT_STATE_SEED_CTRL;
			seed_state = SEED_STATE_INIT;
			if (chassis_TrapezoidPlaning(0.0, 0.0, 0.0, 300.0, 0.0, 200.0, 100.0, 2000, 0.3, 0.55, 1.0, 1.3))
			{
				flag = 1;
				ROBOT_CHASSI.plan_x = 0;
				ROBOT_CHASSI.plan_y = 0;
				ROBOT_CHASSI.plan_w = 0;
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
			//			// 抵达取苗点  2500
			//			if (chassis_TrapezoidPlaning(POSX_TrapezoidPlaning, POSY_TrapezoidPlaning, -3390.0, 290.0, 0.0, 200.0, 400.0, 3000, 0.2, 0.60, 1.3, 1.0))

			if (chassis_TrapezoidPlaning(POSX_TrapezoidPlaning, POSY_TrapezoidPlaning, -3242.0, 290.0, 0.0, 100.0, 100.0, 3500, 0.2, 0.60, 1.3, 1.0))
			{
				//				ROBOT_CHASSI.plan_x = 0;
				//				ROBOT_CHASSI.plan_y = 0;
				//				ROBOT_CHASSI.plan_w = 0;
//				move_state = MOVE_STATE_WAIT_SEED;
//				caculation_ok = 0;
				//			     ROBOT_CHASSI.wVx=0;
				//		         ROBOT_CHASSI.wVy=0;
				//			    move_state = MOVE_STATE_LASER;
				//				move_state = MOVE_STATE_WAIT_SEED;
				//				robot_state = ROBOT_STATE_SEED_CTRL;
							   move_state = MOVE_SEED_LASER;
			}
		}
	}
	// 取第二组苗
	else if (seed == 2)
	{
				// 这些标志位是一次性的，看后面需不需要改成多次
		if (flag == 0)
		{ // 后面的是比例  X Y
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
		// 抵达取苗点 2800
		//		if (chassis_TrapezoidPlaning(POSX_TrapezoidPlaning, POSY_TrapezoidPlaning, -2390.0, 320.0, 0.0, 200.0, 400.0, 3000, 0.2, 0.55, 1.0, 1.3))
		if (chassis_TrapezoidPlaning(POSX_TrapezoidPlaning, POSY_TrapezoidPlaning, -2302.0, 320.0, 0.0, 100.0, 100.0, 3000, 0.2, 0.55, 1.0, 1.3))
		{
			ROBOT_CHASSI.plan_x = 0;
			ROBOT_CHASSI.plan_y = 0;
			ROBOT_CHASSI.plan_w = 0;
			dianjix = can1motorRealInfo[0].TARGET_RPM;
			//			move_state = MOVE_SEED_LASER;
			//			     ROBOT_CHASSI.wVx=0;
			//		         ROBOT_CHASSI.wVy=0;
			//			   move_state = MOVE_STATE_LASER;
//			caculation_ok = 0;
//			move_state = MOVE_STATE_WAIT_SEED;
			move_state = MOVE_SEED_LASER;
			//			robot_state = ROBOT_STATE_SEED_CTRL;
		}
	}
	}
	// 取第三组苗
	else if (seed == 1)
	{
				// 这些标志位是一次性的，看后面需不需要改成多次
		if (flag == 0)
		{ // 后面的是比例  X Y
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
		// 抵达取苗点  2700
		//		if (chassis_TrapezoidPlaning(POSX_TrapezoidPlaning, POSY_TrapezoidPlaning, -1392.0, 320.0, 0.0, 200.0, 400.0, 3000, 0.2, 0.55, 1.0, 1.3))
		if (chassis_TrapezoidPlaning(POSX_TrapezoidPlaning, POSY_TrapezoidPlaning, -1302.0, 320.0, 0.0, 100.0, 100.0, 3000, 0.2, 0.55, 1.0, 1.3))
		{
			ROBOT_CHASSI.plan_x = 0;
			ROBOT_CHASSI.plan_y = 0;
			ROBOT_CHASSI.plan_w = 0;
			//			move_state = MOVE_SEED_LASER;
			//			     ROBOT_CHASSI.wVx=0;
			//		         ROBOT_CHASSI.wVy=0;
			//			   move_state = MOVE_STATE_LASER;
//			move_state = MOVE_STATE_WAIT_SEED;
			move_state = MOVE_SEED_LASER;
//			caculation_ok = 0;
			//			robot_state = ROBOT_STATE_SEED_CTRL;
		}
//				ROBOT_CHASSI.plan_x = 0;
//				num += 10;
//				ROBOT_CHASSI.plan_y = num;
//				if(num > 1000) num = 1000;
	   }
   }
	else
	{
	}
}

void laser_seed_right()
{
//	caculation_ok = 1;
	if (seed == 1)
	{
//		if(point_to_point(-1365.0,320.0,0.0,500)|| feedback1.motor_state[3] == 1 || feedback2.motor_state[3] == 1)
		// 激光zhuizong Y B
		if (Laser_calibration7(3098, 2315, 0, 1300) || feedback1.motor_state[3] == 1 || feedback2.motor_state[3] == 1)
		//		if (Laser_calibration7(3208, 2019, 0, 1500)|| feedback1.motor_state[3] == 1 || feedback2.motor_state[3] == 1)
		//		if(Laser1_calibration(1100,500))
		{
			ROBOT_CHASSI.plan_x = 0;
			ROBOT_CHASSI.plan_y = 0;
			ROBOT_CHASSI.plan_w = 0;
			move_state = MOVE_STATE_WAIT_SEED;
			robot_state = ROBOT_STATE_SEED_CTRL;
//			caculation_ok = 0;
			// 不确定要不要加上倒车，看看测试 这里加上夹爪放苗之类的即可
			//			move_state = MOVE_STATE_LASER_NEAR;
		}
	}
	else if (seed == 2)
	{
		// 激光zhuizong Y B
//		if(point_to_point(-2358.0,320.0,0.0,500)|| feedback1.motor_state[3] == 1 || feedback2.motor_state[3] == 1)
		if (Laser_calibration7(3098, 3315, 0, 1300) || feedback1.motor_state[3] == 1 || feedback2.motor_state[3] == 1)
		//		if (Laser_calibration7(3207, 2935, 0, 1500)|| feedback1.motor_state[3] == 1 || feedback2.motor_state[3] == 1)
		//		if(Laser1_calibration(1100,500))
		{
			ROBOT_CHASSI.plan_x = 0;
			ROBOT_CHASSI.plan_y = 0;
			ROBOT_CHASSI.plan_w = 0;
			move_state = MOVE_STATE_WAIT_SEED;
			robot_state = ROBOT_STATE_SEED_CTRL;
//			caculation_ok = 0;
			// 不确定要不要加上倒车，看看测试 这里加上夹爪放苗之类的即可
			//			move_state = MOVE_STATE_LASER_NEAR;
		}
	}
	else if (seed == 3)
	{
		// 激光zhuizong Y B
//		if(point_to_point(-3358.0,320.0,0.0,500)|| feedback1.motor_state[3] == 1 || feedback2.motor_state[3] == 1)
		if (Laser_calibration7(3098, 4315, 0, 1300) || feedback1.motor_state[3] == 1 || feedback2.motor_state[3] == 1)
		//		if (Laser_calibration7(3207, 3819, 0, 1500)|| feedback1.motor_state[3] == 1 || feedback2.motor_state[3] == 1)
		//		if(Laser1_calibration(1100,500))
		{
			ROBOT_CHASSI.plan_x = 0;
			ROBOT_CHASSI.plan_y = 0;
			ROBOT_CHASSI.plan_w = 0;
			move_state = MOVE_STATE_WAIT_SEED;
			robot_state = ROBOT_STATE_SEED_CTRL;
//			caculation_ok = 0;
			// 不确定要不要加上倒车，看看测试 这里加上夹爪放苗之类的即可
			//			move_state = MOVE_STATE_LASER_NEAR;
		}
	}
	else
	{
	}
}

void put_seed_right()
{
//	caculation_ok = 1;
	if (put == 1)
	{
		if (update_action_put)
		{
			POSX_TrapezoidPlaning = ROBOT_CHASSI.world_x;
			POSY_TrapezoidPlaning = ROBOT_CHASSI.world_y;
			update_action_put = 0;
		}
		// 车体wu需旋转(90)
		//		if (chassis_TrapezoidPlaning(POSX_TrapezoidPlaning, POSY_TrapezoidPlaning, -1547, 2242.0, 0.0, 100.0, 400.0, 3000, 0.2, 0.55, 1.0, 1.0))
		if (chassis_TrapezoidPlaning(POSX_TrapezoidPlaning, POSY_TrapezoidPlaning, -1448, 2000.0, 0.0, 100.0, 100.0, 3000, 0.2, 0.55, 1.0, 1.0))
		//		if (chassis_TrapezoidPlaning(POSX_TrapezoidPlaning, POSY_TrapezoidPlaning, -982, 2041.0, -90.0, 100.0, 400.0, 2500, 0.2, 0.55, 1.0, 1.0))
		{
			ROBOT_CHASSI.plan_x = 0;
			ROBOT_CHASSI.plan_y = 0;
			ROBOT_CHASSI.plan_w = 0;
			move_state = MOVE_PUT_LASER;
//			caculation_ok = 0;
			//						move_state = MOVE_STATE_WAIT_PUT;
			//			robot_state = ROBOT_STATE_SEED_CTRL;
			//			robot_state = ROBOT_STATE_SEED_CTRL;
		}
	}
	else if (put == 2)
	{
		//		Skew_laser();
		if (update_action_put)
		{
			POSX_TrapezoidPlaning = ROBOT_CHASSI.world_x;
			POSY_TrapezoidPlaning = ROBOT_CHASSI.world_y;
			update_action_put = 0;
		}
		target_action[0] = POSX_TrapezoidPlaning + 224.0f;
		target_action[1] = POSY_TrapezoidPlaning + 466.0f;
		// 车体wu需旋转(90)
		if (chassis_TrapezoidPlaning(POSX_TrapezoidPlaning, POSY_TrapezoidPlaning, target_action[0], target_action[1], 0.0, 100.0, 100.0, 1500, 0.2, 0.55, 1.0, 1.0))
		{
			ROBOT_CHASSI.plan_x = 0;
			ROBOT_CHASSI.plan_y = 0;
			ROBOT_CHASSI.plan_w = 0;
			////			move_state = MOVE_PUT_LASER;
//			caculation_ok = 0;
			move_state = MOVE_STATE_WAIT_PUT;
			robot_state = ROBOT_STATE_SEED_CTRL;
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
		// 车体wu需旋转(90)
		if (chassis_TrapezoidPlaning(POSX_TrapezoidPlaning, POSY_TrapezoidPlaning, -2448.0, 2000.0, 0.0, 100.0, 100.0, 3000, 0.2, 0.55, 1.0, 1.3))
		//		if (chassis_TrapezoidPlaning(POSX_TrapezoidPlaning, POSY_TrapezoidPlaning, -2547.0, 2242.0, 0.0, 200.0, 400.0, 3000, 0.2, 0.55, 1.0, 1.3))
		{
			//			ROBOT_CHASSI.plan_x = 0;
			//			ROBOT_CHASSI.plan_y = 0;
			//			ROBOT_CHASSI.plan_w = 0;
//			caculation_ok = 0;
			move_state = MOVE_PUT_LASER;
			//						move_state = MOVE_STATE_WAIT_PUT;
			//			robot_state = ROBOT_STATE_SEED_CTRL;
			//			robot_state = ROBOT_STATE_SEED_CTRL;
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
		target_action[0] = POSX_TrapezoidPlaning + 224.0f;
		target_action[1] = POSY_TrapezoidPlaning + 466.0f;
		// 车体wu需旋转(90)
		if (chassis_TrapezoidPlaning(POSX_TrapezoidPlaning, POSY_TrapezoidPlaning, target_action[0], target_action[1], 0.0, 100.0, 100.0, 1500, 0.2, 0.55, 1.0, 1.0))
		{
			ROBOT_CHASSI.plan_x = 0;
			ROBOT_CHASSI.plan_y = 0;
			ROBOT_CHASSI.plan_w = 0;
			////			move_state = MOVE_PUT_LASER;
			move_state = MOVE_STATE_WAIT_PUT;
			robot_state = ROBOT_STATE_SEED_CTRL;
//			caculation_ok = 0;
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
		// 车体wu需旋转(90)
		if (chassis_TrapezoidPlaning(POSX_TrapezoidPlaning, POSY_TrapezoidPlaning, -3448.0, 2000.0, 0.0, 100.0, 100.0, 3000, 0.2, 0.55, 1.0, 1.0))
		//		if (chassis_TrapezoidPlaning(POSX_TrapezoidPlaning, POSY_TrapezoidPlaning, -3547.0, 2242.0, 0.0, 200.0, 400.0, 3000, 0.2, 0.55, 1.0, 1.0))
		{
			//			ROBOT_CHASSI.plan_x = 0;
			//			ROBOT_CHASSI.plan_y = 0;
			//			ROBOT_CHASSI.plan_w = 0;
			move_state = MOVE_PUT_LASER;
////			caculation_ok = 0;
//								move_state = MOVE_STATE_WAIT_PUT;
			//			robot_state = ROBOT_STATE_SEED_CTRL;
			//			robot_state = ROBOT_STATE_SEED_CTRL;
		}
	}
	else if (put == 6)
	{
		//		Skew_laser();
		if (update_action_put)
		{
			POSX_TrapezoidPlaning = ROBOT_CHASSI.world_x;
			POSY_TrapezoidPlaning = ROBOT_CHASSI.world_y;
			update_action_put = 0;
		}
		target_action[0] = POSX_TrapezoidPlaning + 226.0f;
		target_action[1] = POSY_TrapezoidPlaning + 466.0f;
		// 车体wu需旋转(90)
		if (chassis_TrapezoidPlaning(POSX_TrapezoidPlaning, POSY_TrapezoidPlaning, target_action[0], target_action[1], 0.0, 100.0, 100.0, 1500, 0.2, 0.55, 1.0, 1.0))
		{
			ROBOT_CHASSI.plan_x = 0;
			ROBOT_CHASSI.plan_y = 0;
			ROBOT_CHASSI.plan_w = 0;
			////			move_state = MOVE_PUT_LASER;
			move_state = MOVE_STATE_WAIT_PUT;
			robot_state = ROBOT_STATE_SEED_CTRL;
//			caculation_ok = 0;
		}
	}
	else
	{
	}
}

void laser_put_right()
{
//	caculation_ok = 1;
	// 用于激光放苗
	if (put == 1)
	{
		// 激光zhuizong X K
//		if(point_to_point(-1448, 2268.0,0.0,500)||(DT35.x>1068&&DT35.x<1088))
		if (Laser_calibration_seed_right(1109, 2420, 0.0, 1300))
		//		if (Laser_calibration_seed_right(1108, 2075, 0, 1300))
		//		if(Laser1_calibration(1100,500))
		{
			ROBOT_CHASSI.plan_x = 0;
			ROBOT_CHASSI.plan_y = 0;
			ROBOT_CHASSI.plan_w = 0;
			move_state = MOVE_STATE_WAIT_PUT;
			robot_state = ROBOT_STATE_SEED_CTRL;
//			caculation_ok = 0;
			// 不确定要不要加上倒车，看看测试 这里加上夹爪放苗之类的即可
			//			move_state = MOVE_STATE_LASER_NEAR;
		}
	}
	else if (put == 3)
	{
		// 激光zhuizong Y B
//		if(point_to_point(-2448, 2268.0,0.0,500)||(DT35.x>1068&&DT35.x<1088))
		if (Laser_calibration_seed_right(1109, 3420, 0, 1300))
		//		if (Laser_calibration_seed_right(1108, 3041, 0, 1300))
		//		if(Laser1_calibration(1100,500))
		{
			ROBOT_CHASSI.plan_x = 0;
			ROBOT_CHASSI.plan_y = 0;
			ROBOT_CHASSI.plan_w = 0;
			move_state = MOVE_STATE_WAIT_PUT;
			robot_state = ROBOT_STATE_SEED_CTRL;
//			caculation_ok = 0;
			// 不确定要不要加上倒车，看看测试 这里加上夹爪放苗之类的即可
			//			move_state = MOVE_STATE_LASER_NEAR;
		}
	}
	else if (put == 5)
	{
		// 激光zhuizong Y B
//		if(point_to_point(-3448, 2268.0,0.0,500)||(DT35.x>1068&&DT35.x<1088))
		if (Laser_calibration_seed_right(1109, 4420, 0, 1300))
		//		if (Laser_calibration_seed_right(1108, 3945, 0, 1300))
		//		if(Laser1_calibration(1100,500))
		{
			ROBOT_CHASSI.plan_x = 0;
			ROBOT_CHASSI.plan_y = 0;
			ROBOT_CHASSI.plan_w = 0;
			move_state = MOVE_STATE_WAIT_PUT;
			robot_state = ROBOT_STATE_SEED_CTRL;
//			caculation_ok = 0;
			// 不确定要不要加上倒车，看看测试 这里加上夹爪放苗之类的即可
			//			move_state = MOVE_STATE_LASER_NEAR;
		}
	}
	else
	{
	}
}

float tt14 = 0;
// ACTION取球
void action_catch_ball_right()
{
	// 前面6个都没用
	if (direction == 12)
	{
		if (update_action_ball)
		{
			POSX_TrapezoidPlaning_ball = ROBOT_CHASSI.world_x;
			POSY_TrapezoidPlaning_ball = ROBOT_CHASSI.world_y;
			update_action_ball = 0;
		}
		if (chassis_TrapezoidPlaning_ball(POSX_TrapezoidPlaning_ball, POSY_TrapezoidPlaning_ball, -1915, 668, 0.0, 100.0, 100.0, 1500.0, 0.3, 0.55, 1.0, 1.0))
		{
			ROBOT_CHASSI.plan_x = 0;
			ROBOT_CHASSI.plan_y = 0;
			ROBOT_CHASSI.plan_w = 0;
			move_state = MOVE_STATE_LASER;
			//            move_state = MOVE_STATE_WAIT_CATCH;
		}
	}
	else if (direction == 11)
	{
		if (update_action_ball)
		{
			POSX_TrapezoidPlaning_ball = ROBOT_CHASSI.world_x;
			POSY_TrapezoidPlaning_ball = ROBOT_CHASSI.world_y;
			update_action_ball = 0;
		}
		if (chassis_TrapezoidPlaning_ball(POSX_TrapezoidPlaning_ball, POSY_TrapezoidPlaning_ball, -2415, 668, 0.0, 100.0, 100.0, 1500, 0.3, 0.55, 1.0, 1.0))
		{
			ROBOT_CHASSI.plan_x = 0;
			ROBOT_CHASSI.plan_y = 0;
			ROBOT_CHASSI.plan_w = 0;
			move_state = MOVE_STATE_LASER;
			//        move_state = MOVE_STATE_WAIT_CATCH;
		}
	}
	else if (direction == 10)
	{
		if (update_action_ball)
		{
			POSX_TrapezoidPlaning_ball = ROBOT_CHASSI.world_x;
			POSY_TrapezoidPlaning_ball = ROBOT_CHASSI.world_y;
			update_action_ball = 0;
		}
		if (chassis_TrapezoidPlaning_ball(POSX_TrapezoidPlaning_ball, POSY_TrapezoidPlaning_ball, -2915, 668, 0.0, 100.0, 100.0, 1500, 0.3, 0.55, 1.0, 1.0))
		{
			ROBOT_CHASSI.plan_x = 0;
			ROBOT_CHASSI.plan_y = 0;
			ROBOT_CHASSI.plan_w = 0;
			move_state = MOVE_STATE_LASER;
			//    move_state = MOVE_STATE_WAIT_CATCH;
		}
	}
	else if (direction == 9)
	{
		if (update_action_ball)
		{
			POSX_TrapezoidPlaning_ball = ROBOT_CHASSI.world_x;
			POSY_TrapezoidPlaning_ball = ROBOT_CHASSI.world_y;
			update_action_ball = 0;
		}
		if (chassis_TrapezoidPlaning_ball(POSX_TrapezoidPlaning_ball, POSY_TrapezoidPlaning_ball, -3415, 668, 0.0, 100.0, 100.0, 1500, 0.3, 0.55, 1.0, 1.0))
		{
			ROBOT_CHASSI.plan_x = 0;
			ROBOT_CHASSI.plan_y = 0;
			//			ROBOT_CHASSI.plan_w = 0;
			move_state = MOVE_STATE_LASER;
			//    move_state = MOVE_STATE_WAIT_CATCH;
		}
	}
	else if (direction == 8)
	{
		if (update_action_ball)
		{
			POSX_TrapezoidPlaning_ball = ROBOT_CHASSI.world_x;
			POSY_TrapezoidPlaning_ball = ROBOT_CHASSI.world_y;
			tt14++;
			update_action_ball = 0;
		}
		if (chassis_TrapezoidPlaning_ball(POSX_TrapezoidPlaning_ball, POSY_TrapezoidPlaning_ball, -3915, 668, 0, 100.0, 100.0, 1500, 0.2, 0.65, 1.0, 1.0))
		{
			ROBOT_CHASSI.plan_x = 0;
			ROBOT_CHASSI.plan_y = 0;
			ROBOT_CHASSI.plan_w = 0;
			move_state = MOVE_STATE_LASER;
			//            move_state = MOVE_STATE_WAIT_CATCH;
		}
	}
	else if (direction == 7)
	{
		if (update_action_ball)
		{
			POSX_TrapezoidPlaning_ball = ROBOT_CHASSI.world_x;
			POSY_TrapezoidPlaning_ball = ROBOT_CHASSI.world_y;
			update_action_ball = 0;
		}
		if (chassis_TrapezoidPlaning_ball(POSX_TrapezoidPlaning_ball, POSY_TrapezoidPlaning_ball, -4415, 668, 0.0, 100.0, 100.0, 1500, 0.3, 0.55, 1.0, 1.0))
		{
			ROBOT_CHASSI.plan_x = 0;
			ROBOT_CHASSI.plan_y = 0;
			ROBOT_CHASSI.plan_w = 0;
			//					move_state = MOVE_STATE_WAIT_CATCH;
			move_state = MOVE_STATE_LASER;
		}
	}
	else
	{
	}
}

void laser_catch_right()
{
	// 以下参数均待修改
	//-----------------------前6个球-------------------------------------------
	YawAdjust(0.0);
	if (direction == 1)
	{
		// 激光zhuizong X K
		if (Laser_calibration_ball(1491, 4572, 0.0, 1500) || catch_ball_laser == 2)
		//				if(Laser1_calibration(1940,500))
		{
			//					if(Laser3_calibration(449,100))
			//					{
			ROBOT_CHASSI.plan_x = 0;
			ROBOT_CHASSI.plan_y = 0;
			ROBOT_CHASSI.plan_w = 0;
			move_state = MOVE_STATE_WAIT_CATCH;
		}
	}
	else if (direction == 2)
	{
		// 激光zhuizong X K
		if (Laser_calibration_ball(999, 4072, 0.0, 1500) || catch_ball_laser == 2)
		{
			// 不确定要不要加上倒车，看看测试 这里加上夹爪放苗之类的即可
			ROBOT_CHASSI.plan_x = 0;
			ROBOT_CHASSI.plan_y = 0;
			ROBOT_CHASSI.plan_w = 0;
			move_state = MOVE_STATE_WAIT_CATCH;
		}
		//		     }
	}
	else if (direction == 3)
	{
		// 双激光使车体对正 X Y
		//			if(Laser2_calibration(DT35.y,DT35.x))
		//			{
		// 激光zhuizong Y K
		if (Laser_calibration_ball(994, 3572, 0.0, 1500) || catch_ball_laser == 2)
		{
			// 不确定要不要加上倒车，看看测试 这里加上夹爪放苗之类的即可
			ROBOT_CHASSI.plan_x = 0;
			ROBOT_CHASSI.plan_y = 0;
			ROBOT_CHASSI.plan_w = 0;
			move_state = MOVE_STATE_WAIT_CATCH;
		}
		//		     }
	}
	else if (direction == 4)
	{
		// 双激光使车体对正 X Y
		//			if(Laser2_calibration(DT35.y,DT35.x))
		//			{
		// 激光zhuizong Y K
		if (Laser_calibration_ball(495, 3072, 0.0, 1500) || catch_ball_laser == 2)
		{
			ROBOT_CHASSI.plan_x = 0;
			ROBOT_CHASSI.plan_y = 0;
			ROBOT_CHASSI.plan_w = 0;
			// 不确定要不要加上倒车，看看测试 这里加上夹爪放苗之类的即可
			move_state = MOVE_STATE_WAIT_CATCH;
		}
		//		     }
	}
	else if (direction == 5)
	{
		//		      //双激光使车体对正 X Y
		//			if(Laser2_calibration(DT35.y,DT35.x))
		//			{
		// 激光zhuizong X K
		if (Laser_calibration_ball(506, 2572, 0.0, 1500) || catch_ball_laser == 2)
		{
			ROBOT_CHASSI.plan_x = 0;
			ROBOT_CHASSI.plan_y = 0;
			ROBOT_CHASSI.plan_w = 0;
			// 不确定要不要加上倒车，看看测试 这里加上夹爪放苗之类的即可
			move_state = MOVE_STATE_WAIT_CATCH;
		}
		//		     }
	}
	else if (direction == 6)
	{
		YawAdjust(0.0);
		//		      //双激光使车体对正 X Y
		//			if(Laser2_calibration(DT35.y,DT35.x))
		//			{
		// 激光zhuizong X K
		if (Laser_calibration_ball(506, 2072, 0.0, 1500) || catch_ball_laser == 2)
		{
			// 不确定要不要加上倒车，看看测试 这里加上夹爪放苗之类的即可
			ROBOT_CHASSI.plan_x = 0;
			ROBOT_CHASSI.plan_y = 0;
			ROBOT_CHASSI.plan_w = 0;
			move_state = MOVE_STATE_WAIT_CATCH;
		}
		//		     }
	}
	//---------------------------------------------------------------------
	//-----------------------后5个球加上第7个球特殊取-----------------------
	// 第7个球是横着取的

	else if (direction == 7)
	{
		//		     //双激光使车体对正 X Y
		//			if(Laser2_calibration(DT35.y,DT35.x))
		//			{
		// 激光zhuizong Y B
		if (Laser_calibration_ball(2010, 4572, 0, 1500) || catch_ball_laser == 1 || DT35.b < 2120)
		{
			// 不确定要不要加上倒车，看看测试 这里加上夹爪放苗之类的即可
			ROBOT_CHASSI.plan_x = 0;
			ROBOT_CHASSI.plan_y = 0;
			ROBOT_CHASSI.plan_w = 0;
			move_state = MOVE_STATE_WAIT_CATCH;
		}
		//		     }
	}
	// 后面那几个球全是激光加上action  前面已经重置过action了
	else if (direction == 8)
	{
		// 双激光使车体对正 X Y
		//		if(Laser2_calibration(DT35.y,DT35.x))
		//		  {
		//			  ROBOT_CHASSI.plan_x = 0;
		//			  ROBOT_CHASSI.plan_y = 0;
		//			  ROBOT_CHASSI.plan_w = 0;
		// 激光zhuizong Y K
		if (Laser_calibration_ball(1528, 4072, 0, 1500) || catch_ball_laser == 1 || DT35.b < 1640)
		{
			tr = 1;
			// 不确定要不要加上倒车，看看测试 这里加上夹爪放苗之类的即可
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
		//	     //双激光使车体对正 X Y
		//		if(Laser2_calibration(DT35.y,DT35.x))
		//		  {
		// 激光zhuizong Y K
		if (Laser_calibration_ball(1557, 3572, 0, 1500) || catch_ball_laser == 1 || DT35.b < 1090)
		{
			ROBOT_CHASSI.plan_x = 0;
			ROBOT_CHASSI.plan_y = 0;
			ROBOT_CHASSI.plan_w = 0;
			move_state = MOVE_STATE_WAIT_CATCH;
		}
		//		     }
	}
	else if (direction == 10)
	{
		tr = 2;
		// 激光zhuizong Y K
		if (Laser_calibration_ball(976, 3072, 0, 1500) || catch_ball_laser == 1 || DT35.b < 540)
		{

			ROBOT_CHASSI.plan_x = 0;
			ROBOT_CHASSI.plan_y = 0;
			ROBOT_CHASSI.plan_w = 0;
			move_state = MOVE_STATE_WAIT_CATCH;
		}
		//		     }
	}
	else if (direction == 11)
	{
		// 双激光使车体对正 X Y
		//		if(Laser2_calibration(DT35.y,DT35.x))
		//		  {
		// 激光zhuizong Y K
		if (Laser_calibration_ball(1004, 2572, 0, 1500) || catch_ball_laser == 1)
		{
			ROBOT_CHASSI.plan_x = 0;
			ROBOT_CHASSI.plan_y = 0;
			ROBOT_CHASSI.plan_w = 0;
			move_state = MOVE_STATE_WAIT_CATCH;
		}
		//		     }
	}
	else if (direction == 12)
	{
		// 双激光使车体对正 X Y
		//		if(Laser2_calibration(DT35.y,DT35.x))
		//		  {
		// 激光zhuizong Y K
		tr = 3;
		if (Laser_calibration_ball(1017, 2072, 0, 1500) || catch_ball_laser == 1)
		{

			ROBOT_CHASSI.plan_x = 0;
			ROBOT_CHASSI.plan_y = 0;
			ROBOT_CHASSI.plan_w = 0;
			move_state = MOVE_STATE_WAIT_CATCH;
		}
		//		     }
	}
}
