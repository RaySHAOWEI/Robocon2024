//
// Created by 19115 on 2024/4/1.
//

#include "DT35.h"

// 滤波宽度
int times = 50;

average average_x, average_y, average_k, average_b;

// 平均值滤波
int Get_Adc_Average(int *ch, int time)
{
	int temp_val = 0;
	int t;
	for (t = 0; t < time; t++)
	{
		temp_val += ch[t];
	}
	return temp_val / time;
}

/**
 * @brief  计算八位循环冗余校验，被usartSendData和usartReceiveOneData函数调用
 * @param   数组地址、数组大小
 * @retval
 */
unsigned char getCrc8(unsigned char *ptr, unsigned short len)
{
	unsigned char crc;
	unsigned char i;
	crc = 0;
	while (len--)
	{
		crc ^= *ptr++;
		for (i = 0; i < 8; i++)
		{
			if (crc & 0x01)
				crc = (crc >> 1) ^ 0x8C;
			else
				crc >>= 1;
		}
	}
	return crc;
}

///*--------------------------------------激光--------------------------*/

////激光数据读取
///**DT35消息格式：
//*0xaa 0x55 第三个是一帧数据长度 4567为第一个激光 789 10为第二个激光 11 12 13 14为第三个激光 15 16 17 18 19为第四个激光
//*20为校验位消息0x07 21为数据尾（不用管）
//*
//*最后使用联合体将字符类型转为整型数据
//*/
///**
//  * @brief  四个激光测距usart1  DT35:俩个数据头
//  * @param   返回四个激光的信息
//  * @retval
//  */
union receiveData1
{
	int d;
	unsigned char data[4];
} x_position, y_position, k_position, b_position;

float test3 = 0;
// 做了修改，修改用于四个激光测距
unsigned char receiveBuff_u1[22] = {0};
unsigned char USART1_Receiver = 0; // 用于存储接收到的串口数据的变量
unsigned char header[2] = {0x55, 0xaa};
const unsigned char ender[2] = {0x0d, 0x0a};
void VisionReceiveData_1(int *theta_angle, int *theta_pitch, int *theta_yaw, int *theta_ras)
{
	static unsigned char checkSum = 0;
	static unsigned char USARTBufferIndex = 0;
	static short j = 0, k = 0;
	static unsigned char USARTReceiverFront = 0; // 用于存储消息头的首个字节
	static unsigned char Start_Flag = START;	 // 一帧数据传送开始标志位
	static short dataLength = 0;
	HAL_UART_Receive_IT(&huart5, &USART1_Receiver, 1); // 继续监听  只接受一个字节的数据

	// 接收数据头
	if (Start_Flag == START)
	{
		if (USART1_Receiver == 0xaa)
		{
			if (USARTReceiverFront == 0x55)
			{
				Start_Flag = !START;
				receiveBuff_u1[0] = header[0]; // buf[0]
				receiveBuff_u1[1] = header[1]; // buf[1]
				USARTBufferIndex = 0;		   // 缓冲区初始化
				checkSum = 0x00;			   // 校验和初始化
			}
		}
		else
		{
			USARTReceiverFront = USART1_Receiver;
		}
	}
	else // 开始解析数据
	{
		test3 = 1;
		switch (USARTBufferIndex)
		{
		case 0: // 接收数据的长度
			receiveBuff_u1[2] = USART1_Receiver;
			dataLength = receiveBuff_u1[2]; // buf[2]
			USARTBufferIndex++;
			break;
		case 1:										 // 接收所有数据，并赋值处理
			receiveBuff_u1[j + 3] = USART1_Receiver; // buf[3] buf[4]/buf[5] buf[6]	/	buf[7] buf[8]/		buf[9] buf[10]
			j++;
			if (j >= dataLength - 1)
			{
				j = 0;
				USARTBufferIndex++;
			}
			break;
		case 2: // 接收校验值信息(设定为0x07)
			receiveBuff_u1[2 + dataLength] = USART1_Receiver;
			checkSum = getCrc8(receiveBuff_u1, 3 + dataLength);
			//				checkSum = 0x07;
			// 检查信息校验值
			//				if (checkSum != receiveBuff[3 + dataLength]) //buf[11]
			//				{
			////					printf("Received data check sum error!");
			//					return 0;
			//				}
			USARTBufferIndex++;
			break;
		case 3: // 接收数据尾
			if (k == 0)
			{
				// 数据0d     buf[11]  无需判断
				k++;
			}
			else if (k == 1)
			{
				// 数据0a     buf[12] 无需判断

				// 进行速度赋值操作
				for (k = 0; k < 4; k++)
				{
					x_position.data[k] = receiveBuff_u1[k + 3];	 // buf[3]  buf[4] buf[5]  buf[6] 激光1
					y_position.data[k] = receiveBuff_u1[k + 7];	 // buf[7]  buf[8] buf[9]  buf[10] 激光2
					k_position.data[k] = receiveBuff_u1[k + 11]; // buf[3]  buf[4] buf[5]  buf[6] 激光3
					b_position.data[k] = receiveBuff_u1[k + 15]; // buf[7]  buf[8] buf[9]  buf[10] 激光4
				}
				//					if(x_position.d==12)
				//					{
				//						printf("OK!");
				//					}
				//					else
				//					{
				//						printf("error!");
				//					}
				// 赋值操作
				// y
				*theta_angle = y_position.d ;
				average_y.t++;
				if (average_y.t >= times)
				{
					average_y.t = 0;
				}
				average_y.data[average_y.t] = *theta_angle;
				*theta_angle = Get_Adc_Average(average_y.data, times);
				// x
				*theta_pitch = 2.75 * x_position.d - 2.14;
				average_x.t++;
				if (average_x.t >= times)
				{
					average_x.t = 0;
				}
				average_x.data[average_x.t] = *theta_pitch;
				*theta_pitch = Get_Adc_Average(average_x.data, times);
				// k
				*theta_yaw = 1.88 * k_position.d + 65.5;
				average_k.t++;
				if (average_k.t >= times)
				{
					average_k.t = 0;
				}
				average_k.data[average_k.t] = *theta_yaw;
				*theta_yaw = Get_Adc_Average(average_k.data, times);
				// b
				*theta_ras = 2.75 * b_position.d - 2.75;
//				*theta_ras = 4.37 * b_position.d - 29.1;
//				*theta_ras = 4.21 * b_position.d - 24.2;
//         *theta_ras = 2.8835 * b_position.d + 50.846;
				average_b.t++;
				if (average_b.t >= 50)
				{
					average_b.t = 0;
				}
				average_b.data[average_b.t] = *theta_ras;
				*theta_ras = Get_Adc_Average(average_b.data, 50);
				//					*theta =(int)angle.d;
				//
				//					//ctrlFlag
				//					*flag = receiveBuff[9];                //buf[9]
				//-----------------------------------------------------------------
				// 完成一个数据包的接收，相关变量清零，等待下一字节数据
				USARTBufferIndex = 0;
				USARTReceiverFront = 0;
				Start_Flag = START;
				checkSum = 0;
				dataLength = 0;
				j = 0;
				k = 0;
				//-----------------------------------------------------------------
			}
			break;
		default:
			break;
		}
	}
}

// 激光平均值滤波
averageFilter_TPYE laser_filter_y;
averageFilter_TPYE laser_filter_x;
averageFilter_TPYE laser_filter_k;
averageFilter_TPYE laser_filter_b;

DT35_TPYE origin_DT35 = {0}, DT35 = {0}, DT35_FILTE = {0};
KFP KFP_x = {0.05, 0, 0, 0, 0.001, 1}, KFP_y = {0.05, 0, 0, 0, 0.001, 1}, KFP_k = {0.05, 0, 0, 0, 0.001, 1}, KFP_b = {0.05, 0, 0, 0, 0.001, 1};

PID_T laser_X_pid;
PID_T laser_Y_pid;
PID_T laser_K_pid;
PID_T laser_B_pid;

float aa = 0;
void Update_DT35(int theta_angle, int theta_pitch, int theta_yaw, int theta_ras)
{
	origin_DT35.y = theta_angle;
	origin_DT35.x = theta_pitch;
	origin_DT35.k = theta_yaw;
	origin_DT35.b = theta_ras;

	DT35_FILTE.x = kalmanFilter(&KFP_x, origin_DT35.x);
	DT35_FILTE.y = kalmanFilter(&KFP_y, origin_DT35.y);
	DT35_FILTE.k = kalmanFilter(&KFP_k, origin_DT35.k);
	DT35_FILTE.b = kalmanFilter(&KFP_b, origin_DT35.b);

	DT35.y = 1.88 * origin_DT35.y + 83.3;
	DT35.x = 1.78 * origin_DT35.x + 11.9;
	DT35.k = origin_DT35.k;
	DT35.b = origin_DT35.b;

	aa++;
}

float kalmanFilter(KFP *kfp, float input)
{
	// 预测协方差方程：k时刻系统估算协方差 = k-1时刻的系统协方差 + 过程噪声协方差
	kfp->Now_P = kfp->LastP + kfp->Q;
	// 卡尔曼增益方程：卡尔曼增益 = k时刻系统估算协方差 / （k时刻系统估算协方差 + 观测噪声协方差）
	kfp->Kg = kfp->Now_P / (kfp->Now_P + kfp->R);
	// 更新最优值方程：k时刻状态变量的最优值 = 状态变量的预测值 + 卡尔曼增益 * （测量值 - 状态变量的预测值）
	kfp->out = kfp->out + kfp->Kg * (input - kfp->out); // 因为这一次的预测值就是上一次的输出值
	// 更新协方差方程: 本次的系统协方差付给 kfp->LastP 威下一次运算准备。
	kfp->LastP = (1 - kfp->Kg) * kfp->Now_P;
	return kfp->out;
}

// 单激光简单控制  倒车用
int Laser1_calibration(float k, float v_max)
{
	Laser_error_k = DT35.k - k;

	if (ABS(Laser_error_k) > 5000)
	{
		ROBOT_CHASSI.plan_x = 0;
		ROBOT_CHASSI.plan_y = 0;
		return 0;
	}
	else
	{
		laser_K_pid.MaxOutput = ABS(v_max);
		pid_calc_by_error(&laser_K_pid, Laser_error_k);
		ROBOT_CHASSI.plan_x = -(laser_K_pid.output);
		if (ABS(Laser_error_k) < 3)
		{
			ROBOT_CHASSI.plan_x = 0;
			ROBOT_CHASSI.plan_y = 0;
			return 1;
		}
		else
		{
			return 0;
		}
	}
}

// 单激光简单控制  倒车用
int Laser3_calibration(float k, float v_max)
{
	Laser_error_k = DT35.y - k;

	if (ABS(Laser_error_k) > 5000)
	{
		ROBOT_CHASSI.plan_x = 0;
		ROBOT_CHASSI.plan_y = 0;
		return 0;
	}
	else
	{
		laser_K_pid.MaxOutput = ABS(v_max);
		pid_calc_by_error(&laser_K_pid, Laser_error_k);
		ROBOT_CHASSI.plan_y = -(laser_K_pid.output);
		if (ABS(Laser_error_k) < 3)
		{
			ROBOT_CHASSI.plan_x = 0;
			ROBOT_CHASSI.plan_y = 0;
			return 1;
		}
		else
		{
			return 0;
		}
	}
}

/**
 * @brief  平均值滤波
 * @param  None
 * @retval None
 * @attention
 */
void averageFilter(averageFilter_TPYE *ff)
{
	float sum = 0;
	for (int i = 0; i < 4; i++)
	{
		ff->data[i] = ff->data[i + 1];
		sum = sum + ff->data[i];
	}
	ff->data[4] = ff->indata;
	sum = sum + ff->data[4];

	ff->outdata = sum / 5;
}

int tt = 0;
int tx = 0;
int ty = 0;
int hhh = 0;
// 双激光使底盘保持正  y x
int Laser2_calibration(float location_x1, float location_x2)
{
	ROBOT_CHASSI.plan_x = 0;
	ROBOT_CHASSI.plan_y = 0;
	int laser_error_location = 0;
	laser_error_location = ABS(location_x1 - location_x2);
	if (tt == 2)
	{
		return 1;
	}
	// 往左边转
	if ((location_x1 - location_x2) > 18)
	{
		ROBOT_CHASSI.plan_w = laser_error_location * -6;
		tt = -1;
	}
	else if (((location_x1 - location_x2) > -4) && ((location_x1 - location_x2) < 18))
	{
		ROBOT_CHASSI.plan_w = laser_error_location * -2;
		//		ROBOT_CHASSI.plan_w = -30;
		tt = -1;
	}
	// 往右边转
	else if ((location_x1 - location_x2) < -27)
	{
		ROBOT_CHASSI.plan_w = laser_error_location * 6;
		tt = 1;
	}
	else if (((location_x1 - location_x2) > -27) && ((location_x1 - location_x2) < -7))
	{
		ROBOT_CHASSI.plan_w = laser_error_location * 2;
		//		ROBOT_CHASSI.plan_w = 30;
		tt = 1;
		hhh = 1;
	}
	else
	{
		ROBOT_CHASSI.plan_w = 0;
		tx = location_x2;
		ty = location_x1;
		//		ROBOT_CHASSI.plan_x = 0;
		//		ROBOT_CHASSI.plan_y = 0;
		tt = 2;
		return 1;
	}
	//	if((location_x1 - location_x2)<-12)
	//	{
	//		ROBOT_CHASSI.plan_w = 500;
	//		tt = 1;
	//	}
	//	else
	//	{
	//		ROBOT_CHASSI.plan_w = 0;
	//		ROBOT_CHASSI.plan_x = 0;
	//		ROBOT_CHASSI.plan_y = 0;
	//		tt = 2;
	//		return 1;
	//	}

	return 0;
}
float aa1 = 0;
float Laser_error_y, Laser_error_k, ERROR_SHOOTING;
// 修改定位   取苗左
int Laser_calibration(float x, float y, float yaw, float v_max)
{
	tt = 0;
//	YawAdjust(yaw);
	// 转到指定角度
	Laser_error_y = DT35.x - x;
	Laser_error_k = DT35.k - y;
	ERROR_SHOOTING = sqrt(Laser_error_y * Laser_error_y + Laser_error_k * Laser_error_k);
	// 怎么利用该误差还需填充
	// 判断距离是否合适
	if (Laser_error_y < 8000 && Laser_error_k < 8000)
	{
		if (ABS(Laser_error_y) > 10 || ABS(Laser_error_k) > 10)
		{
			laser_B_pid.MaxOutput = ABS(v_max);
			pid_calc_by_error(&TRACK_PID, ERROR_SHOOTING);
			//1.5 1.7
			ROBOT_CHASSI.plan_x = -1.5f * (Max_Value_Limit((TRACK_PID.output * 0.1f * Laser_error_k / ERROR_SHOOTING), v_max));
			ROBOT_CHASSI.plan_y = -3.7f * (Max_Value_Limit((TRACK_PID.output * 0.1f * Laser_error_y / ERROR_SHOOTING), v_max));

			Laser_error_y = DT35.y - x;
			Laser_error_k = DT35.k - y;
			aa1 = 0;
		}
		else
		{

			ROBOT_CHASSI.plan_x = 0;
			ROBOT_CHASSI.plan_y = 0;
			ROBOT_CHASSI.plan_w = 0;
			return 1;
		}
	}
	return 0;
}
int Laser_calibration_right1(float x, float y, float yaw, float v_max)
{
	tt = 0;
//	YawAdjust(yaw);
	// 转到指定角度
	Laser_error_y = DT35.y - x;
	Laser_error_k = DT35.x - y;
	ERROR_SHOOTING = sqrt(Laser_error_y * Laser_error_y + Laser_error_k * Laser_error_k);
	// 怎么利用该误差还需填充
	// 判断距离是否合适
	if (Laser_error_y < 8000 && Laser_error_k < 8000)
	{
		if (ABS(Laser_error_y) > 3 || ABS(Laser_error_k) > 3)
		{
			laser_B_pid.MaxOutput = ABS(v_max);
			pid_calc_by_error(&TRACK_PID, ERROR_SHOOTING);
			//1.1 1.3
			ROBOT_CHASSI.plan_x = -1.5f * (Max_Value_Limit((TRACK_PID.output * 0.1f * Laser_error_k / ERROR_SHOOTING), v_max));
			ROBOT_CHASSI.plan_y = -1.7f * (Max_Value_Limit((TRACK_PID.output * 0.1f * Laser_error_y / ERROR_SHOOTING), v_max));

			Laser_error_y = DT35.y - x;
			Laser_error_k = DT35.k - y;
			aa1 = 0;
		}
		else
		{

			ROBOT_CHASSI.plan_x = 0;
			ROBOT_CHASSI.plan_y = 0;
			ROBOT_CHASSI.plan_w = 0;
			return 1;
		}
	}
	return 0;
}
//取苗右
int Laser_calibration_right(float x, float y, float yaw, float v_max)
{
	tt = 0;
	YawAdjust(yaw);
	// 转到指定角度
	Laser_error_y = DT35.y - x;
	Laser_error_k = DT35.x - y;
	ERROR_SHOOTING = sqrt(Laser_error_y * Laser_error_y + Laser_error_k * Laser_error_k);
	// 怎么利用该误差还需填充
	// 判断距离是否合适
	if (Laser_error_y < 8000 && Laser_error_k < 8000)
	{
		if (ABS(Laser_error_y) > 28 || ABS(Laser_error_k) > 28)
		{
			laser_B_pid.MaxOutput = ABS(v_max);
			pid_calc_by_error(&TRACK_PID, ERROR_SHOOTING);
			ROBOT_CHASSI.plan_x = 1.3f * (Max_Value_Limit((TRACK_PID.output * 0.1f * Laser_error_k / ERROR_SHOOTING), v_max));
			ROBOT_CHASSI.plan_y = -1.3f * (Max_Value_Limit((TRACK_PID.output * 0.1f * Laser_error_y / ERROR_SHOOTING), v_max));

			Laser_error_y = DT35.y - x;
			Laser_error_k = DT35.x - y;
			aa1 = 0;
		}
		else
		{

			ROBOT_CHASSI.plan_x = 0;
			ROBOT_CHASSI.plan_y = 0;
			ROBOT_CHASSI.plan_w = 0;
			return 1;
		}
	}
	return 0;
}


float aa2 = 0;
float Laser_error_y_ball, Laser_error_k_ball, ERROR_SHOOTING_ball;
// 修改定位  取球左
int Laser_calibration_ball(float x, float y, float yaw, float v_max)
{
	tt = 0;
//	YawAdjust(yaw);

	// 转到指定角度
	Laser_error_y_ball = DT35.x - x;
	Laser_error_k_ball = DT35.b - y;
	ERROR_SHOOTING_ball = sqrt(Laser_error_y_ball * Laser_error_y_ball + Laser_error_k_ball * Laser_error_k_ball);
	// 怎么利用该误差还需填充
	// 判断距离是否合适
	if (Laser_error_y_ball < 8000 && Laser_error_k_ball < 8000)
	{
		if (ABS(Laser_error_y_ball) > 280 || ABS(Laser_error_k_ball) > 280)
		{
			laser_B_pid.MaxOutput = ABS(v_max);
			pid_calc_by_error(&point_traker_x_pid, ERROR_SHOOTING_ball);
			ROBOT_CHASSI.plan_x = 3.0f * (Max_Value_Limit((point_traker_x_pid.output * 0.1f * Laser_error_k_ball / ERROR_SHOOTING_ball), v_max));
			ROBOT_CHASSI.plan_y = -3.0f * (Max_Value_Limit((point_traker_x_pid.output * 0.1f * Laser_error_y_ball / ERROR_SHOOTING_ball), v_max));
			Laser_error_y_ball = DT35.x - x;
			Laser_error_k_ball = DT35.b - y;
			aa2 = 1;
		}
		else if ((ABS(Laser_error_y_ball) > 35 || ABS(Laser_error_k_ball) > 35) && (ABS(Laser_error_y_ball) < 280 || ABS(Laser_error_k_ball) < 280))
		{
			laser_B_pid.MaxOutput = ABS(v_max);
			pid_calc_by_error(&laser_B_pid, ERROR_SHOOTING_ball);
			ROBOT_CHASSI.plan_x = 1.5f * (Max_Value_Limit((laser_B_pid.output * 0.1f * Laser_error_k_ball / ERROR_SHOOTING_ball), v_max));
			ROBOT_CHASSI.plan_y = -1.5f * (Max_Value_Limit((laser_B_pid.output * 0.1f * Laser_error_y_ball / ERROR_SHOOTING_ball), v_max));
			Laser_error_y_ball = DT35.x - x;
			Laser_error_k_ball = DT35.b - y;
			aa1 = 1;
		}
		else
		{

			ROBOT_CHASSI.plan_x = 0;
			ROBOT_CHASSI.plan_y = 0;
			ROBOT_CHASSI.plan_w = 0;
			return 1;
		}
	}
	return 0;
}
int Laser_calibration_ball_left(float x, float y, float yaw, float v_max)
{
	tt = 0;
//	YawAdjust(yaw);

	// 转到指定角度
	Laser_error_y_ball = DT35.x - x;
	Laser_error_k_ball = DT35.k - y;
	ERROR_SHOOTING_ball = sqrt(Laser_error_y_ball * Laser_error_y_ball + Laser_error_k_ball * Laser_error_k_ball);
	// 怎么利用该误差还需填充
	// 判断距离是否合适
	if (Laser_error_y_ball < 8000 && Laser_error_k_ball < 8000)
	{
		if (ABS(Laser_error_y_ball) > 280 || ABS(Laser_error_k_ball) > 280)
		{
			laser_B_pid.MaxOutput = ABS(v_max);
			pid_calc_by_error(&point_traker_x_pid, ERROR_SHOOTING_ball);
			ROBOT_CHASSI.plan_x = -3.0f * (Max_Value_Limit((point_traker_x_pid.output * 0.1f * Laser_error_k_ball / ERROR_SHOOTING_ball), v_max));
			ROBOT_CHASSI.plan_y = -3.0f * (Max_Value_Limit((point_traker_x_pid.output * 0.1f * Laser_error_y_ball / ERROR_SHOOTING_ball), v_max));
			Laser_error_y_ball = DT35.y - x;
			Laser_error_k_ball = DT35.x - y;
			aa2 = 1;
		}
		else if ((ABS(Laser_error_y_ball) > 35 || ABS(Laser_error_k_ball) > 35) && (ABS(Laser_error_y_ball) < 280 || ABS(Laser_error_k_ball) < 280))
		{
			laser_B_pid.MaxOutput = ABS(v_max);
			pid_calc_by_error(&laser_B_pid, ERROR_SHOOTING_ball);
			ROBOT_CHASSI.plan_x = -1.5f * (Max_Value_Limit((laser_B_pid.output * 0.1f * Laser_error_k_ball / ERROR_SHOOTING_ball), v_max));
			ROBOT_CHASSI.plan_y = -1.5f * (Max_Value_Limit((laser_B_pid.output * 0.1f * Laser_error_y_ball / ERROR_SHOOTING_ball), v_max));
			Laser_error_y_ball = DT35.y - x;
			Laser_error_k_ball = DT35.x - y;
			aa1 = 1;
		}
		else
		{

			ROBOT_CHASSI.plan_x = 0;
			ROBOT_CHASSI.plan_y = 0;
			ROBOT_CHASSI.plan_w = 0;
			return 1;
		}
	}
	return 0;
}
int Laser_calibration_laser_left(float x, float y, float yaw, float v_max)
{
	tt = 0;
//	YawAdjust(yaw);

	// 转到指定角度
	Laser_error_y_ball = DT35.x - x;
	Laser_error_k_ball = DT35.k - y;
	ERROR_SHOOTING_ball = sqrt(Laser_error_y_ball * Laser_error_y_ball + Laser_error_k_ball * Laser_error_k_ball);
	// 怎么利用该误差还需填充
	// 判断距离是否合适
	if (Laser_error_y_ball < 8000 && Laser_error_k_ball < 8000)
	{
		if (ABS(Laser_error_y_ball) > 280 || ABS(Laser_error_k_ball) > 280)
		{
			laser_B_pid.MaxOutput = ABS(v_max);
			pid_calc_by_error(&point_traker_x_pid, ERROR_SHOOTING_ball);
			ROBOT_CHASSI.plan_x = -1.5f * (Max_Value_Limit((point_traker_x_pid.output * 0.1f * Laser_error_k_ball / ERROR_SHOOTING_ball), v_max));
			ROBOT_CHASSI.plan_y = -1.5f * (Max_Value_Limit((point_traker_x_pid.output * 0.1f * Laser_error_y_ball / ERROR_SHOOTING_ball), v_max));
			Laser_error_y_ball = DT35.y - x;
			Laser_error_k_ball = DT35.x - y;
			aa2 = 1;
		}
		else if ((ABS(Laser_error_y_ball) > 35 || ABS(Laser_error_k_ball) > 35) && (ABS(Laser_error_y_ball) < 280 || ABS(Laser_error_k_ball) < 280))
		{
			laser_B_pid.MaxOutput = ABS(v_max);
			pid_calc_by_error(&laser_B_pid, ERROR_SHOOTING_ball);
			ROBOT_CHASSI.plan_x = -1.5f * (Max_Value_Limit((laser_B_pid.output * 0.1f * Laser_error_k_ball / ERROR_SHOOTING_ball), v_max));
			ROBOT_CHASSI.plan_y = -1.5f * (Max_Value_Limit((laser_B_pid.output * 0.1f * Laser_error_y_ball / ERROR_SHOOTING_ball), v_max));
			Laser_error_y_ball = DT35.y - x;
			Laser_error_k_ball = DT35.x - y;
			aa1 = 1;
		}
		else
		{

			ROBOT_CHASSI.plan_x = 0;
			ROBOT_CHASSI.plan_y = 0;
			ROBOT_CHASSI.plan_w = 0;
			return 1;
		}
	}
	return 0;
}
float laser_start = 0;
float laser_start1 = 0;
//右侧激光  取球右
int Laser_calibration_ball_right(float x, float y, float yaw, float v_max)
{
	tt = 0;
	YawAdjust(yaw);

	// 转到指定角度
	Laser_error_y_ball = DT35.y - x;
	Laser_error_k_ball = DT35.x - y;
	ERROR_SHOOTING_ball = sqrt(Laser_error_y_ball * Laser_error_y_ball + Laser_error_k_ball * Laser_error_k_ball);
	// 怎么利用该误差还需填充
	// 判断距离是否合适
	if (Laser_error_y_ball < 8000 && Laser_error_k_ball < 8000)
	{
		if (ABS(Laser_error_y_ball) > 280 || ABS(Laser_error_k_ball) > 280)
		{
			laser_B_pid.MaxOutput = ABS(v_max);
			pid_calc_by_error(&point_traker_x_pid, ERROR_SHOOTING_ball);
			ROBOT_CHASSI.plan_x = 4.0f * (Max_Value_Limit((point_traker_x_pid.output * 0.1f * Laser_error_k_ball / ERROR_SHOOTING_ball), v_max));
			ROBOT_CHASSI.plan_y = -4.0f * (Max_Value_Limit((point_traker_x_pid.output * 0.1f * Laser_error_y_ball / ERROR_SHOOTING_ball), v_max));
			Laser_error_y_ball = DT35.y - x;
			Laser_error_k_ball = DT35.x - y;
			aa2 = 1;
//      laser_start = 3.0f * (Max_Value_Limit((laser_B_pid.output * 0.1f * Laser_error_k_ball / ERROR_SHOOTING_ball), v_max));
//      laser_start1= -3.0f * (Max_Value_Limit((laser_B_pid.output * 0.1f * Laser_error_y_ball / ERROR_SHOOTING_ball), v_max));	
		}
		else if ((ABS(Laser_error_y_ball) > 35 || ABS(Laser_error_k_ball) > 35) && (ABS(Laser_error_y_ball) < 280 || ABS(Laser_error_k_ball) < 280))
		{
			laser_B_pid.MaxOutput = ABS(v_max);
			pid_calc_by_error(&laser_B_pid, ERROR_SHOOTING_ball);
			ROBOT_CHASSI.plan_x = 2.0f * (Max_Value_Limit((laser_B_pid.output * 0.1f * Laser_error_k_ball / ERROR_SHOOTING_ball), v_max));
			ROBOT_CHASSI.plan_y = -2.0f * (Max_Value_Limit((laser_B_pid.output * 0.1f * Laser_error_y_ball / ERROR_SHOOTING_ball), v_max));
//      laser_start = 1.4f * (Max_Value_Limit((laser_B_pid.output * 0.1f * Laser_error_k_ball / ERROR_SHOOTING_ball), v_max));
//      laser_start1= -1.4f * (Max_Value_Limit((laser_B_pid.output * 0.1f * Laser_error_y_ball / ERROR_SHOOTING_ball), v_max));	
			Laser_error_y_ball = DT35.y - x;
			Laser_error_k_ball = DT35.x - y;
			aa1 = 1;
		}
		else
		{

			ROBOT_CHASSI.plan_x = 0;
			ROBOT_CHASSI.plan_y = 0;
			ROBOT_CHASSI.plan_w = 0;
			return 1;
		}
     }
	return 0;
}

// 修改定位   放苗左边
int Laser_calibration_seed(float x, float y, float yaw, float v_max)
{
	tt = 0;
		YawAdjust(yaw);
	// 转到指定角度
	Laser_error_y_ball = DT35.x - x;
	Laser_error_k_ball = DT35.k - y;
	ERROR_SHOOTING_ball = sqrt(Laser_error_y_ball * Laser_error_y_ball + Laser_error_k_ball * Laser_error_k_ball);
	// 怎么利用该误差还需填充
	// 判断距离是否合适
	if (Laser_error_y_ball < 8000 && Laser_error_k_ball < 8000)
	{
		if (ABS(Laser_error_y_ball) > 280 || ABS(Laser_error_k_ball) > 280)
   {
      laser_B_pid.MaxOutput = ABS(v_max);
			pid_calc_by_error(&laser_B_pid, ERROR_SHOOTING_ball);
			ROBOT_CHASSI.plan_x = -1.5f * (Max_Value_Limit((laser_B_pid.output * 0.1f * Laser_error_k_ball / ERROR_SHOOTING_ball), v_max));
			ROBOT_CHASSI.plan_y = -1.5f * (Max_Value_Limit((laser_B_pid.output * 0.1f * Laser_error_y_ball / ERROR_SHOOTING_ball), v_max));
			Laser_error_y_ball = DT35.y - x;
			Laser_error_k_ball = DT35.k - y;
   }
		else if ((ABS(Laser_error_y_ball) > 35 || ABS(Laser_error_k_ball) > 35) && (ABS(Laser_error_y_ball) < 280 || ABS(Laser_error_k_ball) < 280))
		{
			laser_B_pid.MaxOutput = ABS(v_max);
			pid_calc_by_error(&laser_B_pid, ERROR_SHOOTING_ball);
			ROBOT_CHASSI.plan_x = -1.5f * (Max_Value_Limit((laser_B_pid.output * 0.1f * Laser_error_k_ball / ERROR_SHOOTING_ball), v_max));
			ROBOT_CHASSI.plan_y = -1.5f * (Max_Value_Limit((laser_B_pid.output * 0.1f * Laser_error_y_ball / ERROR_SHOOTING_ball), v_max));
			Laser_error_y_ball = DT35.y - x;
			Laser_error_k_ball = DT35.k - y;
			aa1 = 0;
		}
		else
		{

			ROBOT_CHASSI.plan_x = 0;
			ROBOT_CHASSI.plan_y = 0;
			ROBOT_CHASSI.plan_w = 0;
			return 1;
		}
	}
	return 0;
}

// 修改定位  放苗右边
int Laser_calibration_seed_right(float x, float y, float yaw, float v_max)
{
	tt = 0;
//	
//		YawAdjust(yaw);
	// 转到指定角度
	Laser_error_y_ball = DT35.x - x;
	Laser_error_k_ball = DT35.b - y;
	ERROR_SHOOTING_ball = sqrt(Laser_error_y_ball * Laser_error_y_ball + Laser_error_k_ball * Laser_error_k_ball);
	// 怎么利用该误差还需填充
	// 判断距离是否合适
	if (Laser_error_y_ball < 8000 && Laser_error_k_ball < 8000)
	{
		if (ABS(Laser_error_y_ball) > 280 || ABS(Laser_error_k_ball) > 280)
   {
      laser_B_pid.MaxOutput = ABS(v_max);
			pid_calc_by_error(&laser_B_pid, ERROR_SHOOTING_ball);
			ROBOT_CHASSI.plan_x = 1.5f * (Max_Value_Limit((laser_B_pid.output * 0.1f * Laser_error_k_ball / ERROR_SHOOTING_ball), v_max));
			ROBOT_CHASSI.plan_y = -1.5f * (Max_Value_Limit((laser_B_pid.output * 0.1f * Laser_error_y_ball / ERROR_SHOOTING_ball), v_max));
			Laser_error_y_ball = DT35.x - x;
			Laser_error_k_ball = DT35.b - y;
   }
		else if ((ABS(Laser_error_y_ball) > 35 || ABS(Laser_error_k_ball) > 35) && (ABS(Laser_error_y_ball) < 280 || ABS(Laser_error_k_ball) < 280))
		{
			laser_B_pid.MaxOutput = ABS(v_max);
			pid_calc_by_error(&laser_B_pid, ERROR_SHOOTING_ball);
			ROBOT_CHASSI.plan_x = 1.5f * (Max_Value_Limit((laser_B_pid.output * 0.1f * Laser_error_k_ball / ERROR_SHOOTING_ball), v_max));
			ROBOT_CHASSI.plan_y = -1.5f * (Max_Value_Limit((laser_B_pid.output * 0.1f * Laser_error_y_ball / ERROR_SHOOTING_ball), v_max));
			Laser_error_y_ball = DT35.x - x;
			Laser_error_k_ball = DT35.b - y;
			aa1 = 0;
		}
		else
		{

			ROBOT_CHASSI.plan_x = 0;
			ROBOT_CHASSI.plan_y = 0;
			ROBOT_CHASSI.plan_w = 0;
			return 1;
		}
	}
	return 0;
}

int Laser_calibration_init(float x, float y, float yaw, float v_max)
{
	tt = 0;
	if (!yaw_disable)
	{
		YawAdjust(yaw);
	}
	// 转到指定角度
	Laser_error_y_ball = DT35.y - x;
	Laser_error_k_ball = DT35.k - y;
	ERROR_SHOOTING_ball = sqrt(Laser_error_y_ball * Laser_error_y_ball + Laser_error_k_ball * Laser_error_k_ball);
	// 怎么利用该误差还需填充
	// 判断距离是否合适
	if (Laser_error_y_ball < 8000 && Laser_error_k_ball < 8000)
	{
		if (ABS(Laser_error_y_ball) > 30 || ABS(Laser_error_k_ball) > 30)
		{
			laser_X_pid.MaxOutput = ABS(v_max);
			pid_calc_by_error(&laser_X_pid, ERROR_SHOOTING_ball);
			ROBOT_CHASSI.plan_x = -1.1f * (Max_Value_Limit((laser_X_pid.output * 0.1f * Laser_error_k_ball / ERROR_SHOOTING_ball), v_max));
			ROBOT_CHASSI.plan_y = -1.1f * (Max_Value_Limit((laser_X_pid.output * 0.1f * Laser_error_y_ball / ERROR_SHOOTING_ball), v_max));

			Laser_error_y_ball = DT35.y - x;
			Laser_error_k_ball = DT35.k - y;
			aa1 = 0;
		}
		else
		{

			ROBOT_CHASSI.plan_x = 0;
			ROBOT_CHASSI.plan_y = 0;
			ROBOT_CHASSI.plan_w = 0;
			return 1;
		}
	}
	return 0;
}

int Laser_calibration_init_right(float x, float y, float yaw, float v_max)
{
	tt = 0;
	if (!yaw_disable)
	{
		YawAdjust(yaw);
	}
	// 转到指定角度
	Laser_error_y_ball = DT35.x - x;
	Laser_error_k_ball = DT35.b - y;
	ERROR_SHOOTING_ball = sqrt(Laser_error_y_ball * Laser_error_y_ball + Laser_error_k_ball * Laser_error_k_ball);
	// 怎么利用该误差还需填充
	// 判断距离是否合适
	if (Laser_error_y_ball < 8000 && Laser_error_k_ball < 8000)
	{
		if (ABS(Laser_error_y_ball) > 60 || ABS(Laser_error_k_ball) > 60)
		{
			laser_X_pid.MaxOutput = ABS(v_max);
			pid_calc_by_error(&laser_X_pid, ERROR_SHOOTING_ball);
			ROBOT_CHASSI.plan_x = 1.1f * (Max_Value_Limit((laser_X_pid.output * 0.1f * Laser_error_k_ball / ERROR_SHOOTING_ball), v_max));
			ROBOT_CHASSI.plan_y = -1.1f * (Max_Value_Limit((laser_X_pid.output * 0.1f * Laser_error_y_ball / ERROR_SHOOTING_ball), v_max));

			Laser_error_y_ball = DT35.y - x;
			Laser_error_k_ball = DT35.b - y;
			aa1 = 0;
		}
		else
		{

			ROBOT_CHASSI.plan_x = 0;
			ROBOT_CHASSI.plan_y = 0;
			ROBOT_CHASSI.plan_w = 0;
			return 1;
		}
	}
	return 0;
}

// 左边场地激光
int Laser_calibration1(float x, float y, float yaw, float v_max)
{
	tt = 0;
	YawAdjust(yaw);
	// 转到指定角度
	Laser_error_y = DT35.y - x;
	Laser_error_k = DT35.k - y;
	ERROR_SHOOTING = sqrt(Laser_error_y * Laser_error_y + Laser_error_k * Laser_error_k);
	// 怎么利用该误差还需填充
	// 判断距离是否合适
	if (Laser_error_y < 8000 && Laser_error_k < 8000)
	{
		if (ABS(Laser_error_y) > 3 || ABS(Laser_error_k) > 3)
		{
			laser_B_pid.MaxOutput = ABS(v_max);
			pid_calc_by_error(&laser_B_pid, ERROR_SHOOTING);
			ROBOT_CHASSI.plan_x = -1.0f * (Max_Value_Limit((laser_B_pid.output * 0.1f * Laser_error_k / ERROR_SHOOTING), v_max));
			ROBOT_CHASSI.plan_y = -1.0f * (Max_Value_Limit((laser_B_pid.output * 0.1f * Laser_error_y / ERROR_SHOOTING), v_max));

			Laser_error_y = DT35.y - x;
			Laser_error_k = DT35.k - y;
			aa1 = 0;
		}
		else
		{

			ROBOT_CHASSI.plan_x = 0;
			ROBOT_CHASSI.plan_y = 0;
			ROBOT_CHASSI.plan_w = 0;
			return 1;
		}
	}
	return 0;
}
int flag_sss = 0;
int Laser_calibration7(float x, float y, float yaw, float v_max)
{
	aa1 = 1;
//	YawAdjust(yaw);
	// 转到指定角度
	Laser_error_y = DT35.x - x;
	Laser_error_k = DT35.b - y;
	ERROR_SHOOTING = sqrt(Laser_error_y * Laser_error_y + Laser_error_k * Laser_error_k);
	// 怎么利用该误差还需填充
	// 判断距离是否合适
	if (Laser_error_y < 8000 && Laser_error_k < 8000)
	{
		if (ABS(Laser_error_y) > 35 || ABS(Laser_error_k) > 35)
		{
			flag_sss = 1;
			aa1 = 2;
			laser_B_pid.MaxOutput = ABS(v_max);
			pid_calc_by_error(&laser_B_pid, ERROR_SHOOTING);
			ROBOT_CHASSI.plan_x = 1.1f * (Max_Value_Limit((laser_B_pid.output * 0.1f * Laser_error_k / ERROR_SHOOTING), v_max));
			ROBOT_CHASSI.plan_y = -1.3f * (Max_Value_Limit((laser_B_pid.output * 0.1f * Laser_error_y / ERROR_SHOOTING), v_max));
			//			ROBOT_CHASSI.plan_x = -100;
			Laser_error_y = DT35.y - x;
			Laser_error_k = DT35.x - y;
		}
		else
		{
			flag_sss = 2;
			ROBOT_CHASSI.plan_x = 0;
			ROBOT_CHASSI.plan_y = 0;
			ROBOT_CHASSI.plan_w = 0;
			return 1;
		}
	}
	return 0;
}
