//
// Created by Ray on 2023/11/24.
//

#include "action.h"

ACTION_GL_POS ACTION_GL_POS_DATA = {0};

/**
 * @brief actionȫ���涨λ�Ĵ������ݰ�����
 */
void action_data_analyse(void)
{
	static uint8_t ch;
	static union
	{
		uint8_t data[24];
		float ActVal[6];
	} posture;

	static uint8_t count = 0;
	static uint8_t i = 0;

	HAL_UART_Receive_IT(&huart4, &ch, 1); // �жϷ�����

	switch (count)
	{
	case 0:
	{
		if (ch == 0x0d)
			count++;
		else
			count = 0;
		break;
	}

	case 1:
	{
		if (ch == 0x0a)
		{
			i = 0;
			count++;
		}
		else if (ch == 0x0d)
			;
		else
			count = 0;
		break;
	}

	case 2:
	{
		posture.data[i] = ch;
		i++;
		if (i >= 24)
		{
			i = 0;
			count++;
		}
		break;
	}

	case 3:
	{
		if (ch == 0x0a)
			count++;
		else
			count = 0;
		break;
	}

	case 4:
	{
		if (ch == 0x0d)
		{
			Update_Action_gl_position(posture.ActVal);
		}
		count = 0;

		break;
	}

	default:
	{
		count = 0;
		break;
	}
	}
}

// ����actionȫ����λ��ֵ
void Update_Action_gl_position(float value[6])
{
	// ������һ�ε�ֵ
	ACTION_GL_POS_DATA.LAST_POS_X = ACTION_GL_POS_DATA.POS_X;
	ACTION_GL_POS_DATA.LAST_POS_Y = ACTION_GL_POS_DATA.POS_Y;

	// ��¼�˴ε�ֵ
	ACTION_GL_POS_DATA.ANGLE_Z = value[0]; // ���á��Ƕ�
	ACTION_GL_POS_DATA.ANGLE_X = value[1];
	ACTION_GL_POS_DATA.ANGLE_Y = value[2];
	ACTION_GL_POS_DATA.POS_X = value[3]; // ���á�x��
	ACTION_GL_POS_DATA.POS_Y = value[4]; // ���á�y��
	ACTION_GL_POS_DATA.W_Z = value[5];

	// �������
	ACTION_GL_POS_DATA.DELTA_POS_X = ACTION_GL_POS_DATA.POS_X - ACTION_GL_POS_DATA.LAST_POS_X;
	ACTION_GL_POS_DATA.DELTA_POS_Y = ACTION_GL_POS_DATA.POS_Y - ACTION_GL_POS_DATA.LAST_POS_Y;

	// ƫ����ֱ�Ӹ�ֵ
	ROBOT_CHASSI.world_w = ACTION_GL_POS_DATA.ANGLE_Z;

	// �۵ó�������ʵλ��
	ACTION_GL_POS_DATA.REAL_X += (ACTION_GL_POS_DATA.DELTA_POS_X);
	ACTION_GL_POS_DATA.REAL_Y += (ACTION_GL_POS_DATA.DELTA_POS_Y); // �����Ҹ��ݳ����˵���

	// �任����������
	ROBOT_CHASSI.world_x = ACTION_GL_POS_DATA.REAL_X - INSTALL_ERROR_X * cos(ROBOT_CHASSI.world_w * PI / 180) + INSTALL_ERROR_Y * sin(ROBOT_CHASSI.world_w * PI / 180) + INSTALL_ERROR_X;
	ROBOT_CHASSI.world_y = ACTION_GL_POS_DATA.REAL_Y - INSTALL_ERROR_X * sin(ROBOT_CHASSI.world_w * PI / 180) - INSTALL_ERROR_Y * cos(ROBOT_CHASSI.world_w * PI / 180) + INSTALL_ERROR_Y;

	//	// �任����������
	//	ROBOT_CHASSI.world_x = ACTION_GL_POS_DATA.REAL_X + INSTALL_ERROR_X * cos(ROBOT_CHASSI.world_w * PI / 180) - INSTALL_ERROR_Y * sin(ROBOT_CHASSI.world_w * PI / 180);
	//	ROBOT_CHASSI.world_y = ACTION_GL_POS_DATA.REAL_Y + INSTALL_ERROR_X * sin(ROBOT_CHASSI.world_w * PI / 180) + INSTALL_ERROR_Y * cos(ROBOT_CHASSI.world_w * PI / 180);

	//	// �任����������
	//	ROBOT_CHASSI.world_x = ACTION_GL_POS_DATA.REAL_X + INSTALL_ERROR_X * cos(ROBOT_CHASSI.world_w * PI / 180) - INSTALL_ERROR_Y * sin(ROBOT_CHASSI.world_w * PI / 180);
	//	ROBOT_CHASSI.world_y = ACTION_GL_POS_DATA.REAL_Y + INSTALL_ERROR_Y * sin(ROBOT_CHASSI.world_w * PI / 180) + INSTALL_ERROR_Y * cos(ROBOT_CHASSI.world_w * PI / 180);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	// ����
	if (huart->Instance == UART5)
	{
		VisionReceiveData_1(&DT35.y, &DT35.x, &DT35.k, &DT35.b);
		//	 //HAL_UART_Receive_IT(&huart1, &USART1_Receiver , 1); // ������
	}
	if (huart->Instance == UART4)
	{
		action_data_analyse();
	}
	// usart1
}

// ���ڶ������� ʹ��action
int action_calibration()
{
	//	if(YawAdjust(action_yaw))

	if (Laser2_calibration(DT35.y, DT35.x))
	{

		tr5++;
		if (tr5 > 300)
		{
			ROBOT_CHASSI.plan_w = 0;
			tt = 0;
			yaw_disable = 1;
			move_state = MOVE_BALL_BY_ACTION;
			//		move_state = MOVE_STATE_INIT;
			tr5 = 0;
			return 1;
		}
	}
	else
	{
		return 0;
	}
	return 0;
}

void Stract(char str1[],char str2[], int num)
{
	int i = 0, j = 0;
	
	while(str1[i] != '\0') i++;
	for(j = 0;j < num; j++)
		str1[i++] = str2[j];
}
//���ڸ���action
void Update_J(float New)
{
	int i = 0;
	char Update[8] = "ACTJ";
	
	static union
	{
		float J;
		char data[4];
	}New_set;
	
	New_set.J = New;
	Stract(Update,New_set.data,4);
	for(i = 0;i<8;i++)
	{
		HAL_UART_Transmit(&huart4, &Update[i], 1, 1);
	}
}
void Update_X(float New)
{
	int i = 0;
	char Update[8] = "ACTX";
	
	static union
	{
		float J;
		char data[4];
	}New_set;
	
	New_set.J = New;
	Stract(Update,New_set.data,4);
	for(i = 0;i<8;i++)
	{
		HAL_UART_Transmit(&huart4, &Update[i], 1, 1);
	}
}
void Update_Y(float New)
{
	int i = 0;
	char Update[8] = "ACTY";
	
	static union
	{
		float J;
		char data[4];
	}New_set;
	
	New_set.J = New;
	Stract(Update,New_set.data,4);
	for(i = 0;i<8;i++)
	{
		HAL_UART_Transmit(&huart4, &Update[i], 1, 1);
	}
}

void Update_ACTION()
{
		int i = 0;
    char data[8] = "ACT0";
    for (i = 0; i < 4; i++)
    {
			HAL_UART_Transmit(&huart4, &data[i], 1, 1);
    }
}

// ���ڶ������� ʹ��action
int Action_calibration()
{
	//	if(YawAdjust(action_yaw))
	if (Laser2_calibration(DT35.y, DT35.x))
	{
		ROBOT_CHASSI.plan_w = 0;
		move_state = MOVE_STATE_LASER;
		return 1;
	}
	else
	{
		return 0;
	}
}
