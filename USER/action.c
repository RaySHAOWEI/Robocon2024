//
// Created by Ray on 2023/11/24.
//

#include "action.h"

ACTION_GL_POS ACTION_GL_POS_DATA = {0};

// ��ʵλ������
ROBOT_REAL_POS ROBOT_REAL_POS_DATA = {0};

/**
 * @brief �������������������ļ����ã���Ҫ�ⲿ����
 */
float pos_x=0;
float pos_y=0;
float zangle=0;
float xangle=0;
float yangle=0;
float w_z=0;

/**
 * @brief actionȫ���涨λ�Ĵ������ݰ�����
 */
void action_data_analyse(void)
{
    static uint8_t ch;
	static union {
		uint8_t data[24];
		float ActVal[6];
	} posture;

	static uint8_t count = 0;
	static uint8_t i = 0;

	HAL_UART_Receive_IT(&huart4, &ch, 1);		//�жϷ�����

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

//����actionȫ����λ��ֵ
void Update_Action_gl_position(float value[6])
{
	//������һ�ε�ֵ
	ACTION_GL_POS_DATA.LAST_POS_X = ACTION_GL_POS_DATA.POS_X;
	ACTION_GL_POS_DATA.LAST_POS_Y = ACTION_GL_POS_DATA.POS_Y;

	//��¼�˴ε�ֵ
	ACTION_GL_POS_DATA.ANGLE_Z = value[0]; // ���á��Ƕ�
	ACTION_GL_POS_DATA.ANGLE_X = value[1];
	ACTION_GL_POS_DATA.ANGLE_Y = value[2];
	ACTION_GL_POS_DATA.POS_X = value[3]; // ���á�x��
	ACTION_GL_POS_DATA.POS_Y = value[4]; // ���á�y��
	ACTION_GL_POS_DATA.W_Z = value[5];

	// �������
	ACTION_GL_POS_DATA.DELTA_POS_X = ACTION_GL_POS_DATA.POS_X - ACTION_GL_POS_DATA.LAST_POS_X;
	ACTION_GL_POS_DATA.DELTA_POS_Y = ACTION_GL_POS_DATA.POS_Y - ACTION_GL_POS_DATA.LAST_POS_Y;

	//ƫ����ֱ�Ӹ�ֵ
	ROBOT_REAL_POS_DATA.world_w = ACTION_GL_POS_DATA.ANGLE_Z;
	ROBOT_CHASSI.world_w = ROBOT_REAL_POS_DATA.world_w;

	//�۵ó�������ʵλ��
	ACTION_GL_POS_DATA.REAL_X += (ACTION_GL_POS_DATA.DELTA_POS_X);
	ACTION_GL_POS_DATA.REAL_Y += (ACTION_GL_POS_DATA.DELTA_POS_Y);//�����Ҹ��ݳ����˵���
	
	//�任����������
	ROBOT_REAL_POS_DATA.world_x = ACTION_GL_POS_DATA.REAL_X + INSTALL_ERROR_X * sin(ROBOT_CHASSI.world_w * PI / 180);
	ROBOT_CHASSI.world_x = ROBOT_REAL_POS_DATA.world_x;
	ROBOT_REAL_POS_DATA.world_y = ACTION_GL_POS_DATA.REAL_Y + INSTALL_ERROR_Y * cos(ROBOT_CHASSI.world_w * PI / 180);
	ROBOT_CHASSI.world_y = ROBOT_REAL_POS_DATA.world_y;
	
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == UART4)
	{
		action_data_analyse();
	}
}
