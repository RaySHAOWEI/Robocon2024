//
// Created by Ray on 2024/1/21.
//

#include "usr_uart.h"

int receflag1 = 0;
int receflag2 = 0;

HAL_StatusTypeDef send1flag;
HAL_StatusTypeDef send2flag;

Usart_struct Usart1 = {0};
Usart_struct Usart2 = {0};
Usart_struct Usart3 = {0};
Usart_struct Usart4 = {0};
Usart_struct Usart5 = {0};

// 发送缓存数组
uint8_t send_Data[Max_DATA_Len] = {0};

// 接收缓存数组
uint8_t receive_Buffer[Max_BUFF_Len] = {0};

// 电机数据可视化
void sent_data(int16_t A, int16_t B, int16_t C, int16_t D)
{
	int i;
	uint8_t sumcheck = 0;
	uint8_t addcheck = 0;
	uint8_t _cnt = 0;
	static uint8_t last_data[Max_DATA_Len];
	send_Data[_cnt++] = 0xAA;	  // 帧头
	send_Data[_cnt++] = 0xFF;	  // 目标地址
	send_Data[_cnt++] = 0XF1;	  // 功能码
	send_Data[_cnt++] = 0x08;	  // 数据长度
	send_Data[_cnt++] = BYTE0(A); // 数据内容,小段模式，低位在前
	send_Data[_cnt++] = BYTE1(A); // 需要将字节进行拆分，调用上面的宏定义即可。
	send_Data[_cnt++] = BYTE0(B);
	send_Data[_cnt++] = BYTE1(B);
	send_Data[_cnt++] = BYTE0(C);
	send_Data[_cnt++] = BYTE1(C);
	send_Data[_cnt++] = BYTE0(D);
	send_Data[_cnt++] = BYTE1(D);
	for (i = 0; i < send_Data[3] + 4; i++)
	{
		sumcheck += send_Data[i];
		addcheck += sumcheck;
	}
	send_Data[_cnt++] = sumcheck;
	send_Data[_cnt++] = addcheck;

	if (memcmp(send_Data, last_data, _cnt) != 0)
	{
		HAL_UART_Transmit_DMA(&huart6, send_Data, _cnt);
		memcpy(last_data, send_Data, _cnt);
	}
	else
	{
	}
}
/*使用方法：
*	int16_t A,B,C,D;
	sent_data(A,B,C,D);
	可以同时发送4个int16_t类型的数据
	例如：
	sent_data(can1motorRealInfo[0].RPM,can1motorRealInfo[1].RPM,can1motorRealInfo[2].RPM,can1motorRealInfo[3].RPM);
*/

// 发送不定长数据 cmd:功能码（f1pid f2控制） data:数据指针 len:数据长度
void Send_command(uint8_t cmd, uint8_t *data, uint16_t len)
{
	static uint8_t Data[Max_DATA_Len];
	uint8_t i;
	uint8_t sumcheck = 0;
	uint8_t addcheck = 0;
	uint8_t _cnt = 0;
	Data[_cnt++] = 0xAA; // 帧头
	Data[_cnt++] = 0xFF; // 目标地址
	Data[_cnt++] = cmd;	 // 功能码
	Data[_cnt++] = len;	 // 数据长度
	for (i = 0; i < len; i++)
	{
		Data[_cnt++] = data[i];
	}
	for (i = 0; i < Data[3] + 4; i++)
	{
		sumcheck += Data[i];
		addcheck += sumcheck;
	}
	Data[_cnt++] = sumcheck;
	Data[_cnt++] = addcheck;
	send1flag = HAL_UART_Transmit_DMA(&huart1, Data, _cnt);
	send2flag = HAL_UART_Transmit_DMA(&huart2, Data, _cnt);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if (huart == &huart1)
	{
		receflag1++;
		processData(&Usart1, Size);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart1, Usart1.ProcessBuff, Max_BUFF_Len);
		__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT); // 关闭DMA半传输中断
	}

	if (huart == &huart2)
	{
		receflag2++;
		processData(&Usart2, Size);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart2, Usart2.ProcessBuff, Max_BUFF_Len);
		__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT); // 关闭DMA半传输中断
	}

//	if (huart == &huart3)
//	{
//		keyboard_feedbacks.receive_cnt++;
//		processData(&Usart3, Size);
//		HAL_UARTEx_ReceiveToIdle_DMA(&huart3, Usart3.ProcessBuff, Max_BUFF_Len);
//		__HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT); // 关闭DMA半传输中断
//	}

	//	if (huart == &huart4) // action
	//	{
	//		processData(&Usart4, Size);
	//		HAL_UARTEx_ReceiveToIdle_DMA(&huart4, Usart4.ProcessBuff, Max_BUFF_Len);
	//		__HAL_DMA_DISABLE_IT(&hdma_uart4_rx, DMA_IT_HT); // 关闭DMA半传输中断
	//	}

	//	if (huart == &huart5) // dt35
	//	{
	//		processData(&Usart5, Size);
	//		HAL_UARTEx_ReceiveToIdle_DMA(&huart5, Usart5.ProcessBuff, Max_BUFF_Len);
	//		__HAL_DMA_DISABLE_IT(&hdma_uart5_rx, DMA_IT_HT); // 关闭DMA半传输中断
	//	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	
	if (huart == &huart2)
	{
		HAL_UART_AbortReceive(&huart2);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart2, Usart2.ProcessBuff, Max_BUFF_Len);
		__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT); // 关闭DMA半传输中断
	}
	else if (huart == &huart1)
	{
		HAL_UART_AbortReceive(&huart1);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart1, Usart1.ProcessBuff, Max_BUFF_Len);
		__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT); // 关闭DMA半传输中断
	}
//	else if (huart == &huart3)
//	{
//		HAL_UART_AbortReceive(&huart3);
//		HAL_UARTEx_ReceiveToIdle_DMA(&huart3, Usart3.ProcessBuff, Max_BUFF_Len);
//		__HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT); // 关闭DMA半传输中断
//	}
}

void Usart1_Init(void)
{
	Usart1.HEAD = 0xAA;
	Usart1.ADDR = 0xFF;
	Usart1.checkMode = Command_data;
}

void Usart2_Init(void)
{
	Usart2.HEAD = 0xAA;
	Usart2.ADDR = 0xFF;
	Usart2.checkMode = Command_data;
}

void Usart3_Init(void)
{
	Usart3.HEAD = 0x55;
	Usart3.ADDR = 0xAA;
	Usart3.checkMode = Keyboard_data;
	Usart3.END[0] = 0xAA;
	Usart3.END[1] = 0x55;
}

void Uart4_Init(void)
{
	Usart4.HEAD = 0x0D;
	Usart4.ADDR = 0x0A;
	Usart4.checkMode = Action_data;
	Usart4.END[0] = 0x0A;
	Usart4.END[1] = 0x0D;
}

void Uart5_Init(void)
{
	Usart5.HEAD = 0x55;
	Usart5.ADDR = 0xAA;
	Usart5.checkMode = DT35_data;
	Usart5.END[0] = 0x0D;
	Usart5.END[1] = 0x0A;
}

void Usr_UART_Init(void)
{
	Usart1_Init();
	Usart2_Init();
	// Usart3_Init();
	// Uart4_Init();
	// Uart5_Init();
}

/*
	处理数据
	参数：data：数据指针
		  len：数据长度
*/
void processData(Usart_struct *data, uint16_t len)
{
	// 数据处理
	uint8_t i;

	data->Size = len;
	data->Checked = 0;
	data->new_data = 1;

	if (data->Size < 7)
	{
		data->new_data = 0;
		memset(data->ProcessBuff, 0, Max_BUFF_Len);
		memset(data->DATA, 0, Max_DATA_Len);
		return;
	} // 接收到的数据太少，直接跳出

	for (i = 0; i < data->Size; i++)
	{
		if (data->ReceiveBuff[i] == data->HEAD && data->ReceiveBuff[i + 1] == data->ADDR)
		{
			if (memcmp(data->ProcessBuff, &data->ReceiveBuff[i], len - i) != 0)
			{
				memcpy(data->ProcessBuff, &data->ReceiveBuff[i], len - i);
			}
			else
			{
				data->new_data = 0;
			}
			break;
		}
	} // 复制到处理缓存

	if (data->checkMode == Command_data)
	{
		command_analysis(data);
	}
	else if (data->checkMode == Action_data)
	{
		action_analysis(data);
	}
	else if (data->checkMode == DT35_data)
	{
		DT35_analysis(data);
	}
	else if (data->checkMode == Keyboard_data)
	{
		Keyboard_analysis(data);
	}
	else
	{
		data->new_data = 0;
		memset(data->ProcessBuff, 0, Max_BUFF_Len);
		memset(data->DATA, 0, Max_DATA_Len);
	}
}

void action_analysis(Usart_struct *data)
{
	static union
	{
		uint8_t data[24];
		float ActVal[6];
	} posture;

	if (data->ProcessBuff[0] == data->HEAD && data->ProcessBuff[1] == data->ADDR)
	{
		memcpy(data->DATA, &data->ProcessBuff[2], 24);
		memcpy(posture.data, data->DATA, 24);
	}
	else
	{
		return;
	}

	if (data->ProcessBuff[26] == data->END[0] && data->ProcessBuff[27] == data->END[1])
	{
		Update_Action_gl_position(posture.ActVal);
	}
	else
	{
		return;
	}
}

/**
 * @brief DT35数据解析 0x55 0xAA lenth [16] crc8 0x0D 0x0A
 *
 * @param data
 */
void DT35_analysis(Usart_struct *data)
{
	union receiveData1
	{
		int d;
		uint8_t data[4];
	} x_position, y_position, k_position, b_position;

	if (data->ProcessBuff[0] == data->HEAD && data->ProcessBuff[1] == data->ADDR)
	{
		data->Len = data->ProcessBuff[2];
		memcpy(data->DATA, &data->ProcessBuff[3], 16);
		memcpy(x_position.data, data->DATA, 4);
		memcpy(y_position.data, data->DATA + 4, 4);
		memcpy(k_position.data, data->DATA + 8, 4);
		memcpy(b_position.data, data->DATA + 12, 4);
	}
	else
	{
		return;
	}
	// 跳过校验
	// 接受数据尾
	if (data->ProcessBuff[20] == data->END[0] && data->ProcessBuff[21] == data->END[1])
	{
		Update_DT35(x_position.d, y_position.d, k_position.d, b_position.d);
	}
	else
	{
		return;
	}
}

void command_analysis(Usart_struct *data)
{
	uint8_t i;
	uint8_t sumcheck = 0;
	uint8_t addcheck = 0;
	if (data->ProcessBuff[0] == data->HEAD && data->ProcessBuff[1] == data->ADDR)
	{
		data->ID = data->ProcessBuff[2];
		data->Len = data->ProcessBuff[3];
		data->SC = data->ProcessBuff[data->Len + 4];
		data->AC = data->ProcessBuff[data->Len + 5];
	}

	if (data->Len > data->Size - 6)
	{
		data->new_data = 0;
		memset(data->ProcessBuff, 0, Max_BUFF_Len);
		memset(data->DATA, 0, Max_DATA_Len);
		return;
	}

	for (i = 0; i < data->Len + 4; i++)
	{
		sumcheck += data->ProcessBuff[i];
		addcheck += sumcheck;
	}

	if (addcheck != 0 && data->SC == sumcheck && data->AC == addcheck)
	{
		data->Checked = 1;
		feedback1.command_cnt++;
		feedback2.command_cnt++;
		if (memcmp(data->DATA, &data->ProcessBuff[4], data->Len) != 0)
		{
			memcpy(data->DATA, &data->ProcessBuff[4], data->Len);
		}
		else
		{
			data->new_data = 0;
		}

		if (data->ID == 0xF1)
		{
			// PID_process(data->DATA);
		}
		else if (data->ID == 0xF2)
		{
			// ctrl_process(data->DATA);
		}
		else if (data->ID == 0xF3)
		{
			feedback_process(data->DATA);
		}
	}
	else
	{
		data->new_data = 0;
		memset(data->ProcessBuff, 0, Max_BUFF_Len);
		memset(data->DATA, 0, Max_DATA_Len);
	}
}

void Keyboard_analysis(Usart_struct *data)
{
	if (data->ProcessBuff[0] == data->HEAD && data->ProcessBuff[1] == data->ADDR)
	{
		memcpy(data->DATA, &data->ProcessBuff[2], 4);
	}
	else
	{
		return;
	}
	// 对数据进行尾部检查 ， 如果匹配，说明数据包完整。
	if (data->ProcessBuff[6] == data->END[0] && data->ProcessBuff[7] == data->END[1])
	{
		keyboard_feedbacks.command_cnt++;
		keyboard_process(data->DATA);
	}
	else
	{
		return;
	}
}
