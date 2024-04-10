//
// Created by 19115 on 2024/3/13.
//

#include "Command.h"

// pid_command pidsend = {0};
ctrl_command ctrlsend = {0};
feedback_command feedback = {0};

// void pid_sent_test(uint8_t motor_id, uint8_t mode, float kp, float ki, float kd)
// {
// 	pidsend.motor_id=motor_id;
// 	pidsend.mode=mode;
// 	pidsend.kp.kp=kp;
// 	pidsend.ki.ki=ki;
// 	pidsend.kd.kd=kd;
// 	uint8_t cmd_arr[14];
// 	memcpy(cmd_arr, &pidsend, 2);
// 	memcpy(cmd_arr+2, pidsend.kp.kp_array, 4);
// 	memcpy(cmd_arr+6, pidsend.ki.ki_array, 4);
// 	memcpy(cmd_arr+10, pidsend.kd.kd_array, 4);
// 	Send_command(0xF1, cmd_arr, 14);
// }

void ctrl_sent_test(uint8_t command)
{
	ctrlsend.command_cnt++;
	ctrlsend.command = command;
	uint8_t cmd_arr[2];
	memcpy(cmd_arr, &ctrlsend, 2);
	Send_command(0xF2, cmd_arr, 2);
}

// void Feedback_sends(void)
// {
// 	uint8_t cmd_arr[8];
// 	memcpy(cmd_arr, &feedback, 8);
// 	Send_command(0xF3, cmd_arr, 8);
// }

// void PID_process(uint8_t *data)
// {
// 	if(Usart1.Checked==1)
// 	{
// 		if(Usart1.ID==0xF1)
// 		{
// 			pidreceive.motor_id=data[0];
// 			pidreceive.mode=data[1];
// 			memcpy(pidreceive.kp.kp_array, data+2, 4);
// 			memcpy(pidreceive.ki.ki_array, data+6, 4);
// 			memcpy(pidreceive.kd.kd_array, data+10, 4);
// 		}
// 	}
// }

// void ctrl_process(uint8_t *data)
// {
// 	if(Usart1.Checked==1)
// 	{
// 		if(Usart1.ID==0xF2)
// 		{
// 			ctrlreceive.command_cnt=data[0];
// 			ctrlreceive.command=data[1];
// 		}
// 		seed_state = ctrlreceive.command;
// 	}
// }

void feedback_process(uint8_t *data)
{
	if(Usart1.Checked==1)
	{
		if(Usart1.ID==0xF3)
		{
			feedback.command_cnt=data[0];
			feedback.receive_cnt=data[1];
			feedback.lost_cnt=data[2];
			feedback.state=data[3];
			memcpy(feedback.motor_state, data+4, 4);
		}
	}
}
