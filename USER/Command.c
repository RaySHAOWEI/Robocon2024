//
// Created by 19115 on 2024/3/13.
//

#include "Command.h"

// pid_command pidsend = {0};
ctrl_command ctrlsend = {0};
feedback_command feedback1 = {0}, feedback2 = {0};
Keyboard_command last_keyboard_commands = {0}, keyboard_commands = {0};
keyboard_feedback last_keyboard_feedbacks = {0}, keyboard_feedbacks = {0};

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
	static uint8_t cmd_arr[2];
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
	feedback1.command_cnt = Usart1.DATA[0];
	feedback1.receive_cnt = Usart1.DATA[1];
	feedback1.lost_cnt = Usart1.DATA[2];
	feedback1.state = Usart1.DATA[3];
	memcpy(feedback1.motor_state, Usart1.DATA + 4, 4);
	feedback2.command_cnt = Usart2.DATA[0];
	feedback2.receive_cnt = Usart2.DATA[1];
	feedback2.lost_cnt = Usart2.DATA[2];
	feedback2.state = Usart2.DATA[3];
	memcpy(feedback2.motor_state, Usart2.DATA + 4, 4);
}

void keyboard_process(uint8_t *data)
{
	memcpy(&keyboard_commands, data, 4);
	switch (keyboard_commands.key_state)
	{
	case 'f':
		caculation_ok = 1;
		update_action_seed = 1;
		seed = 1;
		move_state = MOVE_STATE_SEED;
		break;
	case 'e':
		update_action_seed = 1;
		seed = 2;
		caculation_ok = 1;
		move_state = MOVE_STATE_SEED;
		break;
	case 'd':
		update_action_seed = 1;
		seed = 3;
		caculation_ok = 1;
		move_state = MOVE_STATE_SEED;
		break;
	case 'c':
		update_action_put = 1;
		put = 1;
		move_state = MOVE_STATE_PUT;
		break;
	case 'b':
		update_action_put = 1;
		put = 3;
		move_state = MOVE_STATE_PUT;
		break;
	case 'a':
		update_action_put = 1;
		put = 5;
		move_state = MOVE_STATE_PUT;
		break;
	case 'A':
		direction = 1;
		update_action_ball = 1;
	    move_state = MOVE_STATE_ACTION_NEAR;
//		move_state = MOVE_STATE_LASER;
		break;
	case 'B':
		direction = 2;
		update_action_ball = 1;
	    move_state = MOVE_STATE_ACTION_NEAR;	
//		move_state = MOVE_STATE_LASER;
		break;
	case 'C':
		direction = 3;
		update_action_ball = 1;
	    move_state = MOVE_STATE_ACTION_NEAR;	
//		move_state = MOVE_STATE_LASER;
		break;
	case 'D':
		direction = 4;
		update_action_ball = 1;
	    move_state = MOVE_STATE_ACTION_NEAR;	
//		move_state = MOVE_STATE_LASER;
		break;
	case 'E':
		direction = 5;
		update_action_ball = 1;
	    move_state = MOVE_STATE_ACTION_NEAR;	
//		move_state = MOVE_STATE_LASER;
		break;
	case 'F':
		direction = 6;
		update_action_ball = 1;
	    move_state = MOVE_STATE_ACTION_NEAR;
//		move_state = MOVE_STATE_LASER;
		break;
	case 'G':
		direction = 7;
		update_action_ball = 1;
	    move_state = MOVE_STATE_ACTION_NEAR;
//		move_state = MOVE_STATE_LASER;
		break;
	case 'H':
		direction = 8;
		update_action_ball = 1;
	    move_state = MOVE_STATE_ACTION_NEAR;
//		move_state = MOVE_STATE_LASER;
		break;
	case 'I':
		direction = 9;
		update_action_ball = 1;
	    move_state = MOVE_STATE_ACTION_NEAR;
//		move_state = MOVE_STATE_LASER;
		break;
	case 'J':
		direction = 10;
		update_action_ball = 1;
	    move_state = MOVE_STATE_ACTION_NEAR;
//		move_state = MOVE_STATE_LASER;
		break;
	case 'K':
		direction = 11;
		update_action_ball = 1;
	    move_state = MOVE_STATE_ACTION_NEAR;
//		move_state = MOVE_STATE_LASER;
		break;
	case 'L':
		direction = 12;
		update_action_ball = 1;
	    move_state = MOVE_STATE_ACTION_NEAR;
//		move_state = MOVE_STATE_LASER;
		break;
	default:
		break;
	}

	switch (keyboard_commands.aim_state)
	{
	case 'O':
		gimbal_state = GIMBAL_AIM_1;
		break;
	case 'P':
		gimbal_state = GIMBAL_AIM_2;
		break;
	case 'Q':
		gimbal_state = GIMBAL_AIM_3;
		break;
	case 'R':
		
		//   move_state = MOVE_INIT_ACTION;
		break;
	}
}

void keyboard_feedback_send(void)
{
	static uint8_t cmd_arr[13];
	cmd_arr[0] = 0x55;
	cmd_arr[1] = 0xAA;
	memcpy(cmd_arr + 2, &keyboard_feedbacks, 9);
	cmd_arr[11] = 0xAA;
	cmd_arr[12] = 0x55;
	HAL_UART_Transmit_DMA(&huart3, cmd_arr, 13);
}

uint8_t move_state_switch(uint8_t state)
{
	uint8_t send_state;
	switch (state)
	{
	case MOVE_STATE_INIT:
		break;
	case MOVE_STATE_WAIT_SEED:
		send_state = 'B';
		break;
	case MOVE_STATE_WAIT_PUT:
		send_state = 'D';
		break;
	case MOVE_STATE_WAIT_CATCH:
		send_state = 'F';
		break;
	case MOVE_STATE_SEED:
		send_state = 'A';
		break;
	case MOVE_STATE_PUT:
		send_state = 'C';
		break;
	case MOVE_SEED_LASER:
		break;
	case MOVE_STATE_LASER:
		send_state = 'E';
		break;
	case MOVE_STATE_ACTION_NEAR:
		break;
	case CALIBRATION_BY_ACTION:
		send_state = 'E';
		break;
	case MOVE_BY_ACTION:
		send_state = 'E';
		break;
	default:
		break;
	}
	return send_state;
}
