//
// Created by Ray on 2024/1/21.
//

#ifndef UPPER_TEST_USR_UART_H
#define UPPER_TEST_USR_UART_H

#include "usart.h"
#include <string.h>

//cupΪС��ģʽ�洢��Ҳ�����ڴ洢��ʱ�򣬵�λ������0�ֽڣ���λ��1�ֽ�
#define BYTE0(dwTemp)       (*(char *)(&dwTemp))	        //ȡ��int�ͱ����ĵ��ֽ�
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))	    //	ȡ�洢�ڴ˱�����һ�ڴ��ֽڵ����ݣ����ֽ�
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))

void sent_data(int16_t A,int16_t B,int16_t C,int16_t D);

#endif //UPPER_TEST_USR_UART_H
