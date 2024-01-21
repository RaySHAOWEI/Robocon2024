//
// Created by Ray on 2024/1/21.
//

#ifndef UPPER_TEST_USR_UART_H
#define UPPER_TEST_USR_UART_H

#include "usart.h"
#include <string.h>

//cup为小端模式存储，也就是在存储的时候，低位被存在0字节，高位在1字节
#define BYTE0(dwTemp)       (*(char *)(&dwTemp))	        //取出int型变量的低字节
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))	    //	取存储在此变量下一内存字节的内容，高字节
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))

void sent_data(int16_t A,int16_t B,int16_t C,int16_t D);

#endif //UPPER_TEST_USR_UART_H
