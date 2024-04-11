//
// Created by Ray on 2024/1/21.
//

#ifndef UPPER_TEST_USR_UART_H
#define UPPER_TEST_USR_UART_H

#include "usart.h"
#include <string.h>

extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_uart4_rx;
extern DMA_HandleTypeDef hdma_uart4_tx;
extern DMA_HandleTypeDef hdma_uart5_rx;
extern DMA_HandleTypeDef hdma_uart5_tx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;

#define Max_BUFF_Len 100
#define Max_DATA_Len 50

// cup为小端模式存储，也就是在存储的时候，低位被存在0字节，高位在1字节
#define BYTE0(dwTemp) (*(char *)(&dwTemp))       // 取出int型变量的低字节
#define BYTE1(dwTemp) (*((char *)(&dwTemp) + 1)) //	取存储在此变量下一内存字节的内容，高字节
#define BYTE2(dwTemp) (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp) (*((char *)(&dwTemp) + 3))

typedef enum
{
    Action_data,  // action数据
    DT35_data,    // DT35数据
    Command_data, // 命令数据
    debug_data    // 调试数据(电机数据可视化)
} DataMode;
// 串口数据形式

// 串口管理结构体
typedef struct
{
    uint16_t Size;                     // 接收到的数据长度
    uint8_t ReceiveBuff[Max_BUFF_Len]; // 接收缓存
    uint8_t ProcessBuff[Max_BUFF_Len]; // 处理缓存

    uint8_t HEAD; // 帧头
    uint8_t ADDR; // 地址

    DataMode checkMode;         // 校验方式
    uint8_t ID;                 // 功能码
    uint8_t Len;                // 数据长度
    uint8_t DATA[Max_DATA_Len]; // 数据内容
    uint8_t SC;                 // 和校验
    uint8_t AC;                 // 附加校验
    int new_data;               // 新 且 有效 数据标志
    int Checked;                // 校验结果

    uint8_t END[2]; // 帧尾1
} Usart_struct;

extern Usart_struct Usart1;
extern Usart_struct Usart2;
extern Usart_struct Usart4;
extern Usart_struct Usart5;

extern uint8_t send_Buffer[Max_BUFF_Len];
extern uint8_t receive_Buffer[Max_BUFF_Len];

void sent_data(int16_t A, int16_t B, int16_t C, int16_t D);
void Send_command(uint8_t cmd, uint8_t *data, uint16_t len);

void Usart1_Init(void);
void Usart2_Init(void);
void Uart4_Init(void);
void Uart5_Init(void);
void processData(Usart_struct *data, uint16_t len);

void action_analysis(Usart_struct *data);
void DT35_analysis(Usart_struct *data);
void command_analysis(Usart_struct *data);

#endif // UPPER_TEST_USR_UART_H
