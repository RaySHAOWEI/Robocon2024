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

// cupΪС��ģʽ�洢��Ҳ�����ڴ洢��ʱ�򣬵�λ������0�ֽڣ���λ��1�ֽ�
#define BYTE0(dwTemp) (*(char *)(&dwTemp))       // ȡ��int�ͱ����ĵ��ֽ�
#define BYTE1(dwTemp) (*((char *)(&dwTemp) + 1)) //	ȡ�洢�ڴ˱�����һ�ڴ��ֽڵ����ݣ����ֽ�
#define BYTE2(dwTemp) (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp) (*((char *)(&dwTemp) + 3))

typedef enum
{
    Action_data,  // action����
    DT35_data,    // DT35����
    Command_data, // ��������
    debug_data    // ��������(������ݿ��ӻ�)
} DataMode;
// ����������ʽ

// ���ڹ���ṹ��
typedef struct
{
    uint16_t Size;                     // ���յ������ݳ���
    uint8_t ReceiveBuff[Max_BUFF_Len]; // ���ջ���
    uint8_t ProcessBuff[Max_BUFF_Len]; // ������

    uint8_t HEAD; // ֡ͷ
    uint8_t ADDR; // ��ַ

    DataMode checkMode;         // У�鷽ʽ
    uint8_t ID;                 // ������
    uint8_t Len;                // ���ݳ���
    uint8_t DATA[Max_DATA_Len]; // ��������
    uint8_t SC;                 // ��У��
    uint8_t AC;                 // ����У��
    int new_data;               // �� �� ��Ч ���ݱ�־
    int Checked;                // У����

    uint8_t END[2]; // ֡β1
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
