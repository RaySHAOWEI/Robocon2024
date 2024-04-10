//
// Created by Ray on 2023/11/24.
//

#include "usr_can.h"

void can_filter_init(void)//�����˲���������������
{
    CAN_FilterTypeDef can_filter_st;

    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;

    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);//��ʼ���˲���
    HAL_CAN_Start(&hcan1);//����can
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);//�˲����ж�

    can_filter_st.SlaveStartFilterBank = 14;
    can_filter_st.FilterBank = 14;

    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);//��ʼ���˲���
    HAL_CAN_Start(&hcan2);//����can
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);//�˲����ж�
}


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef	RxHeader;      //����
    uint8_t	RxData[8];  //���ݽ������飬can������ֻ֡��8֡
    uint8_t	RxData2[8];

    if(hcan == &hcan1)
    {
        HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, RxData);
        get_motor_measure(&RxHeader, RxData, &hcan1);  // �󽮵�����ݴ���
        for(int m=0;m<7;m++)
        {
            RM_MOTOR_Angle_Integral(&can1motorRealInfo[m]);//�ǶȻ���
        }

    }
    if(hcan == &hcan2)
    {
        HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &RxHeader, RxData2);
        get_motor_measure(&RxHeader, RxData2, &hcan2);  // �󽮵�����ݴ���
        for(int m=0;m<7;m++)
        {
            RM_MOTOR_Angle_Integral(&can2motorRealInfo[m]);//�ǶȻ���
        }
    }
}
