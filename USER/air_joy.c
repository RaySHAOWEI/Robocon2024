//
// Created by Ray on 2023/11/24.
//

#include "air_joy.h"

/*******************************************************************************************************************
2023/10/11
��ģң��������ȡPPW���塣
ʹ�ã��۲�PPM_Databuf[10]��ߴ��������ֵ�����ô��ڽ��й۲�
********************************************************************************************************************/


uint32_t TIME_ISR_CNT=0,LAST_TIME_ISR_CNT=0;
uint16_t Microsecond_Cnt=0;
uint16_t Time_Sys[4]={0};

static uint16_t PPM_buf[10]={0};
uint16_t PPM_Databuf[10]={0};
uint8_t ppm_update_flag=0;
uint32_t now_ppm_time_send=0;


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    static uint32_t last_ppm_time=0, now_ppm_time=0;
    static uint8_t ppm_ready=0,ppm_sample_cnt=0;
	static uint16_t ppm_time_delta=0;   //�õ����������½��ص�ʱ��

    if(GPIO_Pin == GPIO_PIN_7)		//ʹ���ж�����ΪPF7
    {
        //ϵͳ����ʱ���ȡ����λus
		last_ppm_time=now_ppm_time;//��ȡ��һ�εĵ�ǰʱ����Ϊ�ϴ�ʱ��
		now_ppm_time_send=now_ppm_time=10000 * TIME_ISR_CNT + TIM2->CNT;//us
		ppm_time_delta=now_ppm_time-last_ppm_time;//����õ�һ������ʱ��
    }

    //PPM������ʼ
		if(ppm_ready==1)	//�ж�֡����ʱ����ʼ�����µ�һ��PPM
		{
			if(ppm_time_delta >= 2200)//֡������ƽ����2ms=2000us�����ڲ����ϰ汾ң������//���ջ����PPM�źŲ���׼�������ֽ����쳣ʱ�����Ը�С��ֵ�������������һ����ʹ����ط��ϰ汾ң����
			{
				//memcpy(PPM_Databuf,PPM_buf,ppm_sample_cnt*sizeof(uint16));
				ppm_ready = 1;
				ppm_sample_cnt=0;//��Ӧ��ͨ��ֵ
				ppm_update_flag=1;
			} 
			else if(ppm_time_delta>=950&&ppm_time_delta<=2050)//����PWM������1000-2000us�������趨900-2100��Ӧ����Ϊ�������ݴ�
			{         
				PPM_buf[ppm_sample_cnt++]=ppm_time_delta;//��Ӧͨ��д�뻺���� 
				if(ppm_sample_cnt>=8)//���ν�������0-7��ʾ8��ͨ���������������ʾ10��ͨ���������ֵӦ��Ϊ0-9�������޸�
				{
					memcpy(PPM_Databuf,PPM_buf,ppm_sample_cnt*sizeof(uint16_t));//���Ƶ�PPM_Databuf��
					//ppm_ready=0;
					ppm_sample_cnt=0;
				}
			}
			else  
                ppm_ready=0;
		}
		else if(ppm_time_delta>=2200)//֡������ƽ����2ms=2000us
		{
			ppm_ready=1;
			ppm_sample_cnt=0;
			ppm_update_flag=0;
		}
}
