//
// Created by Ray on 2023/11/24.
//

#include "air_joy.h"

/*******************************************************************************************************************
2023/10/11
航模遥控器：读取PPW脉冲。
使用：观察PPM_Databuf[10]里边储存的脉宽值，利用串口进行观察
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
	static uint16_t ppm_time_delta=0;   //得到上升沿与下降沿的时间

    if(GPIO_Pin == GPIO_PIN_7)		//使用中断引脚为PF7
    {
        //系统运行时间获取，单位us
		last_ppm_time=now_ppm_time;//获取上一次的当前时间作为上次时间
		now_ppm_time_send=now_ppm_time=10000 * TIME_ISR_CNT + TIM2->CNT;//us
		ppm_time_delta=now_ppm_time-last_ppm_time;//相减得到一个周期时间
    }

    //PPM解析开始
		if(ppm_ready==1)	//判断帧结束时，开始解析新的一轮PPM
		{
			if(ppm_time_delta >= 2200)//帧结束电平至少2ms=2000us，由于部分老版本遥控器、//接收机输出PPM信号不标准，当出现解析异常时，尝试改小此值，该情况仅出现一例：使用天地飞老版本遥控器
			{
				//memcpy(PPM_Databuf,PPM_buf,ppm_sample_cnt*sizeof(uint16));
				ppm_ready = 1;
				ppm_sample_cnt=0;//对应的通道值
				ppm_update_flag=1;
			} 
			else if(ppm_time_delta>=950&&ppm_time_delta<=2050)//单个PWM脉宽在1000-2000us，这里设定900-2100，应该是为了提升容错
			{         
				PPM_buf[ppm_sample_cnt++]=ppm_time_delta;//对应通道写入缓冲区 
				if(ppm_sample_cnt>=8)//单次解析结束0-7表示8个通道。我这里可以显示10个通道，故这个值应该为0-9！！待修改
				{
					memcpy(PPM_Databuf,PPM_buf,ppm_sample_cnt*sizeof(uint16_t));//复制到PPM_Databuf中
					//ppm_ready=0;
					ppm_sample_cnt=0;
				}
			}
			else  
                ppm_ready=0;
		}
		else if(ppm_time_delta>=2200)//帧结束电平至少2ms=2000us
		{
			ppm_ready=1;
			ppm_sample_cnt=0;
			ppm_update_flag=0;
		}
}
