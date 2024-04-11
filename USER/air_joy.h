//
// Created by Ray on 2023/11/24.
//

#ifndef INC_2024RC_B_R1_AIR_JOY_H
#define INC_2024RC_B_R1_AIR_JOY_H

#include "main.h"

// #define LEFT_BUTTON         PPM_Databuf[8]
#define YaoGan_LEFT_X PPM_Databuf[0]
#define YaoGan_LEFT_Y PPM_Databuf[1]
#define YaoGan_RIGHT_Y PPM_Databuf[2]
#define YaoGan_RIGHT_X PPM_Databuf[3]
#define SWA PPM_Databuf[4]
#define SWB PPM_Databuf[5]
#define SWC PPM_Databuf[6]
#define SWD PPM_Databuf[7]

#define Hour 3
#define Minute 2
#define Second 1
#define MicroSecond 0

extern uint16_t PPM_Databuf[10];
extern uint32_t TIME_ISR_CNT, LAST_TIME_ISR_CNT;
extern uint16_t Microsecond_Cnt;
extern uint16_t Time_Sys[4];

#endif // INC_2024RC_B_R1_AIR_JOY_H
