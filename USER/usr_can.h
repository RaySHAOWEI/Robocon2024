//
// Created by Ray on 2023/11/24.
//

#ifndef INC_2024RC_B_R1_USR_CAN_H
#define INC_2024RC_B_R1_USR_CAN_H

#include "main.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

void can_filter_init(void);

#endif //INC_2024RC_B_R1_USR_CAN_H
