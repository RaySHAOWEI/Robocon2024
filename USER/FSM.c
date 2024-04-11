//
// Created by Ray on 2023/11/24.
//

#include "FSM.h"

ROBOT_STATE_ITEMS robot_state = ROBOT_STATE_INIT;
CHASSIS_STATE_ITEMS chassis_state = CHASSIS_DISABLE;
SEED_STATE_ITEMS last_seed_state, seed_state = SEED_STATE_DISABLE;
SHOOT_STATE_ITEMS shoot_state = SHOOT_STATE_DISABLE;

// AREA SEED_AREA = {-0.1f, -0.1f, 50.0f, 50.0f};//瞎写的，期末没用
// AREA SHOOT_AREA = {-0.1f, 50.0f, 50.0f, 100.0f};//瞎写的，期末没用

void robot_fsm(void)
{
    switch (robot_state)
    {
    case ROBOT_STATE_INIT:
        chassis_state = CHASSIS_DISABLE;
        if (SWA != 0 && SWB != 0 && SWC != 0 && SWD != 0)
        { // 确认航模初始化成功
            SWA_judge();
            SWB_judge();
            SWC_judge();
            SWD_judge();
        }
        break;

    case ROBOT_STATE_AUTO_CTRL:
        if (SWA == 0)
        {
            robot_state = ROBOT_STATE_INIT;
        }
        else
        {
            SWA_judge();
            SWB_judge();
            SWC_judge();
            SWD_judge();
        }
        break;

    case ROBOT_STATE_SEED_CTRL:
        if (SWA == 0)
        {
            robot_state = ROBOT_STATE_INIT;
        }
        else
        {
            SWA_judge();
            SWB_judge();
            SWC_judge();
            SWD_judge();
        }
        break;

    case ROBOT_STATE_SHOOT_CTRL:
        if (SWA == 0)
        {
            robot_state = ROBOT_STATE_INIT;
        }
        else
        {
            SWA_judge();
            SWB_judge();
            SWC_judge();
            SWD_judge();
        }
        break;
    }
}

void SWA_judge(void)
{
    if (SWA < 1500)
    {
        chassis_state = CHASSIS_STATE_HIGH_SPEED;
    }
    else if (SWA > 1500)
    {
        chassis_state = CHASSIS_STATE_LOW_SPEED;
    }
}

void auto_switch(void)
{
    /*
    自动模式：swb拨到第一档：根据当前action定位决定模式
                swd拨到第一档，半自动，需要操作机器人位置和拨动swc决定具体状态
                swd拨到第二档，全自动，不需要操作机器人
    */
}

void SWB_judge(void)
{
    if (SWB < 1200)
    {
        auto_switch(); // 根据action全场定位判断当前是哪个模式
        // robot_state = ROBOT_STATE_AUTO_CTRL;
    }
    else if (1300 < SWB && SWB < 1700)
    {
        robot_state = ROBOT_STATE_SEED_CTRL;
        shoot_state = SHOOT_STATE_DISABLE;
    }
    else if (SWB > 1800)
    {
        robot_state = ROBOT_STATE_SHOOT_CTRL;
        seed_state = SEED_STATE_DISABLE;
    }
}

void SWC_judge(void)
{
    if (SWC < 1200)
    {
        if (robot_state == ROBOT_STATE_SEED_CTRL)
        {
            seed_state = SEED_STATE_INIT;
        }
        else if (robot_state == ROBOT_STATE_SHOOT_CTRL)
        {
            shoot_state = SHOOT_STATE_INIT;
        }
    }
    else if (1300 < SWC && SWC < 1700)
    {
        if (robot_state == ROBOT_STATE_SEED_CTRL)
        {
            seed_state = SEED_STATE_PEEK_DOWN;
        }
        else if (robot_state == ROBOT_STATE_SHOOT_CTRL)
        {
            shoot_state = SHOOT_STATE_LOAD;
        }
    }
    else if (SWC > 1800)
    {
        if (robot_state == ROBOT_STATE_SEED_CTRL)
        {
            seed_state = SEED_STATE_PREPUT;
        }
        else if (robot_state == ROBOT_STATE_SHOOT_CTRL)
        {
            shoot_state = SHOOT_STATE_SHOOTING;
        }
    }
}

void SWD_judge(void)
{
    if (SWD < 1500)
    {
        // 路径规划关
        ROBOT_CHASSI.World_Move_Flag = 0;
    }
    else if (SWD > 1500)
    {
        // 路径规划开
        ROBOT_CHASSI.World_Move_Flag = 1;
    }
}
