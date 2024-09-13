//
// Created by Ray on 2023/11/24.
//

#include "FSM.h"

AUTO_STATE auto_state = AUTO_DISABLE;
ROBOT_STATE_ITEMS robot_state = ROBOT_STATE_INIT;
CHASSIS_STATE_ITEMS chassis_state = CHASSIS_DISABLE;
SEED_STATE_ITEMS last_seed_state, seed_state = SEED_STATE_DISABLE;
SHOOT_STATE_ITEMS shoot_state = SHOOT_STATE_DISABLE;
GIMBAL_STATE_ITEMS gimbal_state = GIMBAL_STATE_DISABLE;
MOVE_STATE_ITEMS move_state = MOVE_STATE_INIT;

int update_action_seed = 0; // ���ڻ�ȡ�����˵�ǰλ��actionֵ
int update_action_put = 0;
float yaw_disable = 0;
int field = 0;
int catch_ball_laser = 0;
int in2aera = 0;

int once_flag1 = 0;
int once_flag2 = 0;

/**
 * @brief ��״̬��
 * ͨ��SWA�жϻ����˵��Զ������ֶ�״̬��Ȼ�������Ӧ��״̬��
 */
void robot_fsm(void)
{
    if (auto_state == AUTO_ENABLE)
    {
        auto_fsm(field); // �����Զ�״̬��
        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1))
        {
            catch_ball_laser = 0; // δ��λ��־
        }
        else
        {
            catch_ball_laser = 1; // ����ȡ��λ��־
        }
        action_reset(); // ��action yaw������
    }
    else if (auto_state == AUTO_DISABLE)
    {
        ROBOT_CHASSI.plan_x = 0;
        ROBOT_CHASSI.plan_y = 0;
        ROBOT_CHASSI.plan_w = 0; // �Զ�·���滮ʧ��

        once_flag1 = 0;
        once_flag2 = 0;
        caculation_ok = 0;   // ���ֱ�־λ����
        robot_state_judge(); // �����ֶ�״̬��
    }
    else
    {
        // ���������ʧ�ܣ�
    }
}

/**
 * @brief �ֶ�״̬��
 * �ж�SWA��SWB��SWC��SWD��״̬������״̬�жϻ����˵�״̬
 */
void robot_state_judge(void)
{
    if (robot_state == ROBOT_STATE_INIT)
    {
        chassis_state = CHASSIS_DISABLE;
        if (SWA != 0 && SWB != 0 && SWC != 0 && SWD != 0 && ACTION_GL_POS_DATA.POS_X != 0) // ȷ�Ϻ�ģ��ʼ����ACTION�ɹ�
        {
            SWA_judge();
            SWB_judge();
            SWC_judge();
        }
    }
    else
    {
        if (SWA == 0 && ACTION_GL_POS_DATA.POS_X == 0) // ��������
        {
            robot_state = ROBOT_STATE_INIT;
        }
        else
        {
            SWA_judge();
            SWB_judge();
            SWC_judge();
        }
    }
}

void auto_fsm(int f)
{
    static int SWC_ONCE = 0;
    static int SWC_able = 0;
    SWA_judge();
    if (auto_state == AUTO_DISABLE)
    {
        SWC_ONCE = 0;
    }
    if (auto_state == AUTO_ENABLE)
    {
        if (SWB < 1200)
        {
            SWC_able = 0;
        }
        else if (1300 < SWB && SWB < 1700)
        {
            if (in2aera == 0)
            {
                SWC_able = 1;
                robot_state = ROBOT_STATE_SEED_CTRL;
            }
            else
            {
                SWC_able = 0;
            }
        }
        else if (SWB > 1800)
        {
            SWC_able = 1;
            robot_state = ROBOT_STATE_SHOOT_CTRL;
        }

        if (f == Left)
        {
            if (SWC < 1200)
            {
                if (robot_state == ROBOT_STATE_SEED_CTRL)
                {
                    if (SWC_able == 1)
                    {
                        if (SWC_ONCE != 1)
                        {
                            caculation_ok = 1;
                            update_action_seed = 1;
                            seed = 1;
                            move_state = MOVE_STATE_SEED;
                            SWC_ONCE = 1;
                        }
                    }
                }
                else if (robot_state == ROBOT_STATE_SHOOT_CTRL)
                {
                    if (SWC_ONCE == 4)
                    {
                        move_state = MOVE_STATE_WAIT_CATCH;
                        SWC_ONCE = 5;
                    }

                    if (ball_loaded == 0) // ��δװ��
                    {
                        shoot_state = SHOOT_STATE_INIT;
                    }
                    else if (ball_loaded == 1) // ����װ��
                    {
                        shoot_state = SHOOT_STATE_SHOOTING;
                    }
                }
            }
            else if (1300 < SWC && SWC < 1700)
            {
                if (robot_state == ROBOT_STATE_SEED_CTRL)
                {
                    if (SWC_able == 1)
                    {
                        if (SWC_ONCE != 2)
                        {
                            caculation_ok = 1;
                            update_action_seed = 1;
                            seed = 2;
                            move_state = MOVE_STATE_SEED;
                            SWC_ONCE = 2;
                        }
                    }
                }
                else if (robot_state == ROBOT_STATE_SHOOT_CTRL)
                {
                    if (SWC_ONCE != 4)
                    {
                        direction = action_ball;
                        update_action_ball = 1;
                        move_state = MOVE_STATE_ACTION_NEAR;
                        //                        move_state = MOVE_STATE_LASER;
                        SWC_ONCE = 4;
                    }
                }
            }
            else if (SWC > 1800)
            {
                if (robot_state == ROBOT_STATE_SEED_CTRL)
                {
                    if (SWC_able == 1)
                    {
                        if (SWC_ONCE != 3)
                        {
                            caculation_ok = 1;
                            update_action_seed = 1;
                            seed = 3;
                            move_state = MOVE_STATE_SEED;
                            SWC_ONCE = 3;
                        }
                    }
                }
                else if (robot_state == ROBOT_STATE_SHOOT_CTRL)
                {
                    shoot_state = SHOOT_STATE_LOAD;
                }
            }
            else
            {
                robot_state = ROBOT_STATE_INIT;
            }
        }
        else if (f == Right)
        {
            if (SWC < 1200)
            {
                if (robot_state == ROBOT_STATE_SEED_CTRL)
                {
                    if (SWC_ONCE != 1)
                    {
                        caculation_ok = 1;
                        update_action_seed = 1;
                        seed = 3;
                        move_state = MOVE_STATE_SEED;
                        SWC_ONCE = 1;
                    }
                }
                else if (robot_state == ROBOT_STATE_SHOOT_CTRL)
                {
                    if (SWC_ONCE == 4)
                    {
                        move_state = MOVE_STATE_WAIT_CATCH;
                        SWC_ONCE = 5;
                    }

                    if (ball_loaded == 0) // ��δװ��
                    {
                        shoot_state = SHOOT_STATE_INIT;
                    }
                    else if (ball_loaded == 1) // ����װ��
                    {
                        shoot_state = SHOOT_STATE_SHOOTING;
                    }
                }
            }
            else if (1300 < SWC && SWC < 1700)
            {
                if (robot_state == ROBOT_STATE_SEED_CTRL)
                {
                    if (SWC_ONCE != 2)
                    {
                        caculation_ok = 1;
                        update_action_seed = 1;
                        seed = 2;
                        move_state = MOVE_STATE_SEED;
                        SWC_ONCE = 2;
                    }
                }
                else if (robot_state == ROBOT_STATE_SHOOT_CTRL)
                {
                    if (SWC_ONCE != 4)
                    {
                        direction = action_ball;
                        update_action_ball = 1;
                        move_state = MOVE_STATE_ACTION_NEAR;
                        //                        move_state = MOVE_STATE_LASER;
                        SWC_ONCE = 4;
                    }
                }
            }
            else if (SWC > 1800)
            {
                if (robot_state == ROBOT_STATE_SEED_CTRL)
                {
                    if (SWC_ONCE != 3)
                    {
                        caculation_ok = 1;
                        update_action_seed = 1;
                        seed = 1;
                        move_state = MOVE_STATE_SEED;
                        SWC_ONCE = 3;
                    }
                }
                else if (robot_state == ROBOT_STATE_SHOOT_CTRL)
                {
                    shoot_state = SHOOT_STATE_LOAD;
                }
            }
            else
            {
                robot_state = ROBOT_STATE_INIT;
            }
        }
    }
}

void SWA_judge(void)
{
    if (SWA < 1500)
    {
        ROBOT_CHASSI.World_Move_Flag = 0;
        chassis_state = CHASSIS_STATE_HIGH_SPEED;
        auto_state = AUTO_DISABLE; // �Զ���
        move_state = MOVE_STATE_INIT;
        // ·���滮��
    }
    else if (SWA > 1500)
    {
        // ·���滮��
        ROBOT_CHASSI.World_Move_Flag = 1;
        chassis_state = CHASSIS_STATE_LOW_SPEED;
        auto_state = AUTO_ENABLE; // �Զ���
    }
    else
    {
        robot_state = ROBOT_STATE_INIT;
    }
    if (robot_state == ROBOT_UPPER_DISABLE)
    {
        shoot_state = SHOOT_STATE_DISABLE;
        seed_state = SEED_STATE_DISABLE;
        gimbal_state = GIMBAL_STATE_DISABLE;
    }
    else if (robot_state == ROBOT_STATE_SHOOT_CTRL)
    {
        if (auto_state == AUTO_ENABLE)
        {
            if (SWD < 1500)
            {
                gimbal_state = GIMBAL_AIM_3;
            }
            else if (SWD > 1500)
            {
                gimbal_state = GIMBAL_AIM_2;
            }
        }
        else
        {
            gimbal_state = GIMBAL_STATE_DISABLE;
        }
        ball_auto(field);
        seed_state = SEED_STATE_CLOSE;
    }
    else if (robot_state == ROBOT_STATE_SEED_CTRL)
    {
        action_ball = 0;
        shoot_state = SHOOT_STATE_DISABLE;
        gimbal_state = GIMBAL_STATE_DISABLE;
    }
}

void SWB_judge(void)
{
    if (SWB < 1200)
    {
        robot_state = ROBOT_UPPER_DISABLE;
    }
    else if (1300 < SWB && SWB < 1700)
    {
        robot_state = ROBOT_STATE_SEED_CTRL;
        shoot_state = SHOOT_STATE_DISABLE;
    }
    else if (SWB > 1800)
    {
        robot_state = ROBOT_STATE_SHOOT_CTRL;
        seed_state = SEED_STATE_CLOSE;
    }
    else
    {
        robot_state = ROBOT_STATE_INIT;
    }
}

void SWC_judge(void)
{
    if (SWC < 1200)
    {
        if (robot_state == ROBOT_STATE_SEED_CTRL)
        {
            if (flip_state == 0)
            {
                seed_state = SEED_STATE_INIT;
            }
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
            seed_state = field == 0 ? SEED_STATE_PREPUT : SEED_STATE_PREPUT_BLUE;
        }
        else if (robot_state == ROBOT_STATE_SHOOT_CTRL)
        {
            shoot_state = SHOOT_STATE_SHOOTING;
        }
    }
    else
    {
        robot_state = ROBOT_STATE_INIT;
    }
}

// void SWD_judge(void)
// {
//     if (SWD < 1500)
//     {
//     }
//     else if (SWD > 1500)
//     {
//     }
//     else
//     {
//         robot_state = ROBOT_STATE_INIT;
//     }
// }

// float action_yaw;
float wait_ation = 0;
float tt3 = 0;
int caculation_ok = 0;
void action_reset(void)
{
    if (field == Left)
    {
        if ((YaoGan_RIGHT_Y - 1500) > 450)
        {
            in2aera = 1;
            if (once_flag1 == 0)
            {
                caculation_ok = 1;
                move_state = MOVE_STATE_INIT;
                Update_J(0.0);
                ACTION_GL_POS_DATA.REAL_X = (DT35.k - 50.0);
                ACTION_GL_POS_DATA.REAL_Y = 1160.0;
                robot_state = ROBOT_STATE_SHOOT_CTRL;
                shoot_state = SHOOT_STATE_INIT;
                wait_ation++;
                once_flag1 = 1;
            }
        }
        else if (YaoGan_RIGHT_Y - 1500 < -450)
        {
            in2aera = 0;
            if (once_flag2 == 0)
            {
                caculation_ok = 0;
                ROBOT_CHASSI.plan_x = 0;
                ROBOT_CHASSI.plan_y = 0;
                ROBOT_CHASSI.plan_w = 0;
                Update_ACTION();
                flag = 0;
                move_state = MOVE_STATE_INIT;
                ACTION_GL_POS_DATA.REAL_X = 0.0;
                ACTION_GL_POS_DATA.REAL_Y = 0.0;
                once_flag2 = 1;
            }
        }
    }
    else if (field == Right)
    {
        if ((YaoGan_RIGHT_Y - 1500) > 450)
        {
            in2aera = 1;
            if (once_flag1 == 0)
            {
                caculation_ok = 1;
                move_state = MOVE_STATE_INIT;
                Update_J(0.0);
                ACTION_GL_POS_DATA.REAL_X = -(DT35.b - 50.0);
                ACTION_GL_POS_DATA.REAL_Y = 1160.0;
                robot_state = ROBOT_STATE_SHOOT_CTRL;
                shoot_state = SHOOT_STATE_INIT;
                once_flag1 = 1;
            }
        }
        else if (YaoGan_RIGHT_Y - 1500 < -450)
        {
            in2aera = 0;
            if (once_flag2 == 0)
            {
                caculation_ok = 0;
                ROBOT_CHASSI.plan_x = 0;
                ROBOT_CHASSI.plan_y = 0;
                ROBOT_CHASSI.plan_w = 0;
                move_state = MOVE_STATE_INIT;

                Update_ACTION();
                flag = 0;
                ACTION_GL_POS_DATA.REAL_X = 0.0;
                ACTION_GL_POS_DATA.REAL_Y = 0.0;
                once_flag2 = 1;
            }
        }
    }
}
