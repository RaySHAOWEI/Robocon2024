/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for FSM_switch */
osThreadId_t FSM_switchHandle;
const osThreadAttr_t FSM_switch_attributes = {
    .name = "FSM_switch",
    .stack_size = 256 * 4,
    .priority = (osPriority_t)osPriorityAboveNormal,
};
/* Definitions for Chassis */
osThreadId_t ChassisHandle;
const osThreadAttr_t Chassis_attributes = {
    .name = "Chassis",
    .stack_size = 256 * 4,
    .priority = (osPriority_t)osPriorityBelowNormal,
};
/* Definitions for Gimbal */
osThreadId_t GimbalHandle;
const osThreadAttr_t Gimbal_attributes = {
    .name = "Gimbal",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityLow,
};
/* Definitions for Shoot */
osThreadId_t ShootHandle;
const osThreadAttr_t Shoot_attributes = {
    .name = "Shoot",
    .stack_size = 256 * 4,
    .priority = (osPriority_t)osPriorityLow,
};
/* Definitions for Motor */
osThreadId_t MotorHandle;
const osThreadAttr_t Motor_attributes = {
    .name = "Motor",
    .stack_size = 256 * 4,
    .priority = (osPriority_t)osPriorityHigh,
};
/* Definitions for Communicate */
osThreadId_t CommunicateHandle;
const osThreadAttr_t Communicate_attributes = {
    .name = "Communicate",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for Move */
osThreadId_t MoveHandle;
const osThreadAttr_t Move_attributes = {
    .name = "Move",
    .stack_size = 256 * 4,
    .priority = (osPriority_t)osPriorityLow,
};
/* Definitions for sensor */
osThreadId_t sensorHandle;
const osThreadAttr_t sensor_attributes = {
    .name = "sensor",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void FSM_Task(void *argument);
void Chassis_Task(void *argument);
void Gimbal_Task(void *argument);
void Shoot_Task(void *argument);
void MotorTask(void *argument);
void CommTask(void *argument);
void MoveTask(void *argument);
void sensor_fun(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void)
{
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of FSM_switch */
  FSM_switchHandle = osThreadNew(FSM_Task, NULL, &FSM_switch_attributes);

  /* creation of Chassis */
  ChassisHandle = osThreadNew(Chassis_Task, NULL, &Chassis_attributes);

  /* creation of Gimbal */
  GimbalHandle = osThreadNew(Gimbal_Task, NULL, &Gimbal_attributes);

  /* creation of Shoot */
  ShootHandle = osThreadNew(Shoot_Task, NULL, &Shoot_attributes);

  /* creation of Motor */
  MotorHandle = osThreadNew(MotorTask, NULL, &Motor_attributes);

  /* creation of Communicate */
  CommunicateHandle = osThreadNew(CommTask, NULL, &Communicate_attributes);

  /* creation of Move */
  MoveHandle = osThreadNew(MoveTask, NULL, &Move_attributes);

  /* creation of sensor */
  sensorHandle = osThreadNew(sensor_fun, NULL, &sensor_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */
}

/* USER CODE BEGIN Header_FSM_Task */
/**
 * @brief  Function implementing the FSM_switch thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_FSM_Task */
void FSM_Task(void *argument)
{
  /* USER CODE BEGIN FSM_Task */
  /* Infinite loop */
  for (;;)
  {
    // 用于切换场地
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2))
    {
      field = Right;
    }
    else
    {
      field = Left;
    }
    robot_fsm();
    osDelay(1);
  }
  /* USER CODE END FSM_Task */
}

/* USER CODE BEGIN Header_Chassis_Task */
/**
 * @brief Function implementing the Chassis thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Chassis_Task */
void Chassis_Task(void *argument)
{
  /* USER CODE BEGIN Chassis_Task */
  /* Infinite loop */
  for (;;)
  {
    if (chassis_state == CHASSIS_DISABLE)
    {
      ROBOT_CHASSI.Vx_MAX = 0;
      ROBOT_CHASSI.Vy_MAX = 0;
      ROBOT_CHASSI.Vw_MAX = 0;
      chassis_stop();
    }
    else if (chassis_state == CHASSIS_STATE_HIGH_SPEED)
    {
      ROBOT_CHASSI.Vx_MAX = 1600;
      ROBOT_CHASSI.Vy_MAX = 1600;
      ROBOT_CHASSI.Vw_MAX = 2000;
    }
    else if (chassis_state == CHASSIS_STATE_LOW_SPEED)
    {
      //      ROBOT_CHASSI.Vx_MAX = 800;
      //      ROBOT_CHASSI.Vy_MAX = 800;
      //      ROBOT_CHASSI.Vw_MAX = 1000;
      ROBOT_CHASSI.Vx_MAX = 1600;
      ROBOT_CHASSI.Vy_MAX = 1600;
      ROBOT_CHASSI.Vw_MAX = 2000;
    }
    Free_Control();
    Robot_Wheels_RPM_calculate();
    osDelay(1);
  }
  /* USER CODE END Chassis_Task */
}

/* USER CODE BEGIN Header_Gimbal_Task */
/**
 * @brief Function implementing the Gimbal thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Gimbal_Task */
void Gimbal_Task(void *argument)
{
  /* USER CODE BEGIN Gimbal_Task */
  /* Infinite loop */
  for (;;)
  {
    switch (gimbal_state)
    {
    case GIMBAL_STATE_DISABLE:
      Position_Control(&can2motorRealInfo[Motor_SHOOT_GIMBAL], 3300);
      belt_speed = 5000;
      // belt_calc(field, GIMBAL_STATE_DISABLE);
      break;

    case GIMBAL_AIM_1:
      if (field == Left)
      {
        Gimbal.target_point[0] = shooting_point_x[1];
        Gimbal.target_point[1] = shooting_point_y[1];
      }
      else if (field == Right)
      {
        Gimbal.target_point[0] = shooting_point_x[4];
        Gimbal.target_point[1] = shooting_point_y[4];
      }
      Gimbal_controller();
      if (Theta <= 140 && Theta >= -140)
      {
        if (Position_Control(&can2motorRealInfo[Motor_SHOOT_GIMBAL], Alpha))
        {
          Gimbal.aim_flag = Position_Control(&can2motorRealInfo[Motor_SHOOT_GIMBAL], Alpha);
        }
      }
      belt_calc(field, GIMBAL_AIM_1);
      break;

    case GIMBAL_AIM_2:
      if (field == Left)
      {
        Gimbal.target_point[0] = shooting_point_x[2];
        Gimbal.target_point[1] = shooting_point_y[2];
      }
      else if (field == Right)
      {
        Gimbal.target_point[0] = shooting_point_x[5];
        Gimbal.target_point[1] = shooting_point_y[5];
      }
      Gimbal_controller();
      if (Theta <= 140 && Theta >= -140)
      {
        if (Position_Control(&can2motorRealInfo[Motor_SHOOT_GIMBAL], Alpha))
        {
          Gimbal.aim_flag = Position_Control(&can2motorRealInfo[Motor_SHOOT_GIMBAL], Alpha);
        }
      }
      belt_calc(field, GIMBAL_AIM_2);
      break;

    case GIMBAL_AIM_3:
      if (field == Left)
      {
        Gimbal.target_point[0] = shooting_point_x[3];
        Gimbal.target_point[1] = shooting_point_y[3];
      }
      else if (field == Right)
      {
        Gimbal.target_point[0] = shooting_point_x[6];
        Gimbal.target_point[1] = shooting_point_y[6];
      }
      Gimbal_controller();
      if (Theta <= 140 && Theta >= -140)
      {
        if (Position_Control(&can2motorRealInfo[Motor_SHOOT_GIMBAL], Alpha))
        {
          Gimbal.aim_flag = Position_Control(&can2motorRealInfo[Motor_SHOOT_GIMBAL], Alpha);
        }
      }
      belt_calc(field, GIMBAL_AIM_3);
      break;
    default:
      break;
    }
    // // sent_data(-1 * can1motorRealInfo[0].RPM, -1 * can1motorRealInfo[1].RPM, can1motorRealInfo[2].RPM, can1motorRealInfo[3].RPM);
    // if (ball == 0 || shooting_point == 0)
    // {
    //   Position_Control(&can2motorRealInfo[Motor_SHOOT_GIMBAL], 3300);
    // }
    // else
    // {
    //   Auto_shooting_Init(); // 自动瞄准初始化
    // }
    // ball_auto(field);
    osDelay(10);
  }
  /* USER CODE END Gimbal_Task */
}

/* USER CODE BEGIN Header_Shoot_Task */
/**
 * @brief Function implementing the Shoot thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Shoot_Task */
void Shoot_Task(void *argument)
{
  /* USER CODE BEGIN Shoot_Task */

  /* Infinite loop */
  static int catch_start = 0; // 夹取计时开启标志位
  static int catch_time = 0;  // 夹取计时
  int catch_time_max = 150;   // 夹取计时最大值
  for (;;)
  {
    switch (shoot_state)
    {
    case SHOOT_STATE_DISABLE:
      /*失能*/
      flip_motor(flip_init); // 翻转电机抬升
      jaw_open;              // 夹爪张开
      ball_loaded = 0;       // 球未装填
      back_ball;             // 推射气缸回位
      belt_ctrl(belt_stop);  // 摩擦带停止
      gimbal_state = GIMBAL_STATE_DISABLE;
      shooting_point = 0;
      belt_speed = 0;
      break;
    case SHOOT_STATE_INIT:
      /*初始化*/
      jaw_open;              // 夹爪张开
      ball_loaded = 0;       // 球未装填
      back_ball;             // 推射气缸回位
      flip_motor(flip_down); // 翻转电机下降
      catch_start = 0;       // 关闭夹爪计时
                             //	  belt_ctrl(belt_speed);
      break;
    case SHOOT_STATE_LOAD:
      /*取球*/
      belt_ctrl(belt_stop);                // 摩擦带停止
      jaw_close;                           // 夹爪夹紧
      back_ball;                           // 推射气缸回位
      catch_start = 1;                     // 开启计时
      if (catch_time > catch_time_max - 1) // 1000ms后
      {
        flip_motor(flip_up); // 翻转电机抬升
        if (flip_motor(flip_up) == 0)
        {
          ball_loaded = 1; // 球已装填
          jaw_open;        // 装填
        }
      }
      else
      {
        flip_motor(flip_down); // 翻转电机下降
      }
      break;
    case SHOOT_STATE_SHOOTING:
      /*发射*/
      jaw_open; // 夹爪张开
      // if (ball != 0)
      // {
      //   belt_ctrl(belt_speed);
      // }
      // else if (ball == 0)
      // {
      //   belt_base();
      // }
      belt_ctrl(belt_speed);
      if (shooting_enable)
      {
        push_ball; // 推射气缸推出
      }
      if (catch_time > catch_time_max - 1) // 1000ms后
      {
        flip_motor(flip_down); // 翻转电机下降
      }
      if (flip_motor(flip_down) == 1)
      {
        ball_loaded = 0; // 球未装填
        catch_start = 0; // 关闭夹爪计时
      }
      break;
    }
    if (catch_start == 1)
    {
      if (catch_time < catch_time_max)
      {
        catch_time++;
      }
      else
      {
        catch_time = catch_time_max;
      }
    }
    else
    {
      catch_time = 0;
    }
    osDelay(1);
  }
  /* USER CODE END Shoot_Task */
}

/* USER CODE BEGIN Header_MotorTask */
/**
 * @brief Function implementing the Motor thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_MotorTask */
void MotorTask(void *argument)
{
  /* USER CODE BEGIN MotorTask */
  /* Infinite loop */
  for (;;)
  {
    Motor_Control();
    osDelay(1);
  }
  /* USER CODE END MotorTask */
}

/* USER CODE BEGIN Header_CommTask */
/**
 * @brief Function implementing the Communicate thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_CommTask */
void CommTask(void *argument)
{
  /* USER CODE BEGIN CommTask */
  static int seed_lost_back1;
  static int seed_lost_back2;
  /* Infinite loop */
  for (;;)
  {
    if (seed_state != last_seed_state)
    {
      ctrl_sent_test(seed_state);
      last_seed_state = seed_state;
    }
    if (feedback1.lost_cnt != seed_lost_back1 || feedback2.lost_cnt != seed_lost_back2)
    {
      ctrl_sent_test(seed_state);
      seed_lost_back1 = feedback1.lost_cnt;
      seed_lost_back2 = feedback2.lost_cnt;
    }

    // 键盘反馈数据更新
    keyboard_feedbacks.auto_state = auto_state;
    keyboard_feedbacks.move_state = move_state_switch(move_state);
    keyboard_feedbacks.direction = direction;
    keyboard_feedbacks.seed = seed;
    keyboard_feedbacks.put = put;
    keyboard_feedbacks.gimbal_state = gimbal_state;
    keyboard_feedbacks.lost_cnt = keyboard_feedbacks.receive_cnt - keyboard_feedbacks.command_cnt;
    if (memcmp(&keyboard_feedbacks, &last_keyboard_feedbacks, 9) != 0)
    {
      keyboard_feedback_send();
      memcpy(&last_keyboard_feedbacks, &keyboard_feedbacks, 9);
    }

    osDelay(100);
  }
  /* USER CODE END CommTask */
}

/* USER CODE BEGIN Header_MoveTask */
/**
 * @brief Function implementing the Move thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_MoveTask */
void MoveTask(void *argument)
{
  /* USER CODE BEGIN MoveTask */
  /* Infinite loop */
  //  static float tt17 = 0;
  static int seed_preput = 0;
  static int seed_chack = 0;
  for (;;)
  {
    if (caculation_ok)
    {
      YawAdjust(0.0);
    }
    switch (move_state)
    {
    case MOVE_STATE_INIT:
      ROBOT_CHASSI.plan_x = 0;
      ROBOT_CHASSI.plan_y = 0;
      //      ROBOT_CHASSI.plan_w = 0;
      break;

    case MOVE_STATE_WAIT_SEED:
      robot_state = ROBOT_STATE_SEED_CTRL;
      ROBOT_CHASSI.plan_x = 0;
      ROBOT_CHASSI.plan_y = 0;
      //      ROBOT_CHASSI.plan_w = 0;
      if (SWD < 1500)
      {
        seed_state = SEED_STATE_INIT;
      }
      else if (SWD > 1500)
      {
        seed_state = SEED_STATE_PEEK_DOWN;

        if (feedback2.state == SEED_STATE_PEEK_UP && feedback2.motor_state[2] == 2)
        {
          update_action_put = 1;
          put = 2 * seed - 1;
          move_state = MOVE_STATE_PUT;
        }
      }
      break;

    case MOVE_STATE_WAIT_PUT:
      robot_state = ROBOT_STATE_SEED_CTRL;
      ROBOT_CHASSI.plan_x = 0;
      ROBOT_CHASSI.plan_y = 0;
      //      ROBOT_CHASSI.plan_w = 0;
      if (SWD < 1500)
      {
        if (put == 1)
        {
          seed_state = field == 0 ? SEED_STATE_PUT : SEED_STATE_PUT_BLUE;
          seed_chack = 0;
          if (feedback2.motor_state[field] == 0)
          {
            update_action_put = 1;
            put = 2;
            move_state = MOVE_STATE_PUT;
          }
        }
        else if (put == 3)
        {
          seed_state = field == 0 ? SEED_STATE_PUT : SEED_STATE_PUT_BLUE;
          seed_chack = 0;
          if (feedback2.motor_state[field] == 0)
          {
            update_action_put = 1;
            put = 4;
            move_state = MOVE_STATE_PUT;
          }
        }
        else if (put == 5)
        {
          seed_state = field == 0 ? SEED_STATE_PUT : SEED_STATE_PUT_BLUE;
          seed_chack = 0;
          if (feedback2.motor_state[field] == 0)
          {
            update_action_put = 1;
            put = 6;
            move_state = MOVE_STATE_PUT;
          }
        }
        else // put为双数 2 4 6
        {
          if (seed_chack == 0)
          {
            seed_state = field == 0 ? SEED_STATE_PUT : SEED_STATE_PUT_BLUE;
          }
          else if (seed_chack == 1)
          {
            seed_state = SEED_STATE_INIT;
            seed_preput = 0;
          }
        }
      }
      else if (SWD > 1500)
      {
        if (seed_preput == 1)
        {
          seed_chack = 1;
          seed_state = SEED_STATE_HALF;
        }
        else if (seed_preput == 0)
        {
          seed_state = SEED_STATE_PEEK_UP;
        }
      }
      break;

    case MOVE_STATE_WAIT_CATCH:
      robot_state = ROBOT_STATE_SHOOT_CTRL;
      ROBOT_CHASSI.plan_x = 0;
      ROBOT_CHASSI.plan_y = 0;
      //      ROBOT_CHASSI.plan_w = 0;
      break;

    case MOVE_STATE_WAIT_ACTION:

      break;

    case MOVE_INIT_ACTION:

      break;

    case MOVE_STATE_SEED:
      robot_state = ROBOT_STATE_SEED_CTRL;
      seed_state = SEED_STATE_JAW_INIT;
      if (field == Left)
      {
        move_seed();
      }
      else
      {
        move_seed_right();
      }

      break;

    case MOVE_SEED_LASER:
      robot_state = ROBOT_STATE_SEED_CTRL;
      seed_state = SEED_STATE_INIT;
      if (field == Left)
      {
        laser_seed();
      }
      else
      {
        //		  point_to_point(3321,280,0.0,500);
        laser_seed_right();
      }
      break;

    case MOVE_PUT_LASER:
      robot_state = ROBOT_STATE_SEED_CTRL;
      if (put == 1 || put == 3 || put == 5)
      {
        seed_state = SEED_STATE_HALF;
        seed_preput = 1;
      }
      else if (put == 2 || put == 4 || put == 6)
      {
        seed_state = field == 0 ? SEED_STATE_PUT : SEED_STATE_PUT_BLUE;
        seed_chack = 0;
      }

      if (field == Left)
      {
        laser_put();
      }
      else
      {
        laser_put_right();
      }
      break;

    case MOVE_STATE_PUT:
      robot_state = ROBOT_STATE_SEED_CTRL;
      if (field == Left)
      {
        put_seed();
      }
      else
      {
        put_seed_right();
      }

      break;

    case MOVE_STATE_LASER:
      robot_state = ROBOT_STATE_SHOOT_CTRL;
      if (field == Left)
      {
        laser_catch();
      }
      else
      {

        laser_catch_right();
      }

      break;

    case MOVE_STATE_ACTION_NEAR:
      ////		if(Laser2_calibration(DT35.y,DT35.x))
      //	    if(YawAdjust(0.0))
      //		{
      //		 ROBOT_CHASSI.plan_x = 0;
      //		if(Laser3_calibration(449,100))
      //					{
      //					    ROBOT_CHASSI.plan_x = 0;
      //		                ROBOT_CHASSI.plan_y = 0;
      //		                ROBOT_CHASSI.plan_w = 0;
      //					    move_state = MOVE_STATE_INIT;
      //				       //不确定要不要加上倒车，看看测试 这里加上夹爪放苗之类的即可
      ////					   move_state = MOVE_STATE_LASER_NEAR;
      //					}
      //      laser_catch_near();
      robot_state = ROBOT_STATE_SHOOT_CTRL;
      action_catch();

      //				}

      break;

    case CALIBRATION_BY_ACTION:

      action_calibration();

      break;
    case MOVE_BALL_BY_ACTION:
      robot_state = ROBOT_STATE_SHOOT_CTRL;
      if (field == Left)
      {
        action_catch_ball();
      }
      else
      {

        action_catch_ball_right();
      }
      break;

    case MOVE_BY_ACTION:

      action_catch();

      break;
    }
    osDelay(10);
  }
  /* USER CODE END MoveTask */
}

/* USER CODE BEGIN Header_sensor_fun */
/**
 * @brief Function implementing the sensor thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_sensor_fun */
void sensor_fun(void *argument)
{
  /* USER CODE BEGIN sensor_fun */
  /* Infinite loop */
  for (;;)
  {
    //    tcs34725_work();
    sent_data(can1motorRealInfo[0].RPM, can1motorRealInfo[1].RPM, -1 * can1motorRealInfo[2].RPM, -1 * can1motorRealInfo[3].RPM);
    osDelay(10);
  }
  /* USER CODE END sensor_fun */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
