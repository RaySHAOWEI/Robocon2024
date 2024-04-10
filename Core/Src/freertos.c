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
    .stack_size = 128 * 4,
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
      ROBOT_CHASSI.Vx_MAX = 0.0f;
      ROBOT_CHASSI.Vy_MAX = 0.0f;
      ROBOT_CHASSI.Vw_MAX = 0.0f;
      chassis_stop();
    }
    else if (chassis_state == CHASSIS_STATE_HIGH_SPEED)
    {
      ROBOT_CHASSI.Vx_MAX = 1.6f;
      ROBOT_CHASSI.Vy_MAX = 1.6f;
      ROBOT_CHASSI.Vw_MAX = 2.0f;
    }
    else if (chassis_state == CHASSIS_STATE_LOW_SPEED)
    {
      ROBOT_CHASSI.Vx_MAX = 0.4f;
      ROBOT_CHASSI.Vy_MAX = 0.4f;
      ROBOT_CHASSI.Vw_MAX = 0.5f;
    }
    if (robot_state == ROBOT_STATE_SEED_CTRL)
    {
      free_ctrl_change();
    }
    else
    {
      Free_Control();
    }
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
    gimbal_free_ctrl();
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
  static int ball_jaw = 0;  // 夹爪是否夹着球 0:没球 1:有球
  static int ball_load = 0; // 发射仓是否有球 0:没球 1:有球
  /* Infinite loop */
  for (;;)
  {
    switch (shoot_state)
    {
    case SHOOT_STATE_DISABLE:
      /*失能*/
      flip_motor(flip_up);     // 翻转电机上升到高处防止夹苗干涉
      back2init;               // 气缸复位
      belt_ctrl(belt_stop);    // 摩擦带停止
      gimbal_ctrl(gimbal_mid); // 云台回中
      break;
    case SHOOT_STATE_INIT:
      /*初始化*/
      back2init;             // 气缸复位
      flip_motor(flip_down); // 翻转电机下降到初始位置
      jaw_motor(jaw_open);   // 夹爪张开
      belt_ctrl(belt_speed); // 摩擦带转动
      break;
    case SHOOT_STATE_LOAD:
      /*取球*/
      back2init;             // 气缸复位
      belt_ctrl(belt_speed); // 摩擦带转动

      if (ball_load == 1) // 球已上膛
      {
        if (ball_jaw == 0) // 夹爪没球
        {
          flip_motor(flip_up); // 保持翻转上，等待射球
        }
      }
      else if (ball_load == 0) // 球没上膛
      {
        if (ball_jaw == 0) // 夹爪没球
        {
          if (flip_motor(flip_down) == 0) // 等待翻转下
          {
            ball_jaw = jaw_motor(jaw_close); // 夹球 夹紧时判定：夹爪有球
          }
          else // 翻转电机未到位
          {
            jaw_motor(jaw_open); // 夹爪松开
          }
        }
        else if (ball_jaw == 1) // 夹爪有球
        {
          if (flip_motor(flip_up)) // 等待翻转上
          {
            ball_jaw = jaw_motor(jaw_open); // 放球 松开时判定：夹爪没球
            ball_load = 1;                  // 同时判定：球已上膛
          }
          else
          {
            jaw_motor(jaw_close); // 夹爪夹紧防止球滚落
          }
        }
      }
      break;
    case SHOOT_STATE_SHOOTING:
      /*发射*/
      belt_ctrl(belt_speed); // 摩擦带转动
      shootball;             // 发射
      ball_load = 0;         // 球没上膛
      break;
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
  static int lost_back;
  /* Infinite loop */
  for (;;)
  {
    if (seed_state != last_seed_state)
    {
      for (int i = 0; i < 2; i++)
      {
        ctrl_sent_test(seed_state);
      }
      last_seed_state = seed_state;
    }
    if (feedback.lost_cnt != lost_back)
    {
      for (int i = 0; i < 2; i++)
      {
        ctrl_sent_test(seed_state);
      }
      lost_back = feedback.lost_cnt;
    }
    osDelay(10);
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
  for (;;)
  {
    osDelay(1);
  }
  /* USER CODE END MoveTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
