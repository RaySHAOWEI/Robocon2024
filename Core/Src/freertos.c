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
#include "move.h"
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
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Chassis */
osThreadId_t ChassisHandle;
const osThreadAttr_t Chassis_attributes = {
  .name = "Chassis",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for Seed */
osThreadId_t SeedHandle;
const osThreadAttr_t Seed_attributes = {
  .name = "Seed",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Shoot */
osThreadId_t ShootHandle;
const osThreadAttr_t Shoot_attributes = {
  .name = "Shoot",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Move */
osThreadId_t MoveHandle;
const osThreadAttr_t Move_attributes = {
  .name = "Move",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void FSM_Task(void *argument);
void Chassis_Task(void *argument);
void Seed_Task(void *argument);
void Shoot_Task(void *argument);
void Move_Task(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
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

  /* creation of Seed */
  SeedHandle = osThreadNew(Seed_Task, NULL, &Seed_attributes);

  /* creation of Shoot */
  ShootHandle = osThreadNew(Shoot_Task, NULL, &Shoot_attributes);

  /* creation of Move */
  MoveHandle = osThreadNew(Move_Task, NULL, &Move_attributes);

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
  for(;;)
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
  for(;;)
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
    Free_Control();
    Robot_Wheels_RPM_calculate();
    Motor_Control();
    osDelay(1);
  }
  /* USER CODE END Chassis_Task */
}

/* USER CODE BEGIN Header_Seed_Task */
/**
* @brief Function implementing the Seed thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Seed_Task */
void Seed_Task(void *argument)
{
  /* USER CODE BEGIN Seed_Task */
  /* Infinite loop */
  for(;;)
  {
    switch (seed_state)
    {
      case SEED_STATE_DISABLE:
        /*收缩状态*/
        // flip_motor(Flip2ground);
        // lift_motor(lift2ground);
        cylinder_control(open, 0);
        cylinder_control(finger1_3, 0);
        cylinder_control(finger2_4, 0);
        break;
      case SEED_STATE_INIT:
        /*初始化、24放苗*/
        flip_motor(Flip2ground);
        lift_motor(lift2ground);
        cylinder_control(open, 1);
        cylinder_control(finger1_3, 0);
        cylinder_control(finger2_4, 0);
        break;

      case SEED_STATE_PEEK_DOWN:
        /* 夹苗下 */
        lift_motor(lift2ground);
        cylinder_control(finger1_3, 1);
        cylinder_control(finger2_4, 1);
        seed_state = SEED_STATE_PEEK_UP;
        break;
      case SEED_STATE_PEEK_UP:
        /* 夹苗上 */
        vTaskDelay(100);
        lift_motor(lift2top);
        break;

      case SEED_STATE_PRE_PUT:
        /* 预备放苗 */
        lift_motor(lift2half);
        seed_state = SEED_STATE_PUT;
        break;
      case SEED_STATE_PUT:
        /* 13放苗 */
        vTaskDelay(100);
        cylinder_control(finger1_3, 0);
        break;
    }
    osDelay(1);
  }
  /* USER CODE END Seed_Task */
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
  for(;;)
  {
    switch (shoot_state)
    {
      case SHOOT_STATE_INIT:
        /*初始化*/
        servos_control(180);
        cylinder_control(push1, 0);
        cylinder_control(push2, 0);
        belt_ctrl(0);
        break;
      case SHOOT_STATE_LOAD:
        /*取球*/
        servos_control(155);
        break;
      case SHOOT_STATE_SPEED_UP:
        /*加速*/
        belt_ctrl(1000);
        vTaskDelay(500);
        shoot_state = SHOOT_STATE_SHOOTING;
        break;
      case SHOOT_STATE_SHOOTING:
        /*发射*/
        cylinder_control(push1, 1);
        cylinder_control(push2, 1);
        servos_control(100);
        belt_logs();
        break;
    }
    osDelay(1);
  }
  /* USER CODE END Shoot_Task */
}

/* USER CODE BEGIN Header_Move_Task */
/**
* @brief Function implementing the Move thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Move_Task */
void Move_Task(void *argument)
{
  /* USER CODE BEGIN Move_Task */
  /* Infinite loop */
  for(;;)
  {
    //这段代码用于调用路径规划
    osDelay(1);
  }
  /* USER CODE END Move_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

