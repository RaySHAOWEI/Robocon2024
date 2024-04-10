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
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for Chassis */
osThreadId_t ChassisHandle;
const osThreadAttr_t Chassis_attributes = {
  .name = "Chassis",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for Seed */
osThreadId_t SeedHandle;
const osThreadAttr_t Seed_attributes = {
  .name = "Seed",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Shoot */
osThreadId_t ShootHandle;
const osThreadAttr_t Shoot_attributes = {
  .name = "Shoot",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Motor */
osThreadId_t MotorHandle;
const osThreadAttr_t Motor_attributes = {
  .name = "Motor",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for Communicate */
osThreadId_t CommunicateHandle;
const osThreadAttr_t Communicate_attributes = {
  .name = "Communicate",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Move */
osThreadId_t MoveHandle;
const osThreadAttr_t Move_attributes = {
  .name = "Move",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void FSM_Task(void *argument);
void Chassis_Task(void *argument);
void Seed_Task(void *argument);
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
    // 处理数据
    // processData(receive_Buffer);
    osDelay(1000);
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
  for (;;)
  {
    // switch (seed_state)
    // {
    // case SEED_STATE_DISABLE:
    //   /*收缩状态*/
    //   break;
    // case SEED_STATE_INIT:
    //   /*初始化、24放苗*/
    //   break;
    // case SEED_STATE_PEEK_DOWN:
    //   /* 夹苗下 */
    //   break;
    // case SEED_STATE_PREPUT:
    //   /* 13放苗 */
    //   break;
    // }
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
  for (;;)
  {
    switch (shoot_state)
    {
      case SHOOT_STATE_INIT:
        /*初始化*/
			    shoot_jaw_homeing(shoot_homeing_flag);
          HAL_GPIO_WritePin(GPIOF,GPIO_PIN_12,GPIO_PIN_RESET);
          vTaskDelay(300);
          can2motorRealInfo[Motor_SHOOT_GIMBAL].once_flag = 1;
          Position_Control(&can2motorRealInfo[Motor_SHOOT_GIMBAL],3300);
          vTaskDelay(500);
          Speed_Control(&can2motorRealInfo[Motor_SHOOT_MOTOR_2],0);
          Speed_Control(&can2motorRealInfo[Motor_SHOOT_MOTOR_1],0);
        break;
			case SHOOT_STATE_INIT_FINISH:
				  Speed_Control(&can2motorRealInfo[Motor_SHOOT_MOTOR_2],5000);
          Speed_Control(&can2motorRealInfo[Motor_SHOOT_MOTOR_1],-5000); 
			    vTaskDelay(500);
			    Load_Ball(jaw_up);
			
			  break;
      case SHOOT_STATE_LOAD:
        /*取球*/
			    flag=flag_judgment();
			  if(flag == 0){
					shoot_jaw_homeing(shoot_homeing_flag);
					vTaskDelay(1500);
          Load_Ball(catch_ball);
				             }
			  if(flag == 1){
          Position_Control(&can1motorRealInfo[6],115);
          can1motorRealInfo[6].HomingMode.done_flag = 0;					
				             }
        break;
      case SHOOT_STATE_SHOOTING:
        /*发射*/
//        Shooting_ball();
        Pos_Torque_Control(&can1motorRealInfo[Motor_SHOOT_JAW],4000,100);
			  flag = 0;                                       //标志回零
				if(can2motorRealInfo[Motor_SHOOT_GIMBAL].ANGLE <= 4600 && can2motorRealInfo[Motor_SHOOT_GIMBAL].ANGLE >=2000)
				{
					if(YaoGan_RIGHT_Y -1500 > 100)
					{
							Speed_Control(&can2motorRealInfo[Motor_SHOOT_GIMBAL],800);
					}
					else if(YaoGan_RIGHT_Y -1500 < -100)
					{
							Speed_Control(&can2motorRealInfo[Motor_SHOOT_GIMBAL],-800);
					}
					else
					{
							Speed_Control(&can2motorRealInfo[Motor_SHOOT_GIMBAL],0);
					}
				}
				else
				{
						Speed_Control(&can2motorRealInfo[Motor_SHOOT_GIMBAL],0);
				}
          vTaskDelay(500);
          HAL_GPIO_WritePin(GPIOF,GPIO_PIN_12,GPIO_PIN_SET);
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
  /* Infinite loop */
  for (;;)
  {
    static int lost_back;
    if (seed_state != last_seed_state)
    {
      for (int i = 0; i < 5; i++)
      {
        ctrl_sent_test(seed_state);
      }
      last_seed_state = seed_state;
    }
    if (feedback.lost_cnt != lost_back)
    {
      for (int i = 0; i < 5; i++)
      {
        ctrl_sent_test(seed_state);
      }
      lost_back = feedback.lost_cnt;
    }
    osDelay(1);
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
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END MoveTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

