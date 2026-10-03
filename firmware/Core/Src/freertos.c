/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 RoverTech.
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
#include "cmsis_os2.h"
#include "stm32f4xx_hal.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "file.c"
#include "spi.h"
#include "../settings.h"
#include "../W25X_FLASH_FREERTOS/w25.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "../powerSTEP01/ps01.h"

#include "../RArm/rarm.h"

#include "tim.h"
#include "usart.h"
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
typedef StaticSemaphore_t osStaticMutexDef_t;
typedef StaticSemaphore_t osStaticSemaphoreDef_t;
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
w25_info_t w25info;
char comm_buffer[256];
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
uint32_t defaultTaskBuffer[ 128 ];
osStaticThreadDef_t defaultTaskControlBlock;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .cb_mem = &defaultTaskControlBlock,
  .cb_size = sizeof(defaultTaskControlBlock),
  .stack_mem = &defaultTaskBuffer[0],
  .stack_size = sizeof(defaultTaskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ledTask */
osThreadId_t ledTaskHandle;
uint32_t ledTaskBuffer[ 128 ];
osStaticThreadDef_t ledTaskControlBlock;
const osThreadAttr_t ledTask_attributes = {
  .name = "ledTask",
  .cb_mem = &ledTaskControlBlock,
  .cb_size = sizeof(ledTaskControlBlock),
  .stack_mem = &ledTaskBuffer[0],
  .stack_size = sizeof(ledTaskBuffer),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for flashTask */
osThreadId_t flashTaskHandle;
uint32_t flashTaskBuffer[ 1024 ];
osStaticThreadDef_t flashTaskControlBlock;
const osThreadAttr_t flashTask_attributes = {
  .name = "flashTask",
  .cb_mem = &flashTaskControlBlock,
  .cb_size = sizeof(flashTaskControlBlock),
  .stack_mem = &flashTaskBuffer[0],
  .stack_size = sizeof(flashTaskBuffer),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for commTask */
osThreadId_t commTaskHandle;
uint32_t commTaskBuffer[ 1024 ];
osStaticThreadDef_t commTaskControlBlock;
const osThreadAttr_t commTask_attributes = {
  .name = "commTask",
  .cb_mem = &commTaskControlBlock,
  .cb_size = sizeof(commTaskControlBlock),
  .stack_mem = &commTaskBuffer[0],
  .stack_size = sizeof(commTaskBuffer),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for motorControl */
osThreadId_t motorControlHandle;
uint32_t motorControlBuffer[ 256 ];
osStaticThreadDef_t motorControlControlBlock;
const osThreadAttr_t motorControl_attributes = {
  .name = "motorControl",
  .cb_mem = &motorControlControlBlock,
  .cb_size = sizeof(motorControlControlBlock),
  .stack_mem = &motorControlBuffer[0],
  .stack_size = sizeof(motorControlBuffer),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for FlashMutex */
osMutexId_t FlashMutexHandle;
osStaticMutexDef_t FlashMutexControlBlock;
const osMutexAttr_t FlashMutex_attributes = {
  .name = "FlashMutex",
  .cb_mem = &FlashMutexControlBlock,
  .cb_size = sizeof(FlashMutexControlBlock),
};
/* Definitions for MotorDriverMutex */
osMutexId_t MotorDriverMutexHandle;
osStaticMutexDef_t MotorDriverMutexControlBlock;
const osMutexAttr_t MotorDriverMutex_attributes = {
  .name = "MotorDriverMutex",
  .cb_mem = &MotorDriverMutexControlBlock,
  .cb_size = sizeof(MotorDriverMutexControlBlock),
};
/* Definitions for flashSPISemaphore */
osSemaphoreId_t flashSPISemaphoreHandle;
osStaticSemaphoreDef_t flashSPISemaphoreControlBlock;
const osSemaphoreAttr_t flashSPISemaphore_attributes = {
  .name = "flashSPISemaphore",
  .cb_mem = &flashSPISemaphoreControlBlock,
  .cb_size = sizeof(flashSPISemaphoreControlBlock),
};
/* Definitions for driverSPISemaphore */
osSemaphoreId_t driverSPISemaphoreHandle;
osStaticSemaphoreDef_t driverSPISemaphoreControlBlock;
const osSemaphoreAttr_t driverSPISemaphore_attributes = {
  .name = "driverSPISemaphore",
  .cb_mem = &driverSPISemaphoreControlBlock,
  .cb_size = sizeof(driverSPISemaphoreControlBlock),
};
/* Definitions for commDataAvailable */
osSemaphoreId_t commDataAvailableHandle;
osStaticSemaphoreDef_t commDataAvailableControlBlock;
const osSemaphoreAttr_t commDataAvailable_attributes = {
  .name = "commDataAvailable",
  .cb_mem = &commDataAvailableControlBlock,
  .cb_size = sizeof(commDataAvailableControlBlock),
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartLedTask(void *argument);
void StartFlashTask(void *argument);
void StartCommTask(void *argument);
void StartMotorControlTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of FlashMutex */
  FlashMutexHandle = osMutexNew(&FlashMutex_attributes);

  /* creation of MotorDriverMutex */
  MotorDriverMutexHandle = osMutexNew(&MotorDriverMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of flashSPISemaphore */
  flashSPISemaphoreHandle = osSemaphoreNew(1, 0, &flashSPISemaphore_attributes);

  /* creation of driverSPISemaphore */
  driverSPISemaphoreHandle = osSemaphoreNew(1, 0, &driverSPISemaphore_attributes);

  /* creation of commDataAvailable */
  commDataAvailableHandle = osSemaphoreNew(1, 0, &commDataAvailable_attributes);

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
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of ledTask */
  ledTaskHandle = osThreadNew(StartLedTask, NULL, &ledTask_attributes);

  /* creation of flashTask */
  flashTaskHandle = osThreadNew(StartFlashTask, NULL, &flashTask_attributes);

  /* creation of commTask */
  commTaskHandle = osThreadNew(StartCommTask, NULL, &commTask_attributes);

  /* creation of motorControl */
  motorControlHandle = osThreadNew(StartMotorControlTask, NULL, &motorControl_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
PS01Status_t status = {0};
uint32_t abs_pos = 0;
float encoder_degs = 0.0f;
uint32_t full_rotation = 0;
uint32_t tim_cnt = 0;
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Reset */

  uint8_t buf[2] = {0,0};

  HAL_SPI_Transmit(&hspi1, buf, 2, 200);

  HAL_GPIO_WritePin(DRV_RST_GPIO_Port, DRV_RST_Pin, 0);
  osDelay(1);
  HAL_GPIO_WritePin(DRV_RST_GPIO_Port, DRV_RST_Pin, 1);
  osDelay(100);

  StepperBank_t bank;
  Stepper_t motors[N_JOINTS];
  bank.motors = motors;
  RARM_SimpleConfig_t conf;

  RARM_SetBank(&bank);

  //ps01SetParam_chain(MAX_SPEED, 200);
  //uint32_t spd = ps01GetParam_chain(MAX_SPEED);

  uint8_t i = 0;

  HAL_GPIO_WritePin(BK1_GPIO_Port, BK1_Pin, 1);
  HAL_GPIO_WritePin(BK2_GPIO_Port, BK2_Pin, 1);

  // J1
  conf.steps_rev = 200;
  conf.reduction_ratio = 3;
  conf.min_degs = -90;
  conf.max_degs = 90;

  // steps/s or steps/s²
  conf.acceleration = 300;
  conf.deceleration = 300;
  conf.fullstep_speed = 20000;
  conf.max_speed = 100;
  conf.min_speed = 0;

  // volts
  conf.hold_voltage = 2.5;
  conf.run_acc_dec_voltage = 3.5;
  conf.supply_voltage = 24;

  // amps
  conf.oc_threshold = 40;
  conf.stall_threshold = 20;

  conf.OVERCURRENT_SD = OC_NOSHUTDOWN;
  conf.VSCOMP = VSCOMP_DISABLE;
  conf.STEP_MODE = SM_128_MICROSTEP;

  RARM_SetConfig(0, &conf);

  // J2
  conf.steps_rev = 200;
  conf.reduction_ratio = 50;
  conf.min_degs = -70;
  conf.max_degs = 100;

  // steps/s or steps/s²
  conf.acceleration = 800;
  conf.deceleration = 800;
  conf.fullstep_speed = 20000;
  conf.max_speed = 800;
  conf.min_speed = 0;

  // volts
  conf.hold_voltage = 1.5;
  conf.run_acc_dec_voltage = 1.8;
  conf.supply_voltage = 24;

  // amps
  conf.oc_threshold = 40;
  conf.stall_threshold = 20;

  conf.OVERCURRENT_SD = OC_NOSHUTDOWN;
  conf.VSCOMP = VSCOMP_DISABLE;
  conf.STEP_MODE = SM_128_MICROSTEP;

  mot_bank->active = 1;
  ps01SetParam_chain(ST_SLP,      0x19); // starting slope ~0.038%/step/s
  ps01SetParam_chain(FN_SLP_ACC,  0x79); // acceleration slope
  ps01SetParam_chain(FN_SLP_DEC,  0x79); // deceleration slope

  RARM_SetConfig(5, &conf);

  // J4
  conf.steps_rev = 200;
  conf.reduction_ratio = 5;
  conf.min_degs = -90;
  conf.max_degs = 90;

  // steps/s or steps/s²
  conf.acceleration = 400;
  conf.deceleration = 400;
  conf.fullstep_speed = 20000;
  conf.max_speed = 400;
  conf.min_speed = 0;

  // volts
  conf.hold_voltage = 1.5;
  conf.run_acc_dec_voltage = 2.34;
  conf.supply_voltage = 24;

  // amps
  conf.oc_threshold = 40;
  conf.stall_threshold = 20;

  conf.OVERCURRENT_SD = OC_NOSHUTDOWN;
  conf.VSCOMP = VSCOMP_DISABLE;
  conf.STEP_MODE = SM_16_MICROSTEP;

  mot_bank->active = 1;
  ps01SetParam_chain(ST_SLP,      0x89); // starting slope ~0.038%/step/s
  ps01SetParam_chain(FN_SLP_ACC,  0x79); // acceleration slope
  ps01SetParam_chain(FN_SLP_DEC,  0x79); // deceleration slope

  RARM_SetConfig(1, &conf);

  // J5
  conf.steps_rev = 200;
  conf.reduction_ratio = 5;
  conf.min_degs = -90;
  conf.max_degs = 90;

  // steps/s or steps/s²
  conf.acceleration = 400;
  conf.deceleration = 400;
  conf.fullstep_speed = 20000;
  conf.max_speed = 400;
  conf.min_speed = 0;

  // volts
  conf.hold_voltage = 1.5;
  conf.run_acc_dec_voltage = 2.34;
  conf.supply_voltage = 24;

  // amps
  conf.oc_threshold = 40;
  conf.stall_threshold = 20;

  conf.OVERCURRENT_SD = OC_NOSHUTDOWN;
  conf.VSCOMP = VSCOMP_DISABLE;
  conf.STEP_MODE = SM_16_MICROSTEP;

  mot_bank->active = 3;
  ps01SetParam_chain(ST_SLP,      0x89); // starting slope ~0.038%/step/s
  ps01SetParam_chain(FN_SLP_ACC,  0x79); // acceleration slope
  ps01SetParam_chain(FN_SLP_DEC,  0x79); // deceleration slope

  RARM_SetConfig(3, &conf);

  // J3
  conf.steps_rev = 200;
  conf.reduction_ratio = 50;
  conf.min_degs = -140;
  conf.max_degs = 140;

  // steps/s or steps/s²
  conf.acceleration = 1000;
  conf.deceleration = 1000;
  conf.fullstep_speed = 20000;
  conf.max_speed = 1000;
  conf.min_speed = 0;

  // volts
  conf.hold_voltage = 3.5;
  conf.run_acc_dec_voltage = 3.5;
  conf.supply_voltage = 24;

  // amps
  conf.oc_threshold = 40;
  conf.stall_threshold = 20;

  conf.OVERCURRENT_SD = OC_NOSHUTDOWN;
  conf.VSCOMP = VSCOMP_DISABLE;
  conf.STEP_MODE = SM_128_MICROSTEP;

  mot_bank->active = 5;
  ps01SetParam_chain(ST_SLP,      0x89); // starting slope ~0.038%/step/s
  ps01SetParam_chain(FN_SLP_ACC,  0x79); // acceleration slope
  ps01SetParam_chain(FN_SLP_DEC,  0x79); // deceleration slope

  RARM_SetConfig(4, &conf);

  // J6
  conf.steps_rev = 200;
  conf.reduction_ratio = 50;
  conf.min_degs = -120;
  conf.max_degs = 120;

  // steps/s or steps/s²
  conf.acceleration = 4000;
  conf.deceleration = 4000;
  conf.fullstep_speed = 20000;
  conf.max_speed = 1000;
  conf.min_speed = 0;

  // volts
  conf.hold_voltage = 1.5;
  conf.run_acc_dec_voltage = 2.34;
  conf.supply_voltage = 24;

  // amps
  conf.oc_threshold = 40;
  conf.stall_threshold = 20;

  conf.OVERCURRENT_SD = OC_NOSHUTDOWN;
  conf.VSCOMP = VSCOMP_DISABLE;
  conf.STEP_MODE = SM_128_MICROSTEP;

  RARM_SetConfig(2, &conf);

  osDelay(100);

  
  /*if (joint[1] == '2')
      joint[1] = '5';
    else if (joint[1] == '3')
      joint[1] = '4';
    else if (joint[1] == '4')
      joint[1] = '1';
    else if (joint[1] == '5')
      joint[1] = '5';
    else if (joint[1] == '6')
      joint[1] = '2';
    else if (joint[1] == '1')
      joint[1] = '0';*/


  // DRILL

  /*conf.steps_rev = 200;
  conf.reduction_ratio = 1;
  conf.min_degs = -140;
  conf.max_degs = 140;

  // steps/s or steps/s²
  conf.acceleration = 200;
  conf.deceleration = 1000;
  conf.fullstep_speed = 20000;
  conf.max_speed = 1000;
  conf.min_speed = 0;

  // volts
  conf.hold_voltage = 2.5;
  conf.run_acc_dec_voltage = 3.5;
  conf.supply_voltage = 24;

  // amps
  conf.oc_threshold = 40;
  conf.stall_threshold = 20;

  conf.OVERCURRENT_SD = OC_NOSHUTDOWN;
  conf.VSCOMP = VSCOMP_DISABLE;
  conf.STEP_MODE = SM_128_MICROSTEP;

  mot_bank->active = 0;
  ps01SetParam_chain(ST_SLP,      0x89); // starting slope ~0.038%/step/s
  ps01SetParam_chain(FN_SLP_ACC,  0x89); // acceleration slope
  ps01SetParam_chain(FN_SLP_DEC,  0x89); // deceleration slope

  RARM_SetConfig(0, &conf);*/


  osDelay(osWaitForever);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

  //RARM_MoveDegrees(J1_INDEX, CLOCKWISE, 45);


  /* Infinite loop */
  for(;;)
  {
    // random encoder code
    tim_cnt = TIM2->CNT;
    encoder_degs = ((TIM2->CNT > 4000) ? 4000 : TIM2->CNT) / 4000.0f * 360.0f;
    if (TIM2->CNT > 4000)
    {
      full_rotation++;
      TIM2->CNT -= 4000;
    }
    encoder_degs += full_rotation * 360;
    /*if (!HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin)/)
    {
      //ps01SoftStop_chain(&bank);
      //abs_pos = ps01GetParam_chain(ABS_POS);
      ps01GoHome_chain(&bank);
      osDelay(osWaitForever);
      //ps01MoveDegrees_chain(0, 7200);
    }*/
    //status = (PS01Status_t)ps01GetStatus_chain(&bank);
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartLedTask */
/**
* @brief Function implementing the ledTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLedTask */
void StartLedTask(void *argument)
{
  /* USER CODE BEGIN StartLedTask */
  /* Infinite loop */
  for(;;)
  {
    //HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    osDelay(50);
  }
  /* USER CODE END StartLedTask */
}

/* USER CODE BEGIN Header_StartFlashTask */
/**
* @brief Function implementing the flashTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartFlashTask */
void StartFlashTask(void *argument)
{
  /* USER CODE BEGIN StartFlashTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartFlashTask */
}

/* USER CODE BEGIN Header_StartCommTask */
/**
* @brief Function implementing the commTask thread.
* @param argument: Not used
* @retval None
*/
uint16_t recv_size;

// string must be correctly terminated, either by '\0' or the specified terminator
int32_t parseInt(char *str, char terminator)
{
  int32_t n = 0, i = 0;
  while (str[i] != '\0' && str[i] != terminator)
  {
    n += str[i] - '0';
    n *= 10;
  }
  return n;
}
/* USER CODE END Header_StartCommTask */
void StartCommTask(void *argument)
{
  /* USER CODE BEGIN StartCommTask */
  HAL_UARTEx_ReceiveToIdle_DMA(&huart2, (uint8_t*)comm_buffer, 256);
  char cmd[10 + 1];
  char joint[2 + 1];
  char arg[20 + 1];
  char cmd_len;

  char *arg_ptr;
  char txbuf[256];

  RARM_Gearbox_t grbx;
  grbx.mot1_index = 1;
  grbx.mot2_index = 3;
  grbx.rotation_reduction_ratio = 1.4;

  osDelay(100);

  /* Infinite loop */
  for(;;)
  {
    osSemaphoreAcquire(commDataAvailableHandle, osWaitForever);

    memset(cmd, 0, sizeof(cmd));
    memset(joint, 0, sizeof(joint));
    memset(arg, 0, sizeof(arg));
    // command
    if (!strstr(comm_buffer, " ") && !strstr(comm_buffer, "z") && !strstr(comm_buffer, "K"))
    {
      HAL_UART_Transmit(&huart2, (uint8_t*)"STOPPING ALL MOTORS\n", 20, 200);
      for (uint8_t i = 0; i < N_JOINTS; i++)
        RARM_SoftBrake(i);
      continue;
    }
    cmd_len = strstr(comm_buffer, " ") - comm_buffer;
    strncpy(cmd, comm_buffer, cmd_len);

    // joint
    char *joint_ptr = comm_buffer + cmd_len + 1;
    joint[0] = joint_ptr[0];
    joint[1] = joint_ptr[1];

    if (joint[0] != 'j' && !strstr(comm_buffer, "z") && !strstr(comm_buffer, "K"))
    {
      HAL_UART_Transmit(&huart2, (uint8_t*)"STOPPING ALL MOTORS\n", 20, 200);
      for (uint8_t i = 0; i < N_JOINTS; i++)
        RARM_SoftBrake(i);
      continue;
    }

    // argument
    arg_ptr = joint_ptr + 3;
    strncpy(arg, arg_ptr, sizeof(arg) - 1);

    snprintf(txbuf, 256, "cmd: %s; joint: %s; arg: %s\n", cmd, joint, arg);

    if (joint[1] == '2')
      joint[1] = '5';
    else if (joint[1] == '3')
      joint[1] = '4';
    else if (joint[1] == '4')
      joint[1] = '1';
    else if (joint[1] == '5')
      joint[1] = '5';
    else if (joint[1] == '6')
      joint[1] = '2';
    else if (joint[1] == '1')
      joint[1] = '0';

    if (strnstr(comm_buffer, "run", recv_size))
    {
      uint8_t dir;
      uint16_t speed;
      char *end;

      dir = (uint8_t)strtoul(arg, &end, 10);
      speed = (uint16_t)strtoul(end, NULL, 10);

      snprintf(txbuf, 256, "running %s: %d RPM in direction %d\n", joint, speed, dir);
      HAL_UART_Transmit(&huart2, (uint8_t*)txbuf, strlen(txbuf), 200);

      RARM_Run(joint[1] - '0', dir, speed);
    }
    else if (strnstr(comm_buffer, "move", recv_size))
    {
      uint16_t degrees;

      degrees = (uint16_t)strtoul(arg, NULL, 10);
    
      snprintf(txbuf, 256, "moving %s: %d degrees\n", joint, degrees);
      HAL_UART_Transmit(&huart2, (uint8_t*)txbuf, strlen(txbuf), 200);

      RARM_MoveDegrees(joint[1] -'0', degrees);
    }
    else if (strnstr(comm_buffer, "movgr", recv_size))
    {
      int32_t degrees = (int32_t)strtoul(arg, NULL, 10);

      snprintf(txbuf, 256, "moving gearbox: %d degrees\n", degrees);
      HAL_UART_Transmit(&huart2, (uint8_t*)txbuf, strlen(txbuf), 200);

      RARM_GearboxMoveDegrees(&grbx, degrees);
    }
    else if (strnstr(comm_buffer, "rotgr", recv_size))
    {
      int32_t degrees = (int32_t)strtoul(arg, NULL, 10);

      snprintf(txbuf, 256, "moving gearbox: %d degrees\n", degrees);
      HAL_UART_Transmit(&huart2, (uint8_t*)txbuf, strlen(txbuf), 200);

      RARM_GearboxRotateDegrees(&grbx, degrees);
    }
    else if (strnstr(comm_buffer, "z", recv_size))
    {
      RARM_Gearbox_t grbx;
      grbx.mot1_index = 1;
      grbx.mot2_index = 3;
      grbx.rotation_reduction_ratio = 1.4;
      RARM_MoveDegrees(5, -113);
      RARM_MoveDegrees(4, 30);
      //RARM_GearboxRotateDegrees(&grbx, 40);
    }
    else if (strnstr(comm_buffer, "K", recv_size))
    {
      RARM_Gearbox_t grbx;
      grbx.mot1_index = 1;
      grbx.mot2_index = 3;
      grbx.rotation_reduction_ratio = 1.4;

      RARM_MoveDegrees(5, 70);
      RARM_MoveDegrees(4, -30);
      //RARM_GearboxRotateDegrees(&grbx,-40);
      osDelay(5000);
      RARM_MoveDegrees(2, 120);
    }
    osDelay(1);
  }
  /* USER CODE END StartCommTask */
}

/* USER CODE BEGIN Header_StartMotorControlTask */
#define ENCODER_DEADZONE 1 // degrees
/**
* @brief Function implementing the motorControl thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMotorControlTask */
void StartMotorControlTask(void *argument)
{
  /* USER CODE BEGIN StartMotorControlTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(osWaitForever);
    
    for (uint8_t i = 0; i < N_JOINTS; i++)
    {
      mot_bank->active = i;
      uint16_t drv_status = ps01GetStatus_chain();
      uint16_t mot_status = drv_status & STATUS_MASK_MOT_STATUS;
      if (mot_status == MOT_STATUS_STOPPED)
      {
        // motor is stopped. compare the driver position to the encoder position
        // if there is a difference, send command to get the motor to the desired position
        // example: driver position 90°, encoder position 70° (= motor axle). we set the driver position
        // to 70° (which is the real position) and send a move command of 20°
        int32_t drv_position = RARM_GetPositionDegrees(i);
        int32_t enc_position = 0; // implement encoder position logic
        int32_t delta = drv_position - enc_position;
        if (abs(delta) > ENCODER_DEADZONE)
        {
          // enc_position is in degrees. first convert it to steps
          //int32_t real_position getStepsFromAngle(enc_position, mot_bank.m);
          ps01SetParam_chain(ABS_POS, enc_position); // set real position into driver
          RARM_MoveDegrees(i, delta);
        }
      }
    }
    ps01WaitBusy_chain();
    osDelay(1);
  }
  /* USER CODE END StartMotorControlTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
  if (hspi->Instance == FLASH_SPI_HANDLE.Instance) osSemaphoreRelease(spiSemaphore);
  else if (hspi->Instance == PS01_SPI_HANDLE.Instance) osSemaphoreRelease(ps01SPISemaphore);
}
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {
  if (hspi->Instance == FLASH_SPI_HANDLE.Instance) osSemaphoreRelease(spiSemaphore);
}
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
  if (hspi->Instance == PS01_SPI_HANDLE.Instance) osSemaphoreRelease(ps01SPISemaphore);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  recv_size = Size;
  if (Size < sizeof(comm_buffer))
    comm_buffer[Size] = '\0';

  osSemaphoreRelease(commDataAvailableHandle);
  HAL_UARTEx_ReceiveToIdle_DMA(&huart2, (uint8_t*)comm_buffer, 256);
}
/* USER CODE END Application */

