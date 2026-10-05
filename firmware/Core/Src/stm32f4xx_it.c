/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_spi1_rx;
extern DMA_HandleTypeDef hdma_spi1_tx;
extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim8;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim14;

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 stream5 global interrupt.
  */
void DMA1_Stream5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream5_IRQn 0 */

  /* USER CODE END DMA1_Stream5_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart2_rx);
  /* USER CODE BEGIN DMA1_Stream5_IRQn 1 */

  /* USER CODE END DMA1_Stream5_IRQn 1 */
}

/**
  * @brief This function handles SPI1 global interrupt.
  */
void SPI1_IRQHandler(void)
{
  /* USER CODE BEGIN SPI1_IRQn 0 */

  /* USER CODE END SPI1_IRQn 0 */
  HAL_SPI_IRQHandler(&hspi1);
  /* USER CODE BEGIN SPI1_IRQn 1 */

  /* USER CODE END SPI1_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}

/**
  * @brief This function handles TIM8 trigger and commutation interrupts and TIM14 global interrupt.
  */
void TIM8_TRG_COM_TIM14_IRQHandler(void)
{
  /* HAL timebase now uses SysTick (no TIM14). */
}

/**
  * @brief This function handles DMA2 stream0 global interrupt.
  */
void DMA2_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream0_IRQn 0 */

  /* USER CODE END DMA2_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi1_rx);
  /* USER CODE BEGIN DMA2_Stream0_IRQn 1 */

  /* USER CODE END DMA2_Stream0_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream3 global interrupt.
  */
void DMA2_Stream3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream3_IRQn 0 */

  /* USER CODE END DMA2_Stream3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi1_tx);
  /* USER CODE BEGIN DMA2_Stream3_IRQn 1 */

  /* USER CODE END DMA2_Stream3_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/* USER CODE BEGIN NCAP */
/* FreeRTOS kernel exception-vector glue.

   Defensive default so this file also compiles if the Makefile ever forgets
   -DNCAP_FREERTOS=...; bare metal stays the default personality. */
#ifndef NCAP_FREERTOS
#define NCAP_FREERTOS 0
#endif

#if NCAP_FREERTOS == 1
/* start-up linkage for the kernel's exception vectors. See the SVC/PendSV note
   below for why those two come from port.c and not from here. */
#include "FreeRTOS.h"
#include "task.h"

/* SVC_Handler / PendSV_Handler.

   This project's FreeRTOSConfig.h:175-176 already does the CMSIS aliasing
   itself:
       #define vPortSVCHandler    SVC_Handler
       #define xPortPendSVHandler PendSV_Handler
   so the preprocessor renames port.c's own definitions
   (portable/GCC/ARM_CM4F/port.c:242 and :431) to SVC_Handler and PendSV_Handler.
   The kernel therefore ALREADY supplies both vectors, and defining them again
   here is not merely redundant - it is a duplicate-symbol link error.

   The #if below is the escape hatch for a vanilla FreeRTOSConfig.h that does
   not alias (where port.c really does define vPortSVCHandler/
   xPortPendSVHandler and nothing binds them to the CMSIS names). It compiles
   out here. Kept because startup_stm32f446xx.s:270,276 only .weak-aliases
   SVC_Handler and PendSV_Handler to Default_Handler, so if the aliasing in
   FreeRTOSConfig.h were ever dropped, the first task switch would land in
   Default_Handler's fault-loop instead of the context switch.
   The vendored V10.3.1 tree declares none of these three in any header, so the
   forward declarations below are required, not decorative. */
#if !defined(vPortSVCHandler) && !defined(xPortPendSVHandler)
void vPortSVCHandler(void) __attribute__((naked));
void xPortPendSVHandler(void) __attribute__((naked));
void SVC_Handler(void)    { vPortSVCHandler(); }
void PendSV_Handler(void) { xPortPendSVHandler(); }
#endif

/* SysTick is the one vector the config does NOT alias (FreeRTOSConfig.h:180
   sets USE_CUSTOM_SYSTICK_HANDLER_IMPLEMENTATION 0 and only comments that
   SysTick "comes from NVIC"), so this file owns it. xPortSysTickHandler is
   declared in no header of the vendored tree - only defined in port.c:488 -
   hence the declaration. */
void xPortSysTickHandler(void);
#endif /* NCAP_FREERTOS == 1 */

void SysTick_Handler(void)
{
#if NCAP_FREERTOS == 1
  xPortSysTickHandler();
#endif
  /* NOTE: HAL_IncTick() looks redundant right next to xPortSysTickHandler()
     above - it is NOT, and deleting it silently kills the transport.

     xPortSysTickHandler() only advances the FreeRTOS tick; it does not touch
     uwTick. HAL's tick is a separate counter and it is load-bearing:
       - w5500_transport.c, cubemx_transport_read_w5500() (~lines 178-197),
         spins on getSn_RX_RSR() and escapes via
         (int32_t)(HAL_GetTick() - start) >= timeout. If uwTick freezes, every
         XRCE read instantly reports timeout, the session desyncs and the node
         goes quiet with no error message.
       - video_udp.c (~lines 403-420) paces its frame deadline with
         HAL_GetTick() / HAL_Delay().
     So both calls must run on every SysTick. Keep HAL_IncTick() here. */
  HAL_IncTick();
}
/* USER CODE END NCAP */
