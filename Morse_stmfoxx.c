/**
  ******************************************************************************
  * @file    stm32f0xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32f0xx_it.h"
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

/* USER CODE BEGIN EV */
extern TIM_HandleTypeDef htim6;
extern UART_HandleTypeDef huart1;
extern volatile uint16_t morse_sram_index;
volatile uint8_t button_pressed = 0;

volatile uint32_t release_time = 0;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M0 Processor Interruption and Exception Handlers          */
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
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVC_IRQn 0 */

  /* USER CODE END SVC_IRQn 0 */
  /* USER CODE BEGIN SVC_IRQn 1 */

  /* USER CODE END SVC_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f0xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line 0 and 1 interrupts.
  */


void EXTI0_1_IRQHandler(void)
{

    if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_0) != RESET)
    {
        __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_0);

        if (button_pressed == 0)
        {
            // Button is being pressed
            __HAL_TIM_SET_COUNTER(&htim6, 0);
            HAL_TIM_Base_Start(&htim6);
            button_pressed = 1;
        }
        else
        {
            // Button is being released stop timer and take the value
            HAL_TIM_Base_Stop(&htim6);
            release_time = __HAL_TIM_GET_COUNTER(&htim6);
            button_pressed = 0;

            char signal = '?';

            if (release_time <= 40) {
                // Skip noisy signals
                char ignore_msg[64];
            	sprintf(ignore_msg, "Signal: - | Duration: %lu ms | Index: %u | IGNORED\r\n", release_time, morse_sram_index);
                return;
            } else if (release_time < 200) {
                signal = '.';
            } else if (release_time < 600) {
                signal = '-';
            } else if (release_time < 1000) {
                signal = ' ';
            } else {
                signal = '/';
            }

            // Write to SRAM to store
            write_sram_char(morse_sram_index, signal);

            // Read back for verification
            char verify = read_sram_char(morse_sram_index);
            if (verify != signal) {
                char err_msg[64];
                sprintf(err_msg, "SRAM ERROR @%u: wrote %c, read %c\r\n", morse_sram_index, signal, verify);
                HAL_UART_Transmit(&huart1, (uint8_t*)err_msg, strlen(err_msg), 100);
            } else {
                char debug_msg[64];
                sprintf(debug_msg, "Signal: %c | Duration: %lu ms | Index: %u\r\n", signal, release_time, morse_sram_index);
                HAL_UART_Transmit(&huart1, (uint8_t*)debug_msg, strlen(debug_msg), 100);
            }

            morse_sram_index++; // increment the sram index to avoid over write
        }
    }

}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
