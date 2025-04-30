/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include <string.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart1;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

char morse_sequence[10];
uint8_t morse_index = 0;
uint8_t ir_monitoring_enabled = 0;
uint32_t last_ir_activity = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define TIMEOUT 100
#define SRAM_WRITE 0x02
#define SRAM_READ  0x03

// Morse buffer tracking
volatile uint16_t morse_sram_index = 0;
volatile uint8_t store_ready = 0;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */

void write_sram_char(uint16_t addr, char data);
char read_sram_char(uint16_t addr);
void replay_morse_sequence();
char decode_morse_sequence(char *sequence);
void decode_and_print_morse();
void delete_last_morse_char();

typedef struct {
    const char *morse;
    char ascii;
} MorseMap;

MorseMap morse_table[] = {
    {".-", 'A'}, {"-...", 'B'}, {"-.-.", 'C'}, {"-..", 'D'},
    {".", 'E'}, {"..-.", 'F'}, {"--.", 'G'}, {"....", 'H'},
    {"..", 'I'}, {".---", 'J'}, {"-.-", 'K'}, {".-..", 'L'},
    {"--", 'M'}, {"-.", 'N'}, {"---", 'O'}, {".--.", 'P'},
    {"--.-", 'Q'}, {".-.", 'R'}, {"...", 'S'}, {"-", 'T'},
    {"..-", 'U'}, {"...-", 'V'}, {".--", 'W'}, {"-..-", 'X'},
    {"-.--", 'Y'}, {"--..", 'Z'},
    {"-----", '0'}, {".----", '1'}, {"..---", '2'}, {"...--", '3'},
    {"....-", '4'}, {".....", '5'}, {"-....", '6'}, {"--...", '7'},
    {"---..", '8'}, {"----.", '9'}
};


int main(void)
{

  /* USER CODE BEGIN 1 */


  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
 // HAL_GPIO_WritePin(nCS_SRAM_GPIO_Port, nCS_SRAM_Pin, 1);
 // HAL_GPIO_WritePin(NCS_MEMS_SPI_GPIO_Port, NCS_MEMS_SPI_Pin,1);

  __HAL_TIM_SET_PRESCALER(&htim6, 47999); //Timer16 PWM Initialization for LCD, Start timer here to activate refresh.
  __HAL_TIM_SET_AUTORELOAD(&htim6, 10000);

  char menu[] =
    "\r\n======== Morse Code Menu ========\r\n"
    "[p] Print stored Morse and decode\r\n"
    "[c] Clear Morse memory\r\n"
    "[r] Play Morse message via LEDs\r\n"
    "[d] Delete previous morse character\r\n"
    "[h] Show this menu\r\n"
    "=================================\r\n";

  HAL_UART_Transmit(&huart1, (uint8_t*)menu, strlen(menu), TIMEOUT);
  HAL_Delay(10); //This is dumb but SPI wasn't cool without it
  uint8_t cmd;


  HAL_Delay(10); // again, for spi
  write_sram_char(0, 0); // Dummy write to initialize SRAM
  morse_sram_index = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

      if (HAL_UART_Receive(&huart1, &cmd, 1, HAL_MAX_DELAY) == HAL_OK)
      {
          switch (cmd)
          {
              case 'p'://decode the morse message and print it
                  decode_and_print_morse();
                  break;

              case 'c': // clear the sram
                  for (uint16_t i = 0; i < morse_sram_index; i++)
                      write_sram_char(i, 0);  // optional: overwrite with nulls
                  morse_sram_index = 0;
                  HAL_UART_Transmit(&huart1, (uint8_t*)"Memory cleared.\r\n", 17, TIMEOUT);
                  break;

              case 'r'://play the morse sequence on LEDs

            	  replay_morse_sequence();
                  break;

              case 'd':  // backspace/delete only deletes the previous morse char, not the translated letter
            	  delete_last_morse_char();
                  break;

              case 'h':
              default:
                  HAL_UART_Transmit(&huart1, (uint8_t*)menu, strlen(menu), TIMEOUT);
                  break;
          }
      }
  }


  /* USER CODE END 3 */
}


// SRAM WRITE
void write_sram_char(uint16_t addr, char data)
{
    uint8_t tx[4] = {
        SRAM_WRITE,
        (uint8_t)(addr >> 8),
        (uint8_t)(addr & 0xFF),
        (uint8_t)data
    };
    HAL_GPIO_WritePin(nCS_SRAM_GPIO_Port, nCS_SRAM_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, tx, 4, TIMEOUT);
    HAL_GPIO_WritePin(nCS_SRAM_GPIO_Port, nCS_SRAM_Pin, GPIO_PIN_SET);
}

// SRAM READ
char read_sram_char(uint16_t addr)
{
    uint8_t tx[4] = {
        SRAM_READ,
        (uint8_t)(addr >> 8),
        (uint8_t)(addr & 0xFF),
        0x00
    };
    uint8_t rx[4] = {0};

    HAL_GPIO_WritePin(nCS_SRAM_GPIO_Port, nCS_SRAM_Pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi2, tx, rx, 4, TIMEOUT);
    HAL_GPIO_WritePin(nCS_SRAM_GPIO_Port, nCS_SRAM_Pin, GPIO_PIN_SET);

    return (char)rx[3];
}

void replay_morse_sequence() {
    for (uint16_t i = 0; i < morse_sram_index; i++) {
        char c = read_sram_char(i);

        switch (c) {
            case '.':
                // Blink PC8 for dot
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
                HAL_Delay(200);
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
                HAL_Delay(200); // Gap between signals
                break;

            case '-':
                // Blink PC9 for dash
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
                HAL_Delay(600);
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
                HAL_Delay(200); // Gap between signals
                break;

            case ' ':
                HAL_Delay(400); // Space between letters
                break;

            case '/':
                HAL_Delay(1000); // Space between words
                break;

            default:
                break;
        }
    }
    HAL_UART_Transmit(&huart1, (uint8_t*)"Replaying Morse through LEDs\r\n", 30, TIMEOUT);
}

char decode_morse_sequence(char *sequence)
{
    for (int i = 0; i < sizeof(morse_table)/sizeof(MorseMap); i++) {
        if (strcmp(sequence, morse_table[i].morse) == 0) {
            return morse_table[i].ascii;
        }
    }
    return '?'; // If not found
}

void decode_and_print_morse()
{
    HAL_UART_Transmit(&huart1, (uint8_t*)"\r\nStored Morse: ", 16, TIMEOUT);
    for (uint16_t i = 0; i < morse_sram_index; i++) {
        char c = read_sram_char(i);
        HAL_UART_Transmit(&huart1, (uint8_t*)&c, 1, TIMEOUT);
    }
    HAL_UART_Transmit(&huart1, (uint8_t*)"\r\n", 2, TIMEOUT);
    HAL_UART_Transmit(&huart1, (uint8_t*)"\r\nDecoded Text: ", 16, TIMEOUT);

    char buffer[10] = {0};
    uint8_t idx = 0;

    for (uint16_t i = 0; i < morse_sram_index; i++) {
        char c = read_sram_char(i);

        if (c == '.' || c == '-') {
            buffer[idx++] = c;
        }
        else if (c == ' ' || c == '/') {
            if (idx > 0) {
                buffer[idx] = '\0';
                char decoded = decode_morse_sequence(buffer);
                HAL_UART_Transmit(&huart1, (uint8_t*)&decoded, 1, TIMEOUT);
                idx = 0;
            }
            if (c == '/') {
                char space = ' ';
                HAL_UART_Transmit(&huart1, (uint8_t*)&space, 1, TIMEOUT);
            }
        }
    }

    // Flush if leftover
    if (idx > 0) {
        buffer[idx] = '\0';
        char decoded = decode_morse_sequence(buffer);
        HAL_UART_Transmit(&huart1, (uint8_t*)&decoded, 1, TIMEOUT);
    }

    HAL_UART_Transmit(&huart1, (uint8_t*)"\r\n", 2, TIMEOUT);
}

void delete_last_morse_char() {
    if (morse_sram_index > 0) {
        morse_sram_index--;
        write_sram_char(morse_sram_index, 0);
        HAL_UART_Transmit(&huart1, (uint8_t*)"Last character deleted.\r\n", 26, TIMEOUT);
    } else {
        HAL_UART_Transmit(&huart1, (uint8_t*)"Nothing to delete.\r\n", 21, TIMEOUT);
    }
}
/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 47; // changed to take in time in a more manageable manner
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 0xFFFF;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 0;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 65535;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 57600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, NCS_MEMS_SPI_Pin|EXT_RESET_Pin|LD3_Pin|LD6_Pin
                          |LD4_Pin|LD5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(nCS_SRAM_GPIO_Port, nCS_SRAM_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : NCS_MEMS_SPI_Pin EXT_RESET_Pin LD3_Pin LD6_Pin
                           LD4_Pin LD5_Pin */
  GPIO_InitStruct.Pin = NCS_MEMS_SPI_Pin|EXT_RESET_Pin|LD3_Pin|LD6_Pin
                          |LD4_Pin|LD5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA2 PA3 PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF3_TSC;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF3_TSC;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : I2C2_SCL_Pin I2C2_SDA_Pin */
  GPIO_InitStruct.Pin = I2C2_SCL_Pin|I2C2_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF1_I2C2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : nCS_SRAM_Pin */
  GPIO_InitStruct.Pin = nCS_SRAM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(nCS_SRAM_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
