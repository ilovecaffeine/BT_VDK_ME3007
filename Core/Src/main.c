/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"

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

/* USER CODE BEGIN PV */
GPIO_TypeDef* rowPorts[8] = {ROW0_GPIO_Port, ROW1_GPIO_Port, ROW2_GPIO_Port, ROW3_GPIO_Port,
                                  ROW4_GPIO_Port, ROW5_GPIO_Port, ROW6_GPIO_Port, ROW7_GPIO_Port};
uint16_t rowPins[8] = {ROW0_Pin, ROW1_Pin, ROW2_Pin, ROW3_Pin,
                           ROW4_Pin, ROW5_Pin, ROW6_Pin, ROW7_Pin};

GPIO_TypeDef* colPorts[8] = {COL0_GPIO_Port, COL1_GPIO_Port, COL2_GPIO_Port, COL3_GPIO_Port, COL4_GPIO_Port,
                                  COL5_GPIO_Port, COL6_GPIO_Port, COL7_GPIO_Port};
uint16_t colPins[8] = {COL0_Pin, COL1_Pin, COL2_Pin, COL3_Pin, COL4_Pin,
                           COL5_Pin, COL6_Pin, COL7_Pin};

const int MAX_LED_MATRIX = 8;
int index_led_matrix = 0; //0 TO 7
int charIndex = 0; //0 TO 3
uint8_t matrix_buffer[4][8] = {
    // Character 0: '+'
    {0x18, 0x18, 0x18, 0xFF, 0xFF, 0x18, 0x18, 0x18},

    // Character 1: '-'
    {0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18},

    // Character 2: 'x'
    {0x81, 0x42, 0x24, 0x18, 0x18, 0x24, 0x42, 0x81},

    // Character 3: '/'
    {0x18, 0x18, 0x18, 0xDB, 0xDB, 0x18, 0x18, 0x18}
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */




void controlLEDs(uint8_t num) {
    // Analyze each bit of the 8 LSBs and control the LEDs accordingly

    // Bit 0 (rightmost bit)
    if (num & 0x01) {   // Check if bit 0 is 1
        HAL_GPIO_WritePin(ROW0_GPIO_Port, ROW0_Pin, GPIO_PIN_RESET); //ledROW0_on
    } else {
        HAL_GPIO_WritePin(ROW0_GPIO_Port, ROW0_Pin, GPIO_PIN_SET); //ledROW0_off
    }
    // Bit 1
    if (num & 0x02) {   // Check if bit 1 is 1
        HAL_GPIO_WritePin(ROW1_GPIO_Port, ROW1_Pin, GPIO_PIN_RESET); //ledROW1_on
    } else {
        HAL_GPIO_WritePin(ROW1_GPIO_Port, ROW1_Pin, GPIO_PIN_SET); //ledROW1_off
    }

    // Bit 2
    if (num & 0x04) {   // Check if bit 2 is 1
        HAL_GPIO_WritePin(ROW2_GPIO_Port, ROW2_Pin, GPIO_PIN_RESET); //ledROW2_on
    } else {
        HAL_GPIO_WritePin(ROW2_GPIO_Port, ROW2_Pin, GPIO_PIN_SET); //ledROW2_off
    }

    // Bit 3
    if (num & 0x08) {   // Check if bit 3 is 1
        HAL_GPIO_WritePin(ROW3_GPIO_Port, ROW3_Pin, GPIO_PIN_RESET); //ledROW3_on
    } else {
        HAL_GPIO_WritePin(ROW3_GPIO_Port, ROW3_Pin, GPIO_PIN_SET); //ledROW3_off
    }
        // Bit 4
    if (num & 0x10) {
        HAL_GPIO_WritePin(ROW4_GPIO_Port, ROW4_Pin, GPIO_PIN_RESET); //ledROW4_on
    } else {
        HAL_GPIO_WritePin(ROW4_GPIO_Port, ROW4_Pin, GPIO_PIN_SET); //ledROW4_off
    }

    // Bit 5
    if (num & 0x20) {
        HAL_GPIO_WritePin(ROW5_GPIO_Port, ROW5_Pin, GPIO_PIN_RESET); //ledROW5_on
    } else {
        HAL_GPIO_WritePin(ROW5_GPIO_Port, ROW5_Pin, GPIO_PIN_SET); //ledROW5_off
    }

    // Bit 6
    if (num & 0x40) {
        HAL_GPIO_WritePin(ROW6_GPIO_Port, ROW6_Pin, GPIO_PIN_RESET); //ledROW6_on
    } else {
        HAL_GPIO_WritePin(ROW6_GPIO_Port, ROW6_Pin, GPIO_PIN_SET); //ledROW6_off
    }

    // Bit 7
    if (num & 0x80) {
        HAL_GPIO_WritePin(ROW7_GPIO_Port, ROW7_Pin, GPIO_PIN_RESET); //ledROW7_on
    } else {
        HAL_GPIO_WritePin(ROW7_GPIO_Port, ROW7_Pin, GPIO_PIN_SET); //ledROW7_off
    }
}

void offAllMatrixLED() {

    for (int row = 0; row < 8; row++) {
        HAL_GPIO_WritePin(rowPorts[row], rowPins[row], GPIO_PIN_RESET); // Turn off current row
    }


    for (int col = 0; col < 8; col++) {
        HAL_GPIO_WritePin(colPorts[col], colPins[col], GPIO_PIN_RESET); // Turn off current column
    }
}



void updateLEDMatrix(int charIndex, int rowIndex) {
    // Turn off all LEDs first
    offAllMatrixLED();

    // Set the appropriate column LED based on the charIndex
    if (charIndex >= 0 && charIndex < 4 && rowIndex >= 0 && rowIndex < 8) {
        // Set the specific column high
        switch (rowIndex) {
            case 0:
                HAL_GPIO_WritePin(COL0_GPIO_Port, COL0_Pin, GPIO_PIN_SET);
                break;
            case 1:
                HAL_GPIO_WritePin(COL1_GPIO_Port, COL1_Pin, GPIO_PIN_SET);
                break;
            case 2:
                HAL_GPIO_WritePin(COL2_GPIO_Port, COL2_Pin, GPIO_PIN_SET);
                break;
            case 3:
                HAL_GPIO_WritePin(COL3_GPIO_Port, COL3_Pin, GPIO_PIN_SET);
                break;
            case 4:
                HAL_GPIO_WritePin(COL4_GPIO_Port, COL4_Pin, GPIO_PIN_SET);
                break;
            case 5:
                HAL_GPIO_WritePin(COL5_GPIO_Port, COL5_Pin, GPIO_PIN_SET);
                break;
            case 6:
                HAL_GPIO_WritePin(COL6_GPIO_Port, COL6_Pin, GPIO_PIN_SET);
                break;
            case 7:
                HAL_GPIO_WritePin(COL7_GPIO_Port, COL7_Pin, GPIO_PIN_SET);
                break;
            default:
                break;
        }

        // Control the LEDs for the specific row based on the character matrix
        controlLEDs(matrix_buffer[charIndex][rowIndex]);
    }
}





while (1)
  {

      updateLEDMatrix(charIndex,index_led_matrix);

      if (index_led_matrix == 7) {
    	  charIndex = (charIndex + 1) % 4;
      }

      index_led_matrix = (index_led_matrix + 1) % MAX_LED_MATRIX;
      HAL_Delay(100);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, COL1_Pin|COL2_Pin|ROW2_Pin|ROW3_Pin
                          |ROW4_Pin|ROW5_Pin|ROW6_Pin|ROW7_Pin
                          |COL3_Pin|COL4_Pin|COL5_Pin|COL6_Pin
                          |COL7_Pin|ROW0_Pin|ROW1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(COL0_GPIO_Port, COL0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : COL1_Pin COL2_Pin ROW2_Pin ROW3_Pin
                           ROW4_Pin ROW5_Pin ROW6_Pin ROW7_Pin
                           COL3_Pin COL4_Pin COL5_Pin COL6_Pin
                           COL7_Pin ROW0_Pin ROW1_Pin */
  GPIO_InitStruct.Pin = COL1_Pin|COL2_Pin|ROW2_Pin|ROW3_Pin
                          |ROW4_Pin|ROW5_Pin|ROW6_Pin|ROW7_Pin
                          |COL3_Pin|COL4_Pin|COL5_Pin|COL6_Pin
                          |COL7_Pin|ROW0_Pin|ROW1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : COL0_Pin */
  GPIO_InitStruct.Pin = COL0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(COL0_GPIO_Port, &GPIO_InitStruct);

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
