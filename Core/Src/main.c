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




  int hundreds = 0, tens = 0, ones = 0;
  int led_buffer[3] = {0, 0, 0};
  void updateBuffer()
  {
      led_buffer[0] = ones;
      led_buffer[1] = tens;
      led_buffer[2] = hundreds;

  }

    void off_all_SEG(void) {
        HAL_GPIO_WritePin(GPIOB, SEG_0_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, SEG_1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, SEG_2_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, SEG_3_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, SEG_4_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, SEG_5_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, SEG_6_Pin, GPIO_PIN_SET);
    }
    void display7SEG(int num) {
      if (num < 0 || num > 9) return; // Ensure the number is between 0 and 9
      switch (num) {
          case 0:
              HAL_GPIO_WritePin(GPIOB, SEG_0_Pin, GPIO_PIN_RESET);
              HAL_GPIO_WritePin(GPIOB, SEG_1_Pin, GPIO_PIN_RESET);
              HAL_GPIO_WritePin(GPIOB, SEG_2_Pin, GPIO_PIN_RESET);
              HAL_GPIO_WritePin(GPIOB, SEG_3_Pin, GPIO_PIN_RESET);
              HAL_GPIO_WritePin(GPIOB, SEG_4_Pin, GPIO_PIN_RESET);
              HAL_GPIO_WritePin(GPIOB, SEG_5_Pin, GPIO_PIN_RESET);
              HAL_GPIO_WritePin(GPIOB, SEG_6_Pin, GPIO_PIN_SET);
              break;
          case 1:
              HAL_GPIO_WritePin(GPIOB, SEG_0_Pin, GPIO_PIN_SET);
              HAL_GPIO_WritePin(GPIOB, SEG_1_Pin, GPIO_PIN_RESET);
              HAL_GPIO_WritePin(GPIOB, SEG_2_Pin, GPIO_PIN_RESET);
              HAL_GPIO_WritePin(GPIOB, SEG_3_Pin, GPIO_PIN_SET);
              HAL_GPIO_WritePin(GPIOB, SEG_4_Pin, GPIO_PIN_SET);
              HAL_GPIO_WritePin(GPIOB, SEG_5_Pin, GPIO_PIN_SET);
              HAL_GPIO_WritePin(GPIOB, SEG_6_Pin, GPIO_PIN_SET);
              break;
          case 2:
              HAL_GPIO_WritePin(GPIOB, SEG_0_Pin, GPIO_PIN_RESET);
              HAL_GPIO_WritePin(GPIOB, SEG_1_Pin, GPIO_PIN_RESET);
              HAL_GPIO_WritePin(GPIOB, SEG_2_Pin, GPIO_PIN_SET);
              HAL_GPIO_WritePin(GPIOB, SEG_3_Pin, GPIO_PIN_RESET);
              HAL_GPIO_WritePin(GPIOB, SEG_4_Pin, GPIO_PIN_RESET);
              HAL_GPIO_WritePin(GPIOB, SEG_5_Pin, GPIO_PIN_SET);
              HAL_GPIO_WritePin(GPIOB, SEG_6_Pin, GPIO_PIN_RESET);
              break;
          case 3:
              HAL_GPIO_WritePin(GPIOB, SEG_0_Pin, GPIO_PIN_RESET);
              HAL_GPIO_WritePin(GPIOB, SEG_1_Pin, GPIO_PIN_RESET);
              HAL_GPIO_WritePin(GPIOB, SEG_2_Pin, GPIO_PIN_RESET);
              HAL_GPIO_WritePin(GPIOB, SEG_3_Pin, GPIO_PIN_RESET);
              HAL_GPIO_WritePin(GPIOB, SEG_4_Pin, GPIO_PIN_SET);
              HAL_GPIO_WritePin(GPIOB, SEG_5_Pin, GPIO_PIN_SET);
              HAL_GPIO_WritePin(GPIOB, SEG_6_Pin, GPIO_PIN_RESET);
              break;
          case 4:
              HAL_GPIO_WritePin(GPIOB, SEG_0_Pin, GPIO_PIN_SET);
              HAL_GPIO_WritePin(GPIOB, SEG_1_Pin, GPIO_PIN_RESET);
              HAL_GPIO_WritePin(GPIOB, SEG_2_Pin, GPIO_PIN_RESET);
              HAL_GPIO_WritePin(GPIOB, SEG_3_Pin, GPIO_PIN_SET);
              HAL_GPIO_WritePin(GPIOB, SEG_4_Pin, GPIO_PIN_SET);
              HAL_GPIO_WritePin(GPIOB, SEG_5_Pin, GPIO_PIN_RESET);
              HAL_GPIO_WritePin(GPIOB, SEG_6_Pin, GPIO_PIN_RESET);
              break;
          case 5:
              HAL_GPIO_WritePin(GPIOB, SEG_0_Pin, GPIO_PIN_RESET);
              HAL_GPIO_WritePin(GPIOB, SEG_1_Pin, GPIO_PIN_SET);
              HAL_GPIO_WritePin(GPIOB, SEG_2_Pin, GPIO_PIN_RESET);
              HAL_GPIO_WritePin(GPIOB, SEG_3_Pin, GPIO_PIN_RESET);
              HAL_GPIO_WritePin(GPIOB, SEG_4_Pin, GPIO_PIN_SET);
              HAL_GPIO_WritePin(GPIOB, SEG_5_Pin, GPIO_PIN_RESET);
              HAL_GPIO_WritePin(GPIOB, SEG_6_Pin, GPIO_PIN_RESET);
              break;
          case 6:
              HAL_GPIO_WritePin(GPIOB, SEG_0_Pin, GPIO_PIN_RESET);
              HAL_GPIO_WritePin(GPIOB, SEG_1_Pin, GPIO_PIN_SET);
              HAL_GPIO_WritePin(GPIOB, SEG_2_Pin, GPIO_PIN_RESET);
              HAL_GPIO_WritePin(GPIOB, SEG_3_Pin, GPIO_PIN_RESET);
              HAL_GPIO_WritePin(GPIOB, SEG_4_Pin, GPIO_PIN_RESET);
              HAL_GPIO_WritePin(GPIOB, SEG_5_Pin, GPIO_PIN_RESET);
              HAL_GPIO_WritePin(GPIOB, SEG_6_Pin, GPIO_PIN_RESET);
              break;
          case 7:
              HAL_GPIO_WritePin(GPIOB, SEG_0_Pin, GPIO_PIN_RESET);
              HAL_GPIO_WritePin(GPIOB, SEG_1_Pin, GPIO_PIN_RESET);
              HAL_GPIO_WritePin(GPIOB, SEG_2_Pin, GPIO_PIN_RESET);
              HAL_GPIO_WritePin(GPIOB, SEG_3_Pin, GPIO_PIN_SET);
              HAL_GPIO_WritePin(GPIOB, SEG_4_Pin, GPIO_PIN_SET);
              HAL_GPIO_WritePin(GPIOB, SEG_5_Pin, GPIO_PIN_SET);
              HAL_GPIO_WritePin(GPIOB, SEG_6_Pin, GPIO_PIN_SET);
              break;
          case 8:
              HAL_GPIO_WritePin(GPIOB, SEG_0_Pin, GPIO_PIN_RESET);
              HAL_GPIO_WritePin(GPIOB, SEG_1_Pin, GPIO_PIN_RESET);
              HAL_GPIO_WritePin(GPIOB, SEG_2_Pin, GPIO_PIN_RESET);
              HAL_GPIO_WritePin(GPIOB, SEG_3_Pin, GPIO_PIN_RESET);
              HAL_GPIO_WritePin(GPIOB, SEG_4_Pin, GPIO_PIN_RESET);
              HAL_GPIO_WritePin(GPIOB, SEG_5_Pin, GPIO_PIN_RESET);
              HAL_GPIO_WritePin(GPIOB, SEG_6_Pin, GPIO_PIN_RESET);
              break;
          case 9:
              HAL_GPIO_WritePin(GPIOB, SEG_0_Pin, GPIO_PIN_RESET);
              HAL_GPIO_WritePin(GPIOB, SEG_1_Pin, GPIO_PIN_RESET);
              HAL_GPIO_WritePin(GPIOB, SEG_2_Pin, GPIO_PIN_RESET);
              HAL_GPIO_WritePin(GPIOB, SEG_3_Pin, GPIO_PIN_RESET);
              HAL_GPIO_WritePin(GPIOB, SEG_4_Pin, GPIO_PIN_SET);
              HAL_GPIO_WritePin(GPIOB, SEG_5_Pin, GPIO_PIN_RESET);
              HAL_GPIO_WritePin(GPIOB, SEG_6_Pin, GPIO_PIN_RESET);
              break;
          default:
              // Handle invalid input
              break;
      }
    }
   void turnonlyLED(int index) {
        // Turn off all LEDs first
        HAL_GPIO_WritePin(GPIOA, EN0_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOA, EN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOA, EN2_Pin, GPIO_PIN_SET);


        // Turn on the specified LED based on the index
        switch (index) {
            case 0:
                HAL_GPIO_WritePin(GPIOA, EN0_Pin, GPIO_PIN_RESET);
                break;
            case 1:
                HAL_GPIO_WritePin(GPIOA, EN1_Pin, GPIO_PIN_RESET);
                break;
            case 2:
                HAL_GPIO_WritePin(GPIOA, EN2_Pin, GPIO_PIN_RESET);
                break;
            default:
                // Handle invalid index
                break;
        }
    }


    void update7SEG(int index) {
        switch (index) {
            case 0:
                // Display the first 7SEG with led_buffer[0] ones
                display7SEG(led_buffer[0]);
                turnonlyLED(0);
                break;
            case 1:
                // Display the second 7SEG with led_buffer[1] tens
                display7SEG(led_buffer[1]);
                turnonlyLED(1);
                break;
            case 2:
                // Display the third 7SEG with led_buffer[2] hundreds
                display7SEG(led_buffer[2]);
                turnonlyLED(2);
                break;
            default:
                break;
        }
    }

    int NORMAL_STATE = 1;
    int PRESSED_STATE = 0;
    int index_led = 0;
    int buttonPressed = 0;


    while (1)
    {
        int buttonreg = HAL_GPIO_ReadPin(GPIOA, Button_1_Pin);
        if (buttonreg == PRESSED_STATE)
        {
            if (buttonPressed == 0)
            {
                ones++;
                buttonPressed = 1;
            }
        }
        else if (buttonreg == NORMAL_STATE)
        {
            buttonPressed = 0;
        }


      if (ones >= 10)
      {
        ones = 0;
        tens++;
      }
      if (tens >= 10)
      {
        tens = 0;
        hundreds++;
      }
      if (hundreds >= 10)
      {
        hundreds = 0;
      }
      updateBuffer();
      update7SEG(index_led);
      index_led = (index_led + 1) % 3;
        HAL_Delay(200);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, EN0_Pin|EN1_Pin|EN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SEG_0_Pin|SEG_1_Pin|SEG_2_Pin|SEG_3_Pin
                          |SEG_4_Pin|SEG_5_Pin|SEG_6_Pin|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pins : EN0_Pin EN1_Pin EN2_Pin */
  GPIO_InitStruct.Pin = EN0_Pin|EN1_Pin|EN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : Button_1_Pin */
  GPIO_InitStruct.Pin = Button_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Button_1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SEG_0_Pin SEG_1_Pin SEG_2_Pin SEG_3_Pin
                           SEG_4_Pin SEG_5_Pin SEG_6_Pin PB8 */
  GPIO_InitStruct.Pin = SEG_0_Pin|SEG_1_Pin|SEG_2_Pin|SEG_3_Pin
                          |SEG_4_Pin|SEG_5_Pin|SEG_6_Pin|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
