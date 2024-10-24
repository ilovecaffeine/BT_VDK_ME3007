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
#include <string.h>
#include <stdio.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//LCD -------------------------------------------------------------
#define LCD_NIB 4
#define LCD_DATA_REG         GPIO_PIN_SET
#define LCD_COMMAND_REG      GPIO_PIN_RESET
GPIO_TypeDef* ports[] = {D4_GPIO_Port, D5_GPIO_Port, D6_GPIO_Port, D7_GPIO_Port};
uint16_t pins[] = {D4_Pin, D5_Pin, D6_Pin, D7_Pin};

//KEYPAD -------------------------------------------------------------
GPIO_TypeDef* rowPorts[4] = {KeyA_GPIO_Port, KeyB_GPIO_Port, KeyC_GPIO_Port, KeyD_GPIO_Port};
uint16_t rowPins[4] = {KeyA_Pin, KeyB_Pin, KeyC_Pin, KeyD_Pin};

GPIO_TypeDef* colPorts[4] = {Key1_GPIO_Port, Key2_GPIO_Port, Key3_GPIO_Port, Key4_GPIO_Port};
uint16_t colPins[4] = {Key1_Pin, Key2_Pin, Key3_Pin, Key4_Pin};
// Keypad layout
char keys[4][4] = {
  {'7', '8', '9', '/'},
  {'4', '5', '6', '*'},
  {'1', '2', '3', '-'},
  {'C', '0', '=', '+'}
};

int NORMAL_STATE = 1;
 int PRESSED_STATE = 0;
 int index_led = 0;
 int buttonPressed = 0;

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
  void lcd_write( uint8_t data)
  {
  	for(uint8_t i = 0; i < 4; i++)
  	{
  		if( ((data >> i) & 0x01) == 0)
  	   	HAL_GPIO_WritePin(ports[i], pins[i], GPIO_PIN_RESET);
  		else
  			HAL_GPIO_WritePin(ports[i], pins[i], GPIO_PIN_SET);
  	}

  	HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_SET);
  	HAL_Delay(1);
  	HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_RESET); 		// Data receive on falling edge
  }

  void lcd_write_data( uint8_t data)
  {
  	  HAL_GPIO_WritePin(RS_GPIO_Port, RS_Pin, LCD_DATA_REG);			// Write to data register

  	  lcd_write(data >> 4);
  		lcd_write(data & 0x0F);
  }



  void lcd_write_command(uint8_t command)
  {
  	HAL_GPIO_WritePin(RS_GPIO_Port, RS_Pin, LCD_COMMAND_REG);		// Write to command register

  		lcd_write((command >> 4));
  		lcd_write(command & 0x0F);
  }

  void Lcd_init(void) {
      HAL_Delay(50);    // Wait for LCD to power up

      lcd_write_command(0x33);
      HAL_Delay(5);
      lcd_write_command(0x32);// Initialize in 4-bit mode

      lcd_write_command(0x02); // Home
      HAL_Delay(5);

      lcd_write_command(0x28);  // 4-bit mode, 2 lines, 5x8 font
      HAL_Delay(5);

      lcd_write_command(0x0C);  // Display ON, cursor OFF, blink OFF
      HAL_Delay(5);

      lcd_write_command(0x06);  // Auto-increment cursor
      HAL_Delay(5);

      lcd_write_command(0x01);  // Clear display
      HAL_Delay(5);
  }


  void Lcd_clear_display(void)
  {
  	lcd_write_command(0x01);
  	HAL_Delay(2);
  }
  void Lcd_home() {
      lcd_write_command(0x02);  // Gửi lệnh "Return Home"
      HAL_Delay(2);
  }
  void Lcd_cursor_on(void) {
      lcd_write_command(0x0E);  // Display ON, cursor ON
      HAL_Delay(2);
  }

  void Lcd_cursor_off(void) {
      lcd_write_command(0x0C);  // Display ON, cursor OFF
      HAL_Delay(2);
  }


  void Lcd_write_string(char * string)
  {
  	for(uint8_t i = 0; i < strlen(string); i++)
  	{
  		lcd_write_data( string[i]);
  	}
  }

  void Lcd_write_int(int number)
  {
  	char buffer[11];
  	sprintf(buffer, "%d", number);
  	Lcd_write_string(buffer);
  }

  void Lcd_gotoxy(uint8_t x, uint8_t y) {
      uint8_t address;

      // Kiểm tra y để xác định dòng
      switch(y) {
          case 0:
              address = 0x00 + x;  // �?ịa chỉ của dòng 1 bắt đầu từ 0x00
              break;
          case 1:
              address = 0x40 + x;  // �?ịa chỉ của dòng 2 bắt đầu từ 0x40
              break;
          default:
              return;  // Nếu y không hợp lệ (khác 0 hoặc 1), thoát hàm
      }

      // Gửi lệnh "Set DDRAM Address" với lệnh gốc 0x80
      lcd_write_command(0x80 | address);
  }
  void Lcd_clear_xy(uint8_t x,uint8_t y)
  {
  	Lcd_gotoxy( x,y);
  	lcd_write_data(' ');
  }


  char keypad_scan(void) {
      // Loop through each row
      for (int row = 0; row < 4; row++) {
          // Set the current row LOW and the others HIGH
          HAL_GPIO_WritePin(rowPorts[row], rowPins[row], GPIO_PIN_RESET);

          // Set other rows to HIGH
          for (int otherRow = 0; otherRow < 4; otherRow++) {
              if (otherRow != row) {
                  HAL_GPIO_WritePin(rowPorts[otherRow], rowPins[otherRow], GPIO_PIN_SET);
              }
          }



          // Check each column
          for (int col = 0; col < 4; col++) {
              // Check if the key is pressed
              if (HAL_GPIO_ReadPin(colPorts[col], colPins[col]) == GPIO_PIN_RESET) {
            	  return keys[row][col];

              }
          }

          // Set the current row back to HIGH
          HAL_GPIO_WritePin(rowPorts[row], rowPins[row], GPIO_PIN_SET);
      }
      return '\0';
  }





    Lcd_init();
    char lastKey = '\0'; // Variable to hold the last displayed key
    while (1)
    {
        char key = keypad_scan(); // Scan for key press

        if (key == 'C') {
            Lcd_clear_display(); // Clear the LCD display if 'C' is pressed
            lastKey = '\0'; // Reset lastKey to allow new input
        } else if (key != '\0' && key != lastKey) { // If a new key was pressed
            char keyStr[2];  // Array to hold the key and null terminator
            keyStr[0] = key;  // Assign the key to the first element
            keyStr[1] = '\0';  // Null-terminate the string

            Lcd_write_string(keyStr);  // Display the key on the LCD

            lastKey = key; // Update lastKey to the current key
        } else if (key == '\0') {
            lastKey = '\0'; // Reset lastKey when no key is pressed
        }


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
  HAL_GPIO_WritePin(GPIOA, D4_Pin|D5_Pin|D6_Pin|D7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, KeyD_Pin|RS_Pin|EN_Pin|KeyA_Pin
                          |KeyB_Pin|KeyC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : D4_Pin D5_Pin D6_Pin D7_Pin */
  GPIO_InitStruct.Pin = D4_Pin|D5_Pin|D6_Pin|D7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : KeyD_Pin RS_Pin EN_Pin KeyA_Pin
                           KeyB_Pin KeyC_Pin */
  GPIO_InitStruct.Pin = KeyD_Pin|RS_Pin|EN_Pin|KeyA_Pin
                          |KeyB_Pin|KeyC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : Key1_Pin Key2_Pin Key3_Pin Key4_Pin */
  GPIO_InitStruct.Pin = Key1_Pin|Key2_Pin|Key3_Pin|Key4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
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
