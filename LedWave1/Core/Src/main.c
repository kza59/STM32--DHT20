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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define LCD_NEXTLINE 							0xC0
#define LCD_CLEAR								0x01
#define LCD_8BIT_INIT_SIGNAL					0x30
#define LCD_8BIT_2LINES_5x8_FONT				0x38
#define LCD_DISPLAY_ON_CURSOR_OFF_BLINK_OFF		0x0C
#define LCD_ENTRY_MODE							0x06
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
void turnOnLED(int led_number);
void LCD_SendCommand(uint8_t cmd);
void LCD_SendData(uint8_t data);
void LCD_WriteByte(uint8_t byte);
void LCD_PulseEnable(void);
void writeToLCD(const char* str);
void LCD_Init(void);
void DHT20_Init(void);
char* getMeasurement(void);
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

//	HAL_Delay(1);

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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  LCD_Init();

  writeToLCD("LCD WORKS!");
  LCD_SendCommand(LCD_NEXTLINE);
  writeToLCD("WELCOME");


  DHT20_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  turnOnLED(0);
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |LCD2_Pin|LCD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, E_LCD_Pin|LCD_R_W_Pin|RS_Pin|LD2_Pin
                          |LCD0_Pin|LCD1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LCD4_Pin|LCD5_Pin|LCD6_Pin|LCD7_Pin
                          |DHT20_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC1 PC2 PC3
                           LCD2_Pin LCD3_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |LCD2_Pin|LCD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : E_LCD_Pin LCD_R_W_Pin RS_Pin LD2_Pin
                           LCD0_Pin LCD1_Pin */
  GPIO_InitStruct.Pin = E_LCD_Pin|LCD_R_W_Pin|RS_Pin|LD2_Pin
                          |LCD0_Pin|LCD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD4_Pin LCD5_Pin LCD6_Pin LCD7_Pin
                           DHT20_EN_Pin */
  GPIO_InitStruct.Pin = LCD4_Pin|LCD5_Pin|LCD6_Pin|LCD7_Pin
                          |DHT20_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void LCD_SendCommand(uint8_t cmd) {
    HAL_GPIO_WritePin(RS_GPIO_Port, RS_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LCD_R_W_GPIO_Port, LCD_R_W_Pin, GPIO_PIN_RESET);
    LCD_WriteByte(cmd);
    HAL_Delay(1);
}

void LCD_SendData(uint8_t data) {
    HAL_GPIO_WritePin(RS_GPIO_Port, RS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LCD_R_W_GPIO_Port, LCD_R_W_Pin, GPIO_PIN_RESET);
    LCD_WriteByte(data);
    HAL_Delay(1);
}

void LCD_WriteByte(uint8_t byte) {
    // Write each bit to its respective pin
    HAL_GPIO_WritePin(LCD0_GPIO_Port, LCD0_Pin, (byte & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LCD1_GPIO_Port, LCD1_Pin, (byte & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LCD2_GPIO_Port, LCD2_Pin, (byte & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LCD3_GPIO_Port, LCD3_Pin, (byte & 0x08) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LCD4_GPIO_Port, LCD4_Pin, (byte & 0x10) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LCD5_GPIO_Port, LCD5_Pin, (byte & 0x20) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LCD6_GPIO_Port, LCD6_Pin, (byte & 0x40) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LCD7_GPIO_Port, LCD7_Pin, (byte & 0x80) ? GPIO_PIN_SET : GPIO_PIN_RESET);

    // Pulse the enable pin
    LCD_PulseEnable();
}

void LCD_PulseEnable(void) {
    HAL_GPIO_WritePin(E_LCD_GPIO_Port, E_LCD_Pin, GPIO_PIN_SET);
    HAL_Delay(1);  // Enable pulse width (typically > 450ns)
    HAL_GPIO_WritePin(E_LCD_GPIO_Port, E_LCD_Pin, GPIO_PIN_RESET);
    // No need for additional delay here, handled in calling functions
}

void writeToLCD(const char* str) {
    while (*str) {
        LCD_SendData(*str++);
    }
}

void LCD_Init(void) {
    HAL_Delay(50);
    LCD_SendCommand(LCD_8BIT_INIT_SIGNAL);
    HAL_Delay(5);
    LCD_SendCommand(LCD_8BIT_INIT_SIGNAL);
    HAL_Delay(1);
    LCD_SendCommand(LCD_8BIT_INIT_SIGNAL);
    HAL_Delay(1);
    LCD_SendCommand(LCD_8BIT_2LINES_5x8_FONT);
    HAL_Delay(1);
    LCD_SendCommand(LCD_DISPLAY_ON_CURSOR_OFF_BLINK_OFF);
    HAL_Delay(1);
    LCD_SendCommand(LCD_CLEAR);
    HAL_Delay(2);
    LCD_SendCommand(LCD_ENTRY_MODE);
    HAL_Delay(1);
}

void DHT20_Init(void) {
    HAL_GPIO_WritePin(GPIOB, DHT20_EN_Pin, GPIO_PIN_SET);
    HAL_Delay(100);

    uint8_t status_byte = 0;
    uint8_t command_buf[3] = {0xAC, 0x33, 0x00};
    uint8_t receive_buf[6] = {0};
    uint8_t crc_buf = 0;


    HAL_I2C_Master_Receive(&hi2c1, 0x38 << 1, &status_byte, 1, 500);
    if ((status_byte & 0x18) != 0x18) {
        LCD_SendCommand(LCD_CLEAR);
        HAL_Delay(10);
        writeToLCD("Issue: DHT20");
        LCD_SendCommand(LCD_NEXTLINE);
        writeToLCD("Init 0x1B, 0x1C, 0x1E");
        uint8_t init_reg[2];


        init_reg[0] = 0x1B;
        init_reg[1] = 0x00;
        HAL_I2C_Master_Transmit(&hi2c1, 0x38 << 1, init_reg, 2, 500);
        HAL_Delay(5);


        init_reg[0] = 0x1C;
        init_reg[1] = 0x00;
        HAL_I2C_Master_Transmit(&hi2c1, 0x38 << 1, init_reg, 2, 500);
        HAL_Delay(5);


        init_reg[0] = 0x1E;
        init_reg[1] = 0x00;
        HAL_I2C_Master_Transmit(&hi2c1, 0x38 << 1, init_reg, 2, 500);
        HAL_Delay(5);

    }

    HAL_Delay(10);
    HAL_I2C_Master_Transmit(&hi2c1, 0x38 << 1, command_buf, 3, 500);
    HAL_Delay(80);

    HAL_I2C_Master_Receive(&hi2c1, 0x38 << 1, &status_byte, 1, 500);
    if ((status_byte & 0x80) == 0) {
        HAL_I2C_Master_Receive(&hi2c1, 0x38 << 1, receive_buf, 6, 1000);


        uint32_t raw_humidity = ((uint32_t)receive_buf[1] << 12) |
                                ((uint32_t)receive_buf[2] << 4) |
                                ((uint32_t)(receive_buf[3] >> 4));

        uint32_t raw_temperature = (((uint32_t)(receive_buf[3] & 0x0F)) << 16) |
                                   ((uint32_t)receive_buf[4] << 8) |
                                   (uint32_t)receive_buf[5];

        float humidity = (raw_humidity / 1048576.0) * 100.0;
        float temperature = ((raw_temperature / 1048576.0) * 200.0) - 50.0;

        char lcd_buf[32];
        LCD_SendCommand(LCD_CLEAR);
        snprintf(lcd_buf, sizeof(lcd_buf), "Hum: %.1f%%", humidity);
        writeToLCD(lcd_buf);
        LCD_SendCommand(LCD_NEXTLINE);
        snprintf(lcd_buf, sizeof(lcd_buf), "Temp: %.1fC", temperature);
        writeToLCD(lcd_buf);
    } else {
        LCD_SendCommand(LCD_CLEAR);
        writeToLCD("Data not ready");
    }
}


void turnOnLED(int led_number) {
	switch(led_number) {
	case 0:
	    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
	    HAL_Delay(100);
	    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
		break;
	case 1:
	    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
	    HAL_Delay(100);
	    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
		break;
	case 2:
	    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
	    HAL_Delay(100);
	    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
		break;
	case 3:
	    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
	    HAL_Delay(100);
	    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
		break;
	}
}
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
