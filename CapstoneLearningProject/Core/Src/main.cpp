/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */


#include "main.h"
#include "stdio.h"
#include "string.h"

#include "Constants.h"

#include "Classes/LCD_4bitComm.h"
#include "Classes/LED.h"

#include "Classes/MPU6050.h"
#include "Classes/BMP085.h"

// Program constant values
#define FLOAT_VAL_LENGTH 5
#define LCD_REFRESH_RATE 500
#define SERIAL_COMM_TIMEOUT 100
#define DECIMAL_PT_DIV_FACTOR 100
#define TEMPERATURE_THRESHOLD 73.0

// Comm objects
I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart1;

// Custom class object declarations
LCD_4bitComm LCD(
		LCDD4_PORT, LCDD4_PIN,
		LCDD5_PORT, LCDD5_PIN,
		LCDD6_PORT, LCDD6_PIN,
		LCDD7_PORT, LCDD7_PIN,
		LCD_EN_PORT, LCD_EN_PIN,
		LCD_RW_PORT, LCD_RW_PIN,
		LCD_RS_PORT, LCD_RS_PIN,
		LCD_ROWS, LCD_COLS
);

LED gLED(GPIOA, 8, GPIO_PIN_8);
LED rLED(GPIOC, 9, GPIO_PIN_9);

MPU6050 MPU(&hi2c1);

// Buffer used to transmit information to UART
uint8_t buf[32];

// Global vars for sensor data storage
float accel_x = 0;
float accel_y = 0;
float accel_z = 0;

float gyro_x = 0;
float gyro_y = 0;
float gyro_z = 0;

float temp_f;

// Function Prototypes
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);

/**
 * @brief Places X: Y: and Z: on the LCD screen
 */
void DisplayXYZ(void)
{
	LCD.SendString((char*)"X: ");
	LCD.SetCursor(0, 8);
	LCD.SendString((char*)"Y: ");
	LCD.SetCursor(1, 0);
	LCD.SendString((char*)"Z: ");
}

/**
 * @brief Get the accelerometer, gyroscrope, and temperature
 * Data from the MPU6050 device.
 */
void getMPU6050SensorData()
{
	temp_f = MPU.readTemperature();

	accel_x = MPU.readAccelerometerX();
	accel_y = MPU.readAccelerometerY();
	accel_z = MPU.readAccelerometerZ();

	gyro_x = MPU.readGyroX();
	gyro_y = MPU.readGyroY();
	gyro_z = MPU.readGyroZ();
}

/**
 * @brief Displays the Accelerometer data on the LCD screen.
 */
void DisplayAccelData()
{
	LCD.SetCursor(0, 2);
	LCD.SendFloat(accel_x, FLOAT_VAL_LENGTH);

	LCD.SetCursor(0, 10);
	LCD.SendFloat(accel_y, FLOAT_VAL_LENGTH);

	LCD.SetCursor(1, 2);
	LCD.SendFloat(accel_z, FLOAT_VAL_LENGTH);
}

/**
 * @brief Displays the Gyroscope data on the LCD screen.
 */
void DisplayGyroData()
{
	LCD.SetCursor(0, 2);
	LCD.SendFloat(gyro_x, FLOAT_VAL_LENGTH);

	LCD.SetCursor(0, 10);
	LCD.SendFloat(gyro_y, FLOAT_VAL_LENGTH);

	LCD.SetCursor(1, 2);
	LCD.SendFloat(gyro_z, FLOAT_VAL_LENGTH);
}

/**
 * @brief Sends a serial message to the serial console.
 * @param serial_message	Message to send to the console.
 */
void SendSerialMessage(char*serial_message)
{
	sprintf((char*)buf, "%s", serial_message);
	HAL_UART_Transmit(&huart1, buf, strlen((char*)buf), SERIAL_COMM_TIMEOUT);
}

/**
 * @brief Sends a serial message to the serial console followed by a new line.
 * @param serial_message	Message to send to the console.
 */
void SendSerialMessageln(char*serial_message)
{
	sprintf((char*)buf, "%s\r\n", serial_message);
	HAL_UART_Transmit(&huart1, buf, strlen((char*)buf), SERIAL_COMM_TIMEOUT);
}

/**
 * @brief Convert a float value to a form that can be transmitted to the serial console.
 * Sends a value over with two decimal places
 * @param datapoint			Float value to send to the serial console
 * @param serial_message	Any message to directly follow the data point
 * 							(e.g. units of the data point).
 */
void ConvertFloatDataForSerialComm(float datapoint, char *serial_message)
{
	  float ser_datapoint = datapoint * DECIMAL_PT_DIV_FACTOR;

	  // Convert float value into USART-transmittable buffer
	  sprintf((char*)buf,
			  "%u.%u %s\r\n",
			  ((unsigned int)ser_datapoint / DECIMAL_PT_DIV_FACTOR),
			  ((unsigned int)ser_datapoint % DECIMAL_PT_DIV_FACTOR),
			  serial_message);

	HAL_UART_Transmit(&huart1, buf, strlen((char*)buf), SERIAL_COMM_TIMEOUT);
}

/**
 * @brief Send the Accelerometer, gyroscope, and temperature data
 * from the MPU6050 device to the serial console in a easy to read
 * format.
 */
void SerialDataDump()
{
	SendSerialMessageln((char*)"-- Sensor Data --");
	SendSerialMessage((char*)"Temperature: ");
	ConvertFloatDataForSerialComm(temp_f, (char*)"F");

	SendSerialMessage((char*)"Accel X: ");
	ConvertFloatDataForSerialComm(accel_x, (char*)"g");
	SendSerialMessage((char*)"Accel Y: ");
	ConvertFloatDataForSerialComm(accel_y, (char*)"g");
	SendSerialMessage((char*)"Accel Z: ");
	ConvertFloatDataForSerialComm(accel_z, (char*)"g");

	SendSerialMessage((char*)"Gyro X: ");
	ConvertFloatDataForSerialComm(gyro_x, (char*)"deg/s");
	SendSerialMessage((char*)"Gyro Y: ");
	ConvertFloatDataForSerialComm(gyro_y, (char*)"deg/s");
	SendSerialMessage((char*)"Gyro Z: ");
	ConvertFloatDataForSerialComm(gyro_z, (char*)"deg/s");
	SendSerialMessageln((char*)"-------------");
	SendSerialMessageln((char*)" ");

}

/**
 * @brief Updates the status of indicator LEDs based on the current
 * temperature value from the MPU6050 device.
 */
void UpdateTemperatureIndicatorLEDs()
{
	if(temp_f < TEMPERATURE_THRESHOLD)
	{
		gLED.On();
		rLED.Off();
	}
	else
	{
		gLED.Off();
		rLED.On();
	}
}

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  // Init Base functions of controller
  HAL_Init();
  SystemClock_Config();

  // Init Peripherals on controller
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();

  // Init custom objects
  gLED.Init();
  rLED.Init();
  MPU.Init();
  LCD.Init();

  // Place X: Y: and Z: on the LCD ahead of getting data
  DisplayXYZ();

	while (1)
	{
		getMPU6050SensorData();
		DisplayGyroData();
		UpdateTemperatureIndicatorLEDs();
		SerialDataDump();
		HAL_Delay(LCD_REFRESH_RATE);
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  hi2c1.Init.Timing = 0x0010020A;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  huart1.Init.BaudRate = 9600;
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB12 PB13 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PC6 PC7 PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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

