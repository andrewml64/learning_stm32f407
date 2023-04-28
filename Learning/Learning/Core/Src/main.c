/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "usb_host.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"
#include "math.h"
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
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

USART_HandleTypeDef husart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
static const uint8_t BARO_ADDR = 0x77 << 1; //Left shifted to use 8 bit address
static const uint8_t BARO_RESET = 0x1E; //Reset command
static const uint8_t ADC_BARO_READ_ADDR = 0x00; //Command to read Baro's ADC (24 bit read)
static const uint8_t BARO_PROM_ADDR = 0xA0; //Base PROM ADDR for Baro sensor
static const uint8_t BARO_C1 = 0x02;
static const uint8_t BARO_C2 = 0x04;
static const uint8_t BARO_C3 = 0x06;
static const uint8_t BARO_C4 = 0x08;
static const uint8_t BARO_C5 = 0x0A;
static const uint8_t BARO_C6 = 0x0C;
static const uint8_t BARO_CONVERT_PRESSURE = 0x48; //Command to convert baro sensor to reading pressure
static const uint8_t BARO_CONVERT_TEMP = 0x58; //Command to convert baro sensor to reading temperature

static const uint8_t ACCEL_XOUT_H = (0x3B | 0x80); //High bit x accel register address ORed with read bit
static const uint8_t ACCEL_XOUT_L = (0x3C | 0x80); //Low bit x accel register address ORed with read bit
//static const uint8_t ACCEL_YOUT_H = (0x3D | 0x80); //High bit y accel register address ORed with read bit
//static const uint8_t ACCEL_YOUT_L = (0x3E | 0x80); //Low bit y accel register address ORed with read bit
//static const uint8_t ACCEL_ZOUT_H = (0x3F | 0x80); //High bit z accel register address ORed with read bit
//static const uint8_t ACCEL_ZOUT_L = (0x40 | 0x80); //Low bit z accel register address ORed with read bit

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_Init(void);
static void MX_USART3_UART_Init(void);
void MX_USB_HOST_Process(void);

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
	HAL_StatusTypeDef ret;
	uint8_t buf[16];
	char usart_buf[82];
	uint16_t C1 = 1, C2 = 1, C3 = 1, C4 = 1, C5 = 1, C6 = 1;
	int accel_x = 1;
	uint32_t pressure = 0, temp = 0;
	int32_t dT = 0, TEMP, P;
	int64_t OFF = 0, SENS = 0;
	uint8_t spi_buf[2];
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
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USB_HOST_Init();
  MX_USART2_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
	HAL_Delay(100);

	// Send reset command to baro sensor on initialization
	buf[0] = BARO_RESET;
	ret = HAL_I2C_Master_Transmit(&hi2c1, BARO_ADDR, buf, 1, HAL_MAX_DELAY);
	if (ret != HAL_OK){
		strcpy((char*)buf, "Error TX_1\r\n");
	}
	HAL_Delay(50);

	buf[0] = BARO_PROM_ADDR | BARO_C1;
	ret = HAL_I2C_Master_Transmit(&hi2c1, BARO_ADDR, buf, 1, HAL_MAX_DELAY);
	if (ret != HAL_OK){
		strcpy((char*)buf, "Error TX_1a\r\n");
	}else{
		  ret = HAL_I2C_Master_Receive(&hi2c1, BARO_ADDR, buf, 2, HAL_MAX_DELAY);
		  if (ret != HAL_OK){
			  strcpy((char*)buf, "Error Rx\r\n");
		  } else {
			  C1 = ((buf[0] << 8) | buf[1]);
		  }
	}
	HAL_Delay(50);

	buf[0] = BARO_PROM_ADDR | BARO_C2;
	ret = HAL_I2C_Master_Transmit(&hi2c1, BARO_ADDR, buf, 1, HAL_MAX_DELAY);
	if (ret != HAL_OK){
		strcpy((char*)buf, "Error TX_1b\r\n");
	}else{
		  ret = HAL_I2C_Master_Receive(&hi2c1, BARO_ADDR, buf, 2, HAL_MAX_DELAY);
		  if (ret != HAL_OK){
			  strcpy((char*)buf, "Error Rx\r\n");
		  } else {
			  C2 = ((buf[0] << 8) | buf[1]);
		  }
	}
	HAL_Delay(50);

	buf[0] = BARO_PROM_ADDR | BARO_C3;
	ret = HAL_I2C_Master_Transmit(&hi2c1, BARO_ADDR, buf, 1, HAL_MAX_DELAY);
	if (ret != HAL_OK){
		strcpy((char*)buf, "Error TX_1c\r\n");
	}else{
		  ret = HAL_I2C_Master_Receive(&hi2c1, BARO_ADDR, buf, 2, HAL_MAX_DELAY);
		  if (ret != HAL_OK){
			  strcpy((char*)buf, "Error Rx\r\n");
		  } else {
			  C3 = ((buf[0] << 8) | buf[1]);
		  }
	}
	HAL_Delay(50);

	buf[0] = BARO_PROM_ADDR | BARO_C4;
	ret = HAL_I2C_Master_Transmit(&hi2c1, BARO_ADDR, buf, 1, HAL_MAX_DELAY);
	if (ret != HAL_OK){
		strcpy((char*)buf, "Error TX_1d\r\n");
	}else{
		  ret = HAL_I2C_Master_Receive(&hi2c1, BARO_ADDR, buf, 2, HAL_MAX_DELAY);
		  if (ret != HAL_OK){
			  strcpy((char*)buf, "Error Rx\r\n");
		  } else {
			  C4 = ((buf[0] << 8) | buf[1]);
		  }
	}
	HAL_Delay(50);

	buf[0] = BARO_PROM_ADDR | BARO_C5;
	ret = HAL_I2C_Master_Transmit(&hi2c1, BARO_ADDR, buf, 1, HAL_MAX_DELAY);
	if (ret != HAL_OK){
		strcpy((char*)buf, "Error TX_1e\r\n");
	}else{
		  ret = HAL_I2C_Master_Receive(&hi2c1, BARO_ADDR, buf, 2, HAL_MAX_DELAY);
		  if (ret != HAL_OK){
			  strcpy((char*)buf, "Error Rx\r\n");
		  } else {
			  C5 = ((buf[0] << 8) | buf[1]);
		  }
	}
	HAL_Delay(50);

	buf[0] = BARO_PROM_ADDR | BARO_C6;
	ret = HAL_I2C_Master_Transmit(&hi2c1, BARO_ADDR, buf, 1, HAL_MAX_DELAY);
	if (ret != HAL_OK){
		strcpy((char*)buf, "Error TX_1f\r\n");
	}else{
		  ret = HAL_I2C_Master_Receive(&hi2c1, BARO_ADDR, buf, 2, HAL_MAX_DELAY);
		  if (ret != HAL_OK){
			  strcpy((char*)buf, "Error Rx\r\n");
		  } else {
			  C6 = ((buf[0] << 8) | buf[1]);
		  }
	}
	HAL_Delay(50);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  // Convert to reading pressure values

	  buf[0] = BARO_CONVERT_PRESSURE;
	  ret = HAL_I2C_Master_Transmit(&hi2c1, BARO_ADDR, buf, 1, HAL_MAX_DELAY);
	  if (ret != HAL_OK){
		  strcpy((char*)buf, "Error TX_2\r\n");
	  }

	  HAL_Delay(15);

	  // Read Pressure value

	  buf[0] = ADC_BARO_READ_ADDR;
	  ret = HAL_I2C_Master_Transmit(&hi2c1, BARO_ADDR, buf, 1, HAL_MAX_DELAY);
	  if (ret != HAL_OK){
		  strcpy((char*)buf, "Error TX_3\r\n");
	  } else{
		  ret = HAL_I2C_Master_Receive(&hi2c1, BARO_ADDR, buf, 3, HAL_MAX_DELAY);
		  if (ret != HAL_OK){
			  strcpy((char*)buf, "Error Rx\r\n");
		  } else {
			  pressure = ((0x00 << 24) | (buf[0] << 16) | (buf[1] << 8) | buf[2]);
		  }
	  }

	  //Convert to reading temperature value

	  buf[0] = BARO_CONVERT_TEMP;
	 	  ret = HAL_I2C_Master_Transmit(&hi2c1, BARO_ADDR, buf, 1, HAL_MAX_DELAY);
	 	  if (ret != HAL_OK){
	 		  strcpy((char*)buf, "Error TX_2\r\n");
	 	  }

	  HAL_Delay(15);

	  // Read temperature value

	  buf[0] = ADC_BARO_READ_ADDR;
	  ret = HAL_I2C_Master_Transmit(&hi2c1, BARO_ADDR, buf, 1, HAL_MAX_DELAY);
	  if (ret != HAL_OK){
		  strcpy((char*)buf, "Error TX_3\r\n");
	  } else{
		  ret = HAL_I2C_Master_Receive(&hi2c1, BARO_ADDR, buf, 3, HAL_MAX_DELAY);
		  if (ret != HAL_OK){
			  strcpy((char*)buf, "Error Rx\r\n");
		  } else {
			  temp = ((0x00 << 24) | (buf[0] << 16) | (buf[1] << 8) | buf[2]);
		  }
	  }

	  //sprintf((char*)buf, "%u \r\n", temp);
	  //HAL_USART_Transmit(&husart2, buf, strlen((char*)buf), HAL_MAX_DELAY);

	  dT = temp - C5*pow(2,8);
	  TEMP = 2000 + dT*C6/pow(2,23);
	  strcpy((char*)buf, "Temperature: ");
	  HAL_USART_Transmit(&husart2, buf, strlen((char*)buf), HAL_MAX_DELAY);
	  sprintf((char*)buf, "%i.%02u C\r\n", (int)TEMP/100, (unsigned int)TEMP%100);
	  HAL_USART_Transmit(&husart2, buf, strlen((char*)buf), HAL_MAX_DELAY);

	  OFF = C2*pow(2,16) + (C4*dT)/pow(2,7);
	  SENS = C1*pow(2,15) + (C3*dT)/pow(2,8);
	  P = (pressure*SENS/pow(2,21) - OFF)/pow(2,15);

	  strcpy((char*)buf, "Pressure: ");
	  HAL_USART_Transmit(&husart2, buf, strlen((char*)buf), HAL_MAX_DELAY);
	  sprintf((char*)buf, "%i.%02u Pa\r\n", (int)P/100, (unsigned int)P % 100 );
	  HAL_USART_Transmit(&husart2, buf, strlen((char*)buf), HAL_MAX_DELAY);

	  HAL_Delay(500);

	/*  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	  ret = HAL_SPI_Transmit(&hspi1, (uint8_t *)&ACCEL_XOUT_H, 1, 100);
	  if (ret != HAL_OK){
		  strcpy((char*)buf, "Error TX_3\r\n");
	  } else{
		  ret = HAL_SPI_Receive(&hspi1, (uint8_t *)spi_buf, 1, 100);
		  if (ret != HAL_OK){
			  strcpy((char*)buf, "Error Rx\r\n");
		  } else {
			  accel_x = (spi_buf[0]<<8);
		  }
	  }
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
	  HAL_Delay(10);

	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	  ret = HAL_SPI_Transmit(&hspi1, (uint8_t *)&ACCEL_XOUT_L, 1, 100);
	  if (ret != HAL_OK){
		  strcpy((char*)buf, "Error TX_3\r\n");
	  } else{
		  ret = HAL_SPI_Receive(&hspi1, (uint8_t *)spi_buf, 1, 100);
		  if (ret != HAL_OK){
			  strcpy((char*)buf, "Error Rx\r\n");
		  } else {
			  accel_x |= spi_buf[0];
		  }
	  }
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

	  strcpy((char*)buf, "Accel X: ");
	  HAL_USART_Transmit(&husart2, buf, strlen((char*)buf), HAL_MAX_DELAY);
	  sprintf((char*)buf, "%i\r\n", (int)accel_x);
	  HAL_USART_Transmit(&husart2, buf, strlen((char*)buf), HAL_MAX_DELAY);*/

// Tried to read GPS over Uart3 but just got junk. Looks like I might need a ring buffer
//	  HAL_Delay(10);
//
//	  HAL_USART_Receive(&huart3, usart_buf, strlen((char*)usart_buf), 5000);
//
//	  strcpy((char*)buf, "GPS: ");
//	  HAL_USART_Transmit(&husart2, buf, strlen((char*)buf), HAL_MAX_DELAY);
//	  HAL_USART_Transmit(&husart2, (char*)usart_buf, strlen((char*)usart_buf), HAL_MAX_DELAY);
//	  strcpy((char*)buf, "\r\n");
//	  HAL_USART_Transmit(&husart2, buf, strlen((char*)buf), HAL_MAX_DELAY);

	  //HAL_Delay(500);

    /* USER CODE END WHILE */
    MX_USB_HOST_Process();

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  husart2.Instance = USART2;
  husart2.Init.BaudRate = 115200;
  husart2.Init.WordLength = USART_WORDLENGTH_8B;
  husart2.Init.StopBits = USART_STOPBITS_1;
  husart2.Init.Parity = USART_PARITY_NONE;
  husart2.Init.Mode = USART_MODE_TX_RX;
  husart2.Init.CLKPolarity = USART_POLARITY_LOW;
  husart2.Init.CLKPhase = USART_PHASE_1EDGE;
  husart2.Init.CLKLastBit = USART_LASTBIT_DISABLE;
  if (HAL_USART_Init(&husart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : IMU_CS_Pin */
  GPIO_InitStruct.Pin = IMU_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(IMU_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_IN_Pin */
  GPIO_InitStruct.Pin = CLK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : I2S3_MCK_Pin I2S3_SCK_Pin I2S3_SD_Pin */
  GPIO_InitStruct.Pin = I2S3_MCK_Pin|I2S3_SCK_Pin|I2S3_SD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

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
