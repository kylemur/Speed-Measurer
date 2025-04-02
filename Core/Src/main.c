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
#include <math.h>
#include <stdio.h>
#include <stdint.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define CE_PORT GPIOB 		// PB6 chip enable (aka slave select)
#define CE_PIN GPIO_PIN_6
#define DC_PORT GPIOA 		// PA0 data/control
#define DC_PIN GPIO_PIN_0
#define RESET_PORT GPIOA 	// PA1 reset
#define RESET_PIN GPIO_PIN_1
#define GLCD_WIDTH 84
#define GLCD_HEIGHT 48
#define NUM_BANKS 6
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
uint32_t pMillis;
uint32_t val1 = 0;
uint32_t val2 = 0;
uint16_t distance  = 0;
uint16_t old_distance = 0;
uint16_t next_bank = 42;
uint8_t speed_limit_digits = 0; // start the speed limit at 0
uint8_t caution_speed = 0; 		// start the caution speed limit at 0
int speed = 0;
int speed250 = 0;

const char font_table[][6] = {
	{0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, // space	0
	{0x7E, 0x11, 0x11, 0x11, 0x7E, 0x00}, // 'A'	1
	{0x7F, 0x49, 0x49, 0x49, 0x36, 0x00}, // 'B'	2
	{0x3E, 0x41, 0x41, 0x41, 0x22, 0x00}, // 'C'	3
	{0x00, 0x7F, 0x41, 0x41, 0x22, 0x1C}, // 'D'	4
	{0x00, 0x7F, 0x49, 0x49, 0x49, 0x49}, // 'E'	5
	{0x00, 0x7F, 0x09, 0x09, 0x09, 0x09}, // 'F'	6
	{0x3E, 0x41, 0x49, 0x49, 0x49, 0x7A}, // 'G'	7
	{0x7F, 0x08, 0x08, 0x08, 0x7F, 0x00}, // 'H'	8
	{0x00, 0x41, 0x7F, 0x41, 0x00, 0x00}, // 'I'	9
	{0x30, 0x40, 0x40, 0x40, 0x3F, 0x00}, // 'J'	10
	{0x7F, 0x08, 0x14, 0x22, 0x41, 0x00}, // 'K'	11
	{0x00, 0x7F, 0x40, 0x40, 0x40, 0x40}, // 'L'	12
	{0x7F, 0x02, 0x0C, 0x02, 0x7F, 0x00}, // 'M'	13
	{0x7F, 0x04, 0x08, 0x10, 0x7F, 0x00}, // 'N'	14
	{0x3E, 0x41, 0x41, 0x41, 0x3E, 0x00}, // 'O'	15
	{0x7F, 0x09, 0x09, 0x09, 0x06, 0x00}, // 'P'	16
	{0x3E, 0x41, 0x51, 0x21, 0x5E, 0x00}, // 'Q'	17
	{0x7F, 0x09, 0x19, 0x29, 0x46, 0x00}, // 'R'	18
	{0x46, 0x49, 0x49, 0x49, 0x31, 0x00}, // 'S'	19
	{0x01, 0x01, 0x7F, 0x01, 0x01, 0x00}, // 'T'	20
	{0x3F, 0x40, 0x40, 0x40, 0x3F, 0x00}, // 'U'	21
	{0x1F, 0x20, 0x40, 0x20, 0x1F, 0x00}, // 'V'	22
	{0x3F, 0x40, 0x38, 0x40, 0x3F, 0x00}, // 'W'	23
	{0x63, 0x14, 0x08, 0x14, 0x63, 0x00}, // 'X'	24
	{0x07, 0x08, 0x70, 0x08, 0x07, 0x00}, // 'Y'	25
	{0x61, 0x51, 0x49, 0x45, 0x43, 0x00}, // 'Z'	26
	{0x00, 0x00, 0x5F, 0x00, 0x00, 0x00}, // '!'	27
	{0x00, 0x00, 0x7E, 0x81, 0xB5, 0xA1}, // smile_left 28
	{0xA1, 0xB5, 0x81, 0x7E, 0x00, 0x00}, // smile_right 29
	{0x00, 0x3E, 0x41, 0x41, 0x3E, 0x00}, // '0'	30
	{0x00, 0x00, 0x42, 0x7F, 0x40, 0x00}, // '1'	31
	{0x62, 0x51, 0x49, 0x49, 0x46, 0x00}, // '2'	32
	{0x22, 0x41, 0x49, 0x49, 0x36, 0x00}, // '3'	33
	{0x18, 0x14, 0x12, 0x7F, 0x10, 0x00}, // '4'	34
	{0x27, 0x45, 0x45, 0x45, 0x39, 0x00}, // '5'	35
	{0x3E, 0x49, 0x49, 0x49, 0x32, 0x00}, // '6'	36
	{0x01, 0x01, 0x71, 0x09, 0x07, 0x00}, // '7'	37
	{0x36, 0x49, 0x49, 0x49, 0x36, 0x00}, // '8'	38
	{0x26, 0x49, 0x49, 0x49, 0x3E, 0x00}, // '9'	39
	{0x22, 0x14, 0x7F, 0x14, 0x22, 0x00}, // '*'	40
	{0x12, 0x3F, 0x12, 0x3F, 0x12, 0x00}, // '#'	41
	{0x00, 0x00, 0x40, 0x00, 0x00, 0x00}, // '.'	42
	{0x00, 0x08, 0x08, 0x08, 0x08, 0x00}  // '-'	43
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void SPI_write(unsigned char data);
void GLCD_data_write(unsigned char data);
void GLCD_command_write(unsigned char data);
void GLCD_init(void);
void GLCD_setCursor(unsigned char x, unsigned char y);
void GLCD_clear(void);
void GLCD_putchar(int font_table_row);

uint8_t keypad_decode();
void GLCD_puts(const char* str);
void display_distance(uint16_t distance_cm);
void display_distance_ft(uint16_t distance_ft);
void display_distance_in(uint16_t distance_in);
void display_caution_speed(uint16_t caution_speed);
void display_speed_limit(uint16_t speed_limit);
void display_speed(int speed);
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
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  /// Initialize the Timer and PWM
  HAL_TIM_Base_Start(&htim1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); // start PWM on Timer3’s Channel 2



  /// Initialize the GLCD
  GLCD_init(); 	// initialize the screen
  GLCD_clear(); // clear the screen

  // Display "SPEED LIMIT "
  GLCD_setCursor(0,0); // set the cursor to the top left corner
  uint16_t speed_limit[] = {19, 16, 5, 5, 4, 0, 12, 9, 13, 9, 20, 0}; 	// "SPEED LIMIT " array
  for (int i = 0; i < 12; i++) {
	  GLCD_putchar(speed_limit[i]); 									// display "SPEED LIMIT "
  }

  // Display " 0 MPH"
  speed_limit_digits = 0; 				// start the speed limit at 0
  GLCD_setCursor(0, 1); 				// set the cursor to the second row
  GLCD_putchar(speed_limit_digits);		// '0'
  GLCD_putchar(0);						// ' '
  GLCD_putchar(13);						// 'M'
  GLCD_putchar(16);						// 'P'
  GLCD_putchar(8);						// 'H'

  // Display " # YOUR SPEED"
  GLCD_setCursor(0, 3); // set the cursor to the forth row
  uint16_t your_speed[] = {0, 0, 41, 0, 25, 15, 21, 18, 0, 19, 16, 5, 5, 4}; // " # YOUR SPEED" array
  for (int j = 0; j < 14; j++) {
  	  GLCD_putchar(your_speed[j]); 											 // display " # YOUR SPEED"
  }

  // Display " 0 FT  0 IN"
  GLCD_setCursor(0,5); // set the cursor to the sixth row
  uint16_t ft_in[] = {0, 30, 0, 6, 20, 0, 0, 30, 0, 9, 14}; // " 0 FT  0 IN" array
  for (int k = 0; k < 11; k++) {
      GLCD_putchar(ft_in[k]); 								// display " 0 FT  0 IN"
  }

  /// Sources for Help
  // https://randomnerdtutorials.com/complete-guide-for-ultrasonic-sensor-hc-sr04/
  // https://randomnerdtutorials.com/esp32-esp8266-nodemcu-hc-sr04-ultrasonic-sensor/
  // https://ecelabs.njit.edu/fed101/resources/HC-SR04%20Ultrasonic%20Sensor.pdf
  // https://www.electronicwings.com/nodemcu/hc-sr04-ultrasonic-sensor-interfacing-with-nodemcu

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  /// Display the Speed Limit
	  GLCD_setCursor(0, 1); 					// set the cursor to the second row
	  GLCD_putchar(speed_limit_digits + 30);	// display the speed limit


	  /// Ultrasonic Sensor Logic to Calculate Distance
  	  // Send the ultrasonic wave for 10 microseconds
	  HAL_GPIO_WritePin(Trig_GPIO_Port, Trig_Pin, GPIO_PIN_SET); 	// set the Trig pin high
	  __HAL_TIM_SET_COUNTER(&htim1, 0);								// reset the counter
	  while (__HAL_TIM_GET_COUNTER (&htim1) < 10);  				// wait for 10 us (microseconds)
	  HAL_GPIO_WritePin(Trig_GPIO_Port, Trig_Pin, GPIO_PIN_RESET); 	// set the Trig pin low

	  // Record the time it takes for the wave to return
	  pMillis = HAL_GetTick();
	  while (!(HAL_GPIO_ReadPin (Echo_GPIO_Port, Echo_Pin)) && ((pMillis + 10) >  HAL_GetTick())); // wait for the Echo pin to go high
	  val1 = __HAL_TIM_GET_COUNTER(&htim1); 						// record the time

	  // wait for the Echo pin to go low
	  pMillis = HAL_GetTick();
	  while ((HAL_GPIO_ReadPin (Echo_GPIO_Port, Echo_Pin)) && ((pMillis + 50) > HAL_GetTick())); // wait for the Echo pin to go low
	  val2 = __HAL_TIM_GET_COUNTER(&htim1); 						// record the time

	  // calculate the distance in cm
	  distance = (val2-val1) * 0.034/2;



	  /// Speed Logic and Display
	  float speedf250 = (old_distance - distance) * 0.089476; 	// calculate the speed in mph
	  speed250 = (int) speedf250; 								// convert the speed to an integer

	  display_speed(speed250); 									// Display the speed of the car



	  /// Display the Distance in Feet and Inches
	  uint16_t inches = distance * 0.393701; // convert the distance from cm to inches
	  uint16_t feet = inches / 12;
	  inches %= 12; 						 // get the remainder of inches

	  display_distance_ft(feet);
	  display_distance_in(inches);



	  /// Update the old distance to be used in the next iteration
	  old_distance = distance;



	  /// Red LED Logic
	  caution_speed = speed_limit_digits * 0.8; // set the caution speed limit to 20% below the speed limit
      int PWM_PERIOD = 39999; 					// how many clock cycles for each PWM period
      int current_speed = speed250;

      if (current_speed < 0){ 					// if the speed is negative (aka car is moving away from the sensor)
		  current_speed *= -1;					// make the speed positive
	  }

      if (current_speed < caution_speed){		// if the speed is less than the caution speed limit
		  TIM3->CCR2 = (int) (0); 				// store 0% duty cycle (aka red LED off)
  	  } else if (current_speed >= caution_speed && current_speed < speed_limit_digits){	// if the speed is greater than the caution speed limit and less than the speed limit
		  TIM3->CCR2 = (int) (0.1 * PWM_PERIOD);// store 10% duty cycle (aka red LED on dim)
	  }
	  else { 									// if the speed is greater than the speed limit
		  TIM3->CCR2 = (int) (PWM_PERIOD); 		// store 100% duty cycle (aka red LED on bright)
	  }


	  /// Wait for 250 milliseconds
	  HAL_Delay(250);

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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 79;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 39999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Trig_Pin|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Trig_Pin PB6 */
  GPIO_InitStruct.Pin = Trig_Pin|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : Echo_Pin */
  GPIO_InitStruct.Pin = Echo_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Echo_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DATA_AVAILABLE_Pin */
  GPIO_InitStruct.Pin = DATA_AVAILABLE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DATA_AVAILABLE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DATA0_Pin DATA1_Pin DATA2_Pin DATA3_Pin */
  GPIO_InitStruct.Pin = DATA0_Pin|DATA1_Pin|DATA2_Pin|DATA3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void SPI_write(unsigned char data){
	// Chip Enable (low is asserted)
	HAL_GPIO_WritePin(CE_PORT, CE_PIN, GPIO_PIN_RESET);
	// Send data over SPI1
	HAL_SPI_Transmit(&hspi1, (uint8_t*) &data, 1, HAL_MAX_DELAY);
	// Chip Disable
	HAL_GPIO_WritePin(CE_PORT, CE_PIN, GPIO_PIN_SET);
}

void GLCD_data_write(unsigned char data){
	// Switch to "data" mode (D/C pin high)
	HAL_GPIO_WritePin(DC_PORT, DC_PIN, GPIO_PIN_SET);
	// Send data over SPI
	SPI_write(data);
}

void GLCD_command_write(unsigned char data){
	// Switch to "command" mode (D/C pin low)
	HAL_GPIO_WritePin(DC_PORT, DC_PIN, GPIO_PIN_RESET);
	// Send data over SPI
	SPI_write(data);
}

void GLCD_init(void){
	// Keep CE high when not transmitting
	HAL_GPIO_WritePin(CE_PORT, CE_PIN, GPIO_PIN_SET);
	// Reset the screen (low pulse - down & up)
	HAL_GPIO_WritePin(RESET_PORT, RESET_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(RESET_PORT, RESET_PIN, GPIO_PIN_SET);
	// Configure the screen (according to the datasheet)
	GLCD_command_write(0x21); // enter extended command mode
	GLCD_command_write(0xB0); // set LCD Vop for contrast (this may be adjusted)
	GLCD_command_write(0x04); // set temp coefficient
	GLCD_command_write(0x15); // set LCD bias mode (this may be adjusted)
	GLCD_command_write(0x20); // return to normal command mode
	GLCD_command_write(0x0C); // set display mode normal
}

void GLCD_setCursor(unsigned char x, unsigned char y){
	GLCD_command_write(0x80 | x); // column
	GLCD_command_write(0x40 | y); // bank
}

void GLCD_clear(void){
	int i;
	for(i = 0; i < (GLCD_WIDTH * NUM_BANKS); i++){
	GLCD_data_write(0x00); 	// write zeros
	}
	GLCD_setCursor(0,0); 	// return cursor to top left
}

void GLCD_putchar(int font_table_row){
	 int i;
	 for (i=0; i<6; i++){
	 GLCD_data_write(font_table[font_table_row][i]);
	 }
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	// Check if the interrupt was for Data Available Pin
	if (HAL_GPIO_ReadPin(DATA_AVAILABLE_GPIO_Port, DATA_AVAILABLE_Pin) == GPIO_PIN_SET) {
		GLCD_setCursor(next_bank,0); // set the cursor
		next_bank += 6; // move to the next bank

		// Read data
		uint8_t key = keypad_decode();  // determine which key was pressed

		// Based on what key was pressed, do something (each case)
		switch (key) {
			case 0xA: // button "A" was pressed
//				GLCD_putchar(1); // 'A'
				break; // end case 0xA

			case 0xB: // button "B" was pressed
//				GLCD_putchar(2); // 'B'
				break; // end case 0xB

			case 0xC: // button "C" was pressed
//				GLCD_putchar(3); // 'C'
				break; // end case 0xC

			case 0xD: // button "D" was pressed
//				GLCD_putchar(4); // 'D'
				break; // end case 0xD

			case 0xE: // button E (*) was pressed
//				GLCD_putchar(40); // '*'
				break;

			case 0xF: // button F (#) was pressed
//				GLCD_putchar(41); // '#'
				break;

			case 0x0: // button "0" was pressed
//				GLCD_putchar(30); // '0'
				speed_limit_digits = 0;
				break;

			case 0x1: // button "1" was pressed
//				GLCD_putchar(31); // '1'
				speed_limit_digits = 1;
				break;

			case 0x2: // button "2" was pressed
//				GLCD_putchar(32); // '2'
				speed_limit_digits = 2;
				break;

			case 0x3: // button "3" was pressed
//				GLCD_putchar(33); // '3'
				speed_limit_digits = 3;
				break;

			case 0x4: // button "4" was pressed
//				GLCD_putchar(34); // '4'
				speed_limit_digits = 4;
				break;

			case 0x5: // button "5" was pressed
//				GLCD_putchar(35); // '5'
				speed_limit_digits = 5;
				break;

			case 0x6: // button "6" was pressed
//				GLCD_putchar(36); // '6'
				speed_limit_digits = 6;
				break;

			case 0x7: // button "7" was pressed
//				GLCD_putchar(37); // '7'
				speed_limit_digits = 7;
				break;

			case 0x8: // button "8" was pressed
//				GLCD_putchar(38); // '8'
				speed_limit_digits = 8;
				break;

			case 0x9: // button "9" was pressed
//				GLCD_putchar(39); // '9'
				speed_limit_digits = 9;
				break;
		}

	}

}


// This function reads the four data pins from the keypad encoder and maps them to the key value
unsigned char keypad_decode() {
	unsigned char key = 0x0;
	unsigned char data = 0b0000;

	// read the data pins and combine into the 4-bit value: D3_D2_D1_D0
	data |= (HAL_GPIO_ReadPin(DATA3_GPIO_Port, DATA3_Pin) << 3);
	data |= (HAL_GPIO_ReadPin(DATA2_GPIO_Port, DATA2_Pin) << 2);
	data |= (HAL_GPIO_ReadPin(DATA1_GPIO_Port, DATA1_Pin) << 1);
	data |= (HAL_GPIO_ReadPin(DATA0_GPIO_Port, DATA0_Pin) << 0);

	// The key encoder gives the following "data" values:
	// 0 1 2 3
	// 4 5 6 7
	// 8 9 A B
	// C D E F

	// The following switch statement re-maps it to these "key" names:
	// 1 2 3 A
	// 4 5 6 B
	// 7 8 9 C
	// E 0 F D, where E is "*" and F is "#"

   // Finish this switch statement to remap the "data" to the correct "key"
	switch (data) {
      case 0x0:
         key = 0x1;
         break;
      case 0x1:
         key = 0x2;
         break;
      case 0x2:
         key = 0x3;
         break;
      case 0x3:
         key = 0xA;
         break;
      case 0x4:
         key = 0x4;
         break;
      case 0x5:
         key = 0x5;
         break;
      case 0x6:
         key = 0x6;
         break;
      case 0x7:
         key = 0xB;
         break;
      case 0x8:
         key = 0x7;
         break;
      case 0x9:
         key = 0x8;
         break;
      case 0xA:
         key = 0x9;
         break;
      case 0xB:
         key = 0xC;
         break;
      case 0xC:
         key = 0xE; // *
         break;
      case 0xD:
         key = 0x0;
         break;
      case 0xE:
         key = 0xF; // #
         break;
      case 0xF:
         key = 0xD;
         break;
	}

	return key;
}

void GLCD_puts(const char* str) {
    while (*str) {
        if (*str >= '0' && *str <= '9') {	// If the character is a number
            GLCD_putchar(*str - '0' + 30); 	// Convert char to font_table index
        } else if (*str == '-') {
        	GLCD_putchar(43); 				// Put '-' (negative sign)
        }
        else if (*str == ' ') {
        	GLCD_putchar(0); 				// Put ' ' (space)
		}
		str++;
	}
}


void display_speed(int speed) {
    char buffer[4]; 		// Buffer to hold the speed string, including space for the negative sign
    snprintf(buffer, sizeof(buffer), "%3d", speed); // Convert speed to string with 3 characters width
    GLCD_setCursor(0, 3); 	// Set cursor to the forth row
    GLCD_puts(buffer); 		// Display the speed string
}


void display_distance(uint16_t distance_cm) {
    char buffer[5]; 		// Buffer to hold the distance string
    snprintf(buffer, sizeof(buffer), "%3d", distance_cm); // Convert distance to string (3 digits)
    GLCD_setCursor(0, 2); 	// Set cursor to the third row
    GLCD_puts(buffer); 		// Display the distance string
}

void display_distance_ft(uint16_t distance_ft) {
    char buffer[3]; 		// Buffer to hold the distance string
    snprintf(buffer, sizeof(buffer), "%2d", distance_ft); // Convert distance to string (2 digits)
    GLCD_setCursor(0, 5); 	// Set cursor to the sixth row
    GLCD_puts(buffer); 		// Display the distance string
}

void display_distance_in(uint16_t distance_in) {
    char buffer[3]; 		// Buffer to hold the distance string
    snprintf(buffer, sizeof(buffer), "%2d", distance_in); // Convert distance to string (2 digits)
    GLCD_setCursor(36, 5); 	// Set cursor to the sixth row, 36th column
    GLCD_puts(buffer); 		// Display the distance string
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
