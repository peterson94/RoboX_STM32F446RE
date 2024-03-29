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
#define LCD_CLEAR                 ((uint8_t)0x01)
#define LCD_RETURN                ((uint8_t)0x02)
#define LCD_NEWLINE               ((uint8_t)0xC0)
#define LCD_SHIFT_RIGHT           ((uint8_t)0x14)
#define LCD_SHIFT_LEFT            ((uint8_t)0x10)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_rx;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */

uint32_t IC_Value1;
uint32_t IC_Value2;

uint32_t PWM_1;
uint32_t PWM_2;

uint32_t RPM_1;
uint32_t RPM_2;
uint32_t IC_CNT_1=0;
uint32_t IC_CNT_2=0;

uint8_t cmd[10] = {0};
uint8_t cmd_prev[3] = {0};
unsigned int i;

uint8_t gyro_data[14] = {0};
uint8_t COMMAND[2] = {0};
uint8_t data;
HAL_StatusTypeDef IS_READY = HAL_ERROR;

int16_t axis_avg_z[1000] = {0};

int64_t accel_off_x = 0;
int64_t accel_off_y = 0;
int64_t accel_off_z = 0;
int64_t axis_off_x = 0;
int64_t axis_off_y = 0;
double axis_off_z = 0.0;
int64_t temperature_off = 0;
double z_angle = 0.0;

int16_t accel_x = 0;
int16_t accel_y = 0;
int16_t accel_z = 0;
int16_t axis_x = 0;
int16_t axis_y = 0;
int16_t axis_z = 0;
int16_t temperature = 0;
uint8_t config=0x00;
uint8_t sensor_base=0x3B;

uint8_t FLAG_UART_1 = 0x00;
uint8_t FLAG_TIMEOUT = 0x00;
uint8_t FLAG_IC = 0x00;
uint8_t FLAG_LCD = 0x00;
uint8_t FLAG_TIM10 = 0x00;
uint8_t FLAG_TIM11 = 0x00;
uint8_t FLAG_MEAS_1 = 0x00;
uint8_t FLAG_MEAS_2 = 0x00;

// ############################### PID CONTROLLER BEGIN #############################//
float base;

float err_1 = 0.0f;
float err_1_sum = 0.0f;
float err_1_prev = 0.0f;

float kp_1 = 0.25f;
float ki_1 = 0.00001f;
float kd_1 = 0.0f;

float err_2 = 0.0f;
float err_2_sum = 0.0f;
float err_2_prev = 0.0f;

float kp_2 = 0.25f;
float ki_2 = 0.00001f;
float kd_2 = 0.0f;

uint32_t dt = 1;
// ############################### PID CONTROLLER END ##############################//

// ############################### PI CONTROLLER BEGIN #############################//




// ############################### PI CONTROLLER END #############################//

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM10_Init(void);
static void MX_TIM11_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
static void Motor_Set(uint8_t);
static void Motor_Stop(void);
static void LCD_Init(void);
static void LCD_Send_Char(uint8_t, GPIO_PinState);
static void LCD_Send_Variable(int);
static void LCD_Send_String(char*);
static void LCD_Send_CMD(uint8_t);
static void Control_Set(void);
static void Speed_Calc(void);
static void Speed_Control(void);
static void Battery_Meas(void);
static void Gyro_Calib(void);
static void Gyro_Calc(void);
float Base_Calc(float, uint8_t);

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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM6_Init();
  MX_USART2_UART_Init();
  MX_TIM7_Init();
  MX_TIM10_Init();
  MX_TIM11_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  /* ########################################################################################################### */
  /* ############################       Start sequence comes here      ######################################### */
  /* ########################################################################################################### */
  PWM_1 = 0;
  PWM_2 = 0;

  RPM_1 = 0;
  RPM_2 = 0;

  COMMAND[0] = 0x75;
  COMMAND[1] = 0x48;

  LCD_Init();
 //LCD_Send_String("X: Y: ");
 //LCD_Send_CMD(LCD_NEWLINE);
 //LCD_Send_String("Angle: ");
 //LCD_Send_CMD(LCD_RETURN);
 //

  //Gyro_Calib();
  Battery_Meas();

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

  HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim4,TIM_CHANNEL_1);

  HAL_TIM_Base_Start_IT(&htim6);
  HAL_TIM_Base_Start_IT(&htim7);
  HAL_TIM_Base_Start_IT(&htim10);
  HAL_TIM_Base_Start_IT(&htim11);
  HAL_UART_Receive_DMA(&huart1, cmd, 3);
  HAL_I2C_Master_Transmit(&hi2c1, 0x68<<1, &sensor_base, 1, 100);
  HAL_I2C_Master_Receive_DMA(&hi2c1, 0x68<<1, gyro_data, 14);

  /* ########################################################################################################### */
  /* ###############################       Start sequence ends here      ####################################### */
  /* ########################################################################################################### */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  /* ########################################################################################################### */
  /* #####################                     BEGIN WHILE LOOP HERE                       ##################### */
  /* ########################################################################################################### */
  while (1)
  {

	  Control_Set();
	  Speed_Calc();

	//if (FLAG_TIM11)
	//{
	// Gyro_Calc();
	// FLAG_TIM11 =0x00;
	// HAL_TIM_Base_Start_IT(&htim11);
	//}

	  Speed_Control();

      if (FLAG_LCD)
      {
      	 //LCD_Send_Variable(htim2.Instance->CCR1);
      	 //LCD_Send_CMD(LCD_RETURN);

    	 //LCD_Send_Variable(temperature/340 + 36.53);
    	 //LCD_Send_Char(0xDF,1);
    	 //LCD_Send_String("C");

    	  LCD_Send_String("C1:");
    	  LCD_Send_Variable(IC_Value1);
    	  //LCD_Send_Variable(RPM_1);
    	  LCD_Send_String("  ");
    	  LCD_Send_CMD(LCD_RETURN);

    	  LCD_Send_CMD(LCD_SHIFT_RIGHT);
    	  LCD_Send_CMD(LCD_SHIFT_RIGHT);
    	  LCD_Send_CMD(LCD_SHIFT_RIGHT);
    	  LCD_Send_CMD(LCD_SHIFT_RIGHT);
    	  LCD_Send_CMD(LCD_SHIFT_RIGHT);
    	  LCD_Send_CMD(LCD_SHIFT_RIGHT);
    	  LCD_Send_CMD(LCD_SHIFT_RIGHT);
    	  LCD_Send_CMD(LCD_SHIFT_RIGHT);
    	  LCD_Send_CMD(LCD_SHIFT_RIGHT);
    	  LCD_Send_CMD(LCD_SHIFT_RIGHT);

    	  LCD_Send_String("R2:");
    	  LCD_Send_Variable(RPM_2);
    	  LCD_Send_String("  ");
    	  LCD_Send_CMD(LCD_RETURN);

      	  FLAG_LCD = 0x00;
      	  HAL_TIM_Base_Start_IT(&htim7);
      }

      if (FLAG_TIM10)
      {
    	  Battery_Meas();
    	  FLAG_TIM10 = 0x00;
    	  HAL_TIM_Base_Start_IT(&htim10);
      }

  /* ########################################################################################################### */
  /* #####################                      END WHILE LOOP HERE                        ##################### */
  /* ########################################################################################################### */
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV6;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV8;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV8;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 16-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 20000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 4000-1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 16000-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0xFFFF;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 16000-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 0xFFFF;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  htim6.Init.Prescaler = 16000-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 150-1;
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
  htim7.Init.Prescaler = 16000-1;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 200-1;
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
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 16000-1;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 60000-1;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 16-1;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 1000-1;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */

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
  huart1.Init.Mode = UART_MODE_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 7, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 7, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC0 PC1 PC2 PC3
                           PC4 PC5 PC6 PC7
                           PC8 PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/* ############################################################################################################## */
/* ##########                            FUNCTIONS STARTS FROM HERE                                    ########## */
/* ############################################################################################################## */

static void Motor_Set(uint8_t motor_state)
{
	base = 140.5;

	switch(motor_state)
	{
		case '4':
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
			  break;
		case '2':
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
			  break;
		case '1':
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
			  break;
		case '3':
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
			  break;
	}

}

static void Motor_Stop()
{
	  base = 0;
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
}

static void Control_Set()
{
	if(FLAG_UART_1)
	{
		FLAG_UART_1 = 0x00;

		// Set motor if request changed
		if (cmd[1]!=cmd_prev[1])
		{
			HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
			HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
			Motor_Set(cmd[1]);
			cmd_prev[1]=cmd_prev[1];
		}

		// We want to receive the next command
		HAL_UART_Receive_DMA(&huart1, cmd, 3);

		// Reset TIMEOUTER
		__HAL_TIM_SET_COUNTER(&htim6,0);
	}

	if(FLAG_TIMEOUT)
	{
		FLAG_TIMEOUT = 0x00;

		// We do not have control for a while so let us stop motors and clear calc values
		Motor_Stop();
		RPM_1 = 0;
		RPM_2 = 0;
		IC_CNT_1=0;
		IC_CNT_2=0;
		cmd[1]='0';
		cmd_prev[1]='0';

		// And let us start TIMEOUTER again
		HAL_TIM_Base_Start_IT(&htim6);

		// Stop Speed Meas
		HAL_TIM_IC_Stop_IT(&htim3, TIM_CHANNEL_1);
		HAL_TIM_IC_Stop_IT(&htim3, TIM_CHANNEL_1);
		FLAG_MEAS_2 = 0x00;
		IC_Value2 = 0;
		FLAG_MEAS_1 = 0x00;
		IC_Value1 = 0;
		__HAL_TIM_SET_COUNTER(&htim3,0);
		__HAL_TIM_SET_COUNTER(&htim4,0);
	}
}

static void Speed_Calc()
{
	if (FLAG_MEAS_1 && FLAG_MEAS_2 == 0x01)
	{
		if(IC_Value1 && IC_Value2)
		{
			RPM_1 = 3000/IC_Value1;
			RPM_2 = 3000/IC_Value2;
		}

		else
		{
			RPM_1 = 0;
			RPM_2 = 0;
		}
	}
}

static void Speed_Control()
{
	if (base)
	{
		err_1 = 20000 * (base - RPM_1)/base;
		err_2 = 20000 * (base - RPM_2)/base;

		htim2.Instance->CCR1 = kp_1*err_1 + ki_1*err_1_sum;
		htim2.Instance->CCR2 = kp_2*err_2 + ki_2*err_2_sum;
		//htim2.Instance->CCR1 = 2500;
		//htim2.Instance->CCR2 = 2500;
		err_1_sum += err_1;
		err_2_sum += err_2;
	}

	else
	{
		err_1 = 0;
		err_2 = 0;

		err_1_sum = 0;
		err_2_sum = 0;

		htim2.Instance->CCR1 = 20000 * kp_1;
		htim2.Instance->CCR2 = 20000 * kp_2;
	}
}

static void Battery_Meas()
{
	  uint32_t BAT;

	  HAL_ADC_Start(&hadc1);
	  HAL_ADC_PollForConversion(&hadc1, 10);
	  BAT = HAL_ADC_GetValue(&hadc1);
	  BAT = BAT * 8.057 * 2.47 / 1000;

	  LCD_Send_CMD(LCD_NEWLINE);

	  LCD_Send_String("BAT:  [");
	  //LCD_Send_Char(0x7C,1);
	  if(BAT > 60)
		  LCD_Send_Char(0xFF,1);
	  else LCD_Send_String(" ");

	  if(BAT > 64)
		  LCD_Send_Char(0xFF,1);
	  else LCD_Send_String(" ");

	  if(BAT > 68)
		  LCD_Send_Char(0xFF,1);
	  else LCD_Send_String(" ");

	  if(BAT > 72)
		  LCD_Send_Char(0xFF,1);
	  else LCD_Send_String(" ");

	  if(BAT > 76)
		  LCD_Send_Char(0xFF,1);
	  else LCD_Send_String(" ");

	  LCD_Send_String("]");
	  //LCD_Send_Char(0x7C,1);

	  LCD_Send_CMD(LCD_RETURN);
}

static void Gyro_Calib()
{

	  uint8_t i;
	  uint8_t mpu_reset = 0x80;
	  uint8_t config = 0x00;
	  uint8_t sleep = 0x40;

	  HAL_I2C_Mem_Write(&hi2c1, 0x68<<1, 0x6B, 1, &mpu_reset, 1, 10);
	  HAL_Delay(1);
	  HAL_I2C_Mem_Write(&hi2c1, 0x68<<1, 0x6B, 1, &config, 1, 10);
	  HAL_Delay(1);

	  HAL_I2C_Mem_Write(&hi2c1, 0x68<<1, 0x6B, 1, &sleep, 1, 10);
	  HAL_Delay(1);
	  HAL_I2C_Mem_Write(&hi2c1, 0x68<<1, 0x6B, 1, &config, 1, 10);
	  HAL_Delay(1);


	  while (IS_READY != HAL_OK)
	  {
		  IS_READY = HAL_I2C_IsDeviceReady(&hi2c1, 0x68<<1, 1, 100);
	  	  LCD_Send_String("Gyro error. Restart please!");
	  	__HAL_I2C_CLEAR_FLAG(&hi2c1, I2C_FLAG_BUSY);

		 // HAL_I2C_Mem_Write(&hi2c1, 0x68<<1, 0x6B, 1, &mpu_reset, 1, 10);
		 // HAL_Delay(1);
		 // HAL_I2C_Mem_Write(&hi2c1, 0x68<<1, 0x6B, 1, &config, 1, 10);
		 // HAL_Delay(1);
         //
		 // HAL_I2C_Mem_Write(&hi2c1, 0x68<<1, 0x6B, 1, &sleep, 1, 10);
		 // HAL_Delay(1);
		 // HAL_I2C_Mem_Write(&hi2c1, 0x68<<1, 0x6B, 1, &config, 1, 10);
		 // HAL_Delay(1);

	  }

	  LCD_Send_CMD(LCD_CLEAR);
	  LCD_Send_CMD(LCD_RETURN);

	  LCD_Send_String("Gyro calib");
	  HAL_Delay(500);
	  LCD_Send_String(".");
	  HAL_Delay(500);
	  LCD_Send_String(".");
	  HAL_Delay(500);
	  LCD_Send_String(".");
	  HAL_Delay(500);

	  LCD_Send_CMD(LCD_CLEAR);
	  LCD_Send_CMD(LCD_RETURN);

	  for (i=0; i <100; i++)
	  {
		  HAL_I2C_Master_Transmit(&hi2c1, 0x68<<1, &sensor_base, 1, 100);
		  HAL_I2C_Master_Receive(&hi2c1, 0x68<<1, gyro_data, 14 ,100);

	      axis_avg_z[i] = gyro_data[12] << 8 | gyro_data[13];
	      HAL_Delay(1);

	  }

	  for (i=0; i <100; i++)
	  {
		  axis_off_z += (axis_avg_z[i] * 0.00762939453125) / 1000;
	  }


	  axis_off_z /= 100;
}

static void Gyro_Calc()
{
	  accel_x		= (gyro_data[0] << 8 | gyro_data[1]);
	  accel_y		= (gyro_data[2] << 8 | gyro_data[3]);
	  accel_z		= (gyro_data[4] << 8 | gyro_data[5]);

	  temperature	= (gyro_data[6] << 8 | gyro_data[7]);

	  axis_x		= (gyro_data[8] << 8 | gyro_data[9]) - 		axis_off_x;
	  axis_y		= (gyro_data[10] << 8 | gyro_data[11]) - 	axis_off_y;
	  axis_z		= (gyro_data[12] << 8 | gyro_data[13]);

	  z_angle += (axis_z * 0.00762939453125) /1000 - axis_off_z;
}

/* ########################  TIMER_1 interrupt ######################## */
/* ########################  TIMER_3 interrupt ######################## */

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM4)
	{
		if (FLAG_MEAS_1 != 0x00)
			IC_Value1 = HAL_TIM_ReadCapturedValue(&htim4, TIM_CHANNEL_1);
		else FLAG_MEAS_1 = 0x01;

		//IC_CNT_1 += 1;
	}

	if(htim->Instance == TIM3)
	{
		if (FLAG_MEAS_2 != 0x00)
			IC_Value2 = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_1);
		else FLAG_MEAS_2 = 0x01;

		//HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
		//IC_CNT_2 += 1;
	}

	__HAL_TIM_SET_COUNTER(htim,0);
}


void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	HAL_I2C_Master_Transmit(&hi2c1, 0x68<<1, &sensor_base, 1, 100);
	HAL_I2C_Master_Receive_DMA(&hi2c1, 0x68<<1, gyro_data, 14);
}

/* ########################  TIMER GLOBAL interrupt ######################## */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	/* Set flag */
	if (htim->Instance == TIM6)
		FLAG_TIMEOUT = 0x01;

	if (htim->Instance == TIM7)
		FLAG_LCD = 0x01;

	if (htim->Instance == TIM10)
		FLAG_TIM10 = 0x01;

	if (htim->Instance == TIM11)
		FLAG_TIM11 = 0x01;
}

/* ########################  UART_1 interrupt ######################## */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	/* Set flag */
	FLAG_UART_1 = 0x01;
}

static void LCD_Init(void)
{
	// 8 bit initialization
	HAL_Delay(50);  // wait for >40ms
	LCD_Send_CMD (0x30);
	HAL_Delay(5);  // wait for >4.1ms
	LCD_Send_CMD (0x30);
	HAL_Delay(1);  // wait for >100us
	LCD_Send_CMD (0x30);
	HAL_Delay(10);
	LCD_Send_CMD (0x30);  // 8bit mode
	HAL_Delay(10);

  // display initialization
	LCD_Send_CMD (0x38); // Function set --> DL=1 (8 bit mode), N = 1 (2 line display) F = 0 (5x8 characters)
	HAL_Delay(1);
	LCD_Send_CMD (0x08); //Display on/off control --> D=0,C=0, B=0  ---> display off
	HAL_Delay(1);
	LCD_Send_CMD (0x01);  // clear display
	HAL_Delay(1);
	LCD_Send_CMD (0x06); //Entry mode set --> I/D = 1 (increment cursor) & S = 0 (no shift)
	HAL_Delay(1);
	LCD_Send_CMD (0x0C); //Display on/off control --> D = 1, C and B = 0. (Cursor and blink, last two bits)
}

static void LCD_Send_Char(uint8_t BUFF, GPIO_PinState RS)
{
	uint32_t lcd_data;

	//if RS==0 -> cmd register is selected, if RS==1-> data register is selected
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, RS);

	//set enable pin high before sending data
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);

	lcd_data = GPIOC->ODR & ~(0xFF);
	lcd_data |= BUFF;

	//set data pins
	GPIOC->ODR = lcd_data;

	//set enable from high to low, send data out
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
	HAL_Delay(1);
}

static void LCD_Send_Variable(int var)
{
	int i=100000;

	if (var == 0)
	{
		LCD_Send_Char(0x30, GPIO_PIN_SET);
	}

	else
	{
		if (var < 0)
		{
			LCD_Send_Char(0x2D, GPIO_PIN_SET);
			var = -1 *var ;
		}

		while((var%i) == var)
					i=i/10;

		for (;i>0;i=i/10)
			LCD_Send_Char(0x30+(var/i)%10, GPIO_PIN_SET);
	}
}

static void LCD_Send_String (char *str)
{
	while (*str) LCD_Send_Char(*str++, GPIO_PIN_SET);
}

static void LCD_Send_CMD(uint8_t command)
{
	LCD_Send_Char(command, GPIO_PIN_RESET);
}

/* ############################################################################################################## */
/* ##########                                FUNCTIONS ENDS HERE                                       ########## */
/* ############################################################################################################## */
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
