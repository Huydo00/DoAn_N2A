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
#include "stdio.h"
#include "string.h"
#include "i2c-lcd.h"

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
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint16_t	readPOT[1];
int processPOT;
int valuePOT;
int valuePWM_L;
int valuePWM_R;
int DIRR=1;
int DIRL=1;

void Forward(){
		HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, 0);
		HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, 1);
		HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, 0);
		HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, 1);
}
void Back(){
		HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, 1);
		HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, 0);
		HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, 1);
		HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, 0);
}
void Stop(){
		HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, 0);
		HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, 0);
		HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, 0);
		HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, 0);
}

void PWMMOTOR(){
	if(DIRL > 0){
		__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_3, valuePWM_L);
		HAL_GPIO_WritePin(IN3_GPIO_Port, IN1_Pin, 1);
		HAL_GPIO_WritePin(IN4_GPIO_Port, IN2_Pin, 0);
	}
	if(DIRR > 0){
		__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_4, valuePWM_R);
		HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, 1);
		HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, 0);
	}
	
	if(DIRL < 0){
		__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_3, valuePWM_L);
		HAL_GPIO_WritePin(IN3_GPIO_Port, IN1_Pin, 0);
		HAL_GPIO_WritePin(IN4_GPIO_Port, IN2_Pin, 1);
	}
	if(DIRR < 0){
		__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_4, valuePWM_R);
		HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, 0);
		HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, 1);
	}
	
	if(valuePWM_R == 0){
		__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_4, 0);
		HAL_GPIO_WritePin(IN1_GPIO_Port, IN3_Pin, 0);
		HAL_GPIO_WritePin(IN2_GPIO_Port, IN4_Pin, 0);
	}
	if(valuePWM_L == 0){
		__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_3, 0);
		HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, 0);
		HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, 0);
	}
	
}
volatile short eR;
volatile short enR;
volatile short eL;
volatile short enL;
int btnStart=0;
int valueStart=0;
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
		if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == 0){
			btnStart = 1;
			
		}
	
	 if(GPIO_Pin == GPIO_PIN_4) {
			if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) == 0) {
					eR++;
			} else {
					eR--;
			}
    }
	 
	 if(GPIO_Pin == GPIO_PIN_8) {
			if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9) == 0){
					eL--;
			} else {
					eL++;
			}
   }
}

float taget=0;
float valueL=0;
float cnt=0;
float valueE=0;
float valueEXP=0;
float kd1=0.6; //0.6
float kp1=0.7; //5.5 			
float kd2=0.7;  
float kp2=0.7;
float output=0;
float theta0=0;
float theta1=0;
float theta2=0;
float theta3=0;
float W1=0;
float W2=0;
float POT=0;
float Wr1=0;
float Wr2=0;
float e1=0;
float e2=0;
float e3=0;
float e4=0;
float t=0;
float a=0;
float smax=2;
float vmax=0.65;
float tf=3.25;
float t1=0;
float t2=0;
float x=0;
float output1=0;
float output2=0;
float output3=0;
float output4=0;
float start=0;
int value =0;
void cal(){
	if(valuePWM_R > 0 && valuePWM_R <= 500){
		valuePWM_R = 500;
	}
//	if(valuePWM_R >= -500){
//		valuePWM_R = -500;
//	}
	if(valuePWM_R >= 1799){
		valuePWM_R = 1799;
	}
	if(valuePWM_R <= -1799){
		valuePWM_R = -1799;
	}
	
	if(valuePWM_R<0){
		valuePWM_R = valuePWM_R*(-1);
		DIRR = -1;
	}
	else if(valuePWM_R>0){
		valuePWM_R = valuePWM_R*1;
		DIRR = 1;
	}
	
	if(valuePWM_L > 0 && valuePWM_L <= 500){
		valuePWM_L = 500;
	}
//	if(valuePWM_L >= -500){
//		valuePWM_L = -500;
//	}
	if(valuePWM_L >= 1799){
		valuePWM_L = 1799;
	}
	if(valuePWM_L <= -1799){
		valuePWM_L = -1799;
	}
	if(valuePWM_L<0){
		valuePWM_L = valuePWM_L*(-1);
		DIRL = -1;
	}
	else if(valuePWM_L>0){
		valuePWM_L = valuePWM_L*1;
		DIRL = 1;
	}
	
	PWMMOTOR();
}
int buffer[700];
int j=0;
int k=0;
int valueUART = 0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	enL = eL*4;
	enR = eR*4;
//	HAL_ADC_Start(&hadc1);
//	POT = HAL_ADC_GetValue(&hadc1);
//	tf = POT*4/4096;
	if(valueStart == 1){
		a = vmax/t1;
		t1 = (vmax*tf-smax)/vmax;
		t2 = tf-2*t1;
		t += 0.01;
		
		if(j<=900){
			j++;
			buffer[j] = enL;
		}
//		if(j==701){
//			for(k=0; k<=700; k++){	
//				HAL_UART_Transmit(&huart3, (uint8_t*)buffer[k],sizeof(buffer[k]),100);
//			}
//		}
		
		if(t<t1){
			x = 0.5 * a *(t*t);
		}
		else if(t < tf-t1){
			x = 0.5*a*(t1*t1) + vmax*(t-t1);
		}
		else if(t < tf){
			x = 0.5*a*t1*t1 + vmax*t2 + vmax*(t-t1-t2) + 0.5*a*(t-t1-t2)*(t-t1-t2);
		}
	}
	//cnt = (x*968)/(3.14*0.065);
	cnt = (x/(3.14*0.065))*936;
	
	//################RIGHT################
	theta1 = enR;
	e1 = cnt-enR;
	W1 = (theta1-theta0)*100;
	output1 = kp1*e1 - kd1*W1;
	valuePWM_R = output1*1799/936;
	theta0=theta1;
	
	//################LEFT################
	theta3 = enL;
	e1 = cnt-enL;
	W1 = (theta3-theta2)*100;
	output2 = kp2*e1 - kd2*W1;
	valuePWM_L = output2*1799/936;
	theta2=theta3;
	
//	if(t>=tf){
//		valuePWM_L = 0;
//		valuePWM_R = 0;
//		Stop();
//	}
	
	cal();
}
//uint8_t senddata[]="Hello STM ->ESP";
//char rec,null;
//char buffer[100];
//int i=0;
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//	  if(rec!=13) buffer[i++]=rec;
//	if(rec==13) 
//	{
//		i=0;
//		
//		HAL_UART_Transmit(&huart3,(uint8_t *)&buffer, sizeof(buffer),100);
//		
//		for(int cnt=0; cnt < sizeof(buffer);cnt++) buffer[cnt]=NULL;
//	}
//	HAL_UART_Receive_IT(&huart3,(uint8_t *)&rec,1);
//	
//}
void lcd(){
		lcd_clear();
		lcd_put_cur(0,0);
		lcd_send_string("buf:");
		lcd_send_number((int)j);
		
		lcd_put_cur(0,8);
		lcd_send_string("TIME:");
		lcd_send_number((float)start);
		
		lcd_put_cur(1,0);
		lcd_send_string("T:");
		lcd_send_number((float)t);
		
		lcd_put_cur(1,8);
		lcd_send_string("PWM:");
		lcd_send_number((int)valuePWM_L);
}
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
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_I2C1_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
	lcd_init();
	lcd_put_cur(0,3);
	lcd_send_string("DA2 - N2A");
	lcd_put_cur(1,3);
	lcd_send_string("DH CDT17A");
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);
	//HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_1);
	//HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_2);
//	HAL_TIM_Encoder_Start(&htim1,TIM_CHANNEL_1);
//	HAL_TIM_Encoder_Start(&htim1,TIM_CHANNEL_2);
	HAL_TIM_Base_Start_IT(&htim2);
//	for(int i=1; i<=6; i++){
//		HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
//		HAL_Delay(500);
//		start += 0.5;
//		lcd();
//		
//	}
//	btnStart = 1;
	
	


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	uint8_t test[10] = "\n";
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//		if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13  ) == 0){
//			valueUART = 1;
//		}
//		if(valueUART == 1){
//			for(int k=0; k<=700; k++){	
//				HAL_UART_Transmit(&huart3, (uint8_t*)buffer[k],sizeof(buffer[k]),100);
//			}
//		}s
		if(valueStart == 0){
			if(btnStart == 1){
				for(int i=1; i<=6; i++){
				HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13)	;
				HAL_Delay(500);
				}
				valueStart = 1;
			}
		}
		HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
		HAL_Delay(100);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 179;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 3999;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 9;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1799;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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
  huart3.Init.BaudRate = 115200;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, IN3_Pin|IN4_Pin|IN1_Pin|IN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : IN3_Pin IN4_Pin IN1_Pin IN2_Pin */
  GPIO_InitStruct.Pin = IN3_Pin|IN4_Pin|IN1_Pin|IN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
