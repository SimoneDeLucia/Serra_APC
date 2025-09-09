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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DIM 8
#define DIM_SCREEN 50
#define MIN_VALUE 0
#define MAX_VALUE 20
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */
/**
 * MODE E SELECT
 */
volatile uint8_t push_mode_button = 0;
volatile uint8_t push_select_button = 0;

/**
 * ADC
 */

uint8_t adc_data_ready = 0;
uint32_t adc_value = 0;
uint32_t adc_value_prev = 0;
uint32_t cont_misure_adc = 0;
/**
 * ENCODER
 */


int32_t rawCounter = 0;
int32_t last_rawCounter = 0;
int32_t counter = 0;
int32_t last_counter = 0;
uint32_t cont_misure_tim = 0;

/**
 * VALORI DA MOSTRARE A SCHERMO
 */

char SCREEN_TEXT[DIM_SCREEN] = "\0";
char selected_val_luce[DIM+1] = "\0";
char selected_val_temp[DIM+1] = "\0";
char actual_val_luce[DIM+1] = "\0";
char actual_val_temp[DIM+1] = "\0";

/**
 * SEMAFORI
 */

SemaphoreHandle_t sem_sel_luce;
SemaphoreHandle_t sem_sel_temp;
SemaphoreHandle_t sem_act_luce;
SemaphoreHandle_t sem_act_temp;



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
void StartDefaultTask(void *argument);

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
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */




  ssd1306_Init();


  ssd1306_SetCursor(0,0);
  ssd1306_FillRectangle(0,0,128,15, White);
  strcpy(SCREEN_TEXT, "Serra Digitale");
  ssd1306_WriteString(SCREEN_TEXT, Font_16x15, Black);
  ssd1306_UpdateScreen();

  HAL_GPIO_WritePin(RGB_LED_RED_GPIO_Port, RGB_LED_RED_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(RGB_LED_BLUE_GPIO_Port, RGB_LED_BLUE_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(YELLOW_LED_GPIO_Port, YELLOW_LED_Pin, GPIO_PIN_RESET);

  HAL_ADC_Start_IT(&hadc1);
  /* USER CODE END 2 */

  /* Init scheduler */
//  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
//  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
//  sem_sel_luce = xSemaphoreCreateBinary();
//  sem_sel_temp = xSemaphoreCreateBinary();
//  sem_act_luce = xSemaphoreCreateBinary();
//  sem_act_temp = xSemaphoreCreateBinary();
//
//  xSemaphoreGive(sem_sel_luce);
//  xSemaphoreGive(sem_sel_temp);
//  xSemaphoreGive(sem_act_luce);
//  xSemaphoreGive(sem_act_temp);
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
//  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
//  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
//  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
//  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
//  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
//  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if(push_mode_button == 0 && push_select_button == 0){
		  cont_misure_adc = 0;
			ssd1306_SetCursor(5,0);
			ssd1306_FillRectangle(0,0,128,15, White);
			snprintf(SCREEN_TEXT, sizeof(SCREEN_TEXT), "Temperatura \r\n");
			ssd1306_WriteString(SCREEN_TEXT, Font_16x15, Black);
			ssd1306_SetCursor(45, 30);
			snprintf(SCREEN_TEXT, sizeof(SCREEN_TEXT), "%d\r\n", (int)adc_value);
			ssd1306_WriteString(SCREEN_TEXT, Font_16x26, White);
			ssd1306_UpdateScreen();
		  if(adc_data_ready == 1)
			{
				if (adc_value_prev == adc_value) {
				    cont_misure_adc++;
				} else {
				    adc_value_prev = adc_value;
				}
				if(cont_misure_adc == 30){
					snprintf(actual_val_temp, sizeof(actual_val_temp), "%d", (int)adc_value);
					HAL_UART_Transmit(&huart1, (uint8_t *)actual_val_temp, sizeof(actual_val_temp), HAL_MAX_DELAY);
					cont_misure_adc = 0;
				}
				else{
					HAL_ADC_Start_IT(&hadc1);
				}
				adc_data_ready = 0;

			}
	  }
	  else if(push_mode_button == 0 && push_select_button == 1){
		  cont_misure_tim = 0;
			ssd1306_SetCursor(10,0);
			ssd1306_FillRectangle(0,0,128,15, White);
			snprintf(SCREEN_TEXT, sizeof(SCREEN_TEXT), "Luminosita \r\n");
			ssd1306_WriteString(SCREEN_TEXT, Font_16x15, Black);
			ssd1306_SetCursor(45,30);
			snprintf(SCREEN_TEXT, sizeof(SCREEN_TEXT), "%d \r\n", (int)counter);
			ssd1306_WriteString(SCREEN_TEXT, Font_16x26, White);
			ssd1306_UpdateScreen();

		   rawCounter = __HAL_TIM_GET_COUNTER(&htim3);

		   // Controlla se il valore è cambiato e in che direzione
		   if (rawCounter != last_rawCounter)
		   {
		       // Se il contatore del timer è aumentato
		       if (rawCounter > last_rawCounter)
		       {
		           // Incrementa il nostro contatore solo se non ha raggiunto il valore massimo
		           if (counter < MAX_VALUE)
		           {
		               counter++;
		           }
		       }
		       // Se il contatore del timer è diminuito
		       else if (rawCounter < last_rawCounter)
		       {
		           // Decrementa il nostro contatore solo se non ha raggiunto il valore minimo
		           if (counter > MIN_VALUE)
		           {
		               counter--;
		           }
		       }

		       // Aggiorna la variabile last_rawCounter per il prossimo ciclo
		       last_rawCounter = rawCounter;

		       // Ricalcola il delay solo se il counter è cambiato
		       if (counter != last_counter) {
		           last_counter = counter;
		       }
		       else {
		    	   cont_misure_tim ++;
		       }
		       if(cont_misure_tim == 30){
					snprintf(actual_val_luce, sizeof(actual_val_luce), "%d", (int)counter);
					HAL_UART_Transmit(&huart2, (uint8_t *)actual_val_luce, sizeof(actual_val_luce), HAL_MAX_DELAY);
					cont_misure_tim = 0;
		       }
		   }
	  }

//  	if(HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_RESET)
//  	{
//  		HAL_GPIO_TogglePin(RGB_LED_RED_GPIO_Port, RGB_LED_RED_Pin);
//  		HAL_GPIO_TogglePin(RGB_LED_BLUE_GPIO_Port, RGB_LED_BLUE_Pin);
//  		char SCREEN_text[] = "Gabocchia";
//  		ssd1306_Fill(Black);
//  		ssd1306_FillRectangle(0,0,128,15, White);
//  		ssd1306_SetCursor(15,0);
//  		ssd1306_WriteString(SCREEN_text, Font_16x15, Black);
//  		ssd1306_UpdateScreen();
//
//  	}






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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_6B;
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
  sConfig.Channel = ADC_CHANNEL_4;
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  huart1.Init.HwFlowCtl = UART_HWCONTROL_RTS_CTS;
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
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_RTS_CTS;
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
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RGB_LED_BLUE_Pin|RGB_LED_RED_Pin|YELLOW_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RGB_LED_BLUE_Pin RGB_LED_RED_Pin YELLOW_LED_Pin */
  GPIO_InitStruct.Pin = RGB_LED_BLUE_Pin|RGB_LED_RED_Pin|YELLOW_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : BTN_MODE_Pin */
  GPIO_InitStruct.Pin = BTN_MODE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BTN_MODE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BTN_SEL_Pin */
  GPIO_InitStruct.Pin = BTN_SEL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BTN_SEL_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  /* Prevent unused argument(s) compilation warning */
  //PIN_8 SELECT, PIN_3 MODE

/* COMPORTAMENTO DI DEFAULT:
 * il display mostra il valore di luce desiderato, ossia quello selezionato con il potenziometro
 * LOGICA BOTTONI:
 * se MODE = 0, il display mostra il valore desiderato,
 * se MODE = 1, il display mostra il valore attuale,
 * di cosa dipende da SELECT,
 * se SELECT = 0, il display mostra il valore della luce,
 * se SELECT = 1, il display mostra il valore della temperatura,
 * ogni volta che il bottone mode o select viene premuto,
 * si nega il valore di push_mode_button o push_select_button
 * LOGICA LED E LED_RGB:
 * se MODE = 0, RGB_RED_LED si accende e RGB_BLUE_LED si spegne,
 * se MODE = 1, RGB_RED_LED si spegne e RGB_BLUE_LED si accende,
 * se SELECT = 0, YELLOW_LED si spegne,
 * se SELECT = 1, YELLOW_LED si accende*/

  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  if(GPIO_Pin == GPIO_PIN_8){

	  push_mode_button = !(push_mode_button);

	  if(push_mode_button == 0){
		  HAL_GPIO_WritePin(RGB_LED_RED_GPIO_Port, RGB_LED_RED_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(RGB_LED_BLUE_GPIO_Port, RGB_LED_BLUE_Pin, GPIO_PIN_RESET);

//		  if(push_select_button == 0){
//
////			  if(xSemaphoreTakeFromISR(sem_sel_luce,&xHigherPriorityTaskWoken) == pdTRUE){
////			 strncpy(SCREEN_TEXT, selected_val_luce, DIM+1);
////			  }
//
//
//		  }
		 /* else */if(push_select_button == 1){
			  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
//			  if(xSemaphoreTakeFromISR(sem_sel_temp,&xHigherPriorityTaskWoken) == pdTRUE){
//			 strncpy(SCREEN_TEXT, selected_val_temp, DIM+1);
//			  }
		  }
	  }
	  else if(push_mode_button == 1){
		  HAL_GPIO_WritePin(RGB_LED_RED_GPIO_Port, RGB_LED_RED_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(RGB_LED_BLUE_GPIO_Port, RGB_LED_BLUE_Pin, GPIO_PIN_SET);
//		  if(push_select_button == 0){
////			  if(xSemaphoreTakeFromISR(sem_act_luce,&xHigherPriorityTaskWoken) == pdTRUE){
////		  			 strncpy(SCREEN_TEXT, actual_val_luce, DIM+1);
////			  }
//		  		  }
//		  		  else if(push_select_button == 1){
////		  			if(xSemaphoreTakeFromISR(sem_act_temp,&xHigherPriorityTaskWoken) == pdTRUE){
////		  			 strncpy(SCREEN_TEXT, actual_val_temp, DIM+1);
////		  			}
//		  		  }
	  }
  }

  else if(GPIO_Pin == GPIO_PIN_3){
	  push_select_button = !(push_select_button);


	  if(push_select_button == 0){
		  HAL_GPIO_WritePin(YELLOW_LED_GPIO_Port, YELLOW_LED_Pin, GPIO_PIN_RESET);
		  if(push_mode_button == 0){
			  HAL_ADC_Start_IT(&hadc1);
//			  if(xSemaphoreTakeFromISR(sem_sel_luce,&xHigherPriorityTaskWoken) == pdTRUE){
//			  strncpy(SCREEN_TEXT, selected_val_luce, DIM+1);
//			  }


		  }
//		  else if(push_mode_button == 1){
////			  if(xSemaphoreTakeFromISR(sem_act_luce,&xHigherPriorityTaskWoken) == pdTRUE){
////			  strncpy(SCREEN_TEXT, actual_val_luce, DIM+1);
////			  }
//		  }
	  }
	  else if(push_select_button ==1){
		  HAL_GPIO_WritePin(YELLOW_LED_GPIO_Port, YELLOW_LED_Pin, GPIO_PIN_SET);
		  if(push_mode_button == 0){
			  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
//			  if(xSemaphoreTakeFromISR(sem_sel_temp,&xHigherPriorityTaskWoken) == pdTRUE){
//		  	  strncpy(SCREEN_TEXT, selected_val_temp, DIM+1);
//			  }
		  }
//		  else if(push_mode_button == 1){
////			  if(xSemaphoreTakeFromISR(sem_act_temp,&xHigherPriorityTaskWoken) == pdTRUE){
////		  	  strncpy(SCREEN_TEXT, actual_val_temp, DIM+1);
////			  }
//		  }

	  }
  }

//  	ssd1306_Fill(Black);
//		ssd1306_FillRectangle(0,0,128,15, White);
//		ssd1306_SetCursor(15,0);
//		ssd1306_WriteString(SCREEN_TEXT, Font_16x15, Black);
//		ssd1306_UpdateScreen();
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* h){

	if(h == &hadc1){
		adc_value = HAL_ADC_GetValue(&hadc1);
		adc_data_ready = 1;


	}

}
  /* NOTE: This function Should not be modified, when the callback is needed,
           the HAL_GPIO_EXTI_Callback could be implemented in the user file
   */
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

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
#ifdef USE_FULL_ASSERT
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
