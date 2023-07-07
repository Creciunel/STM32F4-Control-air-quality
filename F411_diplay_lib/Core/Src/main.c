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
#include "cmsis_os.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>

#include "usbd_cdc_if.h"
#include "string.h"

#include "st7789.h"
#include "fonts.h"
#include "bitmap.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MINUT 60000
#define SEC 1000
#define HOUR 60000000
#define ONEHOUR 3600000
#define TIMEFORSENSORS 2

#define USB_BUF_SIZE 64
#define SEBSORMAXVAL 3276

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CRC_HandleTypeDef hcrc;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* Definitions for DisplayTask */
osThreadId_t DisplayTaskHandle;
const osThreadAttr_t DisplayTask_attributes = { .name = "DisplayTask",
		.stack_size = 128 * 4, .priority = (osPriority_t) osPriorityNormal, };
/* Definitions for MotorTask */
osThreadId_t MotorTaskHandle;
const osThreadAttr_t MotorTask_attributes = { .name = "MotorTask", .stack_size =
		128 * 4, .priority = (osPriority_t) osPriorityNormal, };
/* Definitions for ADCTask */
osThreadId_t ADCTaskHandle;
const osThreadAttr_t ADCTask_attributes = { .name = "ADCTask", .stack_size = 128
		* 4, .priority = (osPriority_t) osPriorityNormal, };
/* Definitions for USBTask */
osThreadId_t USBTaskHandle;
const osThreadAttr_t USBTask_attributes = { .name = "USBTask", .stack_size = 128
		* 4, .priority = (osPriority_t) osPriorityNormal, };
/* Definitions for MotorSpeedTask */
osThreadId_t MotorSpeedTaskHandle;
const osThreadAttr_t MotorSpeedTask_attributes = { .name = "MotorSpeedTask",
		.stack_size = 128 * 4, .priority = (osPriority_t) osPriorityNormal, };
/* USER CODE BEGIN PV */
uint32_t T;

uint16_t adcVal;

struct Time {
	uint32_t h, m, s;
} TIME_ON;
struct GAS {
	uint16_t actual, last, BIT12;
} ch4;

struct Flag {
	uint8_t Heat, stratSend;
} flagt = { .Heat = 0 };

struct Motor {
	uint16_t count;
	uint32_t TimeForM;
	uint16_t speed;
} motor;

uint8_t data[USB_BUF_SIZE];
uint8_t USB_Buff[USB_BUF_SIZE];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CRC_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
void StartDisplayTask(void *argument);
void StartMotorTask(void *argument);
void StartADCTask(void *argument);
void StartUSBTask(void *argument);
void StartMotorSpeedTask(void *argument);

/* USER CODE BEGIN PFP */
void printTime(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */
//	char str[10];
	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */
	T = HAL_GetTick();
	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_CRC_Init();
	MX_SPI1_Init();
	MX_TIM2_Init();
	MX_ADC1_Init();
	MX_TIM3_Init();
	MX_TIM4_Init();
	/* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start(&htim4);
	MX_USB_DEVICE_Init();

	ST7789_Init();
	ST7789_rotation(2);

//	c
//	ST7789_DrawCircleFilled(160, 120, 80, ST7789_RED);
	/* USER CODE END 2 */

	/* Init scheduler */
	osKernelInitialize();

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* creation of DisplayTask */
	DisplayTaskHandle = osThreadNew(StartDisplayTask, NULL,
			&DisplayTask_attributes);

	/* creation of MotorTask */
	MotorTaskHandle = osThreadNew(StartMotorTask, NULL, &MotorTask_attributes);

	/* creation of ADCTask */
	ADCTaskHandle = osThreadNew(StartADCTask, NULL, &ADCTask_attributes);

	/* creation of USBTask */
	USBTaskHandle = osThreadNew(StartUSBTask, NULL, &USBTask_attributes);

	/* creation of MotorSpeedTask */
	MotorSpeedTaskHandle = osThreadNew(StartMotorSpeedTask, NULL,
			&MotorSpeedTask_attributes);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */

	/* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
	/* USER CODE END RTOS_EVENTS */

	/* Start scheduler */
	osKernelStart();
	/* We should never get here as control is now taken by the scheduler */
	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

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
	RCC_OscInitStruct.PLL.PLLM = 15;
	RCC_OscInitStruct.PLL.PLLN = 144;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
	RCC_OscInitStruct.PLL.PLLQ = 5;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

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
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_6;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief CRC Initialization Function
 * @param None
 * @retval None
 */
static void MX_CRC_Init(void) {

	/* USER CODE BEGIN CRC_Init 0 */

	/* USER CODE END CRC_Init 0 */

	/* USER CODE BEGIN CRC_Init 1 */

	/* USER CODE END CRC_Init 1 */
	hcrc.Instance = CRC;
	if (HAL_CRC_Init(&hcrc) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN CRC_Init 2 */

	/* USER CODE END CRC_Init 2 */

}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void) {

	/* USER CODE BEGIN SPI1_Init 0 */

	/* USER CODE END SPI1_Init 0 */

	/* USER CODE BEGIN SPI1_Init 1 */

	/* USER CODE END SPI1_Init 1 */
	/* SPI1 parameter configuration*/
	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_MASTER;
	hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
	hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi1.Init.NSS = SPI_NSS_SOFT;
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SPI1_Init 2 */

	/* USER CODE END SPI1_Init 2 */

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 44999;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 15;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */

}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 0;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 1023;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */
	HAL_TIM_MspPostInit(&htim3);

}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void) {

	/* USER CODE BEGIN TIM4_Init 0 */

	/* USER CODE END TIM4_Init 0 */

	TIM_SlaveConfigTypeDef sSlaveConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM4_Init 1 */

	/* USER CODE END TIM4_Init 1 */
	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 0;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 10000;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim4) != HAL_OK) {
		Error_Handler();
	}
	sSlaveConfig.SlaveMode = TIM_SLAVEMODE_EXTERNAL1;
	sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
	sSlaveConfig.TriggerPolarity = TIM_TRIGGERPOLARITY_RISING;
	sSlaveConfig.TriggerFilter = 15;
	if (HAL_TIM_SlaveConfigSynchro(&htim4, &sSlaveConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM4_Init 2 */

	/* USER CODE END TIM4_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

	/* DMA controller clock enable */
	__HAL_RCC_DMA2_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA2_Stream2_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, RST_Pin | DC_Pin | CS_Pin | M_IN1_Pin | M_IN2_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pins : RST_Pin DC_Pin CS_Pin M_IN1_Pin
	 M_IN2_Pin */
	GPIO_InitStruct.Pin = RST_Pin | DC_Pin | CS_Pin | M_IN1_Pin | M_IN2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void printTime() {
	char str_s[10];
	TIME_ON.h = HAL_GetTick() / ONEHOUR;
	TIME_ON.m = HAL_GetTick() / MINUT;
	TIME_ON.s = HAL_GetTick() / SEC;

	if (TIME_ON.s > 59) {
		if (TIME_ON.h > 0) {
			TIME_ON.s = (HAL_GetTick() - (ONEHOUR * TIME_ON.h)) / SEC;

		} else {
			TIME_ON.s = (HAL_GetTick() - (MINUT * TIME_ON.m)) / SEC;
		}
	}
	if (TIME_ON.m > 59) {
		TIME_ON.m = (HAL_GetTick() - (MINUT * TIME_ON.h)) / SEC;
	}
	if (TIME_ON.h >= HAL_ERROR) {
		TIME_ON.h = (HAL_GetTick() - (ONEHOUR * TIME_ON.h)) / SEC;
	}

	sprintf(str_s, "%li:%li:%li ", TIME_ON.h, TIME_ON.m, TIME_ON.s);

	ST7789_DrawFillRoundRect(0, 0, 90, 50, 0, ST7789_BLACK);

	ST7789_print(5, 5, ST7789_GREEN, ST7789_BLACK, 0, &Font_11x18, 1, str_s);
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDisplayTask */
/**
 * @brief  Function implementing the DisplayTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDisplayTask */
void StartDisplayTask(void *argument) {
	/* init code for USB_DEVICE */
	MX_USB_DEVICE_Init();
	/* USER CODE BEGIN 5 */
	char str[10];
	ST7789_FillScreen(ST7789_BLACK);
	ST7789_DrawCircleFilled(160, 120, 80, ST7789_RED);
	flagt.stratSend = 0;
	flagt.Heat = 0;

	/* Infinite loop */
	for (;;) {
		if (TIME_ON.m <= TIMEFORSENSORS) {
			if (ch4.actual != ch4.last) {
				sprintf(str, "%i %%", ch4.actual);
				ST7789_DrawCircleFilled(160, 120, 80, ST7789_RED);

				ST7789_print(130, 70, ST7789_WHITE, ST7789_BLACK, 0,
						&Font_16x26, 1, "Wait");

				ST7789_print(135, 110, ST7789_WHITE, ST7789_BLACK, 0,
						&Font_16x26, 1, str);
				sprintf(str, "CH4");
				ST7789_print(135, 150, ST7789_WHITE, ST7789_BLACK, 0,
						&Font_16x26, 1, str);

				ch4.last = ch4.actual;
			}
		} else {
			if (flagt.Heat == 0) {
				sprintf(str, "%i %%", ch4.actual);

				ST7789_DrawCircleFilled(160, 120, 80, ST7789_BLUE);

				ST7789_print(135, 110, ST7789_WHITE, ST7789_BLACK, 0,
						&Font_16x26, 1, str);
				sprintf(str, "CH4");
				ST7789_print(135, 150, ST7789_WHITE, ST7789_BLACK, 0,
						&Font_16x26, 1, str);
				ch4.last = ch4.actual;
				flagt.Heat = 1;
			}
			if (ch4.actual != ch4.last) {
				sprintf(str, "%i %%", ch4.actual);

				ST7789_DrawCircleFilled(160, 120, 80, ST7789_BLUE);

				ST7789_print(135, 110, ST7789_WHITE, ST7789_BLACK, 0,
						&Font_16x26, 1, str);
				sprintf(str, "CH4");
				ST7789_print(135, 150, ST7789_WHITE, ST7789_BLACK, 0,
						&Font_16x26, 1, str);

				ch4.last = ch4.actual;
			}
			flagt.stratSend = 1;
		}

		printTime();
		osDelay(SEC);
	}
	/* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartMotorTask */
/**
 * @brief Function implementing the MotorTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartMotorTask */
void StartMotorTask(void *argument) {
	/* USER CODE BEGIN StartMotorTask */
	/* Infinite loop */
	for (;;) {
		if (ch4.actual > 20) {
			HAL_GPIO_WritePin(M_EN_GPIO_Port, M_EN_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(M_IN1_GPIO_Port, M_IN1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(M_IN2_GPIO_Port, M_IN2_Pin, GPIO_PIN_RESET);
		} else {
			HAL_GPIO_WritePin(M_EN_GPIO_Port, M_EN_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(M_IN1_GPIO_Port, M_IN1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(M_IN2_GPIO_Port, M_IN2_Pin, GPIO_PIN_RESET);
		}
		osDelay(1);
	}
	/* USER CODE END StartMotorTask */
}

/* USER CODE BEGIN Header_StartADCTask */
/**
 * @brief Function implementing the ADCTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartADCTask */
void StartADCTask(void *argument) {
	/* USER CODE BEGIN StartADCTask */

	/* Infinite loop */
	for (;;) {
		HAL_ADCEx_InjectedStart(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, 100);

		adcVal = HAL_ADCEx_InjectedGetValue(&hadc1,
		ADC_INJECTED_RANK_1);
		ch4.BIT12 = HAL_ADCEx_InjectedGetValue(&hadc1,
		ADC_INJECTED_RANK_1);

		HAL_ADCEx_InjectedStop(&hadc1);

		ch4.actual = (ch4.BIT12 * 100) / SEBSORMAXVAL;
		osDelay(900);
	}
	/* USER CODE END StartADCTask */
}

/* USER CODE BEGIN Header_StartUSBTask */
/**
 * @brief Function implementing the USBTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartUSBTask */
void StartUSBTask(void *argument) {
	/* USER CODE BEGIN StartUSBTask */
	/* Infinite loop */
	for (;;) {
		if (flagt.stratSend == 1) {
			if (ch4.actual != ch4.last) {
//			sprintf((char*) data, "CH4 val: %i\n", ch4.BIT16);
				sprintf((char*) data, " %i", ch4.BIT12);
				CDC_Transmit_FS(data, strlen((char*) data));
			}
		}
		osDelay(SEC / 5);

	}
	/* USER CODE END StartUSBTask */
}

/* USER CODE BEGIN Header_StartMotorSpeedTask */
/**
 * @brief Function implementing the MotorSpeedTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartMotorSpeedTask */
void StartMotorSpeedTask(void *argument) {
	/* USER CODE BEGIN StartMotorSpeedTask */
	motor.TimeForM = T;
	/* Infinite loop */
	for (;;) {
		motor.TimeForM = HAL_GetTick() - motor.TimeForM;
		motor.TimeForM /= MINUT;

		motor.count = __HAL_TIM_GetCounter(&htim4);
		motor.speed = motor.count / motor.TimeForM / 2; // 2 signal per revolution
		motor.TimeForM = HAL_GetTick();
		osDelay(1000);
	}
	/* USER CODE END StartMotorSpeedTask */
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM1 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM1) {
		HAL_IncTick();
	}
	/* USER CODE BEGIN Callback 1 */

	/* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
