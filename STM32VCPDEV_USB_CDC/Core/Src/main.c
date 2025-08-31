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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
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

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// data recived from out ep (from PC->dev) is hold here in this buffer for the gadget driver(stm32 usbcdc device)
extern uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];
volatile int isStatusSent = 0;
char TxBuffer[50];
char sensor_val[30];

static volatile uint32_t g_adc_raw = 0;
static volatile uint8_t  g_adc_done = 0;
/* Called by HAL from ADC IRQ when conversion completes */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  if (hadc->Instance == ADC1)
  {
		g_adc_raw = HAL_ADC_GetValue(hadc);
		g_adc_done = 1;
  }
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
  MX_USB_DEVICE_Init();
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  /* Start first ADC conversion in interrupt mode */
  HAL_ADC_Start_IT(&hadc1);
  //HAL_Delay(5000);
  //sprintf(TxBuffer,"Hello from STM32 DISC1\r\n");
  //CDC_Transmit_FS((uint8_t*)TxBuffer,strlen(TxBuffer));
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	if(g_adc_done)
	{
		g_adc_done = 0;

		/* Convert ADC->temperature (typical values from STM32F4 datasheet) */
		/* VDDA assumed 3.0 V on DISC1 */
		float vsense = ((float)g_adc_raw * 3.0f) / 4095.0f;
		float temp_c = ((vsense - 0.76f) / 0.0025f) + 25.0f;

		sprintf(sensor_val, (char*)"DieTemp: %.2f C\r\n", temp_c);
		CDC_Transmit_FS((uint8_t*)sensor_val, strlen(sensor_val));
		//HAL_UART_Transmit(&huart2, (uint8_t*)sensor_val, sizeof(sensor_val), HAL_MAX_DELAY);
		HAL_Delay(1000);
		HAL_ADC_Start_IT(&hadc1);
	}
	/*
	size_t len = strlen((char *)UserRxBufferFS); // get current string length
	if (len + 2 < APP_RX_DATA_SIZE)  // ensure space for '\n', '\r' and '\0'
	{
			len = len+1;
	        UserRxBufferFS[len] = '\n';
	        len = len+1;
	        UserRxBufferFS[len] = '\r';
	        len = len+1;
	        UserRxBufferFS[len] = '\0'; // keep it a valid string
	}*/
	HAL_UART_Transmit(&huart2, (uint8_t*)UserRxBufferFS, sizeof(UserRxBufferFS), HAL_MAX_DELAY);
	if(strncmp((char*)UserRxBufferFS, "A", 1) == 0){
	  	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
	}else if(strncmp((char*)UserRxBufferFS, "a", 1) == 0){
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
	}else if(strncmp((char*)UserRxBufferFS, "B", 1) == 0){
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
	}else if(strncmp((char*)UserRxBufferFS, "b", 1) == 0){
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
	}else if(strncmp((char*)UserRxBufferFS, "C", 1) == 0){
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
	}else if(strncmp((char*)UserRxBufferFS, "c", 1) == 0){
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
	}else if(strncmp((char*)UserRxBufferFS, "D", 1) == 0){
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
	}else if(strncmp((char*)UserRxBufferFS, "d", 1) == 0){
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
	}else if(strncmp((char*)UserRxBufferFS, "1", 1) == 0){
		if(isStatusSent == 0){
			if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_12) == GPIO_PIN_SET){
				sprintf(TxBuffer,"\r\nGREEN LED is turned ON\r\n");
				CDC_Transmit_FS((uint8_t*)TxBuffer,strlen(TxBuffer));
			}else{
				sprintf(TxBuffer,"\r\nGREEN LED is turned OFF\r\n");
				CDC_Transmit_FS((uint8_t*)TxBuffer,strlen(TxBuffer));
			}
			isStatusSent = 1;
		}
	}else if(strncmp((char*)UserRxBufferFS, "2", 1) == 0){
		if(isStatusSent == 0){
			if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_13) == GPIO_PIN_SET){
				sprintf(TxBuffer,"\r\nORANGE LED is turned ON\r\n");
				CDC_Transmit_FS((uint8_t*)TxBuffer,strlen(TxBuffer));
			}else{
				sprintf(TxBuffer,"\r\nORANGE LED is turned OFF\r\n");
				CDC_Transmit_FS((uint8_t*)TxBuffer,strlen(TxBuffer));
			}
			isStatusSent = 1;
		}
	}else if(strncmp((char*)UserRxBufferFS, "3", 1) == 0){
		if(isStatusSent == 0){
			if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_14) == GPIO_PIN_SET){
				sprintf(TxBuffer,"\r\nRED LED is turned ON\r\n");
				CDC_Transmit_FS((uint8_t*)TxBuffer,strlen(TxBuffer));
			}else{
				sprintf(TxBuffer,"\r\nRED LED is turned OFF\r\n");
				CDC_Transmit_FS((uint8_t*)TxBuffer,strlen(TxBuffer));
			}
			isStatusSent = 1;
		}
	}else if(strncmp((char*)UserRxBufferFS, "4", 1) == 0){
		if(isStatusSent == 0){
			if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_15) == GPIO_PIN_SET){
				sprintf(TxBuffer,"\r\nBLUE LED is turned ON\r\n");
				CDC_Transmit_FS((uint8_t*)TxBuffer,strlen(TxBuffer));
			}else{
				sprintf(TxBuffer,"\r\nBLUE LED is turned OFF\r\n");
				CDC_Transmit_FS((uint8_t*)TxBuffer,strlen(TxBuffer));
			}
			isStatusSent = 1;
		}

	}else if(strncmp((char*)UserRxBufferFS, "L", 1) == 0)
	{
		if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_12) == GPIO_PIN_SET){
			TxBuffer[0] = '1';
			//sprintf(TxBuffer,"1\r\n");
		}else{
			TxBuffer[0] = '0';
		}
		if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_13) == GPIO_PIN_SET){
			TxBuffer[1] = '1';
		}else{
			TxBuffer[1] = '0';
		}
		if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_14) == GPIO_PIN_SET){
			TxBuffer[2] = '1';
		}else{
			TxBuffer[2] = '0';
		}
		if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_15) == GPIO_PIN_SET){
			TxBuffer[3] = '1';
		}else{
			TxBuffer[3] = '0';
		}
		TxBuffer[4] = '\n';
		TxBuffer[5] = '\r';
		CDC_Transmit_FS((uint8_t*)TxBuffer,strlen(TxBuffer));
		isStatusSent = 1;
	}else if(strncmp((char*)UserRxBufferFS, "r", 1) == 0){
		isStatusSent = 0;
	}else{
		// do nothing
	}
	//HAL_Delay(1000);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
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
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

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
