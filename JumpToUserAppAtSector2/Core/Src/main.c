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
#include "lcd16x2_i2c.h"
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
CAN_HandleTypeDef hcan2;

CRC_HandleTypeDef hcrc;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_CAN2_Init(void);
static void MX_CRC_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
char DataBuffer[64];

// define a global buffer(array) of size 200 to hold received command packet
#define BL_RX_LEN  200
uint8_t bl_rx_buffer[BL_RX_LEN];

/* BL Commands */
#define BL_GET_VER					0x51
#define BL_GET_HELP					0x52
#define BL_GET_CID					0x53
#define BL_GET_RDP_STATUS			0x54
#define BL_GO_TO_ADDR				0x55
#define BL_FLASH_ERASE          	0x56
#define BL_MEM_WRITE				0x57

/* CAN DEBUG MSG send Fun */
void CAN_Send_64Bytes(uint16_t msg_id, char *pDataBuffer);

uint8_t bootloader_verify_crc (uint8_t *pData, uint32_t len, uint32_t crc_host);
void bootloader_send_ack(uint8_t command_code, uint8_t follow_len);
void bootloader_send_nack(void);
uint8_t get_bootloader_version(void);
void bootloader_uart_write_data(uint8_t *pBuffer,uint32_t len);
void bootloader_handle_getver_cmd(uint8_t *pbl_rx_buffer);

// BL supported cmd codes
uint8_t supported_commands[] =
{
      BL_GET_VER ,						//command code: 0x51
      BL_GET_HELP,						//command code: 0x52
      BL_GET_CID,						//command code: 0x53
      BL_GET_RDP_STATUS,				//command code: 0x54
      BL_GO_TO_ADDR,					//command code: 0x55
      BL_FLASH_ERASE,					//command code: 0x56
      BL_MEM_WRITE,						//command code: 0x57
      //BL_READ_SECTOR_P_STATUS			//command code: 0x08
};
void bootloader_handle_gethelp_cmd(uint8_t *pBuffer);

uint16_t get_mcu_chip_id(void);
void bootloader_handle_getcid_cmd(uint8_t *pBuffer);

uint8_t get_flash_rdp_level(void);
void bootloader_handle_getrdp_cmd(uint8_t *pBuffer);

#define ADDR_VALID 				0x00
#define ADDR_INVALID 			0x01
#define INVALID_SECTOR 			0x04

/* Start and End addresses of different memories of STM32F407VGT6 MCU */
#define SRAM1_SIZE            (112*1024)     								// STM32F407VGT6 has 112KB of SRAM1
#define SRAM1_END             (SRAM1_BASE + SRAM1_SIZE)
#define SRAM2_SIZE            (16*1024)     								// STM32F407VGT6 has 16KB of SRAM2
#define SRAM2_END             (SRAM2_BASE + SRAM2_SIZE)
#define FLASH_SIZE            (1024*1024)     								// STM32F407VGT6 has 1MB(1024KB) of Internal/Embedded Flash Memory

#define BKPSRAM_SIZE          (4*1024)     									// STM32F407VGT6 has 4KB of Backup SRAM
#define BKPSRAM_END           (BKPSRAM_BASE + BKPSRAM_SIZE)

uint8_t verify_address(uint32_t go_address);
void bootloader_handle_go_cmd(uint8_t *pBuffer);

uint8_t execute_flash_erase(uint8_t sector_number , uint8_t number_of_sector);
void bootloader_handle_flash_erase_cmd(uint8_t *pBuffer);

uint8_t execute_mem_write(uint8_t *pBuffer, uint32_t mem_address, uint32_t len);
void bootloader_handle_mem_write_cmd(uint8_t *pBuffer);

void bootloader_uart_read_data(void)
{
	uint8_t rcv_len = 0;

	while(1)
	{
		// here we will read and decode the commands coming from host
		memset(bl_rx_buffer,0,200);

		// 1. first read only one byte from the host , which is the "length" field of the command packet
		HAL_UART_Receive(&huart3, bl_rx_buffer, 1, HAL_MAX_DELAY);
		rcv_len = bl_rx_buffer[0];
		// then read actual command packet of "len" number of bytes in bl_rx_buffer[1] onwards (cmd code + crc bytes + other required data if any!)
		HAL_UART_Receive(&huart3, &bl_rx_buffer[1], rcv_len, HAL_MAX_DELAY);

		// switch to respective handler fun as per recived cmd code
		// 2. bl_rx_buffer[1] holds "cmd code" field of the command packet.
		switch(bl_rx_buffer[1])
		//@@
		{
			case BL_GET_VER:
				 // pass the complete received bl cmd from host:
				 bootloader_handle_getver_cmd(bl_rx_buffer);  		// HAL_CRC_Accumulate, ok
		 		 break;
			case BL_GET_HELP:
				bootloader_handle_gethelp_cmd(bl_rx_buffer);   		// ok
				break;
			case BL_GET_CID:
				bootloader_handle_getcid_cmd(bl_rx_buffer);	    	// ok
				break;
			case BL_GET_RDP_STATUS:
				bootloader_handle_getrdp_cmd(bl_rx_buffer);	    	// ok
				break;
			case BL_GO_TO_ADDR:
				bootloader_handle_go_cmd(bl_rx_buffer);		    	// ok. but code doesn't run?
				break;
			case BL_FLASH_ERASE:
				bootloader_handle_flash_erase_cmd(bl_rx_buffer); 	// ok. Uses HAL_FLASHEx_Erase API internally
				break;
			case BL_MEM_WRITE:
				bootloader_handle_mem_write_cmd(bl_rx_buffer);  	// nok. Uses HAL_FLASH_UNLOCK -> HAL_FLASH_Program -> HAL_FLASH_UNLOCK APIs internally
				break;
			default:
				//printf("BL_DEBUG_MSG: Invalid command code received from host\n");
				sprintf(DataBuffer, "Invalid command code received from host \n\r");
				break;
		}
	}
}

#define USER_APP_ADDR  0x08008000
//typedef void (*pFunction)(void);
void (*pFunction)(void);
void bootloader_jump_to_user_app(void)
{
    uint32_t msp_value     = *((volatile uint32_t*)(USER_APP_ADDR));          // 1. Load initial MSP
    uint32_t reset_handler = *((volatile uint32_t*)(USER_APP_ADDR+4));        // 2. Load Reset_Handler

    __disable_irq();  				// Important: stop any pending interrupts

    __set_MSP(msp_value);          // 3. Set stack pointer
    SCB->VTOR = USER_APP_ADDR;     // 4. Relocate vector table


    //pFunction JumpToApp = (pFunction) reset_handler;
    //JumpToApp();                   // 5. Jump to user application
    pFunction = (void(*)(void))reset_handler;
    pFunction();
}


/* CRC check status: Failed/Success */
#define VERIFY_CRC_FAIL    		1
#define VERIFY_CRC_SUCCESS 		0

/* ACK and NACK identification bytes */
#define BL_ACK   				0XA5
#define BL_NACK  				0X7F

// version 1.0
#define BL_VERSION 0x10

void SPI1_WriteData(char *pDataBuffer)
{
	uint8_t lenDataBuffer = strlen(pDataBuffer);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); // NSS = LOW
	HAL_SPI_Transmit(&hspi1, &lenDataBuffer, 1, HAL_MAX_DELAY);
	HAL_SPI_Transmit(&hspi1, (uint8_t*)pDataBuffer, lenDataBuffer, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); // NSS = HIGH
	//HAL_Delay(500);
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
  MX_USART3_UART_Init();
  MX_CAN2_Init();
  MX_CRC_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  lcd16x2_i2c_init(&hi2c1);
  lcd16x2_i2c_1stLine();					//lcd16x2_i2c_setCursor(0, 0);
  lcd16x2_i2c_printf("BL Application");
  lcd16x2_i2c_2ndLine();					//lcd16x2_i2c_setCursor(1, 0);
  lcd16x2_i2c_printf("is running ....");
  SPI1_WriteData("Bootloader code is running...");
  HAL_CAN_Start(&hcan2);

  //bootloader_jump_to_user_app();
  if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET){			// continue in bootloader mode for IAP and ... atcivities
  	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
	  sprintf(DataBuffer, "BL_DEBUG_MSG: BL is running...\n\r");
	  //SPI1_WriteData(DataBuffer);
	  CAN_Send_64Bytes(0x123, DataBuffer);
  	  bootloader_uart_read_data();
    }else{															// jump to user appln
  	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
  	  sprintf(DataBuffer, "BL_DEBUG_MSG: Jumping to user Appln...\n\r");
  	  //SPI1_WriteData(DataBuffer);
  	  CAN_Send_64Bytes(0x123, DataBuffer);
  	  bootloader_jump_to_user_app();
    }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 72;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 18;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_10TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_5TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */

  /* USER CODE END CAN2_Init 2 */

}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
void CAN_Send_64Bytes(uint16_t msg_id, char *pDataBuffer)
{
	CAN_TxHeaderTypeDef TxHeader;
	uint32_t TxMailbox;

    TxHeader.StdId = msg_id;        // CAN ID
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.DLC = 8;              // Always sending 8 bytes
    TxHeader.TransmitGlobalTime = DISABLE;

    for (int i = 0; i < 64; i += 8)
    {
        // Take 8 bytes from array
        uint8_t TxData[8];
        for (int j = 0; j < 8; j++)
        {
            TxData[j] = pDataBuffer[i + j];
        }

        // Send frame
        if (HAL_CAN_AddTxMessage(&hcan2, &TxHeader, TxData, &TxMailbox) != HAL_OK)
        {
            // Handle transmission error
        }

        // Optional: wait for Tx complete before sending next
        while (HAL_CAN_IsTxMessagePending(&hcan2, TxMailbox));
    }
}

// This verifies the CRC of the given buffer in pData .
uint8_t bootloader_verify_crc (uint8_t *pData, uint32_t len, uint32_t crc_host)
{
    uint32_t uwCRCValue = 0xff;

    for (uint32_t i=0 ; i < len ; i++)
	{
        uint32_t i_data = pData[i];
        uwCRCValue = HAL_CRC_Accumulate(&hcrc, &i_data, 1);
	}

	 /* Reset CRC Calculation Unit */
     __HAL_CRC_DR_RESET(&hcrc);

	if( uwCRCValue == crc_host)
	{
		return VERIFY_CRC_SUCCESS;
	}
	return VERIFY_CRC_FAIL;
}
/*This function sends ACK if CRC matches along with "len to follow"*/
void bootloader_send_ack(uint8_t command_code, uint8_t follow_len)
{
	 // here we send 2 byte.. first byte is ack and the second byte is len value
	uint8_t ack_buf[2];
	ack_buf[0] = BL_ACK;
	ack_buf[1] = follow_len;
	HAL_UART_Transmit(&huart3, ack_buf, 2, HAL_MAX_DELAY);
}
/*This function sends NACK */
void bootloader_send_nack(void)
{
	uint8_t nack = BL_NACK;
	HAL_UART_Transmit(&huart3, &nack, 1, HAL_MAX_DELAY);
}
/* Just returns the BL_VERSION macro value */
uint8_t get_bootloader_version(void)
{
  return (uint8_t)BL_VERSION;
}
/* This function writes data in to C_UART */
void bootloader_uart_write_data(uint8_t *pBuffer,uint32_t len)
{
    /*you can replace the below ST's USART driver API call with your MCUs driver API call */
	HAL_UART_Transmit(&huart3, pBuffer, len, HAL_MAX_DELAY);
}
/* Note: p_bl_rx_buffer has "ptr to the whole cmd packet" received from host */
void bootloader_handle_getver_cmd(uint8_t *pbl_rx_buffer)
{
	uint8_t bl_version;

	// 1. get the value of cmd packet len
	uint32_t command_packet_len = pbl_rx_buffer[0]+1;
	// 2. extract the value of host crc
	uint32_t host_crc = *((uint32_t * )(pbl_rx_buffer + command_packet_len - 4));

	// 3. verify checksum (crc check)
	if (!bootloader_verify_crc(&pbl_rx_buffer[0], command_packet_len-4, host_crc))
	{
			sprintf(DataBuffer, "BL_DEBUG_MSG: Checksum success !! \n\r");
	        // checksum is correct send ack code with 1 byte(version number) len to follow.
	        bootloader_send_ack(pbl_rx_buffer[0], 1);
	        // get version number
	        bl_version = get_bootloader_version();
	        sprintf(DataBuffer, "BL_DEBUG_MSG: BL_VER : %d %#x\n",bl_version, bl_version);
	        // send it over communication uart interface
	        bootloader_uart_write_data(&bl_version, 1);
	}else
	{
			sprintf(DataBuffer, "BL_DEBUG_MSG: Checksum fail !! \n\r");
	        // checksum is wrong send nack
	        bootloader_send_nack();
	}
}
void bootloader_handle_gethelp_cmd(uint8_t *pBuffer)
{
    sprintf(DataBuffer, "BL_DEBUG_MSG:bootloader_handle_gethelp_cmd \n\r");

	// 1. Get Total length of the command packet
	uint32_t command_packet_len = bl_rx_buffer[0]+1 ;

	//2. extract the CRC32 sent by the Host
	uint32_t host_crc = *((uint32_t * ) (bl_rx_buffer+command_packet_len - 4) ) ;

	// 3. verify checksum
	if (!bootloader_verify_crc(&bl_rx_buffer[0],command_packet_len-4,host_crc))
	{
        sprintf(DataBuffer, "BL_DEBUG_MSG:checksum success !!\n\r");
        bootloader_send_ack(pBuffer[0],sizeof(supported_commands));
        bootloader_uart_write_data(supported_commands,sizeof(supported_commands));

	}else
	{
        sprintf(DataBuffer, "BL_DEBUG_MSG:checksum fail !!\n\r");
        bootloader_send_nack();
	}

}

// Read the chip identifier or device Identifier
uint16_t get_mcu_chip_id(void)
{
/*
	The STM32F407vgt6 MCUs integrate an MCU ID code. This ID identifies the ST MCU partnumber
	and the die revision. It is part of the DBG_MCU component and is mapped on the
	external PPB bus (see Section 33.16 on page 1304). This code is accessible using the
	JTAG debug pCat.2ort (4 to 5 pins) or the SW debug port (two pins) or by the user software.
	It is even accessible while the MCU is under system reset. */
	uint16_t cid;
	// 0xE0042000UL
	cid = (uint16_t)( (DBGMCU->IDCODE) & 0x0FFF );
	return  cid;

}
/*Helper function to handle BL_GET_CID command */
void bootloader_handle_getcid_cmd(uint8_t *pBuffer)
{
	uint16_t bl_cid_num = 0;
	sprintf(DataBuffer, "bootloader_handle_getcid_cmd\n\r");

    // 1. Total length of the command packet
	uint32_t command_packet_len = bl_rx_buffer[0]+1 ;

	// 2. extract the CRC32 sent by the Host
	uint32_t host_crc = *((uint32_t * ) (bl_rx_buffer+command_packet_len - 4) ) ;

	// 3. verify data integrity
	if (! bootloader_verify_crc(&bl_rx_buffer[0],command_packet_len-4,host_crc))
	{
		sprintf(DataBuffer, "BL_DEBUG_MSG:checksum success !!\n\r");
        bootloader_send_ack(pBuffer[0],2);
        bl_cid_num = get_mcu_chip_id();
        bootloader_uart_write_data((uint8_t *)&bl_cid_num,2);
	}else
	{
		sprintf(DataBuffer, "BL_DEBUG_MSG:checksum fail !!\n\r");
        bootloader_send_nack();
	}


}

/*This function reads the RDP ( Read protection option byte) value
 *For more info refer "Table 16. Description of the option bytes" in stm32f407vgt6 RM0090
 *Bits 15:8
 *0xAA: Level 0, no protection
 *0xCC: Level 2, chip protection (debug and boot from RAM features disabled)
 *Others: Level 1, read protection of memories (debug features limited)
 */
uint8_t get_flash_rdp_level(void)
{

	uint8_t rdp_status=0;
#if 0
	FLASH_OBProgramInitTypeDef  ob_handle;
	HAL_FLASHEx_OBGetConfig(&ob_handle);
	rdp_status = (uint8_t)ob_handle.RDPLevel;
#else
	 volatile uint32_t *pOB_addr = (uint32_t*) 0x1FFFC000;
	 rdp_status =  (uint8_t)(*pOB_addr >> 8) ;
#endif

	return rdp_status;

}
/*Helper function to handle BL_GET_RDP_STATUS command */
void bootloader_handle_getrdp_cmd(uint8_t *pBuffer)
{
    uint8_t rdp_level = 0x00;
    sprintf(DataBuffer, "BL_DEBUG_MSG:bootloader_handle_getrdp_cmd\n\r");

    // 1. GetTotal length of the command packet
	uint32_t command_packet_len = bl_rx_buffer[0]+1 ;

	// 2. extract the CRC32 sent by the Host
	uint32_t host_crc = *((uint32_t * ) (bl_rx_buffer+command_packet_len - 4) ) ;

	// 3. verify crc
	if (! bootloader_verify_crc(&bl_rx_buffer[0],command_packet_len-4,host_crc))
	{
		sprintf(DataBuffer, "BL_DEBUG_MSG:checksum success !!\n\r");
        bootloader_send_ack(pBuffer[0],1);
        rdp_level = get_flash_rdp_level();
        bootloader_uart_write_data(&rdp_level,1);

	}else
	{
		sprintf(DataBuffer, "BL_DEBUG_MSG:checksum fail !!\n\r");
        bootloader_send_nack();
	}
}

/* this fun verify the address sent by the host */
uint8_t verify_address(uint32_t go_address)
{
	/*
	 	so, what are the valid addresses to which we can jump ?
		can we jump to system memory ? yes
		can we jump to sram1 memory ?  yes
		can we jump to sram2 memory ? yes
		can we jump to backup sram memory ? yes
		can we jump to peripheral memory ? its possible , but dont allow. so no
		can we jump to external memory ? yes.
	*/

	// incomplete - poorly written .. optimize it
	if ( go_address >= SRAM1_BASE && go_address <= SRAM1_END)
	{
		return ADDR_VALID;
	}
	else if ( go_address >= SRAM2_BASE && go_address <= SRAM2_END)
	{
		return ADDR_VALID;
	}
	else if ( go_address >= FLASH_BASE && go_address <= FLASH_END)
	{
		return ADDR_VALID;
	}
	else if ( go_address >= BKPSRAM_BASE && go_address <= BKPSRAM_END)
	{
		return ADDR_VALID;
	}
	else
		return ADDR_INVALID;
}
/* Helper function to handle BL_GO_TO_ADDR command */
void bootloader_handle_go_cmd(uint8_t *pBuffer)
{
    uint32_t go_address = 0;
    uint8_t addr_valid = ADDR_VALID;
    uint8_t addr_invalid = ADDR_INVALID;
    sprintf(DataBuffer, "BL_DEBUG_MSG: bootloader_handle_go_cmd\n\r");

    // 1. Get Total length of the command packet
	uint32_t command_packet_len = bl_rx_buffer[0]+1 ;

	// 2. extract the CRC32 sent by the Host
	uint32_t host_crc = *((uint32_t * ) (bl_rx_buffer + command_packet_len - 4) ) ;

	// 3. calculate CRC
	if (!bootloader_verify_crc(&bl_rx_buffer[0],command_packet_len-4,host_crc))
	{
		sprintf(DataBuffer, "BL_DEBUG_MSG:checksum success !!\n\r");

        bootloader_send_ack(pBuffer[0],1);

        // extract the go address from cmd packet sent by host
        go_address = *((uint32_t *)&pBuffer[2] );
        //printf("BL_DEBUG_MSG:GO addr: %#x\n",go_address);

        if( verify_address(go_address) == ADDR_VALID )
        {
            // tell host that address is fine
            bootloader_uart_write_data(&addr_valid, 1);

            /*jump to "go" address.
            we dont care what is being done there.
            host must ensure that valid code is present over there
            Its not the duty of bootloader. so just trust and jump */

            /* Not doing the below line will result in hardfault exception for ARM cortex M */
            go_address += 1; // make T bit = 1 (thumb mode, arm cm3/4 can't execute in arm mode)

            void (*lets_jump)(void) = (void *)go_address;
            sprintf(DataBuffer, "BL_DEBUG_MSG: jumping to go address! \n\r");
            lets_jump();
		}else
		{
            sprintf(DataBuffer, "BL_DEBUG_MSG: GO addr invalid ! \n\r");
            //tell host that address is invalid
            bootloader_uart_write_data(&addr_invalid, 1);
		}
	}else
	{
		sprintf(DataBuffer, "BL_DEBUG_MSG:checksum fail !!\n\r");
        bootloader_send_nack();
	}
}
uint8_t execute_flash_erase(uint8_t sector_number , uint8_t number_of_sector)
{
    // we have totally 11 sectors in STM32F407vgt6 mcu .. sector[0 to 11]
	// number_of_sector has to be in the range of 0 to 11
	// if sector_number = 0xff , that means mass erase !
	FLASH_EraseInitTypeDef flashErase_handle;
	uint32_t sectorError;
	HAL_StatusTypeDef status;


	if( number_of_sector > 11 )
		return INVALID_SECTOR;

	if( (sector_number == 0xff ) || (sector_number <= 11) )
	{
		if(sector_number == (uint8_t) 0xff)
		{
			flashErase_handle.TypeErase = FLASH_TYPEERASE_MASSERASE;
		}else
		{
		    /*Here we are just calculating how many sectors needs to erased */
			uint8_t remanining_sector = 11 - sector_number;
            if( number_of_sector > remanining_sector)
            {
            	number_of_sector = remanining_sector;
            }
			flashErase_handle.TypeErase = FLASH_TYPEERASE_SECTORS;
			flashErase_handle.Sector = sector_number; // this is the initial sector
			flashErase_handle.NbSectors = number_of_sector;
		}
		flashErase_handle.Banks = FLASH_BANK_1;

		/*Get access to touch the flash registers */
		HAL_FLASH_Unlock();
		flashErase_handle.VoltageRange = FLASH_VOLTAGE_RANGE_3;  // our mcu will work on this voltage range
		status = (uint8_t) HAL_FLASHEx_Erase(&flashErase_handle, &sectorError);
		HAL_FLASH_Lock();

		return status;
	}

	return INVALID_SECTOR;
}

/*Helper function to handle BL_FLASH_ERASE command */
void bootloader_handle_flash_erase_cmd(uint8_t *pBuffer)
{
    uint8_t erase_status = 0x00;
    sprintf(DataBuffer, "BL_DEBUG_MSG: bootloader_handle_flash_erase_cmd \n\r");

    // 1. Get Total length of the command packet
	uint32_t command_packet_len = bl_rx_buffer[0]+1 ;

	// 2. extract the CRC32 sent by the Host
	uint32_t host_crc = *((uint32_t * ) (bl_rx_buffer+command_packet_len - 4) ) ;

	// 3. verify data integrity
	if (! bootloader_verify_crc(&bl_rx_buffer[0],command_packet_len-4,host_crc))
	{
		sprintf(DataBuffer, "BL_DEBUG_MSG:checksum success !!\n\r");
        bootloader_send_ack(pBuffer[0],1);
        sprintf(DataBuffer, "BL_DEBUG_MSG:initial_sector : %d  no_ofsectors: %d \n\r",pBuffer[2],pBuffer[3]);
        //HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin,1);
        erase_status = execute_flash_erase(pBuffer[2] , pBuffer[3]);
        //HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin,0);

        bootloader_uart_write_data(&erase_status,1);
	}else
	{
		sprintf(DataBuffer, "BL_DEBUG_MSG:checksum fail !!\n\r");
        bootloader_send_nack();
	}
}

uint8_t execute_mem_write(uint8_t *pBuffer, uint32_t mem_address, uint32_t len)
{
    uint8_t status=HAL_OK;

    //We have to unlock flash module to get control of registers
    HAL_FLASH_Unlock();

    for(uint32_t i = 0 ; i <len ; i++)
    {
        //Here we program the flash byte by byte
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE,mem_address+i,pBuffer[i]);
    }

    HAL_FLASH_Lock();

    return status;
}
/*Helper function to handle BL_MEM_WRITE command */
void bootloader_handle_mem_write_cmd(uint8_t *pBuffer)
{
	uint8_t write_status = 0x00;
	uint8_t payload_len = pBuffer[6];
	uint32_t mem_address = *((uint32_t *) ( &pBuffer[2]) );
	sprintf(DataBuffer, "BL_DEBUG_MSG: bootloader_handle_mem_write_cmd \n\r");

    // 1. Get Total length of the command packet
	uint32_t command_packet_len = bl_rx_buffer[0]+1 ;

	// 2. extract the CRC32 sent by the Host
	uint32_t host_crc = *((uint32_t * ) (bl_rx_buffer+command_packet_len - 4) ) ;

	// 3. verify data integrity
	if (! bootloader_verify_crc(&bl_rx_buffer[0],command_packet_len-4,host_crc))
	{
		sprintf(DataBuffer, "BL_DEBUG_MSG:checksum success !!\n\r");
        bootloader_send_ack(pBuffer[0],1);

		if( verify_address(mem_address) == ADDR_VALID )
		{
			sprintf(DataBuffer, "BL_DEBUG_MSG: valid mem write address \n\r");

            //glow the led to indicate bootloader is currently writing to memory
            //HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);

            //execute mem write
            write_status = execute_mem_write(&pBuffer[7],mem_address, payload_len);

            //turn off the led to indicate memory write is over
            //HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

            //inform host about the status
            bootloader_uart_write_data(&write_status,1);
		}else
		{
            sprintf(DataBuffer, "BL_DEBUG_MSG: invalid mem write address \n\r");
            write_status = ADDR_INVALID;
            //inform host that address is invalid
            bootloader_uart_write_data(&write_status,1);
		}
	}else
	{
		sprintf(DataBuffer, "BL_DEBUG_MSG:checksum fail !!\n\r");
        bootloader_send_nack();
	}
}
//@#
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
