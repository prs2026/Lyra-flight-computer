/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "string.h"
#include <stdbool.h>
#include <string.h>
#include <math.h>
//#include <lora.h>
#include <basiclib.h>
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
ADC_HandleTypeDef hadc;

RTC_HandleTypeDef hrtc;

SUBGHZ_HandleTypeDef hsubghz;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_RTC_Init(void);
static void MX_SUBGHZ_Init(void);
static void MX_ADC_Init(void);
/* USER CODE BEGIN PFP */

#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define transmitinterval 1000



void LED_on();
void LED_off();

void SetStandbyXOSC();
void SetPacketTypeLora();
void SetPacketTypeFSK();
uint32_t ComputeRfFreq(double frequencyMhz);
void SetRfFreq(uint32_t rfFreq);
void SetPaLowPower();
void SetPa22dB();
void SetTxPower(int8_t powerdBm);
void SetContinuousWave();
void SetTxInfinitePreamble();
void SetTx(uint32_t timeout);
void SetRx(uint32_t timeout);
void SetModulationParamsLora(const uint8_t params[4]);
void SetModulationParamsFSK(uint32_t bitrate, uint8_t pulseshape, uint8_t bandwidth, uint32_t freq_dev);
void SetPacketParamsLora(uint16_t preamble_length, bool header_fixed, uint8_t payload_length, bool crc_enabled, bool invert_iq);
void FSKBeep(int8_t powerdBm, uint32_t toneHz, uint32_t lengthMs);
int CWBeep(int8_t powerdBm, uint32_t lengthMs);

uint16_t GetIRQStatus();
void ClearIRQ();
uint16_t SubGHz_CheckAndClearTxIRQ(void);
void CfgIRQ(uint16_t bitmask);


void setupLoRa();
void sendPacket(union telempacket _packet,uint32_t timeout);
union telempacket recievePacket(uint32_t timeout);


union telempacket recievePacketACK(uint8_t command,uint32_t timeout);
uint8_t sendPacketACK(union telempacket _packet,uint32_t timeout);

void printbuffer(uint8_t *buf,uint8_t size);



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
  MX_USART2_UART_Init();
  MX_RTC_Init();
  MX_SUBGHZ_Init();
  MX_ADC_Init();
  /* USER CODE BEGIN 2 */

  uint32_t uptime = 0;
  setupLoRa();
  uint8_t commandtosend = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_12);
		uptime = HAL_GetTick();

		/*------------------------TRANSMIT-------------------*/

		union telempacket packet;
		uint8_t Astatus = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4);
		uint8_t Bstatus = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5);

		uint8_t boardstatus = Astatus | (Bstatus << 1);
		printf(" status = %x",boardstatus);
		printf(" transmitting @ %d ms ",uptime);
		packet.r.uptime = uptime;
		packet.r.ID = 0x12;
		packet.r.status = boardstatus;
		//printbuffer(packet.data, sizeof(packet.data));
		//printf(" ---- ");
		uint8_t ackcommand = 0;
		ackcommand = sendPacketACK(packet,1000);

		printf(" command 0x%x \n\r ",ackcommand);


		if (ackcommand & 0b1){
			printf("enabling A ");
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
		}
		if (ackcommand & 0b10){
			printf("enabling B ");
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
		}
		if (ackcommand & 0b100){
			printf("disabling A ");
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
		}
		if (ackcommand & 0b1000){
			printf("disabling B ");
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
		}

		HAL_Delay(transmitinterval);
		/*------------------------RECEIVE-------------------*/
//		uint8_t serialcommands[8] = {0,0,0,0,0,0,0,0};
//		HAL_UART_Receive(&huart2, serialcommands, sizeof(serialcommands), 20);
//		if(serialcommands[0] != 0){
//			printbuffer(serialcommands, sizeof(serialcommands));
//			printf("recieved command 0x%x",serialcommands[0]);
//			commandtosend = serialcommands[0];
//		}
//
//		union telempacket newpacket = recievePacketACK(commandtosend,300);
//		if (newpacket.r.ID == 0x12){
//			printf("packet received - uptime %d ms, status %d \n\r",(int)newpacket.r.uptime,(int)newpacket.r.status);
//			commandtosend = 0;
//		}
//		//printf(" packet uptime: %d ID: %x \n\r",newpacket.r.uptime,newpacket.r.ID);
//		HAL_Delay(200);


		/*------------------------ALL-------------------*/


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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.LSIDiv = RCC_LSI_DIV1;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK3|RCC_CLOCKTYPE_HCLK
                              |RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1
                              |RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK3Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC;
  hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.NbrOfConversion = 1;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_1CYCLE_5;
  hadc.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_1CYCLE_5;
  hadc.Init.OversamplingMode = DISABLE;
  hadc.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.OutPutPullUp = RTC_OUTPUT_PULLUP_NONE;
  hrtc.Init.BinMode = RTC_BINARY_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SUBGHZ Initialization Function
  * @param None
  * @retval None
  */
static void MX_SUBGHZ_Init(void)
{

  /* USER CODE BEGIN SUBGHZ_Init 0 */

  /* USER CODE END SUBGHZ_Init 0 */

  /* USER CODE BEGIN SUBGHZ_Init 1 */

  /* USER CODE END SUBGHZ_Init 1 */
  hsubghz.Init.BaudratePrescaler = SUBGHZSPI_BAUDRATEPRESCALER_8;
  if (HAL_SUBGHZ_Init(&hsubghz) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SUBGHZ_Init 2 */

  /* USER CODE END SUBGHZ_Init 2 */

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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(PRW_A_EN_GPIO_Port, PRW_A_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, PWR_B_EN_Pin|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin : PRW_A_EN_Pin */
  GPIO_InitStruct.Pin = PRW_A_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PRW_A_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PWR_B_EN_Pin PA12 */
  GPIO_InitStruct.Pin = PWR_B_EN_Pin|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART1 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);
  return ch;
}


void LED_on() {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
}
void LED_off() {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
}

void SetStandbyXOSC() {
    uint8_t txbuf[2] = {0x80, 0x01};
    HAL_SUBGHZ_ExecSetCmd(&hsubghz, txbuf[0], txbuf+1, sizeof(txbuf)-1);
}

void SetPacketTypeLora() {
    uint8_t txbuf[2] = {0x8A, 0x01};
    HAL_SUBGHZ_ExecSetCmd(&hsubghz, txbuf[0], txbuf+1, sizeof(txbuf)-1);
}

void SetPacketTypeFSK() {
    uint8_t txbuf[2] = {0x8A, 0x00};
    HAL_SUBGHZ_ExecSetCmd(&hsubghz, txbuf[0], txbuf+1, sizeof(txbuf)-1);
}

uint32_t ComputeRfFreq(double frequencyMhz) {
    return (uint32_t)(frequencyMhz * 1048576L); //2^25/(32e6)
}

void SetRfFreq(uint32_t rfFreq) {
    uint8_t txbuf[5] = {0x86, (rfFreq & 0xFF000000) >> 24, (rfFreq & 0x00FF0000) >> 16, (rfFreq & 0x0000FF00) >> 8, rfFreq & 0x000000FF};
    HAL_SUBGHZ_ExecSetCmd(&hsubghz, txbuf[0], txbuf+1, sizeof(txbuf)-1);
}

void SetPaLowPower() {
    // set Pa to 14 dB.
    uint8_t txbuf[5] = {0x95, 0x02, 0x02, 0x00, 0x01};
    HAL_SUBGHZ_ExecSetCmd(&hsubghz, txbuf[0], txbuf+1, sizeof(txbuf)-1);
}

void SetPa22dB() {
    // set Pa to the highest 22 dBm
    uint8_t txbuf[5] = {0x95, 0x04, 0x07, 0x00, 0x01};
    HAL_SUBGHZ_ExecSetCmd(&hsubghz, txbuf[0], txbuf+1, sizeof(txbuf)-1);
}

void SetTxPower(int8_t powerdBm) {
    // Between -9 and 22
    int8_t power = powerdBm < -9 ? -9 : ((powerdBm > 22) ? 22 : powerdBm);
    uint8_t txbuf[3] = {0x8E, (uint8_t) power, 0x02};
    HAL_SUBGHZ_ExecSetCmd(&hsubghz, txbuf[0], txbuf+1, sizeof(txbuf)-1);
}

void SetContinuousWave() {
    uint8_t txbuf[1] = {0xD1};
    HAL_SUBGHZ_ExecSetCmd(&hsubghz, txbuf[0], txbuf, 0);
}

void SetTxInfinitePreamble() {
    uint8_t txbuf[1] = {0xD2};
    HAL_SUBGHZ_ExecSetCmd(&hsubghz, txbuf[0], txbuf, 0);
}

void SetTx(uint32_t timeout) {
    // Timeout * 15.625 µs
    uint8_t txbuf[4] = {0x83, (timeout & 0x00FF0000) >> 16, (timeout & 0x0000FF00) >> 8, timeout & 0x000000FF};
    HAL_SUBGHZ_ExecSetCmd(&hsubghz, txbuf[0], txbuf+1, sizeof(txbuf)-1);
}

void SetRx(uint32_t timeout) {
    // Timeout * 15.625 µs
    // 0x000000 No timeout. Rx Single mode
    // 0xFFFFFF Rx Continuous mode. The device remains in RX mode until the host sends a command to change the operation mode
    uint8_t txbuf[4] = {0x82, (timeout & 0x00FF0000) >> 16, (timeout & 0x0000FF00) >> 8, timeout & 0x000000FF};
    HAL_SUBGHZ_ExecSetCmd(&hsubghz, txbuf[0], txbuf+1, sizeof(txbuf)-1);
}

void SetModulationParamsLora(const uint8_t params[4]) {
    uint8_t txbuf[5] = {0x8B, params[0], params[1], params[2], params[3]};
    HAL_SUBGHZ_ExecSetCmd(&hsubghz, txbuf[0], txbuf+1, sizeof(txbuf)-1);
}

void SetModulationParamsFSK(uint32_t bitrate, uint8_t pulseshape, uint8_t bandwidth, uint32_t freq_dev) {
    uint32_t BR = 32 * 32e6 / bitrate;
    uint32_t fdev = (uint32_t) (freq_dev * 1.048576L); // 2^25/32e6 = 1.048576
    uint8_t txbuf[9] = {0x8B, (BR & 0x00FF0000) >> 16, (BR & 0x0000FF00) >> 8, BR & 0x000000FF, pulseshape, bandwidth, (fdev & 0x00FF0000) >> 16, (fdev & 0x0000FF00) >> 8, fdev & 0x000000FF};
    HAL_SUBGHZ_ExecSetCmd(&hsubghz, txbuf[0], txbuf+1, sizeof(txbuf)-1);
}

void SetPacketParamsLora(uint16_t preamble_length, bool header_fixed, uint8_t payload_length, bool crc_enabled, bool invert_iq) {
    uint8_t txbuf[7] = {0x8C, (uint8_t)((preamble_length >> 8) & 0xFF), (uint8_t)(preamble_length & 0xFF),
                        (uint8_t) header_fixed, payload_length, (uint8_t) crc_enabled, (uint8_t) invert_iq};

    HAL_SUBGHZ_ExecSetCmd(&hsubghz, txbuf[0], txbuf+1, sizeof(txbuf)-1);
}
/*
void WriteBuffer(uint8_t offset, uint8_t *data, uint8_t len) {
    HAL_SUBGHZ_WriteBuffer(&hsubghz, offset, data, len);
}

void ReadBuffer(uint8_t offset, uint8_t *data, uint8_t len) {
    HAL_SUBGHZ_ReadBuffer(&hsubghz, offset, data, len);
}
*/

uint16_t GetIRQStatus(){
	uint8_t rxbuf[3];
	HAL_SUBGHZ_GetError(&hsubghz);
	HAL_SUBGHZ_ExecGetCmd(&hsubghz, 0x12, rxbuf, sizeof(rxbuf));
	//printbuffer(rxbuf, sizeof(rxbuf));
	uint16_t irqstatus = (rxbuf[2] << 4) | rxbuf[1];
	return irqstatus;
}

void ClearIRQ() {
    uint8_t txbuf[2];
    HAL_SUBGHZ_ExecGetCmd(&hsubghz, 0x02, txbuf, sizeof(txbuf));
    //printbuffer(txbuf, sizeof(txbuf));
}

void CfgIRQ(uint16_t bitmask) {
    uint8_t txbuf[3] = {0x08,(uint8_t)(bitmask>>8),(uint8_t)(bitmask)};
    //printbuffer(txbuf, 3);
    HAL_SUBGHZ_ExecSetCmd(&hsubghz, txbuf[0], txbuf, 3);
}


/*
void FSKBeep(int8_t powerdBm, uint32_t toneHz, uint32_t lengthMs) {
    // assume in standbyXOSC already.
    HAL_Delay(1);
    SetTxPower(powerdBm);
    SetModulationParamsFSK(toneHz*2,    0x09,     0x1E,      2500);
    HAL_Delay(5);
    SetTxInfinitePreamble();
    HAL_Delay(lengthMs);
    SetStandbyXOSC();
    HAL_Delay(5);
}

int CWBeep(int8_t powerdBm, uint32_t lengthMs) {
    HAL_Delay(1);
    SetTxPower(powerdBm);
    HAL_Delay(5);
    SetContinuousWave();
    HAL_Delay(lengthMs);
    SetStandbyXOSC();
    HAL_Delay(5);
    return 0;
}
*/

void setupLoRa() {
	LED_on();
	HAL_Delay(100);
	LED_off();
	SetStandbyXOSC();
	HAL_Delay(1);
	SetPacketTypeLora();
	HAL_Delay(1);
	SetRfFreq(ComputeRfFreq(915.225));
	CfgIRQ(0xffff);

	//SetPaLowPower(); // For powers up to 14 dBm
	SetPa22dB(); // Allows powers up to 22 dBm
	HAL_Delay(1);
	SetTxPower(22);
	HAL_Delay(1);
	//uint8_t LORA_SF12_BW62_CR45[4] = {0x0C, 0x03, 0x01, 0x00};
	uint8_t LORA_SF8_BW62_CR45[4] = {0x8, 0x03, 0x01, 0x00};
	SetModulationParamsLora(LORA_SF8_BW62_CR45);
	HAL_Delay(1);

	SetPacketParamsLora(4, true, sizeof(union telempacket), true, false); // Send 4 bytes
	HAL_Delay(1);
	uint8_t buffer[4] = {0x00, 0x00, 0x00, 0x00};
	HAL_SUBGHZ_WriteBuffer(&hsubghz, 0, buffer, 4);
}



void sendPacket(union telempacket _packet,uint32_t timeout) {
	uint8_t buffer[sizeof(_packet)];

	memcpy(buffer,_packet.data,sizeof(_packet));

	printbuffer(buffer, sizeof(buffer));

	HAL_SUBGHZ_WriteBuffer(&hsubghz, 0, buffer, sizeof(_packet));
	HAL_Delay(1);
	SetTx(0);
	LED_on();
	uint32_t starttime =  HAL_GetTick();
	while ((GetIRQStatus() & 0x1) == 0){
			HAL_Delay(05);
			//printf("waiting... %d \n\r",GetIRQStatus());
			if(HAL_GetTick() - starttime > timeout) break;
	}
}


union telempacket recievePacket(uint32_t timeout){
	ClearIRQ();
	SetRx(0);
	uint32_t starttime =  HAL_GetTick();
	while ((GetIRQStatus() & 0x6) == 0){
		HAL_Delay(05);
		//printf("waiting... %d \n\r",GetIRQStatus());
		if(HAL_GetTick() - starttime > timeout) break;
	} // wait for rx irq


	ClearIRQ();
	union telempacket _packet;
	uint8_t buffer[sizeof(_packet)];

	HAL_SUBGHZ_ReadBuffer(&hsubghz, 0, buffer, sizeof(buffer));
	//printbuffer(buffer, sizeof(buffer));

	memcpy(_packet.data,buffer,sizeof(buffer));

	return _packet;
}


uint8_t sendPacketACK(union telempacket _packet, uint32_t timeout) {
	uint8_t txbuffer[sizeof(_packet)];

	memcpy(txbuffer,_packet.data,sizeof(_packet));

	printbuffer(txbuffer, sizeof(txbuffer));

	HAL_SUBGHZ_WriteBuffer(&hsubghz, 0, txbuffer, sizeof(_packet));
	HAL_Delay(1);
	SetTx(0);
	LED_on();
	uint32_t starttime =  HAL_GetTick();
	while ((GetIRQStatus() & 0x1) == 0){
			HAL_Delay(05);
			//printf("waiting... %d \n\r",GetIRQStatus());
			if(HAL_GetTick() - starttime > timeout) break;
	}

	uint8_t buffer[sizeof(_packet)];

	HAL_SUBGHZ_ReadBuffer(&hsubghz, 0, buffer, sizeof(buffer));

	//wait for ack
	union telempacket ackpack;
	while (HAL_GetTick() - starttime < timeout){
		printf(" - waiting for ack - ");
		ackpack = recievePacket(timeout);

		if (ackpack.r.ID == 0x34){
			printf(" recieved ack with %d uptime ",ackpack.r.uptime);
			return ackpack.r.status;
		}
		else{
			printf(" recieved something ",ackpack.r.uptime);
		}
		}

	return 0;
}


union telempacket recievePacketACK(uint8_t command,uint32_t timeout){
	ClearIRQ();
	SetRx(0);
	uint32_t starttime =  HAL_GetTick();
	while ((GetIRQStatus() & 0x6) == 0){
		HAL_Delay(05);
		//printf("waiting... %d \n\r",GetIRQStatus());
		if(HAL_GetTick() - starttime > timeout) break;
	}
	//printf(" clearing irqs ");
	ClearIRQ();
	union telempacket _packet;
	uint8_t rxbuffer[sizeof(_packet)];

	HAL_SUBGHZ_ReadBuffer(&hsubghz, 0, rxbuffer, sizeof(rxbuffer));


	memcpy(_packet.data,rxbuffer,sizeof(rxbuffer));
	if (_packet.r.ID == 0x12) {
		printf(" got good packet - receive buffer: \n\r  ");
		printbuffer(rxbuffer, sizeof(rxbuffer));
		printf(" \n\r");
		memcpy(_packet.data,rxbuffer,sizeof(rxbuffer));

	} else {
	  LED_off();
	  //printf(" ID error ");
	  return _packet;

	}

	HAL_Delay(100);

	union telempacket _ackpacket;
	_ackpacket.r.ID = 0x34;
	_ackpacket.r.status = command;
	_ackpacket.r.uptime = HAL_GetTick();

	printf("sendingack - \n\r  ");

	sendPacket(_ackpacket,timeout);

	printf(" done acking \n\r ");

	return _packet;
}


// prints the contents of a buffer
void printbuffer(uint8_t *buf,uint8_t size){
	for (uint8_t i = 0; i < size; i++){
		printf(" %x",buf[i]);
	}
	return;
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
	  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_12);
	  HAL_Delay(50);
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
