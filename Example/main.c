/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>

#include "TI_ADS_ADC.h"
#include "ADI_DF_ADC.h"
#include "LTC26x1.h"
#include "EEPROM.h"

#include "Miscellaneous.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
uint8_t log_data[512];

#define SIMPLE_LOG_INFO(fmt, ...)												\
	sprintf((char *)log_data, fmt, ## __VA_ARGS__);								\
	HAL_UART_Transmit(&huart3, log_data, strlen((const char *)log_data), 100)
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static volatile uint32_t tim2_counter = 0;

static bool spiReady = true;

static uint8_t randomWriteSize = 25;
static uint8_t randomWriteData = 0x15;
static uint8_t randomPageData  = 0xAE;

static uint16_t randomWriteAddr = 0x2200;
static uint16_t randomPageAddr  = 0x3000;

static uint16_t dac_value = 37899;

static uint8_t writeData[32] = {0};
static uint8_t readData[32] = {0};

static uint8_t pageWriteData[64] = {0};
static uint8_t pageReadData[64]  = {0};

static uint8_t gRawADCCBuffer_au8[4];

static int32_t gLastRawADCData_i32;
static int32_t gRawADCData_i32;
static int32_t gVolt_i32;

static float gVolt_f;

static DFADC_State_t  gLTCStatus_en;
static DAC_State_t    gLTCDACStatus_en;
static ADSADC_State_t gADSStatus_en;
static EEPROM_State_t gEEPROMStatus_en;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
DFADC_State_t LTC250032_SPIHandle(DFADC_SPI_Event_t locSPIEvent_en, uint8_t *locData_pu8, uint16_t locDataSize_u16, void *locContext_p);
DFADC_State_t LTC250032_CtrlHandle(DFADC_Control_Event_t locContEvent_en, uint32_t locSignal_u32, void *locContext_p);
DAC_State_t LTC2601_SPIHandle(DAC_SPI_Event_t locSPIEvent_en, uint8_t *locData_pu8, uint16_t locDataSize_u16, void *locContext_p);
ADSADC_State_t ADS8881_SPIHandle(ADSADC_SPI_Event_t locSPIEvent_en, uint8_t *locData_pu8, uint16_t locDataSize_u16, void *locContext_p);
ADSADC_State_t ADS8881_CtrlHandle(ADSADC_Control_Event_t locContEvent_en, uint32_t locSignal_u32, void *locContext_p);

EEPROM_State_t MCHP_EEPHandle(EEPROM_Event_t locEEPEvent_en,
									uint16_t locDeviceAddress_u16,
									uint16_t locMemAddress_u16,
									uint8_t *locData_p8,
									uint16_t locDataSize_u16,
									void * locContext_p);

static void LTC250032_Init();
static void LTC250032_Run();

static void LTC2601_Init();
static void LTC2601_Run();

static void ADS8881_Init();
static void ADS8881_Run();

static void AT24C256_Init();
static void AT24C256_Run();

static void _25LC256_Init();
static void _25LC256_Run();

static void delay_5us(uint32_t delayTime);
__STATIC_INLINE void DWT_Delay_us(volatile uint32_t microseconds);

static DFADC_Def_t gLTCADCDef_s =
{
		.spiHandle = LTC250032_SPIHandle,
		.controlHandle = LTC250032_CtrlHandle,
		.busyHandle = NULL
};

static DAC_Def_t gLTCDACDef_s =
{
		.spiHandle = LTC2601_SPIHandle,
		.busyHandle = NULL
};

static ADSADC_Def_t gADSADCDef_s =
{
		.spiHandle = ADS8881_SPIHandle,
		.controlHandle = ADS8881_CtrlHandle,
		.delayHandle = NULL
};

static EEPROM_Def_t gEEPROMDef_s =
{
		.commHandle = MCHP_EEPHandle,
		.busyHandle = NULL
};

static EEPROM_Def_t gSPIEEPROMDef_s =
{
		.commHandle = MCHP_EEPHandle,
		.busyHandle = NULL
};
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
  MX_SPI1_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim2);

//  _25LC256_Init();
//
//  _25LC256_Run();

//  LTC250032_Init();
//  LTC2601_Init();
  ADS8881_Init();
//  AT24C256_Init();

//  AT24C256_Run();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//	  LTC2601_Run();
//	  HAL_Delay(10);
//	  LTC250032_Run();
	  ADS8881_Run();

	  HAL_Delay(50);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure LSE Drive Capability 
  */
  HAL_PWR_EnableBkUpAccess();
  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode 
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInitStruct.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
DFADC_State_t LTC250032_SPIHandle(DFADC_SPI_Event_t locSPIEvent_en, uint8_t *locData_pu8, uint16_t locDataSize_u16, void *locContext_p)
{
	DFADC_State_t locRet = HAL_OK;

	switch (locSPIEvent_en)
	{
		case SPI_DF_EVENT_TRANSMIT:
		{
			HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
			locRet = HAL_SPI_Transmit(&hspi1, locData_pu8, locDataSize_u16, 100);
			HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
			break;
		}

		case SPI_DF_EVENT_RECEIVE:
		{
			HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
			locRet = HAL_SPI_Receive(&hspi1, locData_pu8, locDataSize_u16, 100);
			HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
			break;
		}

		case SPI_DF_EVENT_TRANSMIT_RECEIVE:
		{
			break;
		}

		default: break;
	}

	return locRet;
}


DFADC_State_t LTC250032_CtrlHandle(DFADC_Control_Event_t locContEvent_en, uint32_t locSignal_u32, void *locContext_p)
{
	DFADC_State_t locRet = DFADC_OK;

	switch (locContEvent_en)
	{
		case CONTROL_DF_EVENT_CHECK_BUSY:
		{
			if (GPIO_PIN_SET == HAL_GPIO_ReadPin(BUSY_GPIO_Port, BUSY_Pin))
			{
				locRet = DFADC_DEVICE_BUSY;
			}
			else
			{
				locRet = DFADC_DEVICE_NOT_BUSY;
			}
			break;
		}

		case CONTROL_DF_EVENT_CHECK_DATA_READY:
		{
			if (GPIO_PIN_SET == HAL_GPIO_ReadPin(DRL_GPIO_Port, DRL_Pin))
			{
				locRet = DFADC_DATA_NOT_READY;
			}
			else
			{
				locRet = DFADC_DATA_READY;
			}
			break;
		}

		case CONTROL_DF_EVENT_SET_CLOCK:
		{
			HAL_GPIO_WritePin(MCLK_GPIO_Port, MCLK_Pin, (locSignal_u32 == 0)? GPIO_PIN_RESET : GPIO_PIN_SET);
			break;
		}

		case CONTROL_DF_EVENT_PRESET_MODE:
		{
			HAL_GPIO_WritePin(PRE_GPIO_Port, PRE_Pin, (locSignal_u32 == 0)? GPIO_PIN_RESET : GPIO_PIN_SET);
			break;
		}

		case CONTROL_DF_EVENT_WAIT:
		{
			delay_5us(locSignal_u32);
			break;
		}

		default: break;
	}

	return locRet;
}


DAC_State_t LTC2601_SPIHandle(DAC_SPI_Event_t locSPIEvent_en, uint8_t *locData_pu8, uint16_t locDataSize_u16, void *locContext_p)
{
	DAC_State_t locRet = HAL_OK;

	switch (locSPIEvent_en)
	{
		case SPI_DAC_EVENT_TRANSMIT:
		{
			HAL_GPIO_WritePin(DAC_CS_GPIO_Port, DAC_CS_Pin, GPIO_PIN_RESET);
			locRet = HAL_SPI_Transmit(&hspi1, locData_pu8, locDataSize_u16, 100);
			HAL_GPIO_WritePin(DAC_CS_GPIO_Port, DAC_CS_Pin, GPIO_PIN_SET);
			break;
		}

		case SPI_DAC_EVENT_CLEAR_INPUT:
		{
			HAL_GPIO_WritePin(DAC_CLR_GPIO_Port, DAC_CLR_Pin, GPIO_PIN_RESET);
			HAL_Delay(1);
			HAL_GPIO_WritePin(DAC_CLR_GPIO_Port, DAC_CLR_Pin, GPIO_PIN_SET);
			break;
		}

		default: break;
	}

	return locRet;
}


ADSADC_State_t ADS8881_SPIHandle(ADSADC_SPI_Event_t locSPIEvent_en, uint8_t *locData_pu8, uint16_t locDataSize_u16, void *locContext_p)
{
	ADSADC_State_t locRet = ADSADC_OK;

	switch (locSPIEvent_en)
	{
		case SPI_ADS_EVENT_RECEIVE:
		{
			locRet = HAL_SPI_Receive_IT(&hspi1, locData_pu8, locDataSize_u16);
			break;
		}

		default: break;
	}

	return locRet;
}


ADSADC_State_t ADS8881_CtrlHandle(ADSADC_Control_Event_t locContEvent_en, uint32_t locSignal_u32, void *locContext_p)
{
	ADSADC_State_t locRet = ADSADC_OK;

	switch (locContEvent_en)
	{
		case CONTROL_ADS_EVENT_3WIRE_CONVST:
		{
			if (0 == locSignal_u32)
			{
				if (false == spiReady)
				{
					break;
				}
				spiReady = false;
				HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
			}
			else
			{
				while (false == spiReady)
				{

				}
				HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
			}
			break;
		}

		case CONTROL_ADS_EVENT_4WIRE_CONVST:
		{
			HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, (locSignal_u32 == 0)? GPIO_PIN_RESET : GPIO_PIN_SET);
			break;
		}

		case CONTROL_ADS_EVENT_4WIRE_DIN:
		{
			break;
		}

		default: break;
	}

	return locRet;
}


EEPROM_State_t MCHP_EEPHandle(EEPROM_Event_t locEEPEvent_en,
									uint16_t locDeviceAddress_u16,
									uint16_t locMemAddress_u16,
									uint8_t *locData_p8,
									uint16_t locDataSize_u16,
									void * locContext_p)
{
	EEPROM_State_t locRet = EEPROM_OK;

	switch (locEEPEvent_en)
	{
		case SPI_EEP_EVENT_TRANSMIT:
		{
			locRet = HAL_SPI_Transmit(&hspi1, locData_p8, locDataSize_u16, 100);
			break;
		}

		case SPI_EEP_EVENT_RECEIVE:
		{
			locRet = HAL_SPI_Receive(&hspi1, locData_p8, locDataSize_u16, 100);
			break;
		}

		case SPI_EVENT_CS_LOW:
		{
			HAL_GPIO_WritePin(DAC_CS_GPIO_Port, DAC_CS_Pin, GPIO_PIN_RESET);
			break;
		}

		case SPI_EVENT_CS_HIGH:
		{
			HAL_GPIO_WritePin(DAC_CS_GPIO_Port, DAC_CS_Pin, GPIO_PIN_SET);
			break;
		}

		case I2C_EEP_EVENT_TRANSMIT:
		{
			locRet = HAL_I2C_Mem_Write(&hi2c1, locDeviceAddress_u16, locMemAddress_u16, 2, locData_p8, locDataSize_u16, 100);
			break;
		}

		case I2C_EEP_EVENT_RECEIVE:
		{
			locRet = HAL_I2C_Mem_Read(&hi2c1, locDeviceAddress_u16, locMemAddress_u16, 2, locData_p8, locDataSize_u16, 100);
			break;
		}

		default: break;
	}

	return locRet;
}


static void LTC250032_Init()
{
	DFADC_Config_t locLTCConfig_s;

	locLTCConfig_s.deviceType = LTC2500_32;
	locLTCConfig_s.gain = DFADC_GAIN_DISABLE;
	locLTCConfig_s.downSamplingFactor = DF_4096;
	locLTCConfig_s.filterType = AVERAGING_FILTER;
	locLTCConfig_s.refVoltage = 4.096;
	locLTCConfig_s.timeout = 80 * (HAL_RCC_GetHCLKFreq() / 1000000);

	gLTCStatus_en = DFADC_Init(&gLTCADCDef_s);
	SIMPLE_LOG_INFO("LTC2500-32 init status %d\r\n", gLTCStatus_en);

	DFADC_Control(&gLTCADCDef_s, CONTROL_DF_EVENT_PRESET_MODE, 0, NULL);

	gLTCStatus_en = DFADC_Config(&gLTCADCDef_s, locLTCConfig_s);
	SIMPLE_LOG_INFO("LTC2500-32 config status %d\r\n", gLTCStatus_en);
}


static void LTC2601_Init()
{
	DAC_Config_t locLTCConfig_s;

	locLTCConfig_s.deviceType = LTC2601;
	locLTCConfig_s.refVoltage = 4.096;
	locLTCConfig_s.timeout = 80 * (HAL_RCC_GetHCLKFreq() / 1000000);

	gLTCDACStatus_en = DAC_Init(&gLTCDACDef_s);
	SIMPLE_LOG_INFO("LTC2601 init status %d\r\n", gLTCDACStatus_en);

	gLTCDACStatus_en = DAC_Config(&gLTCDACDef_s, locLTCConfig_s);
	SIMPLE_LOG_INFO("LTC2601 config status %d\r\n", gLTCDACStatus_en);
}


static void ADS8881_Init()
{
	ADSADC_Config_t locADSConfig_s;

	locADSConfig_s.deviceType = ADS8881;
	locADSConfig_s.refVoltage = 4.096;
	locADSConfig_s.convTime   = 600 * (HAL_RCC_GetHCLKFreq() / 1000000);
	locADSConfig_s.timeout    = 80 * (HAL_RCC_GetHCLKFreq() / 1000000);

	gADSStatus_en = ADSADC_Init(&gADSADCDef_s);
	SIMPLE_LOG_INFO("ADS8881 init status %d\r\n", gADSStatus_en);

	gADSStatus_en = ADSADC_Config(&gADSADCDef_s, locADSConfig_s);
	SIMPLE_LOG_INFO("ADS8881 config status %d\r\n", gADSStatus_en);
}


static void AT24C256_Init()
{
	gEEPROMDef_s.config.blockSize     = 64;
	gEEPROMDef_s.config.blocks        = 512;
	gEEPROMDef_s.config.communication = I2C_COMM;
	gEEPROMDef_s.config.deviceAddress = 0xA0;
	gEEPROMDef_s.config.timeout       = 200 * (HAL_RCC_GetHCLKFreq() / 1000000);

	gEEPROMStatus_en = EEPROM_Init(&gEEPROMDef_s);
	SIMPLE_LOG_INFO("AT24C256 init status %d\r\n", gEEPROMStatus_en);
}


static void _25LC256_Init()
{
	gSPIEEPROMDef_s.config.blockSize     = 64;
	gSPIEEPROMDef_s.config.blocks        = 512;
	gSPIEEPROMDef_s.config.communication = SPI_COMM;
	gSPIEEPROMDef_s.config.timeout       = 200 * (HAL_RCC_GetHCLKFreq() / 1000000);

	gEEPROMStatus_en = EEPROM_Init(&gSPIEEPROMDef_s);
	SIMPLE_LOG_INFO("25LC256 init status %d\r\n", gEEPROMStatus_en);
}


static void LTC250032_Run()
{
	DFADC_StartADConversion(&gLTCADCDef_s);
	gLTCStatus_en = DFADC_ReadDFDistributedData(&gLTCADCDef_s, gRawADCCBuffer_au8);
	gLastRawADCData_i32 = gRawADCData_i32;
	CONV_TO_INT32_RAW_DATA(gRawADCCBuffer_au8, gRawADCData_i32);
	DFADC_ConvToVoltage(&gLTCADCDef_s, gRawADCCBuffer_au8, &gVolt_f);
	gVolt_i32 = (int32_t)(gVolt_f * 100000.0);

	SIMPLE_LOG_INFO("gLTCStatus_en %d - Raw ADC %d - Diff %d - Votlage (x100k) %d\r\n",
							gLTCStatus_en,
							gRawADCData_i32,
							gRawADCData_i32 - gLastRawADCData_i32,
							gVolt_i32);
}


static void LTC2601_Run()
{
	gLTCDACStatus_en = DAC_StartDAConversion(&gLTCDACDef_s, WRITE_TO_INPUT_REGISTER, dac_value);

	SIMPLE_LOG_INFO("gLTCStatus_en %d - Raw ADC %d\r\n",
							gLTCDACStatus_en,
							dac_value);
}


static void ADS8881_Run()
{
	ADSADC_StartADConversion(&gADSADCDef_s);
	gADSStatus_en = ADSADC_ReadDistributedData(&gADSADCDef_s, gRawADCCBuffer_au8);
	gLastRawADCData_i32 = gRawADCData_i32;
	CONV_18BITS_TO_INT32_RAW_DATA(gRawADCCBuffer_au8, &gRawADCData_i32);
	ADSADC_ConvToVoltage(&gADSADCDef_s, gRawADCCBuffer_au8, &gVolt_f);
	gVolt_i32 = gVolt_f * 100000;
	SIMPLE_LOG_INFO("gADSStatus_en %d - Raw ADC %d - Diff %d - Votlage (x100k) %d\r\n",
							gADSStatus_en,
							gRawADCData_i32,
							gRawADCData_i32 - gLastRawADCData_i32,
							gVolt_i32);
}


static void AT24C256_Run()
{
	memset(writeData, randomWriteData, SIZE(writeData));
	memset(pageWriteData, randomPageData, SIZE(pageWriteData));

	gEEPROMStatus_en = EEPROM_WriteMemory(&gEEPROMDef_s, randomWriteAddr, writeData, randomWriteSize);
	SIMPLE_LOG_INFO("Write Memory state %d\r\n", gEEPROMStatus_en);
	HAL_Delay(20);

	gEEPROMStatus_en = EEPROM_ReadMemory(&gEEPROMDef_s, randomWriteAddr, readData, randomWriteSize);
	SIMPLE_LOG_INFO("Read Memory state %d\r\n", gEEPROMStatus_en);
	HAL_Delay(20);

	gEEPROMStatus_en = EEPROM_WriteMemoryPage(&gEEPROMDef_s, randomPageAddr, pageWriteData);
	SIMPLE_LOG_INFO("Write Memory Page state %d\r\n", gEEPROMStatus_en);
	HAL_Delay(20);

	gEEPROMStatus_en= EEPROM_ReadMemory(&gEEPROMDef_s, randomPageAddr, pageReadData, SIZE(pageReadData));
	SIMPLE_LOG_INFO("Read Memory state %d\r\n", gEEPROMStatus_en);
	HAL_Delay(20);
}


static void _25LC256_Run()
{
	memset(writeData, randomWriteData, SIZE(writeData));
	memset(pageWriteData, randomPageData, SIZE(pageWriteData));

	gEEPROMStatus_en = EEPROM_WriteMemory(&gSPIEEPROMDef_s, randomWriteAddr, writeData, randomWriteSize);
	SIMPLE_LOG_INFO("Write Memory state %d\r\n", gEEPROMStatus_en);
	HAL_Delay(20);

	gEEPROMStatus_en = EEPROM_ReadMemory(&gSPIEEPROMDef_s, randomWriteAddr, readData, randomWriteSize);
	SIMPLE_LOG_INFO("Read Memory state %d\r\n", gEEPROMStatus_en);
	HAL_Delay(20);

	gEEPROMStatus_en = EEPROM_WriteMemoryPage(&gSPIEEPROMDef_s, randomPageAddr, pageWriteData);
	SIMPLE_LOG_INFO("Write Memory Page state %d\r\n", gEEPROMStatus_en);
	HAL_Delay(20);

	gEEPROMStatus_en = EEPROM_ReadMemory(&gSPIEEPROMDef_s, randomPageAddr, pageReadData, SIZE(pageReadData));
	SIMPLE_LOG_INFO("Read Memory state %d\r\n", gEEPROMStatus_en);
	HAL_Delay(20);
}


static void delay_5us(uint32_t delayTime)
{
	uint32_t startTime = tim2_counter;

	while ((startTime + delayTime) > tim2_counter)
	{

	}
}


__STATIC_INLINE void DWT_Delay_us(volatile uint32_t microseconds)
{
	 uint32_t clk_cycle_start = DWT->CYCCNT;

	/* Go to number of cycles for system */
	 microseconds *= (HAL_RCC_GetHCLKFreq() / 1000000);

	/* Delay till end */
	 while ((DWT->CYCCNT - clk_cycle_start) < microseconds);
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (TIM2 == htim->Instance)
	{
		tim2_counter++;
	}
}


void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{

}


void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if (hspi->Instance == hspi1.Instance)
	{
		spiReady = true;
	}
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
