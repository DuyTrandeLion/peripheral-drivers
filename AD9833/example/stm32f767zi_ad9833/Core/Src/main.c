/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "spi.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ad9833.h"
#include "mcp4xxx.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_RESISTANCE		10000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static AD983x_State_t AD9833_State;
static MCP4xxx_State_t MCP41010_State;

static DDS_Def_t AD9833_st;
static PTM_Def_t MCP41010_st;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static AD983x_State_t AD9833_SPI_Event_Handle(DDS_SPI_Event_t commEvent_en, uint8_t *data_p8, uint16_t dataSize_u16, void *context_p);
static MCP4xxx_State_t MCP41010_SPI_Event_Handle(PTM_SPI_Event_t commEvent_en, uint8_t *data_p8, uint16_t dataSize_u16, void *context_p);
static void AD9833_Delay_Handle(uint32_t delayTime_u32);
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
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_USB_OTG_FS_PCD_Init();
  /* USER CODE BEGIN 2 */
  MCP41010_st.device = MCP42xx;
  MCP41010_st.mode = POTENTIOMETER_MODE;
  MCP41010_st.maxResistance = MAX_RESISTANCE;
  MCP41010_st.spiHandle = MCP41010_SPI_Event_Handle;

  MCP41010_State = MCP4xxx_Init(&MCP41010_st);
  MCP41010_State = MCP4xxx_WritePotentiometerValue(&MCP41010_st, MCP4xxx_SELECT_PTM_1, 5000);

  AD9833_st.outputEnabled = 1;
  AD9833_st.dacEnabled = 1;
  AD9833_st.internalClockEnabled = 1;
  AD9833_st.referenceFrequency = 25000000UL;
  AD9833_st.spiHandle = AD9833_SPI_Event_Handle;
  AD9833_st.delayHandle = AD9833_Delay_Handle;

  AD9833_State = AD983x_Init(&AD9833_st);
  AD9833_State = AD983x_ApplySignal(&AD9833_st, REG0, 1000000, REG0, 0.0, REG0, SQUARE_WAVE);
  AD9833_State = AD983x_EnableOutput(&AD9833_st, 1);

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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
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
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  /** Initializes the CPU, AHB and APB buses clocks
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
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
static AD983x_State_t AD9833_SPI_Event_Handle(DDS_SPI_Event_t commEvent_en, uint8_t *data_p8, uint16_t dataSize_u16, void *context_p)
{
	HAL_StatusTypeDef ret;

	if (SPI_DDS_EVENT_TRANSMIT == commEvent_en)
	{
		HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
		ret = HAL_SPI_Transmit(&hspi1, data_p8, dataSize_u16, 1000);
		HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
	}

	switch (ret)
	{
		case HAL_ERROR:
		{
			ret = AD983x_COMM_ERROR;
			break;
		}

		case HAL_BUSY:
		{
			ret = AD983x_DEVICE_BUSY;
			break;
		}

		case HAL_TIMEOUT:
		{
			ret = AD983x_COMM_TIMEOUT;
			break;
		}

		default: break;
	}

	return ret;
}

static MCP4xxx_State_t MCP41010_SPI_Event_Handle(PTM_SPI_Event_t commEvent_en, uint8_t *data_p8, uint16_t dataSize_u16, void *context_p)
{
	HAL_StatusTypeDef ret;

	if (SPI_PTM_EVENT_TRANSMIT == commEvent_en)
	{
		HAL_GPIO_WritePin(SPI1_CS1_GPIO_Port, SPI1_CS1_Pin, GPIO_PIN_RESET);
		ret = HAL_SPI_Transmit(&hspi1, data_p8, dataSize_u16, 1000);
		HAL_GPIO_WritePin(SPI1_CS1_GPIO_Port, SPI1_CS1_Pin, GPIO_PIN_SET);
	}

	switch (ret)
	{
		case HAL_ERROR:
		{
			ret = MCP4xxx_COMM_ERROR;
			break;
		}

		case HAL_BUSY:
		{
			ret = MCP4xxx_DEVICE_BUSY;
			break;
		}

		case HAL_TIMEOUT:
		{
			ret = MCP4xxx_COMM_TIMEOUT;
			break;
		}

		default: break;
	}

	return ret;
}

static void AD9833_Delay_Handle(uint32_t delayTime_u32)
{
	HAL_Delay(delayTime_u32);
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
