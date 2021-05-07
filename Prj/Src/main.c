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
#include "spi.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "nRF24L01.h"
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

/* USER CODE BEGIN PV */
t_nRF24L01 nRF_1;
t_nRF24L01 nRF_2;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// module 1
void CsSetLo(void)
{ HAL_GPIO_WritePin(CE_GPIO_Port, CE_Pin, GPIO_PIN_RESET); }
void CsSetHi(void)
{ HAL_GPIO_WritePin(CE_GPIO_Port, CE_Pin, GPIO_PIN_SET); }

void CsnSetLo(void)
{ HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_RESET); }
void CsnSetHi(void)
{ HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_SET); }

void SPI_Transmit(uint8_t *data, uint16_t size)
{
  HAL_SPI_Transmit(&hspi1, data, size, 100);
}
void SPI_Receive(uint8_t *data, uint16_t size)
{
  HAL_SPI_Receive(&hspi1, data, size, 100);
}
// module 2
void Cs2SetLo(void)
{ HAL_GPIO_WritePin(CE_2_GPIO_Port, CE_2_Pin, GPIO_PIN_RESET); }
void Cs2SetHi(void)
{ HAL_GPIO_WritePin(CE_2_GPIO_Port, CE_2_Pin, GPIO_PIN_SET); }

void Csn2SetLo(void)
{ HAL_GPIO_WritePin(CSN_2_GPIO_Port, CSN_2_Pin, GPIO_PIN_RESET); }
void Csn2SetHi(void)
{ HAL_GPIO_WritePin(CSN_2_GPIO_Port, CSN_2_Pin, GPIO_PIN_SET); }

void SPI2_Transmit(uint8_t *data, uint16_t size)
{
  HAL_SPI_Transmit(&hspi2, data, size, 100);
}
void SPI2_Receive(uint8_t *data, uint16_t size)
{
  HAL_SPI_Receive(&hspi2, data, size, 100);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  uint32_t ticks = HAL_GetTick();
  uint16_t wr_counter = 0;
  uint16_t rd_counter = 0;
uint8_t buf[2] = {0xF1, 0xF2};
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
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_Delay(100);
  
  HAL_GPIO_WritePin(LED_PC13_GPIO_Port, LED_PC13_Pin, GPIO_PIN_SET);
  nRF_Setup(&nRF_1, CsSetHi, CsSetLo, CsnSetHi, CsnSetLo, SPI_Transmit, SPI_Receive);
  nRF_Setup(&nRF_2, Cs2SetHi, Cs2SetLo, Csn2SetHi, Csn2SetLo, SPI2_Transmit, SPI2_Receive);
//  HAL_Delay(1);
//  nRf_SwitchReceiveMode(&nRF_2);
//  HAL_Delay(1);
    

  while(1)
  {
    if(nRF_1.nRfStatusStruct.STATUS.TX_DS == 1)
    {
//      nRf_SwitchReceiveMode(&nRF_2);
      HAL_GPIO_WritePin(LED_PC13_GPIO_Port, LED_PC13_Pin, GPIO_PIN_RESET);
    }
    else
    {
//      nRF_2.ceSetLo();
      HAL_GPIO_WritePin(LED_PC13_GPIO_Port, LED_PC13_Pin, GPIO_PIN_SET);
    }

    if( (HAL_GetTick() - ticks) > 100)
    {
//      if(nRF_2.nRfStatusStruct.STATUS.RX_P_NO == 1)
//        nRf_ReadCMD(&nRF_2, CMD_R_RX_PAYLOAD, &buf[0], 2);
      ticks = HAL_GetTick();
      nRf_Send(&nRF_1, (uint8_t*)&wr_counter, 2);
      ++wr_counter;
    }

    nRfPollingRegisters(&nRF_1);

    nRfPollingRegisters(&nRF_2);
//    HAL_Delay(2000);
//    if(nRF_0.nRfConfigStruct.CONFIG.PWR_UP == 0)
//    {
//      nRF_0.nRfConfigStruct.CONFIG.PWR_UP = 1;
//      nRfRegisterWrite(&nRF_0, &nRF_0.nRfConfigReg);
//    }
//    HAL_Delay(10);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
