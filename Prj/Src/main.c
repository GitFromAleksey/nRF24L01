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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
t_nRfConfig       nRfConfig;
t_nRF_EN_AA       nRfEnAa;
t_nRf_EN_RXADDR   nRfEnRxAddr;
t_nRf_SETUP_AW    nRfSetupAw;
t_nRf_SETUP_RETR  nRfSetupRetr;
t_nRF_RF_CH       nRfRfCh;
t_nRF_RF_SETUP    nRfRfSetup;
t_nRF_STATUS      nRfStatus;
t_nRF_OBSERVE_TX  nRfObserveTx;
t_nRF_CD          nRfCd;

t_register RegistersArr[10];

void CsnOn(void)
{ HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_RESET); }
void CsnOff(void)
{ HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_SET); }

void SPI_Transmit(uint8_t *data, uint16_t size)
{
  HAL_SPI_Transmit(&hspi1, data, size, 100);
}

void SPI_Receive(uint8_t *data, uint16_t size)
{
  HAL_SPI_Receive(&hspi1, data, size, 100);
}

void nRfRegisterRead(t_register *pReg)
{
  CsnOn();
  
  SPI_Transmit(&pReg->addr, 1);
  SPI_Receive(pReg->reg_union, 1);
  
  CsnOff();
}

void nRfRegisterWrite(t_register *pReg)
{
  uint8_t buf[2] = {0};
  buf[0] = pReg->addr | (1<<5);
  buf[1] = *pReg->reg_union;
  CsnOn();
  
  SPI_Transmit(buf, 2);
  
  CsnOff();
}

int main(void)
{
  int i = 0;
  uint8_t spi_data_out[10] = {0};
  uint8_t spi_data_in[10] = {0};

  t_register reg;

  reg.addr = REG_CONFIG;
  reg.reg_union = &nRfConfig.byte;
  RegistersArr[0] = reg;
  
  reg.addr = REG_EN_AA;
  reg.reg_union = &nRfEnAa.byte;
  RegistersArr[1] = reg;
  
  reg.addr = REG_EN_RXADDR;
  reg.reg_union = &nRfEnRxAddr.byte;
  RegistersArr[2] = reg;

  reg.addr = REG_SETUP_AW;
  reg.reg_union = &nRfSetupAw.byte;
  RegistersArr[3] = reg;

  reg.addr = REG_SETUP_REPR;
  reg.reg_union = &nRfSetupRetr.byte;
  RegistersArr[4] = reg;

  reg.addr = REG_RF_CH;
  reg.reg_union = &nRfRfCh.byte;
  RegistersArr[5] = reg;

  reg.addr = REG_RF_SETUP;
  reg.reg_union = &nRfRfSetup.byte;
  RegistersArr[6] = reg;

  reg.addr = REG_STATUS;
  reg.reg_union = &nRfStatus.byte;
  RegistersArr[7] = reg;

  reg.addr = REG_OBSERVE_TX;
  reg.reg_union = &nRfObserveTx.byte;
  RegistersArr[8] = reg;
  
  reg.addr = REG_CD;
  reg.reg_union = &nRfCd.byte;
  RegistersArr[9] = reg;
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
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    for(i = 0; i < sizeof(RegistersArr)/sizeof(t_register); ++i)
    {
      nRfRegisterRead(&RegistersArr[i]);
      HAL_Delay(100);
    }
     
    nRfRegisterRead(&RegistersArr[0]);
    if(nRfConfig.CONFIG.PWR_UP == 0)
    {
      nRfConfig.CONFIG.PWR_UP = 1;
      nRfRegisterWrite(&RegistersArr[0]);
    }
//    HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_RESET);
//    
//    HAL_SPI_Transmit(&hspi1, spi_data_out, 1, 100);
//    HAL_SPI_Receive(&hspi1, spi_data_in, 2, 100);
//    
//    HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_SET);
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
