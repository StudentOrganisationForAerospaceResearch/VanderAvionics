/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include "ReadAccelGyroMagnetism.h"
#include "ReadExternalPressure.h"
#include "ReadExternalTemperature.h"
#include "ReadGps.h"
#include "ReadIntegratedTemperature.h"
#include "ReadOxidizerTankConditions.h"
#include "MonitorForEmergencyShutoff.h"
#include "EngineControl.h"
#include "ParachutesControl.h"
#include "LogData.h"
#include "TransmitData.h"
#include "Data.h"
#include "FlightPhase.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;

UART_HandleTypeDef huart1;

osThreadId defaultTaskHandle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
static osThreadId readAccelGyroMagnetismTaskHandle;
static osThreadId readExternalPressureTaskHandle;
static osThreadId readExternalTemperatureTaskHandle;
static osThreadId readGpsTaskHandle;
static osThreadId readIntegratedTemperatureTaskHandle;
static osThreadId readOxidizerTankConditionsTaskHandle;
// Controls that will perform actions
static osThreadId monitorForEmergencyShutoffTaskHandle;
static osThreadId engineControlTaskHandle;
static osThreadId parachutesControlTaskHandle;
// Storing data
static osThreadId logDataTaskHandle;
static osThreadId transmitDataTaskHandle;

FlightPhase currentFlightPhase = PRELAUNCH;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI2_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_SPI3_Init();
  MX_USART1_UART_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
    // data primitive structs
    AccelGyroMagnetismData* accelGyroMagnetismData =
        malloc(sizeof(AccelGyroMagnetismData));
    ExternalPressureData* externalPressureData =
        malloc(sizeof(ExternalPressureData));
    ExternalTemperatureData* externalTemperatureData =
        malloc(sizeof(ExternalTemperatureData));
    GpsData* gpsData =
        malloc(sizeof(GpsData));
    IntegratedTemperatureData* integratedTemperatureData =
        malloc(sizeof(IntegratedTemperatureData));
    OxidizerTankConditionsData* oxidizerTankConditionsData =
        malloc(sizeof(OxidizerTankConditionsData));

    osMutexDef(ACCEL_GYRO_MAGNETISM_DATA_MUTEX);
    accelGyroMagnetismData->mutex_ = osMutexCreate(osMutex(ACCEL_GYRO_MAGNETISM_DATA_MUTEX));
    accelGyroMagnetismData->accelX_ = -1.f;
    accelGyroMagnetismData->accelY_ = -2.f;
    accelGyroMagnetismData->accelZ_ = -3.f;
    accelGyroMagnetismData->gyroX_ = -4.f;
    accelGyroMagnetismData->gyroY_ = -5.f;
    accelGyroMagnetismData->gyroZ_ = -6.f;
    accelGyroMagnetismData->magnetoX_ = -7.f;
    accelGyroMagnetismData->magnetoY_ = -8.f;
    accelGyroMagnetismData->magnetoZ_ = -9.f;

    osMutexDef(EXTERNAL_PRESSURE_DATA_MUTEX);
    externalPressureData->mutex_ = osMutexCreate(osMutex(EXTERNAL_PRESSURE_DATA_MUTEX));
    externalPressureData->externalPressure_ = -10;

    osMutexDef(EXTERNAL_TEMPERATURE_DATA_MUTEX);
    externalTemperatureData->mutex_ = osMutexCreate(osMutex(EXTERNAL_TEMPERATURE_DATA_MUTEX));
    externalTemperatureData->externalTemperature_ = -11;

    osMutexDef(INTEGRATED_TEMPERATURE_DATA_MUTEX);
    integratedTemperatureData->mutex_ = osMutexCreate(osMutex(INTEGRATED_TEMPERATURE_DATA_MUTEX));
    integratedTemperatureData->integratedTemperature_ = -12;

    osMutexDef(GPS_DATA_MUTEX);
    gpsData->mutex_ = osMutexCreate(osMutex(GPS_DATA_MUTEX));
    gpsData->altitude_ = -13;
    gpsData->epochTimeMsec_ = -14;
    gpsData->latitude_ = -15;
    gpsData->longitude_ = -16;

    osMutexDef(OXIDIZER_TANK_CONDITIONS_DATA_MUTEX);
    oxidizerTankConditionsData->mutex_ = osMutexCreate(osMutex(OXIDIZER_TANK_CONDITIONS_DATA_MUTEX));
    oxidizerTankConditionsData->pressure_ = -17.f;
    oxidizerTankConditionsData->temperature_ = -18.f;

    // data containers
    AllData* allData =
        malloc(sizeof(AllData));
    allData->accelGyroMagnetismData_ = accelGyroMagnetismData;
    allData->externalPressureData_ = externalPressureData;
    allData->externalTemperatureData_ = externalTemperatureData;
    allData->gpsData_ = gpsData;
    allData->integratedTemperatureData_ = integratedTemperatureData;
    allData->oxidizerTankConditionsData_ = oxidizerTankConditionsData;

    ParachutesControlData* parachutesControlData =
        malloc(sizeof(ParachutesControlData));
    parachutesControlData->accelGyroMagnetismData_ = accelGyroMagnetismData;
    parachutesControlData->externalPressureData_ = externalPressureData;
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
    /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
    /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */

    osThreadDef(
        readAccelGyroMagnetismThread,
        readAccelGyroMagnetismTask,
        osPriorityNormal,
        1,
        configMINIMAL_STACK_SIZE
    );
    readAccelGyroMagnetismTaskHandle =
        osThreadCreate(osThread(readAccelGyroMagnetismThread), accelGyroMagnetismData);

    osThreadDef(
        readExternalPressureThread,
        readExternalPressureTask,
        osPriorityNormal,
        1,
        configMINIMAL_STACK_SIZE
    );
    readExternalPressureTaskHandle =
        osThreadCreate(osThread(readExternalPressureThread), externalPressureData);

    osThreadDef(
        readExternalTemperatureThread,
        readExternalTemperatureTask,
        osPriorityLow,
        1,
        configMINIMAL_STACK_SIZE
    );
    readExternalTemperatureTaskHandle =
        osThreadCreate(osThread(readExternalTemperatureThread), externalTemperatureData);

    osThreadDef(
        readIntegratedTemperatureThread,
        readIntegratedTemperatureTask,
        osPriorityLow,
        1,
        configMINIMAL_STACK_SIZE
    );
    readIntegratedTemperatureTaskHandle =
        osThreadCreate(osThread(readIntegratedTemperatureThread), integratedTemperatureData);

    osThreadDef(
        readGpsThread,
        readGpsTask,
        osPriorityBelowNormal,
        1,
        configMINIMAL_STACK_SIZE
    );
    readGpsTaskHandle =
        osThreadCreate(osThread(readGpsThread), gpsData);

    osThreadDef(
        readOxidizerTankConditionsThread,
        readOxidizerTankConditionsTask,
        osPriorityAboveNormal,
        1,
        configMINIMAL_STACK_SIZE
    );
    readOxidizerTankConditionsTaskHandle =
        osThreadCreate(osThread(readOxidizerTankConditionsThread), oxidizerTankConditionsData);

    osThreadDef(
        monitorForEmergencyShutoffThread,
        monitorForEmergencyShutoffTask,
        osPriorityHigh,
        1,
        configMINIMAL_STACK_SIZE
    );
    monitorForEmergencyShutoffTaskHandle =
        osThreadCreate(osThread(monitorForEmergencyShutoffThread), accelGyroMagnetismData);

    osThreadDef(
        engineControlThread,
        engineControlTask,
        osPriorityNormal,
        1,
        configMINIMAL_STACK_SIZE * 2
    );
    engineControlTaskHandle =
        osThreadCreate(osThread(engineControlThread), NULL);

    osThreadDef(
        parachutesControlThread,
        parachutesControlTask,
        osPriorityAboveNormal,
        1,
        configMINIMAL_STACK_SIZE * 2
    );
    parachutesControlTaskHandle =
        osThreadCreate(osThread(parachutesControlThread), parachutesControlData);

    osThreadDef(
        logDataThread,
        logDataTask,
        osPriorityNormal,
        1,
        configMINIMAL_STACK_SIZE * 3
    );
    logDataTaskHandle =
        osThreadCreate(osThread(logDataThread), allData);

    osThreadDef(
        transmitDataThread,
        transmitDataTask,
        osPriorityNormal,
        1,
        configMINIMAL_STACK_SIZE * 3
    );
    transmitDataTaskHandle =
        osThreadCreate(osThread(transmitDataThread), allData);

  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
 

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
    }

    free(accelGyroMagnetismData);
    free(externalPressureData);
    free(externalTemperatureData);
    free(gpsData);
    free(integratedTemperatureData);
    free(oxidizerTankConditionsData);
    free(allData);
    free(parachutesControlData);
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* SPI2 init function */
static void MX_SPI2_Init(void)
{

  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI3 init function */
static void MX_SPI3_Init(void)
{

  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED1_Pin|LED2_Pin|BARO_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED1_Pin LED2_Pin BARO_CS_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin|BARO_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : SD_CS_Pin */
  GPIO_InitStruct.Pin = SD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SD_CS_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN 5 */
    /* Infinite loop */
    uint32_t prevWakeTime = osKernelSysTick();

    for (;;)
    {
        osDelayUntil(&prevWakeTime, 250);
        HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
    }

  /* USER CODE END 5 */ 
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
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
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
      ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
