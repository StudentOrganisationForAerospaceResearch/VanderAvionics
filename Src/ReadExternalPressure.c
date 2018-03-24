#include "stm32f4xx.h"
#include "stm32f4xx_hal_conf.h"
#include "cmsis_os.h"

#include "ReadExternalPressure.h"
#include "Data.h"
#include "main.h"

#define BARO_CS_HIGH HAL_GPIO_WritePin(BARO_CS_GPIO_Port, BARO_CS_Pin, GPIO_PIN_SET);
#define BARO_CS_LOW HAL_GPIO_WritePin(BARO_CS_GPIO_Port, BARO_CS_Pin, GPIO_PIN_RESET);

static const int READ_EXTERNAL_PRESSURE_PERIOD = 1000;
static const uint8_t RESET_COMMAND = 0x1E;
static const int COMMAND_SIZE = 1;
static const int COMMAND_TIMEOUT = 3000;
static const uint8_t READ_COMMAND1 = 0x42;
static const uint8_t READ_COMMAND2 = 0x00;
static uint8_t dataIn;
static int temp;

void readExternalPressureTask(void const* arg)
{
    ExternalPressureData* data = (ExternalPressureData*) arg;
    uint32_t prevWakeTime = osKernelSysTick();

	BARO_CS_LOW;
	HAL_SPI_Transmit(&hspi2, &RESET_COMMAND, COMMAND_SIZE, COMMAND_TIMEOUT);
    BARO_CS_HIGH;

	BARO_CS_LOW;
	HAL_SPI_Transmit(&hspi2, &READ_COMMAND1, COMMAND_SIZE, 30000);
	BARO_CS_HIGH;

	BARO_CS_LOW;
	HAL_SPI_Transmit(&hspi2, &READ_COMMAND2, COMMAND_SIZE, COMMAND_TIMEOUT);

    HAL_SPI_Transmit(&hspi2, &READ_COMMAND2, COMMAND_SIZE, COMMAND_TIMEOUT);
    HAL_SPI_Receive(&hspi2, &dataIn, COMMAND_SIZE, COMMAND_TIMEOUT);

    for (;;)
    {
        // BARO_CS_LOW
        osDelayUntil(&prevWakeTime, READ_EXTERNAL_PRESSURE_PERIOD);
        // 255
        HAL_SPI_Transmit(&hspi2, &READ_COMMAND2, COMMAND_SIZE, COMMAND_TIMEOUT);
        HAL_SPI_Receive(&hspi2, &dataIn, COMMAND_SIZE, COMMAND_TIMEOUT);
		temp=65536*dataIn;

		//255
        HAL_SPI_Transmit(&hspi2, &READ_COMMAND2, COMMAND_SIZE, COMMAND_TIMEOUT);
     	HAL_SPI_Receive(&hspi2, &dataIn, COMMAND_SIZE, COMMAND_TIMEOUT);
     	temp=temp+256*dataIn;

     	// 255
    	HAL_SPI_Transmit(&hspi2, &READ_COMMAND2, COMMAND_SIZE, COMMAND_TIMEOUT);
	  	HAL_SPI_Receive(&hspi2, &dataIn, COMMAND_SIZE, COMMAND_TIMEOUT);
     	temp=temp+dataIn;

		// BARO_CS_HIGH;

     	osMutexWait(data->mutex_, 0);
        data->externalPressure_ = temp;
        osMutexRelease(data->mutex_);
    }
}
