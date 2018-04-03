#include "stm32f4xx.h"
#include "stm32f4xx_hal_conf.h"
#include "cmsis_os.h"

#include "ReadExternalPressure.h"
#include "Data.h"

static const int READ_EXTERNAL_PRESSURE_PERIOD = 100;

static const int CMD_SIZE = 1;
static const int CMD_TIMEOUT = 150;
static const uint8_t ADC_512_CONV_CMD = 0x42;
static const uint8_t ADC_READ_CMD = 0x00;
static const uint8_t RESET_CMD = 0x1E;

static uint8_t dataIn;

void readExternalPressureTask(void const* arg)
{
    ExternalPressureData* data = (ExternalPressureData*) arg;
    uint32_t prevWakeTime = osKernelSysTick();

    HAL_GPIO_WritePin(BARO_CS_GPIO_Port, BARO_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, &RESET_CMD, CMD_SIZE, CMD_TIMEOUT);
    osDelay(3);   // 2.8ms reload after Reset command
    HAL_GPIO_WritePin(BARO_CS_GPIO_Port, BARO_CS_Pin, GPIO_PIN_SET);

    uint32_t pressureReading;   // Stores a 24 bit value

    for (;;)
    {
        osDelayUntil(&prevWakeTime, READ_EXTERNAL_PRESSURE_PERIOD);
        HAL_GPIO_WritePin(BARO_CS_GPIO_Port, BARO_CS_Pin, GPIO_PIN_RESET);
        HAL_SPI_Transmit(&hspi2, &ADC_512_CONV_CMD, CMD_SIZE, CMD_TIMEOUT);
        HAL_GPIO_WritePin(BARO_CS_GPIO_Port, BARO_CS_Pin, GPIO_PIN_SET);

        osDelay(2); // 1.17ms max conversion time for OSR 512

        HAL_GPIO_WritePin(BARO_CS_GPIO_Port, BARO_CS_Pin, GPIO_PIN_RESET);
        HAL_SPI_Transmit(&hspi2, &ADC_READ_CMD, CMD_SIZE, CMD_TIMEOUT);

        pressureReading = 0;

        HAL_SPI_Transmit(&hspi2, &ADC_READ_CMD, CMD_SIZE, CMD_TIMEOUT);
        HAL_SPI_Receive(&hspi2, &dataIn, CMD_SIZE, CMD_TIMEOUT);
        pressureReading += dataIn << 16;

        HAL_SPI_Transmit(&hspi2, &ADC_READ_CMD, CMD_SIZE, CMD_TIMEOUT);
        HAL_SPI_Receive(&hspi2, &dataIn, CMD_SIZE, CMD_TIMEOUT);
        pressureReading += dataIn << 8;

        HAL_SPI_Transmit(&hspi2, &ADC_READ_CMD, CMD_SIZE, CMD_TIMEOUT);
        HAL_SPI_Receive(&hspi2, &dataIn, CMD_SIZE, CMD_TIMEOUT);
        pressureReading += dataIn;

        HAL_GPIO_WritePin(BARO_CS_GPIO_Port, BARO_CS_Pin, GPIO_PIN_SET);

        osMutexWait(data->mutex_, 0);
        data->externalPressure_ = pressureReading;
        osMutexRelease(data->mutex_);
    }
}
