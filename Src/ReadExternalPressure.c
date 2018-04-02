#include "stm32f4xx.h"
#include "stm32f4xx_hal_conf.h"
#include "cmsis_os.h"

#include "ReadExternalPressure.h"
#include "Data.h"
#include "main.h"

static const int READ_EXTERNAL_PRESSURE_PERIOD = 1000;
static const uint8_t RESET_COMMAND = 0x1E;
static const int CMD_SIZE = 1;
static const int CMD_TIMEOUT = 3000;
static const uint8_t ADC_512_CONV_CMD = 0x42;
static const uint8_t ADC_READ_CMD = 0x00;
static uint8_t dataIn;
static int temp;

void readExternalPressureTask(void const* arg)
{
    ExternalPressureData* data = (ExternalPressureData*) arg;
    uint32_t prevWakeTime = osKernelSysTick();

    baroCSLow();
    HAL_SPI_Transmit(&hspi2, &RESET_COMMAND, CMD_SIZE, CMD_TIMEOUT);
    HAL_Delay(10);
    baroCSHigh();

    for (;;)
    {
        osDelayUntil(&prevWakeTime, READ_EXTERNAL_PRESSURE_PERIOD);
        baroCSLow();
        HAL_SPI_Transmit(&hspi2, &ADC_512_CONV_CMD, CMD_SIZE, CMD_TIMEOUT);
        baroCSHigh();

        osDelay(10);

        baroCSLow();
        HAL_SPI_Transmit(&hspi2, &ADC_READ_CMD, CMD_SIZE, CMD_TIMEOUT);

        HAL_SPI_Transmit(&hspi2, &ADC_READ_CMD, CMD_SIZE, CMD_TIMEOUT);
        HAL_SPI_Receive(&hspi2, &dataIn, CMD_SIZE, CMD_TIMEOUT);
        temp = 65536 * dataIn;

        HAL_SPI_Transmit(&hspi2, &ADC_READ_CMD, CMD_SIZE, CMD_TIMEOUT);
        HAL_SPI_Receive(&hspi2, &dataIn, CMD_SIZE, CMD_TIMEOUT);
        temp = temp + 256 * dataIn;

        HAL_SPI_Transmit(&hspi2, &ADC_READ_CMD, CMD_SIZE, CMD_TIMEOUT);
        HAL_SPI_Receive(&hspi2, &dataIn, CMD_SIZE, CMD_TIMEOUT);
        temp = temp + dataIn;

        baroCSHigh();

        osMutexWait(data->mutex_, 0);
        data->externalPressure_ = temp;
        osMutexRelease(data->mutex_);
    }
}
void baroCSLow()
{
    HAL_GPIO_WritePin(BARO_CS_GPIO_Port, BARO_CS_Pin, GPIO_PIN_RESET);
}
void baroCSHigh()
{
    HAL_GPIO_WritePin(BARO_CS_GPIO_Port, BARO_CS_Pin, GPIO_PIN_SET);
}
