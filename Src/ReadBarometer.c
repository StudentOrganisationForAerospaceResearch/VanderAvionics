#include "stm32f4xx.h"
#include "stm32f4xx_hal_conf.h"
#include "cmsis_os.h"
#include "math.h"

#include "ReadBarometer.h"
#include "Data.h"

static const int READ_BAROMETER_PERIOD = 1000;

static const int CMD_SIZE = 1;
static const int CMD_TIMEOUT = 150;

static const uint8_t ADC_D1_512_CONV_CMD = 0x42;
static const uint8_t ADC_D2_512_CONV_CMD = 0x52;
static const uint8_t ADC_READ_CMD = 0x00;
static const uint8_t PROM_READ_CMD_SENS = 0xA2;
static const uint8_t PROM_READ_CMD_OFF = 0xA4;
static const uint8_t PROM_READ_CMD_TCS = 0xA6;
static const uint8_t PROM_READ_CMD_TCO = 0xA8;
static const uint8_t PROM_READ_CMD_TREF = 0xAA;
static const uint8_t PROM_READ_CMD_TEMPSENS = 0xAC;
static const uint8_t RESET_CMD = 0x1E;

void readBarometerTask(void const* arg)
{
    BarometerData* data = (BarometerData*) arg;
    uint32_t prevWakeTime = osKernelSysTick();

    HAL_GPIO_WritePin(BARO_CS_GPIO_Port, BARO_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, &RESET_CMD, CMD_SIZE, CMD_TIMEOUT);
    osDelay(3);   // 2.8ms reload after Reset command
    HAL_GPIO_WritePin(BARO_CS_GPIO_Port, BARO_CS_Pin, GPIO_PIN_SET);

    uint8_t dataInBuffer;

    // PROM read for calibration coefficients
    uint16_t c1SENS = readCalibrationCoefficient(PROM_READ_CMD_SENS);
    uint16_t c2OFF = readCalibrationCoefficient(PROM_READ_CMD_OFF);
    uint16_t c3TCS = readCalibrationCoefficient(PROM_READ_CMD_TCS);
    uint16_t c4TCO = readCalibrationCoefficient(PROM_READ_CMD_TCO);
    uint16_t c5TREF = readCalibrationCoefficient(PROM_READ_CMD_TREF);
    uint16_t c6TEMPSENS = readCalibrationCoefficient(PROM_READ_CMD_TEMPSENS);

    uint32_t pressureReading;   // Stores a 24 bit value
    uint32_t temperatureReading;   // Stores a 24 bit value

    for (;;)
    {
        osDelayUntil(&prevWakeTime, READ_BAROMETER_PERIOD);

        // Read D1 pressure
        HAL_GPIO_WritePin(BARO_CS_GPIO_Port, BARO_CS_Pin, GPIO_PIN_RESET);
        HAL_SPI_Transmit(&hspi2, &ADC_D1_512_CONV_CMD, CMD_SIZE, CMD_TIMEOUT);
        HAL_GPIO_WritePin(BARO_CS_GPIO_Port, BARO_CS_Pin, GPIO_PIN_SET);

        osDelay(2); // 1.17ms max conversion time for OSR 512

        HAL_GPIO_WritePin(BARO_CS_GPIO_Port, BARO_CS_Pin, GPIO_PIN_RESET);
        HAL_SPI_Transmit(&hspi2, &ADC_READ_CMD, CMD_SIZE, CMD_TIMEOUT);

        pressureReading = 0;

        HAL_SPI_TransmitReceive(&hspi2, &ADC_READ_CMD, &dataInBuffer, CMD_SIZE, CMD_TIMEOUT);
        pressureReading += dataInBuffer << 16;

        HAL_SPI_TransmitReceive(&hspi2, &ADC_READ_CMD, &dataInBuffer, CMD_SIZE, CMD_TIMEOUT);
        pressureReading += dataInBuffer << 8;

        HAL_SPI_TransmitReceive(&hspi2, &ADC_READ_CMD, &dataInBuffer, CMD_SIZE, CMD_TIMEOUT);
        pressureReading += dataInBuffer;

        HAL_GPIO_WritePin(BARO_CS_GPIO_Port, BARO_CS_Pin, GPIO_PIN_SET);

        // Read D2 temperature
        HAL_GPIO_WritePin(BARO_CS_GPIO_Port, BARO_CS_Pin, GPIO_PIN_RESET);
        HAL_SPI_Transmit(&hspi2, &ADC_D2_512_CONV_CMD, CMD_SIZE, CMD_TIMEOUT);
        HAL_GPIO_WritePin(BARO_CS_GPIO_Port, BARO_CS_Pin, GPIO_PIN_SET);

        osDelay(2); // 1.17ms max conversion time for OSR 512

        HAL_GPIO_WritePin(BARO_CS_GPIO_Port, BARO_CS_Pin, GPIO_PIN_RESET);
        HAL_SPI_Transmit(&hspi2, &ADC_READ_CMD, CMD_SIZE, CMD_TIMEOUT);

        temperatureReading = 0;

        HAL_SPI_TransmitReceive(&hspi2, &ADC_READ_CMD, &dataInBuffer, CMD_SIZE, CMD_TIMEOUT);
        temperatureReading += dataInBuffer << 16;

        HAL_SPI_TransmitReceive(&hspi2, &ADC_READ_CMD, &dataInBuffer, CMD_SIZE, CMD_TIMEOUT);
        temperatureReading += dataInBuffer << 8;

        HAL_SPI_TransmitReceive(&hspi2, &ADC_READ_CMD, &dataInBuffer, CMD_SIZE, CMD_TIMEOUT);
        temperatureReading += dataInBuffer;

        HAL_GPIO_WritePin(BARO_CS_GPIO_Port, BARO_CS_Pin, GPIO_PIN_SET);

        // calculate 1st order pressure and temperature (MS5607 1st order algorithm from datasheet)
        double dT = temperatureReading - c5TREF * pow(2, 8);
        double TEMP = 2000 + dT * c6TEMPSENS / pow(2, 23);
        double OFF = c2OFF * pow(2, 17) + dT * c4TCO / pow(2, 6);
        double SENS = c1SENS * pow(2, 16) + dT * c3TCS / pow(2, 7);
        double P = (pressureReading * SENS / pow(2, 21) - OFF) / pow(2, 15) / 100; // divide by 100 to get mbar
        double T = (TEMP) / 100; // divide by 100 to get degrees Celcius

        osMutexWait(data->mutex_, 0);
        data->pressure_ = (int32_t) P;
        data->temperature_ = (int32_t) T;
        osMutexRelease(data->mutex_);
    }
}

uint16_t readCalibrationCoefficient(const uint8_t PROM_READ_CMD)
{
    uint8_t dataInBuffer;
    uint16_t coefficient = 0;
    HAL_GPIO_WritePin(BARO_CS_GPIO_Port, BARO_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, &PROM_READ_CMD, CMD_SIZE, CMD_TIMEOUT);
    HAL_SPI_TransmitReceive(&hspi2, &ADC_READ_CMD, &dataInBuffer, CMD_SIZE, CMD_TIMEOUT);
    coefficient = dataInBuffer << 8;
    HAL_SPI_TransmitReceive(&hspi2, &ADC_READ_CMD, &dataInBuffer, CMD_SIZE, CMD_TIMEOUT);
    coefficient += dataInBuffer;
    HAL_GPIO_WritePin(BARO_CS_GPIO_Port, BARO_CS_Pin, GPIO_PIN_SET);
    return coefficient;
}
