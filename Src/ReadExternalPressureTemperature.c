#include "stm32f4xx.h"
#include "stm32f4xx_hal_conf.h"
#include "cmsis_os.h"
#include "math.h"

#include "ReadExternalPressureTemperature.h"
#include "Data.h"

static const int READ_EXTERNAL_PRESSURE_TEMPERATURE_PERIOD = 1000;

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

static uint8_t dataIn;

void readExternalPressureTemperatureTask(void const* arg)
{
    ExternalPressureTemperatureData* data = (ExternalPressureTemperatureData*) arg;
    uint32_t prevWakeTime = osKernelSysTick();

    HAL_GPIO_WritePin(BARO_CS_GPIO_Port, BARO_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, &RESET_CMD, CMD_SIZE, CMD_TIMEOUT);
    osDelay(3);   // 2.8ms reload after Reset command
    HAL_GPIO_WritePin(BARO_CS_GPIO_Port, BARO_CS_Pin, GPIO_PIN_SET);

    uint16_t C1_SENS = 0;
    uint16_t C2_OFF = 0;
    uint16_t C3_TCS = 0;
    uint16_t C4_TCO = 0;
    uint16_t C5_TREF = 0;
    uint16_t C6_TEMPSENS = 0;

    // PROM read for calibration coefficients
    HAL_GPIO_WritePin(BARO_CS_GPIO_Port, BARO_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, &PROM_READ_CMD_SENS, CMD_SIZE, CMD_TIMEOUT);
    HAL_SPI_TransmitReceive(&hspi2, &ADC_READ_CMD, &dataIn, CMD_SIZE, CMD_TIMEOUT);
    C1_SENS = dataIn << 8;
    HAL_SPI_TransmitReceive(&hspi2, &ADC_READ_CMD, &dataIn, CMD_SIZE, CMD_TIMEOUT);
    C1_SENS += dataIn;
    HAL_GPIO_WritePin(BARO_CS_GPIO_Port, BARO_CS_Pin, GPIO_PIN_SET);

    HAL_GPIO_WritePin(BARO_CS_GPIO_Port, BARO_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, &PROM_READ_CMD_OFF, CMD_SIZE, CMD_TIMEOUT);
    HAL_SPI_TransmitReceive(&hspi2, &ADC_READ_CMD, &dataIn, CMD_SIZE, CMD_TIMEOUT);
    C2_OFF = dataIn << 8;
    HAL_SPI_TransmitReceive(&hspi2, &ADC_READ_CMD, &dataIn, CMD_SIZE, CMD_TIMEOUT);
    C2_OFF += dataIn;
    HAL_GPIO_WritePin(BARO_CS_GPIO_Port, BARO_CS_Pin, GPIO_PIN_SET);

    HAL_GPIO_WritePin(BARO_CS_GPIO_Port, BARO_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, &PROM_READ_CMD_TCS, CMD_SIZE, CMD_TIMEOUT);
    HAL_SPI_TransmitReceive(&hspi2, &ADC_READ_CMD, &dataIn, CMD_SIZE, CMD_TIMEOUT);
    C3_TCS = dataIn << 8;
    HAL_SPI_TransmitReceive(&hspi2, &ADC_READ_CMD, &dataIn, CMD_SIZE, CMD_TIMEOUT);
    C3_TCS += dataIn;
    HAL_GPIO_WritePin(BARO_CS_GPIO_Port, BARO_CS_Pin, GPIO_PIN_SET);

    HAL_GPIO_WritePin(BARO_CS_GPIO_Port, BARO_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, &PROM_READ_CMD_TCO, CMD_SIZE, CMD_TIMEOUT);
    HAL_SPI_TransmitReceive(&hspi2, &ADC_READ_CMD, &dataIn, CMD_SIZE, CMD_TIMEOUT);
    C4_TCO = dataIn << 8;
    HAL_SPI_TransmitReceive(&hspi2, &ADC_READ_CMD, &dataIn, CMD_SIZE, CMD_TIMEOUT);
    C4_TCO += dataIn;
    HAL_GPIO_WritePin(BARO_CS_GPIO_Port, BARO_CS_Pin, GPIO_PIN_SET);

    HAL_GPIO_WritePin(BARO_CS_GPIO_Port, BARO_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, &PROM_READ_CMD_TREF, CMD_SIZE, CMD_TIMEOUT);
    HAL_SPI_TransmitReceive(&hspi2, &ADC_READ_CMD, &dataIn, CMD_SIZE, CMD_TIMEOUT);
    C5_TREF = dataIn << 8;
    HAL_SPI_TransmitReceive(&hspi2, &ADC_READ_CMD, &dataIn, CMD_SIZE, CMD_TIMEOUT);
    C5_TREF += dataIn;
    HAL_GPIO_WritePin(BARO_CS_GPIO_Port, BARO_CS_Pin, GPIO_PIN_SET);

    HAL_GPIO_WritePin(BARO_CS_GPIO_Port, BARO_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, &PROM_READ_CMD_TEMPSENS, CMD_SIZE, CMD_TIMEOUT);
    HAL_SPI_TransmitReceive(&hspi2, &ADC_READ_CMD, &dataIn, CMD_SIZE, CMD_TIMEOUT);
    C6_TEMPSENS = dataIn << 8;
    HAL_SPI_TransmitReceive(&hspi2, &ADC_READ_CMD, &dataIn, CMD_SIZE, CMD_TIMEOUT);
    C6_TEMPSENS += dataIn;

    HAL_GPIO_WritePin(BARO_CS_GPIO_Port, BARO_CS_Pin, GPIO_PIN_SET);

    uint32_t pressureReading;   // Stores a 24 bit value
    uint32_t temperatureReading;   // Stores a 24 bit value

    for (;;)
    {
        osDelayUntil(&prevWakeTime, READ_EXTERNAL_PRESSURE_TEMPERATURE_PERIOD);
        // Read D1 pressure
        HAL_GPIO_WritePin(BARO_CS_GPIO_Port, BARO_CS_Pin, GPIO_PIN_RESET);
        HAL_SPI_Transmit(&hspi2, &ADC_D1_512_CONV_CMD, CMD_SIZE, CMD_TIMEOUT);
        HAL_GPIO_WritePin(BARO_CS_GPIO_Port, BARO_CS_Pin, GPIO_PIN_SET);

        osDelay(2); // 1.17ms max conversion time for OSR 512

        HAL_GPIO_WritePin(BARO_CS_GPIO_Port, BARO_CS_Pin, GPIO_PIN_RESET);
        HAL_SPI_Transmit(&hspi2, &ADC_READ_CMD, CMD_SIZE, CMD_TIMEOUT);

        pressureReading = 0;

        HAL_SPI_TransmitReceive(&hspi2, &ADC_READ_CMD, &dataIn, CMD_SIZE, CMD_TIMEOUT);
        pressureReading += dataIn << 16;

        HAL_SPI_TransmitReceive(&hspi2, &ADC_READ_CMD, &dataIn, CMD_SIZE, CMD_TIMEOUT);
        pressureReading += dataIn << 8;

        HAL_SPI_TransmitReceive(&hspi2, &ADC_READ_CMD, &dataIn, CMD_SIZE, CMD_TIMEOUT);
        pressureReading += dataIn;

        HAL_GPIO_WritePin(BARO_CS_GPIO_Port, BARO_CS_Pin, GPIO_PIN_SET);

        // Read D2 temperature
        HAL_GPIO_WritePin(BARO_CS_GPIO_Port, BARO_CS_Pin, GPIO_PIN_RESET);
        HAL_SPI_Transmit(&hspi2, &ADC_D2_512_CONV_CMD, CMD_SIZE, CMD_TIMEOUT);
        HAL_GPIO_WritePin(BARO_CS_GPIO_Port, BARO_CS_Pin, GPIO_PIN_SET);

        osDelay(2); // 1.17ms max conversion time for OSR 512

        HAL_GPIO_WritePin(BARO_CS_GPIO_Port, BARO_CS_Pin, GPIO_PIN_RESET);
        HAL_SPI_Transmit(&hspi2, &ADC_READ_CMD, CMD_SIZE, CMD_TIMEOUT);

        temperatureReading = 0;

        HAL_SPI_TransmitReceive(&hspi2, &ADC_READ_CMD, &dataIn, CMD_SIZE, CMD_TIMEOUT);
        temperatureReading += dataIn << 16;

        HAL_SPI_TransmitReceive(&hspi2, &ADC_READ_CMD, &dataIn, CMD_SIZE, CMD_TIMEOUT);
        temperatureReading += dataIn << 8;

        HAL_SPI_TransmitReceive(&hspi2, &ADC_READ_CMD, &dataIn, CMD_SIZE, CMD_TIMEOUT);
        temperatureReading += dataIn;

        HAL_GPIO_WritePin(BARO_CS_GPIO_Port, BARO_CS_Pin, GPIO_PIN_SET);

        // Default values from the datasheet
        // C1_SENS = 46372;
        // C2_OFF = 43981;

        // calcualte 1st order pressure and temperature (MS5607 1st order algorithm)
        double dT = temperatureReading - C5_TREF * pow(2, 8);
        double TEMP = 2000 + dT * C6_TEMPSENS / pow(2, 23);
        double OFF = C2_OFF * pow(2, 17) + dT * C4_TCO / pow(2, 6);
        double SENS = C1_SENS * pow(2, 16) + dT * C3_TCS / pow(2, 7);
        double P = (pressureReading * SENS / pow(2, 21) - OFF) / pow(2, 15) / 100; // divide by 100 to get mbar
        double T = (TEMP) / 100; // divide by 100 to get degrees Celcius

        osMutexWait(data->mutex_, 0);
        data->externalPressure_ = (int32_t) P;
        data->externalTemperature_ = (int32_t) T;
        osMutexRelease(data->mutex_);
    }
}
