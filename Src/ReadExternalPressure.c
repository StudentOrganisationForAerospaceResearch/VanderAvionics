#include "stm32f4xx.h"
#include "stm32f4xx_hal_conf.h"
#include "cmsis_os.h"
#include "math.h"

#include "ReadExternalPressure.h"
#include "Data.h"

static const int READ_EXTERNAL_PRESSURE_PERIOD = 1000;

static const int CMD_SIZE = 1;
static const int CMD_TIMEOUT = 150;
static const uint8_t ADC_D1_512_CONV_CMD = 0x42;
static const uint8_t ADC_D2_512_CONV_CMD = 0x52;
static const uint8_t ADC_READ_CMD = 0x00;
static const uint8_t RESET_CMD = 0x1E;

static const uint8_t PROM_READ_CMD_SENS = 0xA0;
static const uint8_t PROM_READ_CMD_OFF = 0xA2;
static const uint8_t PROM_READ_CMD_TCS = 0xA4;
static const uint8_t PROM_READ_CMD_TCO = 0xA6;
static const uint8_t PROM_READ_CMD_TREF = 0xA8;
static const uint8_t PROM_READ_CMD_TEMPSENS = 0xAA;

static uint8_t dataIn;

void readExternalPressureTask(void const* arg)
{
    ExternalPressureData* data = (ExternalPressureData*) arg;
    uint32_t prevWakeTime = osKernelSysTick();

    HAL_GPIO_WritePin(BARO_CS_GPIO_Port, BARO_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, &RESET_CMD, CMD_SIZE, CMD_TIMEOUT);
    osDelay(3);   // 2.8ms reload after Reset command
    HAL_GPIO_WritePin(BARO_CS_GPIO_Port, BARO_CS_Pin, GPIO_PIN_SET);

    uint16_t SENS = 0;
    uint16_t OFF = 0;
    uint16_t TCS = 0;
    uint16_t TCO = 0;
    uint16_t TREF = 0;
    uint16_t TEMPSENS = 0;

    // PROM read for calibration coefficients
    osDelay(5);
    HAL_GPIO_WritePin(BARO_CS_GPIO_Port, BARO_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, &PROM_READ_CMD_SENS, CMD_SIZE, CMD_TIMEOUT);
    HAL_SPI_TransmitReceive(&hspi2, &ADC_READ_CMD, &dataIn, CMD_SIZE, CMD_TIMEOUT);
    SENS = dataIn << 8;
    HAL_SPI_TransmitReceive(&hspi2, &ADC_READ_CMD, &dataIn, CMD_SIZE, CMD_TIMEOUT);
    SENS += dataIn;
    HAL_GPIO_WritePin(BARO_CS_GPIO_Port, BARO_CS_Pin, GPIO_PIN_SET);

    HAL_GPIO_WritePin(BARO_CS_GPIO_Port, BARO_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, &PROM_READ_CMD_OFF, CMD_SIZE, CMD_TIMEOUT);
    HAL_SPI_TransmitReceive(&hspi2, &ADC_READ_CMD, &dataIn, CMD_SIZE, CMD_TIMEOUT);
    OFF = dataIn << 8;
    HAL_SPI_TransmitReceive(&hspi2, &ADC_READ_CMD, &dataIn, CMD_SIZE, CMD_TIMEOUT);
    OFF += dataIn;
    HAL_GPIO_WritePin(BARO_CS_GPIO_Port, BARO_CS_Pin, GPIO_PIN_SET);

    HAL_GPIO_WritePin(BARO_CS_GPIO_Port, BARO_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, &PROM_READ_CMD_TCS, CMD_SIZE, CMD_TIMEOUT);
    HAL_SPI_TransmitReceive(&hspi2, &ADC_READ_CMD, &dataIn, CMD_SIZE, CMD_TIMEOUT);
    TCS = dataIn << 8;
    HAL_SPI_TransmitReceive(&hspi2, &ADC_READ_CMD, &dataIn, CMD_SIZE, CMD_TIMEOUT);
    TCS += dataIn;
    HAL_GPIO_WritePin(BARO_CS_GPIO_Port, BARO_CS_Pin, GPIO_PIN_SET);

    HAL_GPIO_WritePin(BARO_CS_GPIO_Port, BARO_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, &PROM_READ_CMD_TCO, CMD_SIZE, CMD_TIMEOUT);
    HAL_SPI_TransmitReceive(&hspi2, &ADC_READ_CMD, &dataIn, CMD_SIZE, CMD_TIMEOUT);
    TCO = dataIn << 8;
    HAL_SPI_TransmitReceive(&hspi2, &ADC_READ_CMD, &dataIn, CMD_SIZE, CMD_TIMEOUT);
    TCO += dataIn;
    HAL_GPIO_WritePin(BARO_CS_GPIO_Port, BARO_CS_Pin, GPIO_PIN_SET);

    HAL_GPIO_WritePin(BARO_CS_GPIO_Port, BARO_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, &PROM_READ_CMD_TREF, CMD_SIZE, CMD_TIMEOUT);
    HAL_SPI_TransmitReceive(&hspi2, &ADC_READ_CMD, &dataIn, CMD_SIZE, CMD_TIMEOUT);
    TREF = dataIn << 8;
    HAL_SPI_TransmitReceive(&hspi2, &ADC_READ_CMD, &dataIn, CMD_SIZE, CMD_TIMEOUT);
    TREF += dataIn;
    HAL_GPIO_WritePin(BARO_CS_GPIO_Port, BARO_CS_Pin, GPIO_PIN_SET);

    HAL_GPIO_WritePin(BARO_CS_GPIO_Port, BARO_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, &PROM_READ_CMD_TEMPSENS, CMD_SIZE, CMD_TIMEOUT);
    HAL_SPI_TransmitReceive(&hspi2, &ADC_READ_CMD, &dataIn, CMD_SIZE, CMD_TIMEOUT);
    TEMPSENS = dataIn << 8;
    HAL_SPI_TransmitReceive(&hspi2, &ADC_READ_CMD, &dataIn, CMD_SIZE, CMD_TIMEOUT);
    TEMPSENS += dataIn;

    HAL_GPIO_WritePin(BARO_CS_GPIO_Port, BARO_CS_Pin, GPIO_PIN_SET);

    osDelay(10);   // 2.8ms reload after Reset command

    uint32_t pressureReading;   // Stores a 24 bit value
    uint32_t temperatureReading;   // Stores a 24 bit value

    for (;;)
    {
        osDelayUntil(&prevWakeTime, READ_EXTERNAL_PRESSURE_PERIOD);
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

        osDelay(10);

        // Read D2
        HAL_GPIO_WritePin(BARO_CS_GPIO_Port, BARO_CS_Pin, GPIO_PIN_RESET);
        HAL_SPI_Transmit(&hspi2, &ADC_D2_512_CONV_CMD, CMD_SIZE, CMD_TIMEOUT);
        HAL_GPIO_WritePin(BARO_CS_GPIO_Port, BARO_CS_Pin, GPIO_PIN_SET);

        osDelay(5); // 1.17ms max conversion time for OSR 512

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

        // pressureReading = 6465444;
        // temperatureReading = 8077636;
        //SENS = 46372;
        // OFF = 43981;
        // TCS = 29059;
        // TCO = 27842;
        // TREF = 31553;
        // TEMPSENS = 28165;

        // calcualte 1st order pressure and temperature (MS5607 1st order algorithm)
        double dT = temperatureReading-TREF*pow(2,8);
        double TEMP = 2000+dT*TEMPSENS/pow(2,23);
        double OFFCALC = OFF*pow(2,17)+dT*TCO/pow(2,6);
        double SENSCALC = SENS*pow(2,16)+dT*TCS/pow(2,7);
        double P = (pressureReading*SENSCALC/pow(2,21)-OFFCALC)/pow(2,15);

        osMutexWait(data->mutex_, 0);
        data->externalPressure_ = SENS;
        osMutexRelease(data->mutex_);
    }
}
