#include "stm32f4xx.h"
#include "stm32f4xx_hal_conf.h"
#include "cmsis_os.h"

#include "TransmitData.h"

#include "FlightPhase.h"
#include "Data.h"

static const int TRANSMIT_DATA_PERIOD = 250;

static const uint8_t IMU_HEADER_BYTE = 0x31;
static const uint8_t BAROMETER_HEADER_BYTE = 0x32;
static const uint8_t GPS_HEADER_BYTE = 0x33;
static const uint8_t OXIDIZER_TANK_HEADER_BYTE = 0x34;
static const uint8_t COMBUSTION_CHAMBER_HEADER_BYTE = 0x35;
static const uint8_t FLIGHT_PHASE_HEADER_BYTE = 0x36;

static const uint8_t UART_TIMEOUT = 100;

void transmitImuData(AllData* data)
{
    int32_t accelX = -1;
    // TODO send data
    int32_t accelY = -1;
    int32_t accelZ = -1;
    int32_t gyroX = -1;
    int32_t gyroY = -1;
    int32_t gyroZ = -1;
    int32_t magnetoX = -1;
    int32_t magnetoY = -1;
    int32_t magnetoZ = -1;

    if (osMutexWait(data->accelGyroMagnetismData_->mutex_, 0) == osOK)
    {
        accelX = data->accelGyroMagnetismData_->accelX_;
        accelY = data->accelGyroMagnetismData_->accelY_;
        accelZ = data->accelGyroMagnetismData_->accelZ_;
        gyroX = data->accelGyroMagnetismData_->gyroX_;
        gyroY = data->accelGyroMagnetismData_->gyroY_;
        gyroZ = data->accelGyroMagnetismData_->gyroZ_;
        magnetoX = data->accelGyroMagnetismData_->magnetoX_;
        magnetoY = data->accelGyroMagnetismData_->magnetoY_;
        magnetoZ = data->accelGyroMagnetismData_->magnetoZ_;
        osMutexRelease(data->accelGyroMagnetismData_->mutex_);
    }
}

void transmitBarometerData(AllData* data)
{
    int32_t pressure = -1;
    // TODO send data
    int32_t temperature = -1;

    if (osMutexWait(data->barometerData_->mutex_, 0) == osOK)
    {
        pressure = data->barometerData_->pressure_;
        temperature = data->barometerData_->temperature_;
        osMutexRelease(data->barometerData_->mutex_);
    }

    uint8_t buffer [] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09};
    HAL_UART_Transmit(&huart1, &buffer, sizeof(buffer), UART_TIMEOUT);
}

void transmitGpsData(AllData* data)
{
    int32_t altitude = -1;
    // TODO send data
    int32_t epochTimeMsec = -1;
    int32_t latitude = -1;
    int32_t longitude = -1;

    if (osMutexWait(data->gpsData_->mutex_, 0) == osOK)
    {
        altitude = data->gpsData_->altitude_;
        epochTimeMsec = data->gpsData_->epochTimeMsec_;
        latitude = data->gpsData_->latitude_;
        longitude = data->gpsData_->longitude_;
        osMutexRelease(data->gpsData_->mutex_);
    }
}

void transmitOxidizerTankData(AllData* data)
{
    int32_t oxidizerTankPressure = -1;
    // TODO send data


    if (osMutexWait(data->oxidizerTankPressureData_->mutex_, 0) == osOK)
    {
        oxidizerTankPressure = data->oxidizerTankPressureData_->pressure_;
        osMutexRelease(data->oxidizerTankPressureData_->mutex_);
    }
}

void transmitCombustionChamberData(AllData* data)
{
    int32_t combustionChamberPressure = -1;
    // TODO send data

    if (osMutexWait(data->combustionChamberPressureData_->mutex_, 0) == osOK)
    {
        combustionChamberPressure = data->combustionChamberPressureData_->pressure_;
        osMutexRelease(data->combustionChamberPressureData_->mutex_);
    }
}

void transmitFlightPhaseData(AllData* data)
{
    uint8_t flightPhase = getCurrentFlightPhase();
    // TODO send data
}

void transmitDataTask(void const* arg)
{
    AllData* data = (AllData*) arg;
    uint32_t prevWakeTime = osKernelSysTick();

    for (;;)
    {
        osDelayUntil(&prevWakeTime, TRANSMIT_DATA_PERIOD);

        transmitImuData(data);
        transmitBarometerData(data);
        transmitGpsData(data);
        transmitOxidizerTankData(data);
        transmitCombustionChamberData(data);
        transmitFlightPhaseData(data);
    }
}
