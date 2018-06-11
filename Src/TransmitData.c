#include "stm32f4xx.h"
#include "stm32f4xx_hal_conf.h"
#include "cmsis_os.h"

#include "TransmitData.h"

#include "FlightPhase.h"
#include "Data.h"

static const int TRANSMIT_DATA_PERIOD = 500;

static const int8_t IMU_HEADER_BYTE = 0x31;
static const int8_t BAROMETER_HEADER_BYTE = 0x32;
static const int8_t GPS_HEADER_BYTE = 0x33;
static const int8_t OXIDIZER_TANK_HEADER_BYTE = 0x34;
static const int8_t COMBUSTION_CHAMBER_HEADER_BYTE = 0x35;
static const uint8_t FLIGHT_PHASE_HEADER_BYTE = 0x36;
static const uint8_t VENT_VALVE_STATUS_HEADER_BYTE = 0x37;

static const uint8_t UART_TIMEOUT = 100;
static const uint32_t MASK_32to24 = 0xff000000;
static const uint32_t MASK_24to16 = 0x00ff0000;
static const uint32_t MASK_16to8 = 0x0000ff00;
static const uint32_t MASK_8to0 = 0x000000ff;

void transmitImuData(AllData* data)
{
    int32_t accelX = -1;
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

    int8_t buffer [] = {IMU_HEADER_BYTE,
                        (int8_t) (accelX & MASK_8to0), (int8_t) ((accelX & MASK_16to8) >> 8), (int8_t) ((accelX & MASK_24to16) >> 16), (int8_t) ((accelX & MASK_32to24) >> 24),
                        (int8_t) (accelY & MASK_8to0), (int8_t) ((accelY & MASK_16to8) >> 8), (int8_t) ((accelY & MASK_24to16) >> 16), (int8_t) ((accelY & MASK_32to24) >> 24),
                        (int8_t) (accelZ & MASK_8to0), (int8_t) ((accelZ & MASK_16to8) >> 8), (int8_t) ((accelZ & MASK_24to16) >> 16), (int8_t) ((accelZ & MASK_32to24) >> 24),
                        (int8_t) (gyroX & MASK_8to0), (int8_t) ((gyroX & MASK_16to8) >> 8), (int8_t) ((gyroX & MASK_24to16) >> 16), (int8_t) ((gyroX & MASK_32to24) >> 24),
                        (int8_t) (gyroY & MASK_8to0), (int8_t) ((gyroY & MASK_16to8) >> 8), (int8_t) ((gyroY & MASK_24to16) >> 16), (int8_t) ((gyroY & MASK_32to24) >> 24),
                        (int8_t) (gyroZ & MASK_8to0), (int8_t) ((gyroZ & MASK_16to8) >> 8), (int8_t) ((gyroZ & MASK_24to16) >> 16), (int8_t) ((gyroZ & MASK_32to24) >> 24),
                        (int8_t) (magnetoX & MASK_8to0), (int8_t) ((magnetoX & MASK_16to8) >> 8), (int8_t) ((magnetoX & MASK_24to16) >> 16), (int8_t) ((magnetoX & MASK_32to24) >> 24),
                        (int8_t) (magnetoY & MASK_8to0), (int8_t) ((magnetoY & MASK_16to8) >> 8), (int8_t) ((magnetoY & MASK_24to16) >> 16), (int8_t) ((magnetoY & MASK_32to24) >> 24),
                        (int8_t) (magnetoZ & MASK_8to0), (int8_t) ((magnetoZ & MASK_16to8) >> 8), (int8_t) ((magnetoZ & MASK_24to16) >> 16), (int8_t) ((magnetoZ & MASK_32to24) >> 24),
                        0X00
                        };

    if( (getCurrentFlightPhase() == PRELAUNCH) || (getCurrentFlightPhase() == ABORT) ) // Add RESET phase here too
    {
        HAL_UART_Transmit(&huart2, &buffer, sizeof(buffer), UART_TIMEOUT);  // Launch Systems
    }
    HAL_UART_Transmit(&huart1, &buffer, sizeof(buffer), UART_TIMEOUT);	// Radio
}

void transmitBarometerData(AllData* data)
{
    int32_t pressure = -1;
    int32_t temperature = -1;

    if (osMutexWait(data->barometerData_->mutex_, 0) == osOK)
    {
        pressure = data->barometerData_->pressure_;
        temperature = data->barometerData_->temperature_;
        osMutexRelease(data->barometerData_->mutex_);
    }

    int8_t buffer [] = {BAROMETER_HEADER_BYTE,
                        (int8_t) (pressure & MASK_8to0), (int8_t) ((pressure & MASK_16to8) >> 8), (int8_t) ((pressure & MASK_24to16) >> 16), (int8_t) ((pressure & MASK_32to24) >> 24),
                        (int8_t) (temperature & MASK_8to0), (int8_t) ((temperature & MASK_16to8) >> 8), (int8_t) ((temperature & MASK_24to16) >> 16), (int8_t) ((temperature & MASK_32to24) >> 24),
                        0X00
                        };
    if( (getCurrentFlightPhase() == PRELAUNCH) || (getCurrentFlightPhase() == ABORT) ) // Add RESET phase here too
    {
        HAL_UART_Transmit(&huart2, &buffer, sizeof(buffer), UART_TIMEOUT);	// Launch Systems
    }
    HAL_UART_Transmit(&huart1, &buffer, sizeof(buffer), UART_TIMEOUT);	// Radio
}

void transmitGpsData(AllData* data)
{
    int32_t altitude = -1;
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

    int8_t buffer [] = {GPS_HEADER_BYTE,
                        (int8_t) (altitude & MASK_8to0), (int8_t) ((altitude & MASK_16to8) >> 8), (int8_t) ((altitude & MASK_24to16) >> 16), (int8_t) ((altitude & MASK_32to24) >> 24),
                        (int8_t) (epochTimeMsec & MASK_8to0), (int8_t) ((epochTimeMsec & MASK_16to8) >> 8), (int8_t) ((epochTimeMsec & MASK_24to16) >> 16), (int8_t) ((epochTimeMsec & MASK_32to24) >> 24),
                        (int8_t) (latitude & MASK_8to0), (int8_t) ((latitude & MASK_16to8) >> 8), (int8_t) ((latitude & MASK_24to16) >> 16), (int8_t) ((latitude & MASK_32to24) >> 24),
                        (int8_t) (longitude & MASK_8to0), (int8_t) ((longitude & MASK_16to8) >> 8), (int8_t) ((longitude & MASK_24to16) >> 16), (int8_t) ((longitude & MASK_32to24) >> 24),
                        0x00
                        };

    if( (getCurrentFlightPhase() == PRELAUNCH) || (getCurrentFlightPhase() == ABORT) ) // Add RESET phase here too
    {
        HAL_UART_Transmit(&huart2, &buffer, sizeof(buffer), UART_TIMEOUT); // Launch Systems
    }
    HAL_UART_Transmit(&huart1, &buffer, sizeof(buffer), UART_TIMEOUT);	// Radio
}

void transmitOxidizerTankData(AllData* data)
{
    int32_t oxidizerTankPressure = -1;

    if (osMutexWait(data->oxidizerTankPressureData_->mutex_, 0) == osOK)
    {
        oxidizerTankPressure = data->oxidizerTankPressureData_->pressure_;
        osMutexRelease(data->oxidizerTankPressureData_->mutex_);
    }

    int8_t buffer [] = {OXIDIZER_TANK_HEADER_BYTE,
                        (int8_t) (oxidizerTankPressure & MASK_8to0), (int8_t) ((oxidizerTankPressure & MASK_16to8) >> 8), (int8_t) ((oxidizerTankPressure & MASK_24to16) >> 16), (int8_t) ((oxidizerTankPressure & MASK_32to24) >> 24),
                        0x00
                        };

    if( (getCurrentFlightPhase() == PRELAUNCH) || (getCurrentFlightPhase() == ABORT) ) // Add RESET phase here too
    {
        HAL_UART_Transmit(&huart2, &buffer, sizeof(buffer), UART_TIMEOUT);  // Launch Systems
    }
    HAL_UART_Transmit(&huart1, &buffer, sizeof(buffer), UART_TIMEOUT);	// Radio
}

void transmitCombustionChamberData(AllData* data)
{
    int32_t combustionChamberPressure = -1;

    if (osMutexWait(data->combustionChamberPressureData_->mutex_, 0) == osOK)
    {
        combustionChamberPressure = data->combustionChamberPressureData_->pressure_;
        osMutexRelease(data->combustionChamberPressureData_->mutex_);
    }

    int8_t buffer [] = {COMBUSTION_CHAMBER_HEADER_BYTE,
                        (int8_t) (combustionChamberPressure & MASK_8to0), (int8_t) ((combustionChamberPressure & MASK_16to8) >> 8), (int8_t) ((combustionChamberPressure & MASK_24to16) >> 16), (int8_t) ((combustionChamberPressure & MASK_32to24) >> 24),
                        0x00
                        };
    if(getCurrentFlightPhase() == PRELAUNCH)
    {
        HAL_UART_Transmit(&huart2, &buffer, sizeof(buffer), UART_TIMEOUT);  // Launch Systems
    }
    HAL_UART_Transmit(&huart1, &buffer, sizeof(buffer), UART_TIMEOUT);	// Radio
}

void transmitFlightPhaseData(AllData* data)
{
    uint8_t flightPhase = getCurrentFlightPhase();

    uint8_t buffer [] = {FLIGHT_PHASE_HEADER_BYTE,
                        flightPhase,
                        0x00
                        };

    if( (getCurrentFlightPhase() == PRELAUNCH) || (getCurrentFlightPhase() == ABORT) ) // Add RESET phase here too
    {
        HAL_UART_Transmit(&huart2, &buffer, sizeof(buffer), UART_TIMEOUT);  // Launch Systems
    }
    HAL_UART_Transmit(&huart1, &buffer, sizeof(buffer), UART_TIMEOUT);	// Radio
}

void transmitVentValveStatus()
{
    uint8_t ventValveStatus = ventValveIsOpen;

    uint8_t buffer [] = {VENT_VALVE_STATUS_HEADER_BYTE,
                        (uint8_t) ((ventValveStatus)),
                        0x00
                        };

    if( (getCurrentFlightPhase() == PRELAUNCH) || (getCurrentFlightPhase() == ABORT) ) // Add RESET phase here too
    {
        HAL_UART_Transmit(&huart2, &buffer, sizeof(buffer), UART_TIMEOUT); // Launch Systems
    }
    HAL_UART_Transmit(&huart1, &buffer, sizeof(buffer), UART_TIMEOUT);  // Radio
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
        transmitVentValveStatus();
    }
}