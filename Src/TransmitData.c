#include "stm32f4xx.h"
#include "stm32f4xx_hal_conf.h"
#include "cmsis_os.h"

#include "TransmitData.h"

#include "Data.h"

static int TRANSMIT_DATA_PERIOD = 250;

void transmitData(
    int32_t altitude,
    int32_t epochTimeMsec,
    int32_t latitude,
    int32_t longitude,
    int32_t pressure,
    int32_t temperature,
    FlightPhase phase)
{
    // TODO
    // Send data via radio
}

void transmitDataTask(void const* arg)
{
    AllData* data = (AllData*) arg;
    uint32_t prevWakeTime = osKernelSysTick();

    for (;;)
    {
        osDelayUntil(&prevWakeTime, TRANSMIT_DATA_PERIOD);

        osMutexWait(data->gpsData_->mutex_, 0);
        int32_t altitude = data->gpsData_->altitude_;
        int32_t epochTimeMsec = data->gpsData_->epochTimeMsec_;
        int32_t latitude = data->gpsData_->latitude_;
        int32_t longitude = data->gpsData_->longitude_;
        osMutexRelease(data->gpsData_->mutex_);

        osMutexWait(data->oxidizerTankConditionsData_->mutex_, 0);
        int32_t pressure = data->oxidizerTankConditionsData_->pressure_;
        int32_t temperature = data->oxidizerTankConditionsData_->temperature_;
        osMutexRelease(data->oxidizerTankConditionsData_->mutex_);

        transmitData(
            altitude,
            epochTimeMsec,
            latitude,
            longitude,
            pressure,
            temperature,
            currentFlightPhase
        );
    }
}
