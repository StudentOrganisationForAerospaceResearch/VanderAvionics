#include "stm32f4xx.h"
#include "stm32f4xx_hal_conf.h"
#include "cmsis_os.h"

#include "TransmitData.h"

#include "Data.h"

static int TRANSMIT_DATA_PERIOD = 250;

void transmitData(
    int altitude,
    int epochTimeMsec,
    int latitude,
    int longitude,
    float pressure,
    float temperature,
    uint8_t drogueParachuteStatus)
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
        int altitude = data->gpsData_->altitude_;
        int epochTimeMsec = data->gpsData_->epochTimeMsec_;
        int latitude = data->gpsData_->latitude_;
        int longitude = data->gpsData_->longitude_;
        osMutexRelease(data->gpsData_->mutex_);

        osMutexWait(data->oxidizerTankConditionsData_->mutex_, 0);
        float pressure = data->oxidizerTankConditionsData_->pressure_;
        float temperature = data->oxidizerTankConditionsData_->temperature_;
        osMutexRelease(data->oxidizerTankConditionsData_->mutex_);

        transmitData(
            altitude,
            epochTimeMsec,
            latitude,
            longitude,
            pressure,
            temperature,
            drogueParachuteLaunched
        );
    }
}
