#include "stm32f4xx.h"
#include "stm32f4xx_hal_conf.h"
#include "cmsis_os.h"

#include "LogData.h"

#include "Data.h"

static int LOG_DATA_PERIOD = 1000;

int writeToSdCard(const char* entry)
{
    return 0;
}

void logDataTask(void const* arg)
{
    AllData* data = (AllData*) arg;
    uint32_t prevWakeTime = osKernelSysTick();

    for (;;)
    {
        osDelayUntil(&prevWakeTime, LOG_DATA_PERIOD);

        const char* entry = "";
        sprintf(
            entry,
            "%f,%f,%f,%f,%f,%f,%f,%f,%f,%d,%d,%d,%d,%d,%d,%d,%f,%f,",
            data->accelGyroMagnetismData_->accelX_,
            data->accelGyroMagnetismData_->accelY_,
            data->accelGyroMagnetismData_->accelZ_,
            data->accelGyroMagnetismData_->gyroX_,
            data->accelGyroMagnetismData_->gyroY_,
            data->accelGyroMagnetismData_->gyroZ_,
            data->accelGyroMagnetismData_->magnetoX_,
            data->accelGyroMagnetismData_->magnetoY_,
            data->accelGyroMagnetismData_->magnetoZ_,
            data->externalPressureData_->externalPressure_,
            data->externalTemperatureData_->externalTemperature_,
            data->gpsData_->altitude_,
            data->gpsData_->epochTimeMsec_,
            data->gpsData_->latitude_,
            data->gpsData_->longitude_,
            data->integratedTemperatureData_->integratedTemperature_,
            data->oxidizerTankConditionsData_->pressure_,
            data->oxidizerTankConditionsData_->temperature_
        );

        int error = writeToSdCard(entry);
    }
}

