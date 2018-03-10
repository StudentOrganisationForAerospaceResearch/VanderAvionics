#include "stm32f4xx.h"
#include "stm32f4xx_hal_conf.h"
#include "cmsis_os.h"

#include "defines.h"
#include "tm_stm32_delay.h"
#include "tm_stm32_fatfs.h"

#include "LogData.h"
#include "Data.h"
#include "main.h"

static int LOG_DATA_PERIOD = 1000;

FATFS fatfs;
FIL file;

int writeToSdCard(const char* entry)
{
    if (f_mount(&fatfs, "SD:", 1) == FR_OK)
    {
        HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 1);

        if (f_open(&file, "SD:VanderAvionics.log", FA_OPEN_APPEND | FA_WRITE) == FR_OK)
        {
            f_puts(entry, &file);
            f_close(&file);
        }
        else
        {
            return 2;
        }

        f_mount(NULL, "SD:", 1);
        HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 0);
    }
    else
    {
        return 1;
    }

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
        osMutexWait(*(data->accelGyroMagnetismData_->mutex_), 0);
        float accelX = data->accelGyroMagnetismData_->accelX_;
        float accelY = data->accelGyroMagnetismData_->accelY_;
        float accelZ = data->accelGyroMagnetismData_->accelZ_;
        float gyroX = data->accelGyroMagnetismData_->gyroX_;
        float gyroY = data->accelGyroMagnetismData_->gyroY_;
        float gyroZ = data->accelGyroMagnetismData_->gyroZ_;
        float magnetoX = data->accelGyroMagnetismData_->magnetoX_;
        float magnetoY = data->accelGyroMagnetismData_->magnetoY_;
        float magnetoZ = data->accelGyroMagnetismData_->magnetoZ_;
        osMutexRelease(*(data->accelGyroMagnetismData_->mutex_));

        osMutexWait(*(data->externalPressureData_->mutex_), 0);
        int externalPressure = data->externalPressureData_->externalPressure_;
        osMutexRelease(*(data->externalPressureData_->mutex_));

        osMutexWait(*(data->externalTemperatureData_->mutex_), 0);
        int externalTemperature = data->externalTemperatureData_->externalTemperature_;
        osMutexRelease(*(data->externalTemperatureData_->mutex_));

        osMutexWait(*(data->integratedTemperatureData_->mutex_), 0);
        int integratedTemperature = data->integratedTemperatureData_->integratedTemperature_;
        osMutexRelease(*(data->integratedTemperatureData_->mutex_));

        osMutexWait(*(data->gpsData_->mutex_), 0);
        int altitude = data->gpsData_->altitude_;
        int epochTimeMsec = data->gpsData_->epochTimeMsec_;
        int latitude = data->gpsData_->latitude_;
        int longitude = data->gpsData_->longitude_;
        osMutexRelease(*(data->gpsData_->mutex_));

        osMutexWait(*(data->oxidizerTankConditionsData_->mutex_), 0);
        float pressure = data->oxidizerTankConditionsData_->pressure_;
        float temperature = data->oxidizerTankConditionsData_->temperature_;
        osMutexRelease(*(data->oxidizerTankConditionsData_->mutex_));

        sprintf(
            entry,
            "%f,%f,%f,%f,%f,%f,%f,%f,%f,%d,%d,%d,%d,%d,%d,%d,%f,%f,",
            accelX,
            accelY,
            accelZ,
            gyroX,
            gyroY,
            gyroZ,
            magnetoX,
            magnetoY,
            magnetoZ,
            externalPressure,
            externalTemperature,
            integratedTemperature,
            altitude,
            epochTimeMsec,
            latitude,
            longitude,
            pressure,
            temperature
        );

        int error = writeToSdCard(entry);
    }
}

