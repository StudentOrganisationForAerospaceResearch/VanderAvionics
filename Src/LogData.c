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
        osMutexWait(accelGyroMagnetismDataMutex, 0);
        float accelX = data->accelGyroMagnetismData_->accelX_;
        float accelY = data->accelGyroMagnetismData_->accelY_;
        float accelZ = data->accelGyroMagnetismData_->accelZ_;
        float gyroX = data->accelGyroMagnetismData_->gyroX_;
        float gyroY = data->accelGyroMagnetismData_->gyroY_;
        float gyroZ = data->accelGyroMagnetismData_->gyroZ_;
        float magnetoX = data->accelGyroMagnetismData_->magnetoX_;
        float magnetoY = data->accelGyroMagnetismData_->magnetoY_;
        float magnetoZ = data->accelGyroMagnetismData_->magnetoZ_;
        osMutexRelease(accelGyroMagnetismDataMutex);

        osMutexWait(externalPressureDataMutex, 0);
        int externalPressure = data->externalPressureData_->externalPressure_;
        osMutexRelease(externalPressureDataMutex);

        osMutexWait(externalTemperatureDataMutex, 0);
        int externalTemperature = data->externalTemperatureData_->externalTemperature_;
        osMutexRelease(externalTemperatureDataMutex);

        osMutexWait(integratedTemperatureDataMutex, 0);
        int integratedTemperature = data->integratedTemperatureData_->integratedTemperature_;
        osMutexRelease(integratedTemperatureDataMutex);

        osMutexWait(gpsDataMutex, 0);
        int altitude = data->gpsData_->altitude_;
        int epochTimeMsec = data->gpsData_->epochTimeMsec_;
        int latitude = data->gpsData_->latitude_;
        int longitude = data->gpsData_->longitude_;
        osMutexRelease(gpsDataMutex);

        osMutexWait(oxidizerTankConditionsDataMutex, 0);
        float pressure = data->oxidizerTankConditionsData_->pressure_;
        float temperature = data->oxidizerTankConditionsData_->temperature_;
        osMutexRelease(oxidizerTankConditionsDataMutex);

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

