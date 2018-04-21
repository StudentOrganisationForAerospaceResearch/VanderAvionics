#include "stm32f4xx.h"
#include "stm32f4xx_hal_conf.h"
#include "cmsis_os.h"

#include "defines.h"
#include "tm_stm32_delay.h"
#include "tm_stm32_fatfs.h"

#include "LogData.h"
#include "Data.h"
#include "main.h"

static int SLOW_LOG_DATA_PERIOD = 1000;
static int FAST_LOG_DATA_PERIOD = 20;

static FATFS fatfs;
static FIL file;

void buildLogEntry(AllData* data, char* buffer)
{
    osMutexWait(data->accelGyroMagnetismData_->mutex_, 0);
    int32_t accelX = data->accelGyroMagnetismData_->accelX_;
    int32_t accelY = data->accelGyroMagnetismData_->accelY_;
    int32_t accelZ = data->accelGyroMagnetismData_->accelZ_;
    int32_t gyroX = data->accelGyroMagnetismData_->gyroX_;
    int32_t gyroY = data->accelGyroMagnetismData_->gyroY_;
    int32_t gyroZ = data->accelGyroMagnetismData_->gyroZ_;
    int32_t magnetoX = data->accelGyroMagnetismData_->magnetoX_;
    int32_t magnetoY = data->accelGyroMagnetismData_->magnetoY_;
    int32_t magnetoZ = data->accelGyroMagnetismData_->magnetoZ_;
    osMutexRelease(data->accelGyroMagnetismData_->mutex_);

    osMutexWait(data->barometerData_->mutex_, 0);
    int32_t pressure = data->barometerData_->pressure_;
    int32_t temperature = data->barometerData_->temperature_;
    osMutexRelease(data->barometerData_->mutex_);

    osMutexWait(data->gpsData_->mutex_, 0);
    int32_t altitude = data->gpsData_->altitude_;
    int32_t epochTimeMsec = data->gpsData_->epochTimeMsec_;
    int32_t latitude = data->gpsData_->latitude_;
    int32_t longitude = data->gpsData_->longitude_;
    osMutexRelease(data->gpsData_->mutex_);

    osMutexWait(data->oxidizerTankConditionsData_->mutex_, 0);
    int32_t tankPressure = data->oxidizerTankConditionsData_->pressure_;
    int32_t tankTemperature = data->oxidizerTankConditionsData_->temperature_;
    osMutexRelease(data->oxidizerTankConditionsData_->mutex_);

    sprintf(
        buffer,
        "%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%d\n",
        accelX,
        accelY,
        accelZ,
        gyroX,
        gyroY,
        gyroZ,
        magnetoX,
        magnetoY,
        magnetoZ,
        pressure,
        temperature,
        altitude,
        epochTimeMsec,
        latitude,
        longitude,
        tankPressure,
        tankTemperature,
        currentFlightPhase
    );
}

void lowFrequencyLogToSdRoutine(AllData* data, char* buffer, FlightPhase entryPhase)
{
    uint32_t prevWakeTime = osKernelSysTick();

    for (;;)
    {
        osDelayUntil(&prevWakeTime, SLOW_LOG_DATA_PERIOD);

        if (currentFlightPhase != entryPhase)
        {
            // New phase has started, exit low frequency logging
            return;
        }

        buildLogEntry(data, buffer);

        if (f_mount(&fatfs, "SD:", 1) == FR_OK)
        {
            HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 1);

            if (f_open(&file, "SD:VanderAvionics.log", FA_OPEN_APPEND | FA_READ | FA_WRITE) == FR_OK)
            {
                f_puts(buffer, &file);
                f_close(&file);
            }

            f_mount(NULL, "SD:", 1);
            HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 0);
        }
    }
}

void highFrequencyLogToSdRoutine(AllData* data, char* buffer)
{
    // Get card mounted
    uint8_t mounted = 0;

    while (!mounted)
    {
        // Keep trying, really need to log during this time
        mounted = (f_mount(&fatfs, "SD:", 1) == FR_OK);

        if ( currentFlightPhase != BURN &&
                currentFlightPhase != COAST &&
                currentFlightPhase != DROGUE_DESCENT)
        {
            // couldn't mount during important phases, too bad :(
            return;
        }
    }

    // Card mounted, start writing at high frequency
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 1);
    uint32_t prevWakeTime = osKernelSysTick();

    for (;;)
    {
        osDelayUntil(&prevWakeTime, FAST_LOG_DATA_PERIOD);

        if ( currentFlightPhase != BURN &&
                currentFlightPhase != COAST &&
                currentFlightPhase != DROGUE_DESCENT)
        {
            // done important phases, unmont card and exit high frequency logging
            break;
        }

        buildLogEntry(data, buffer);

        if (f_open(&file, "SD:VanderAvionics.log", FA_OPEN_APPEND | FA_READ | FA_WRITE) == FR_OK)
        {
            f_puts(buffer, &file);
            f_close(&file); // close to save the file
        }
    }

    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 0);
    f_mount(NULL, "SD:", 1);
}

void logDataTask(void const* arg)
{
    AllData* data = (AllData*) arg;
    char buffer[256];

    sprintf(
        buffer,
        "accelX,"
        "accelY,"
        "accelZ,"
        "gyroX,"
        "gyroY,"
        "gyroZ,"
        "magnetoX,"
        "magnetoY,"
        "magnetoZ,"
        "pressure,"
        "temperature,"
        "altitude,"
        "epochTimeMsec,"
        "latitude,"
        "longitude,"
        "tankPressure,"
        "tankTemperature,"
        "currentFlightPhase\n"
    );

    if (f_mount(&fatfs, "SD:", 1) == FR_OK)
    {
        HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 1);

        if (f_open(&file, "SD:VanderAvionics.log", FA_OPEN_APPEND | FA_READ | FA_WRITE) == FR_OK)
        {
            f_puts(buffer, &file);
            f_close(&file);
        }

        f_mount(NULL, "SD:", 1);
        HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 0);
    }

    for (;;)
    {
        switch (currentFlightPhase)
        {
            case PRELAUNCH:
                lowFrequencyLogToSdRoutine(data, buffer, PRELAUNCH);
                break;

            case BURN:
            case COAST:
            case DROGUE_DESCENT:
                highFrequencyLogToSdRoutine(data, buffer);
                break;

            case MAIN_DESCENT:
            default:
                lowFrequencyLogToSdRoutine(data, buffer, MAIN_DESCENT);
                break;
        }
    }
}
