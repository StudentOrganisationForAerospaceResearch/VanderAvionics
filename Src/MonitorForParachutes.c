#include <math.h>

#include "stm32f4xx.h"
#include "stm32f4xx_hal_conf.h"
#include "cmsis_os.h"

#include "MonitorForParachutes.h"
#include "Data.h"
#include "main.h"

static int MONITOR_FOR_PARACHUTES_PERIOD = 1000;

float readAccel(AccelGyroMagnetismData* data)
{
    osMutexWait(data->mutex_, 0);
    float accelX = data->accelX_;
    float accelY = data->accelY_;
    float accelZ = data->accelZ_;
    osMutexRelease(data->mutex_);

    float accelMagnitude =
        sqrt(
            accelX * accelX +
            accelY * accelY +
            accelZ * accelZ
        );

    return accelMagnitude;
}

float readPressure(ExternalPressureData* data)
{
    osMutexWait(data->mutex_, 0);
    int pressure = data->externalPressure_;
    osMutexRelease(data->mutex_);

    return (float)pressure;
}

void filterSensors(float current_accel, float current_pressure, float positionVector[3])
{
    // TODO
    positionVector[0] = 0.f;
    positionVector[1] = 0.f;
    positionVector[2] = 0.f;
}

int detectApogee(float positionVector[3])
{
    // TODO
    return 0;
}

void ejectDrogueParachute()
{
    // TODO
}

void ejectMainParachute()
{
    // TODO
}

void parachutesControlPrelaunchRoutine()
{
    uint32_t prevWakeTime = osKernelSysTick();

    for (;;)
    {
        osDelayUntil(&prevWakeTime, MONITOR_FOR_PARACHUTES_PERIOD);

        if (currentFlightPhase > PRELAUNCH)
        {
            // Ascent has begun
            return;
        }
    }
}

void parachutesControlAscentRoutine(
    AccelGyroMagnetismData* accelGyroMagnetismData,
    ExternalPressureData* externalPressureData
)
{
    uint32_t prevWakeTime = osKernelSysTick();

    for (;;)
    {
        osDelayUntil(&prevWakeTime, MONITOR_FOR_PARACHUTES_PERIOD);

        float currentAccel = readAccel(accelGyroMagnetismData);
        float currentPressure = readPressure(externalPressureData);

        float positionVector[3];
        filterSensors(currentAccel, currentPressure, positionVector);

        if (detectApogee(positionVector))
        {
            ejectDrogueParachute();
            // Begin descent phases
            currentFlightPhase = DROGUE_DESCENT;
            return;
        }
    }
}

void parachutesControlDrogueDescentRoutine()
{
    uint32_t prevWakeTime = osKernelSysTick();

    for (;;)
    {
        osDelayUntil(&prevWakeTime, MONITOR_FOR_PARACHUTES_PERIOD);
        // TODO
        // detect 4600 ft above sea level and eject main parachute
        if (0)
        {

            currentFlightPhase = MAIN_DESCENT;
            ejectMainParachute();
            return;
        }
    }
}

void parachutesControlMainDescentRoutine()
{
    uint32_t prevWakeTime = osKernelSysTick();

    for (;;)
    {
        osDelayUntil(&prevWakeTime, MONITOR_FOR_PARACHUTES_PERIOD);
        // idle
    }
}

void monitorForParachutesTask(void const* arg)
{
    MonitorForParachuteData* data = (MonitorForParachuteData*) arg;

    switch (currentFlightPhase)
    {
        case PRELAUNCH:
            parachutesControlPrelaunchRoutine();
            break;

        case BURN: // fall through
        case COAST:
            parachutesControlAscentRoutine(
                data->accelGyroMagnetismData_,
                data->externalPressureData_
            );
            break;

        case DROGUE_DESCENT:
            parachutesControlDrogueDescentRoutine();
            break;

        case MAIN_DESCENT:
            parachutesControlMainDescentRoutine();
            break;

        default:
            break;
    }
}
