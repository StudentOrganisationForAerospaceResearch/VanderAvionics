#include <math.h>

#include "stm32f4xx.h"
#include "stm32f4xx_hal_conf.h"
#include "cmsis_os.h"

#include "ParachutesControl.h"
#include "Data.h"
#include "main.h"

static int MONITOR_FOR_PARACHUTES_PERIOD = 1000;

int32_t readAccel(AccelGyroMagnetismData* data)
{
    osMutexWait(data->mutex_, 0);
    int32_t accelX = data->accelX_;
    int32_t accelY = data->accelY_;
    int32_t accelZ = data->accelZ_;
    osMutexRelease(data->mutex_);

    int32_t accelMagnitude =
        sqrt(
            accelX * accelX +
            accelY * accelY +
            accelZ * accelZ
        );

    return accelMagnitude;
}

int32_t readPressure(ExternalPressureData* data)
{
    osMutexWait(data->mutex_, 0);
    int pressure = data->externalPressure_;
    osMutexRelease(data->mutex_);

    return (int32_t)pressure;
}

void filterSensors(int32_t current_accel, int32_t current_pressure, int32_t positionVector[3])
{
    // TODO
    positionVector[0] = 0.f;
    positionVector[1] = 0.f;
    positionVector[2] = 0.f;
}

int detectApogee(int32_t positionVector[3])
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

/**
 * This routine just waits for the currentFlightPhase to get out of PRELAUNCH
 */
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


/**
 * This routine monitors for apogee.
 * Once apogee has been detected,
 * eject the drogue parachute and update the currentFlightPhase.
 */
void parachutesControlAscentRoutine(
    AccelGyroMagnetismData* accelGyroMagnetismData,
    ExternalPressureData* externalPressureData
)
{
    uint32_t prevWakeTime = osKernelSysTick();
    int32_t positionVector[3];

    for (;;)
    {
        osDelayUntil(&prevWakeTime, MONITOR_FOR_PARACHUTES_PERIOD);

        int32_t currentAccel = readAccel(accelGyroMagnetismData);
        int32_t currentPressure = readPressure(externalPressureData);

        filterSensors(currentAccel, currentPressure, positionVector);

        if (detectApogee(positionVector))
        {
            ejectDrogueParachute();
            currentFlightPhase = DROGUE_DESCENT;
            return;
        }
    }
}

/**
 * This routine detects reaching a certain altitude
 * Once that altitude has been reached, eject the main parachute
 * and update the currentFlightPhase.
 */
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

            ejectMainParachute();
            currentFlightPhase = MAIN_DESCENT;
            return;
        }
    }
}

/**
 * This routine does nothing since there is nothing left to do
 * after the main parachute has been launched.
 */
void parachutesControlMainDescentRoutine()
{
    for (;;)
    {
        osThreadSuspend(osThreadGetId());
    }
}

void parachutesControlTask(void const* arg)
{
    ParachutesControlData* data = (ParachutesControlData*) arg;

    for (;;)
    {
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
}
