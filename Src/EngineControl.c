#include "stm32f4xx.h"
#include "stm32f4xx_hal_conf.h"
#include "cmsis_os.h"

#include "EngineControl.h"
#include "Data.h"

static const int PRELAUNCH_PHASE_PERIOD = 50;
static const int BURN_DURATION = 10000;
static const int POST_BURN_PERIOD = 10;

static const int MAX_TANK_PRESSURE = 50000;
static const int MAX_DURATION_VENT_VALVE_OPEN = 7000;

void openVentValve()
{
    // TODO
}

void closeVentValve()
{
    // TDOD
}

void openInjectionValve()
{
    // TODO
}

void closeInjectionValve()
{
    // TDOD
}

/**
 * This routine keeps the injection valve closed during prelaunch.
 * This routine exits when the currentFlightPhase is no longer PRELAUNCH.
 */
void engineControlPrelaunchRoutine(OxidizerTankConditionsData* data)
{
    uint32_t prevWakeTime = osKernelSysTick();
    int32_t tankPressure = -1;
    int32_t durationVentValveOpen = 0;

    for (;;)
    {
        osDelayUntil(&prevWakeTime, PRELAUNCH_PHASE_PERIOD);
        // Ensure valve is closed
        closeInjectionValve();

        // Vent tank if over pressure
        if (osMutexWait(data->oxidizerTankConditionsData_->mutex_, 0) == osOK)
        {
            // read tank pressure
            tankPressure = data->oxidizerTankConditionsData_->pressure_;
            osMutexRelease(data->oxidizerTankConditionsData_->mutex_);

            // open or close valve based on tank pressure
            // also do not open valve if it's been open for too long
            // otherwise the vent valve will break
            if (tankPressure > MAX_TANK_PRESSURE &&
                durationVentValveOpen < MAX_DURATION_VENT_VALVE_OPEN) {

                durationVentValveOpen += PRELAUNCH_PHASE_PERIOD;
                openVentValve();
            } else {
                durationVentValveOpen = 0;
                closeVentValve();
            }
        }

        if (currentFlightPhase != PRELAUNCH) {
            return;
        }
    }
}

/**
 * This routine opens the injection valve for the burn phase
 * for a preconfigured amount of time. Once the preconfigured amount
 * of time has passed, this routine updates the currentFlightPhase.
 */
void engineControlBurnRoutine()
{
    openInjectionValve();
    osDelay(BURN_DURATION);
    currentFlightPhase = COAST;
    return;
}

/**
 * This routine keeps the injection valve closed for all phases past the burn phase.
 * The injection valve is closed to avoid overshooting the goal altitude and during descent.
 * This routine is the final phase.
 */
void engineControlPostBurnRoutine()
{
    uint32_t prevWakeTime = osKernelSysTick();

    for (;;)
    {
        osDelayUntil(&prevWakeTime, POST_BURN_PERIOD);
        closeInjectionValve();
    }
}

void engineControlTask(void const* arg)
{
    OxidizerTankConditionsData* data = (OxidizerTankConditionsData*) arg;
    for (;;)
    {
        switch (currentFlightPhase)
        {
            case PRELAUNCH:
                engineControlPrelaunchRoutine(data);
                break;

            case BURN:
                engineControlBurnRoutine();
                break;

            case COAST:  // fall through
            case DROGUE_DESCENT:
            case MAIN_DESCENT:
                engineControlPostBurnRoutine();
                break;

            default:
                break;
        }
    }
}
