#include "stm32f4xx.h"
#include "stm32f4xx_hal_conf.h"
#include "cmsis_os.h"

#include "EngineControl.h"
#include "FlightPhase.h"
#include "Data.h"

static const int PRELAUNCH_PHASE_PERIOD = 50;
static const int BURN_DURATION = 10000;
static const int POST_BURN_PERIOD = 10;

static const int MAX_TANK_PRESSURE_KPA = 5660; // 820psi, 25 deg C at saturation
static const int MAX_DURATION_VENT_VALVE_OPEN = 8000;
static const int REQUIRED_DURATION_VENT_VALVE_CLOSED = 4000;

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
 * This routine exits when the current flight phase is no longer PRELAUNCH.
 */
void engineControlPrelaunchRoutine(OxidizerTankConditionsData* data)
{
    uint32_t prevWakeTime = osKernelSysTick();
    int32_t tankPressure = -1;
    int32_t durationVentValveControlled = 0;

    for (;;)
    {
        osDelayUntil(&prevWakeTime, PRELAUNCH_PHASE_PERIOD);
        // Ensure valve is closed
        closeInjectionValve();

        // Vent tank if over pressure
        if (osMutexWait(data->mutex_, 0) == osOK)
        {
            // read tank pressure
            tankPressure = data->pressure_;
            osMutexRelease(data->mutex_);

            // open or close valve based on tank pressure
            // also do not open valve if it's been open for too long
            // otherwise the vent valve will break
            if (tankPressure > MAX_TANK_PRESSURE_KPA)
            {
                if (durationVentValveControlled < MAX_DURATION_VENT_VALVE_OPEN)
                {
                    // open vent valve
                    durationVentValveControlled += PRELAUNCH_PHASE_PERIOD;
                    openVentValve();
                }
                else if (durationVentValveControlled <
                         (MAX_DURATION_VENT_VALVE_OPEN + REQUIRED_DURATION_VENT_VALVE_CLOSED))
                {
                    // vent valve has been open for more than max time it can be open
                    durationVentValveControlled += PRELAUNCH_PHASE_PERIOD;
                    closeVentValve();
                }
                else
                {
                    // vent valve has closed to reset itself as long as is necessary
                    openVentValve();
                    durationVentValveControlled = 0;
                }
            }
        }

        if (getCurrentFlightPhase() != PRELAUNCH)
        {
            return;
        }
    }
}

/**
 * This routine opens the injection valve for the burn phase
 * for a preconfigured amount of time. Once the preconfigured amount
 * of time has passed, this routine updates the current flight phase.
 */
void engineControlBurnRoutine()
{
    openInjectionValve();
    osDelay(BURN_DURATION);
    newFlightPhase(COAST);
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
        switch (getCurrentFlightPhase())
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
