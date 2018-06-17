#include "stm32f4xx.h"
#include "stm32f4xx_hal_conf.h"
#include "cmsis_os.h"

#include "EngineControl.h"
#include "FlightPhase.h"
#include "Data.h"
#include "ValveControl.h"

static const int PRELAUNCH_PHASE_PERIOD = 50;
static const int BURN_DURATION = 11000;
static const int POST_BURN_PERIOD = 1000;

static const int POST_BURN_REOPEN_INJECTION_VALVE_DURATION = 20 * 60 * 1000; // 20 minutes
static const int MAX_TANK_PRESSURE = 820000; // 820 psi, 5660 kPa, 25 deg C at saturation

/**
 * This routine keeps the injection valve closed during prelaunch.
 * This routine exits when the current flight phase is no longer PRELAUNCH.
 */
void engineControlPrelaunchRoutine(OxidizerTankPressureData* data)
{
    uint32_t prevWakeTime = osKernelSysTick();
    int32_t tankPressure = -1;

    // If not 0, a vent pulse is in progress
    // greater than REQUIRED_DURATION_VENT_VALVE_CLOSED is open period
    // 0 to REQUIRED_DURATION_VENT_VALVE_CLOSED is closed period
    int32_t ventPulseCounter = 0;
    closeInjectionValve();

    for (;;)
    {
        osDelayUntil(&prevWakeTime, PRELAUNCH_PHASE_PERIOD);

        // Vent tank if over pressure
        if (ventPulseCounter == 0) // vent pulse is not active, check for over pressure
        {
            if (osMutexWait(data->mutex_, 0) == osOK)
            {
                // read tank pressure
                tankPressure = data->pressure_;
                osMutexRelease(data->mutex_);
            }

            if (tankPressure > MAX_TANK_PRESSURE)
            {
                ventPulseCounter =
                    REQUIRED_DURATION_VENT_VALVE_CLOSED + MAX_DURATION_VENT_VALVE_OPEN;
            }
        }

        // do vent pulse routine
        if (ventPulseCounter)
        {
            if (ventPulseCounter > REQUIRED_DURATION_VENT_VALVE_CLOSED)
            {
                openVentValve();
            }
            else
            {
                closeVentValve();
            }

            ventPulseCounter -= PRELAUNCH_PHASE_PERIOD;
        }

        if (launchCmdReceived != 0)
        {
            newFlightPhase(BURN);
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
    closeVentValve();
    openInjectionValve();
    osDelay(BURN_DURATION);
    newFlightPhase(COAST);
    return;
}

/**
 * This routine is the final phase.
 */
void engineControlPostBurnRoutine()
{
    uint32_t prevWakeTime = osKernelSysTick();
    uint32_t timeInPostBurn = 0;

    for (;;)
    {
        osDelayUntil(&prevWakeTime, POST_BURN_PERIOD);
        FlightPhase phase = getCurrentFlightPhase();

        if (phase != COAST && phase != DROGUE_DESCENT && phase != MAIN_DESCENT)
        {
            return;
        }

        // requires 49 days to overflow, not handling this case
        timeInPostBurn += POST_BURN_PERIOD;

        if (timeInPostBurn < POST_BURN_REOPEN_INJECTION_VALVE_DURATION)
        {
            closeInjectionValve();
        }
        else
        {
            openInjectionValve();
        }
    }
}

void engineControlTask(void const* arg)
{
    OxidizerTankPressureData* data = (OxidizerTankPressureData*) arg;

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

            case ABORT:

                // Do nothing and let other code do what needs to be done
                osDelay(PRELAUNCH_PHASE_PERIOD);

                break;

            default:
                break;
        }
    }
}
