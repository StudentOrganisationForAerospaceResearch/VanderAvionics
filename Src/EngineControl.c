#include "stm32f4xx.h"
#include "stm32f4xx_hal_conf.h"
#include "cmsis_os.h"

#include "EngineControl.h"

// Ground Station Commands
#define LAUNCH_CMD 0xAA
#define OPEN_VENT_CMD 0x01
#define CLOSE_VENT_CMD 0x02

// prelaunch
static const int PRELAUNCH_PHASE_PERIOD = 50;
// burn
static const int BURN_DURATION = 10000;
// coast
static const int COAST_PHASE_PERIOD = 10;
// descent
static const int DESCENT_PHASE_PERIOD = 100;
static const int VENT_VALVE_TOGGLE_PERIOD = 5000;

uint8_t readCommandFromGroundStation()
{
    // TODO
    // uint8_t buffer[1];
    // buffer[0] = 0;
    // HAL_UART_Receive(&huart1, buffer, sizeof(buffer), PRELAUNCH_READ_TIMEOUT);
    return 0;
}

void openInjectionValve()
{
    // TODO
}

void closeInjectionValve()
{
    // TDOD
}

void openVentValve()
{
    // TODO
}

void closeVentValve()
{
    // TDOD
}

void toggleVentValve()
{
    // TODO
}

/**
 * This routine listens for and reacts to commands from the ground station.
 * The ground station will provide commands to open and close the ventilation valve
 * for the purpose of avoiding pressure build up in the oxidizer tank.
 * The ground station will also send a launch command to begin the burn phase.
 */
void engineControlPrelaunchRoutine()
{
    uint32_t prevWakeTime = osKernelSysTick();

    for (;;)
    {
        osDelayUntil(&prevWakeTime, PRELAUNCH_PHASE_PERIOD);
        // Ensure valve is closed
        closeInjectionValve();

        switch (readCommandFromGroundStation())
        {
            case LAUNCH_CMD:
                currentFlightPhase = BURN;
                return; // Launch signal received, go to burn phase
                break;

            case OPEN_VENT_CMD:
                openVentValve();
                break;

            case CLOSE_VENT_CMD:
                closeVentValve();
                break;
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
 * This routine keeps the injection valve and ventilation valves closed for the coast phase.
 * The injection valve is closed to avoid overshooting the goal altitude.
 * The ventilation valve is closed to avoid a trajectory change from oxidizer discharge.
 * The routine exits when the currentFlightPhase is past the coast phase.
 */
void engineControlCoastRoutine()
{
    uint32_t prevWakeTime = osKernelSysTick();

    for (;;)
    {
        osDelayUntil(&prevWakeTime, COAST_PHASE_PERIOD);
        closeInjectionValve();
        closeVentValve();

        // Wait for the coast phase to end
        if (currentFlightPhase > COAST)
        {
            return;
        }
    }
}

/**
 * This routine closes the injection valve and vents any remaining oxidizer.
 * The injection valve is closed to prevent combustion during descent.
 * The ventilation valve is toggled to vent any remaining oxidizer.
 * The ventilation valve must be toggled because the valve will break
 * if open for over ~10 seconds.
 */
void engineControlPostCoastRoutine()
{
    uint8_t ventValveToggleCounter = 0;
    uint32_t prevWakeTime = osKernelSysTick();

    for (;;)
    {
        osDelayUntil(&prevWakeTime, DESCENT_PHASE_PERIOD);
        closeInjectionValve(); // Ensure valve is closedp
        ventValveToggleCounter += DESCENT_PHASE_PERIOD;

        if (ventValveToggleCounter > VENT_VALVE_TOGGLE_PERIOD)
        {
            toggleVentValve();
            ventValveToggleCounter = 0;
        }
    }
}

void engineControlTask(void const* arg)
{
    switch (currentFlightPhase)
    {
        case PRELAUNCH:
            engineControlPrelaunchRoutine();
            break;

        case BURN:
            engineControlBurnRoutine();
            break;

        case COAST:
            engineControlCoastRoutine();
            break;

        case DROGUE_DESCENT: // fall through
        case MAIN_DESCENT:
            engineControlPostCoastRoutine();
            break;

        default:
            break;
    }
}
