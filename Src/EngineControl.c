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
// post burn
static const int POST_BURN_PERIOD = 10;

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
                currentFlightPhase != BURN;
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
    for (;;)
    {
        switch (currentFlightPhase)
        {
            case PRELAUNCH:
                engineControlPrelaunchRoutine();
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
