#include "stm32f4xx.h"
#include "stm32f4xx_hal_conf.h"
#include "cmsis_os.h"

#include "MonitorForLaunch.h"

// prelaunch
static const int PRELAUNCH_PHASE_PERIOD = 50;
static const int PRELAUNCH_READ_TIMEOUT = 5;
// burn
static const int BURN_DURATION = 10000;
// coast
static const int COAST_PHASE_PERIOD = 10;
// descent
static const int DESCENT_PHASE_PERIOD = 100;
static const int VENT_VALVE_TOGGLE_PERIOD = 5000;


// Cmmands
static const int LAUNCH_CMD = 0xAA;
static const int OPEN_VENT_CMD = 0xAA;
static const int CLOSE_VENT_CMD = 0xAA;

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

void monitorForLaunchTask(void const* arg)
{
    /** PRELAUNCH PHASE **/
    uint32_t prevWakeTime = osKernelSysTick();
    uint8_t buffer[1];

    for (;;)
    {
        osDelayUntil(&prevWakeTime, PRELAUNCH_PHASE_PERIOD);
        // Ensure valve is closed
        closeInjectionValve();

        buffer[0] = 0;
        HAL_UART_Receive(&huart1, buffer, sizeof(buffer), PRELAUNCH_READ_TIMEOUT);

        if (buffer[0] == LAUNCH_CMD)
        {
            // Launch signal received, go to burn phase
            break;
        }
        else if (buffer[0] == OPEN_VENT_CMD)
        {
            openVentValve();
        }
        else if (buffer[0] == CLOSE_VENT_CMD)
        {
            closeVentValve();
        }
    }

    /** BURN PHASE **/
    openInjectionValve();
    osDelay(BURN_DURATION);
    closeInjectionValve();

    /** COAST PHASE **/
    prevWakeTime = osKernelSysTick();

    for (;;)
    {
        osDelayUntil(&prevWakeTime, COAST_PHASE_PERIOD);

        if (apogeeDetected)
        {
            break;
        }
    }

    /** DESCENT PHASE **/
    uint8_t ventValveToggleCounter = 0;
    prevWakeTime = osKernelSysTick();

    for (;;)
    {
        osDelayUntil(&prevWakeTime, DESCENT_PHASE_PERIOD);
        closeInjectionValve(); // Ensure valve is closed
        ventValveToggleCounter += DESCENT_PHASE_PERIOD;

        if (ventValveToggleCounter > VENT_VALVE_TOGGLE_PERIOD)
        {
            toggleVentValve();
            ventValveToggleCounter = 0;
        }

    }
}
