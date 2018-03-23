#include "stm32f4xx.h"
#include "stm32f4xx_hal_conf.h"
#include "cmsis_os.h"

#include "MonitorForLaunch.h"

static const int MONITOR_FOR_LAUNCH_PERIOD = 50;
static const int LAUNCH_READ_TIMEOUT = 5;
static const int BURN_DURATION = 10000; // 10 seconds

void openInjectionValve()
{
    // TODO
}

void closeInjectionValve()
{
    // TDOD
}

void monitorForLaunchTask(void const* arg)
{
    /** PRE LAUNCH PHASE **/
    uint32_t prevWakeTime = osKernelSysTick();
    uint8_t buffer[1];

    for (;;)
    {
        osDelayUntil(&prevWakeTime, MONITOR_FOR_LAUNCH_PERIOD);
        // Ensure valve is closed
        closeInjectionValve();

        buffer[0] = 0;
        HAL_UART_Receive(&huart1, buffer, sizeof(buffer), LAUNCH_READ_TIMEOUT);

        if (buffer[0] == 0xAA)
        {
            // Launch signal received, go to burn phase
            break;
        }
    }

    /** BURN PHASE **/
    openInjectionValve();
    osDelay(BURN_DURATION);
    closeInjectionValve();

    /** POST BURN PHASE **/
    for (;;)
    {
        // Ensure valve is closed
        osDelay(MONITOR_FOR_LAUNCH_PERIOD);
        closeInjectionValve();
    }
}
