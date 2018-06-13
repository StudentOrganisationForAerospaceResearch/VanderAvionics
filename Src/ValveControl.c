#include "stm32f4xx.h"
#include "stm32f4xx_hal_conf.h"
#include "cmsis_os.h"

#include "ValveControl.h"

static const int INJECTION_VALVE_PULSE_PERIOD = 500; 	// 0.5s high pulse to change state of injection valve

int ventValveIsOpen = 0;
static int injectionValveIsOpen = 0;

void openVentValve()
{
    // Powered is open
    HAL_GPIO_WritePin(GPIOB, VENT_VALVE_Pin, GPIO_PIN_SET);
    ventValveIsOpen = 1;
}

void closeVentValve()
{
    // Unpowered is closed
    HAL_GPIO_WritePin(GPIOB, VENT_VALVE_Pin, GPIO_PIN_RESET);
    ventValveIsOpen = 0;
}
// High pulse is sent to change state of injection valve.
void openInjectionValve()
{
    if (injectionValveIsOpen)
    {
        // already open
        return;
    }

    injectionValveIsOpen = 1;

    HAL_GPIO_WritePin(GPIOB, INJECTION_VALVE_Pin, GPIO_PIN_SET);
    osDelay(INJECTION_VALVE_PULSE_PERIOD);
    HAL_GPIO_WritePin(GPIOB, INJECTION_VALVE_Pin, GPIO_PIN_RESET);
}

void closeInjectionValve()
{
    if (!injectionValveIsOpen)
    {
        // already closed
        return;
    }

    injectionValveIsOpen = 0;

    HAL_GPIO_WritePin(GPIOB, INJECTION_VALVE_Pin, GPIO_PIN_SET);
    osDelay(INJECTION_VALVE_PULSE_PERIOD);
    HAL_GPIO_WritePin(GPIOB, INJECTION_VALVE_Pin, GPIO_PIN_RESET);
}

void isInjectionValveOpen()
{
    return injectionValveIsOpen;
}