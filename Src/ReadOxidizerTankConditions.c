#include "stm32f4xx.h"
#include "stm32f4xx_hal_conf.h"
#include "cmsis_os.h"
#include "math.h"

#include "ReadOxidizerTankConditions.h"

#include "Data.h"

static int READ_OXIDIZER_TANK_CONDITIONS_PERIOD = 1000;

void readOxidizerTankConditionsTask(void const* arg)
{
    OxidizerTankConditionsData* data = (OxidizerTankConditionsData*) arg;
    uint32_t prevWakeTime = osKernelSysTick();

    HAL_ADC_Start(&hadc1);
    double lowPressure = 0;

    for (;;)
    {
        osDelayUntil(&prevWakeTime, READ_OXIDIZER_TANK_CONDITIONS_PERIOD);

        if (HAL_ADC_PollForConversion(&hadc1, 1000000) == HAL_OK)
        {
            lowPressure = HAL_ADC_GetValue(&hadc1);
        }

        // Vin varies between 0.5V-4.5V, but the board requires a voltage of 3.3V or less.
        // R1 = 60kOhm, R2 = 166kOhm -> Change resistors so max value is at 3V?
        // Vo = R2/(R1+R2)*Vin, and Vo varies between 0.367V-3.3V

        lowPressure = 3.3 / pow(2, 12) * lowPressure;

        // Vin = (R1+R2)/R2*Vo to get the original value.
        lowPressure = (166 + 60) / 166 * lowPressure;

        if (osMutexWait(data->mutex_, 0) != osOK)
        {
            continue;
        }

        lowPressure *= 1000; // Multiply by 1000 to keep decimal places - Final value in mV
        data->pressure_ = (int32_t) lowPressure;
        osMutexRelease(data->mutex_);
    }
}
