#include "stm32f4xx.h"
#include "stm32f4xx_hal_conf.h"
#include "cmsis_os.h"
#include "math.h"

#include "ReadOxidizerTankConditions.h"

#include "Data.h"

static int READ_OXIDIZER_TANK_CONDITIONS_PERIOD = 250;  // Sampling delay set to 50 Hz to match high frequency logging

static const int ADC_POLL_TIMEOUT = 150;

void readOxidizerTankConditionsTask(void const* arg)
{
    OxidizerTankConditionsData* data = (OxidizerTankConditionsData*) arg;
    uint32_t prevWakeTime = osKernelSysTick();

    uint16_t adcRead = 0;   // Stores a 12 but value
    double vo = 0;  // The voltage across R2
    double vi = 0;  // The voltage across R1 + R2 in series
    double tankPressure = 0;

    int counter = 0;

    // Resistor values in kOhms
    int R1 = 60; //100
    int R2 = 165; //133

    HAL_ADC_Start(&hadc1);  // Start the ADC peripheral

    for (;;)
    {
        osDelayUntil(&prevWakeTime, READ_OXIDIZER_TANK_CONDITIONS_PERIOD);

        if (HAL_ADC_PollForConversion(&hadc1, ADC_POLL_TIMEOUT ) == HAL_OK)
        {
            adcRead += HAL_ADC_GetValue(&hadc1);
            counter++;
        }
        else
        {
            continue;
        }

        if (counter < 10)
        {
            continue;
        }

        adcRead /= 10;  // Average 10 ADC readings

        // vi to voltage divider varies between 0.5V-4.5V, but the board requires a voltage less than 3.3V.
        // After the voltage divider, the voltage varies between 0.285V-2.57V
        vo = 3.3 / pow(2, 12) * adcRead;

        vi = (R2 + R1) / R2 * vo;   // Calculate the original voltage before the voltage divider.

        // The low pressure sensor is ratiometric. The pressure is 0 psi when the voltage is 0.5V, and is 300
        // psi when the voltage is 4.5V. The equations is derived from this information.
        tankPressure = (vi - 0.5) * 300 / 4;  // Tank pressure in psi
        tankPressure = tankPressure * 1000;   // Multiply by 1000 to keep decimal places

        // Set back to 0 to
        adcRead = 0;
        counter = 0;

        if (osMutexWait(data->mutex_, 0) != osOK)
        {
            continue;
        }

        data->pressure_ = (int32_t) tankPressure;
        osMutexRelease(data->mutex_);
    }
}
