#include "stm32f4xx.h"
#include "stm32f4xx_hal_conf.h"
#include "cmsis_os.h"
#include "math.h"

#include "ReadOxidizerTankPressure.h"

#include "Data.h"

static int READ_OXIDIZER_TANK_PRESSURE_PERIOD = 20;  // Sampling delay set to 50 Hz to match high frequency logging

static const int ADC_POLL_TIMEOUT = 150;

void readOxidizerTankPressureTask(void const* arg)
{
    OxidizerTankPressureData* data = (OxidizerTankPressureData*) arg;
    uint32_t prevWakeTime = osKernelSysTick();

    uint16_t adcRead = 0;   // Stores a 12 bit value ( maximum resolution for this ADC )
    double vo = 0;  // The pressure sensor voltage after amplification
    double vi = 0;  // The original pressure sensor output
    double tankPressure = 0;

    int counter = 0;    // Counts up to 5 ADC readings before averaging

    HAL_ADC_Start(&hadc2);  // Start the ADC peripheral

    for (;;)
    {
        osDelayUntil(&prevWakeTime, READ_OXIDIZER_TANK_PRESSURE_PERIOD);

        if (HAL_ADC_PollForConversion(&hadc2, ADC_POLL_TIMEOUT ) == HAL_OK)
        {
            adcRead += HAL_ADC_GetValue(&hadc2);
            counter++;
        }

        if (counter >= 5)
        {
            adcRead /= counter;  // Average 5 ADC readings

            vo = 3.3 / pow(2, 12) * adcRead;    // Calculate voltage from the 12 bit ADC reading

            // Since the voltage output of the pressure sensor is very small ( below 0.1V ), an opamp was used to amplify
            // the voltage to be more accuractely read by the ADC. See AndromedaV2 PCB schematic for details.
            vi = 13 / (2 * 200) * adcRead; // Calculate the original voltage output of the sensor

            // The pressure sensor is ratiometric. The pressure is 0 psi when the voltage is 0V, and is 1000
            // psi when the voltage is 0.1V. The equation is derived from this information.
            tankPressure = vi * 1000 / 0.1;  // Tank pressure in psi
            tankPressure = tankPressure * 1000;   // Multiply by 1000 to keep decimal places

            adcRead = 0;
            counter = 0;
        }

        if (osMutexWait(data->mutex_, 0) != osOK)
        {
            continue;
        }

        data->pressure_ = (int32_t) tankPressure;
        osMutexRelease(data->mutex_);
    }
}
