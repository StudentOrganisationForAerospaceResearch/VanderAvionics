#include "stm32f4xx.h"
#include "stm32f4xx_hal_conf.h"
#include "cmsis_os.h"
#include "math.h"

#include "ReadCombustionTankPressure.h"

#include "Data.h"

static int READ_COMBUSTION_TANK_PRESSURE_PERIOD = 300;

static const int ADC_POLL_TIMEOUT = 150;

void readCombustionTankPressureTask(void const* arg)
{
    CombustionTankPressureData* data = (CombustionTankPressureData*) arg;
    uint32_t prevWakeTime = osKernelSysTick();

    uint16_t adcRead = 0;   // Stores a 12 bit value ( maximum resolution for this ADC )
    double vo = 0;  // The voltage across the 133k resistor
    double vi = 0;  // The pressure sensor output
    double tankPressure = 0;

    int counter = 0;    // Counts up to 5 ADC readings before averaging

    // Resistor values in kOhms
    int R1 = 100;
    int R2 = 133;

    HAL_ADC_Start(&hadc1);  // Start the ADC peripheral

    for (;;)
    {
        osDelayUntil(&prevWakeTime, READ_COMBUSTION_TANK_PRESSURE_PERIOD);

        if (HAL_ADC_PollForConversion(&hadc1, ADC_POLL_TIMEOUT ) == HAL_OK)
        {
            adcRead += HAL_ADC_GetValue(&hadc1);
            counter++;
        }

        if (counter >= 5)
        {
            adcRead /= counter;  // Average 5 ADC readings

            vo = 3.3 / pow(2, 12) * adcRead;    // Calculate voltage from the 12 bit ADC reading

            // vi to voltage divider varies between 0.5V-4.5V, but the board requires a voltage less than 3.3V.
            // After the voltage divider, the voltage varies between 0.285V-2.57V
            vi = (R2 + R1) / R2 * vo;   // Calculate the original voltage output of the sensor

            // The pressure sensor is ratiometric. The pressure is 0 psi when the voltage is 0.5V, and is 300
            // psi when the voltage is 4.5V. The equation is derived from this information.
            tankPressure = (vi - 0.5) * 300 / 4;  // Tank pressure in psi
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
