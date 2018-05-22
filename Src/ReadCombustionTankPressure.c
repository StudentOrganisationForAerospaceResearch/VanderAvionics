#include "stm32f4xx.h"
#include "stm32f4xx_hal_conf.h"
#include "cmsis_os.h"
#include "math.h"

#include "ReadCombustionTankPressure.h"

#include "Data.h"

#define ADC1_QUEUE_SIZE 5

static int READ_COMBUSTION_TANK_PRESSURE_PERIOD = 300;

static const int ADC1_POLL_TIMEOUT = 150;

static uint16_t adc1ValuesQueue[ADC1_QUEUE_SIZE] = {0};    // Average ADC1_QUEUE_SIZE ADC readings that are each 12 bits in size
// ( maximum resolution for this ADC )

void readCombustionTankPressureTask(void const* arg)
{
    CombustionTankPressureData* data = (CombustionTankPressureData*) arg;
    uint32_t prevWakeTime = osKernelSysTick();

    double vo = 0;  // The voltage across the 133k resistor
    double vi = 0;  // The pressure sensor output
    double tankPressure = 0;

    int adc1QueueIndex = 0;    // Counts up to ADC1_QUEUE_SIZE ADC readings before averaging

    // Resistor values in kOhms
    int R1 = 60; //100;
    int R2 = 165; //133;

    HAL_ADC_Start(&hadc1);  // Start the ADC peripheral

    osDelay(5);

    for (;;)
    {
        osDelayUntil(&prevWakeTime, READ_COMBUSTION_TANK_PRESSURE_PERIOD);

        if (HAL_ADC_PollForConversion(&hadc1, ADC1_POLL_TIMEOUT ) == HAL_OK)
        {
            adc1ValuesQueue[adc1QueueIndex++] = HAL_ADC_GetValue(&hadc1);
        }

        if (adc1QueueIndex >= ADC1_QUEUE_SIZE)
        {
            uint16_t adcRead = getAverageAdc1Reading();

            vo = 3.3 / pow(2, 12) * adcRead;    // Calculate voltage from the 12 bit ADC reading

            // vi to voltage divider varies between 0.5V-4.5V, but the board requires a voltage less than 3.3V.
            // After the voltage divider, the voltage varies between 0.285V-2.57V
            vi = (R2 + R1) / R2 * vo;   // Calculate the original voltage output of the sensor

            // The pressure sensor is ratiometric. The pressure is 0 psi when the voltage is 0.5V, and is 300
            // psi when the voltage is 4.5V. The equation is derived from this information.
            tankPressure = (vi - 0.5) * 300 / 4;  // Tank pressure in psi
            tankPressure = tankPressure * 1000;   // Multiply by 1000 to keep decimal places

            adc1QueueIndex %= ADC1_QUEUE_SIZE;
        }

        if (osMutexWait(data->mutex_, 0) != osOK)
        {
            continue;
        }

        data->pressure_ = (int32_t) tankPressure;
        osMutexRelease(data->mutex_);
    }
}

uint16_t getAverageAdc1Reading()
{
    uint16_t sum = 0;

    for (int i = 0; i < ADC1_QUEUE_SIZE; i++)
    {
        sum += adc1ValuesQueue[i];
    }

    return (sum / ADC1_QUEUE_SIZE);
}