#include "stm32f4xx.h"
#include "stm32f4xx_hal_conf.h"
#include "cmsis_os.h"

#include <Utils.h>

uint16_t averageArray(uint16_t array[], int size)
{
    uint16_t sum = 0;

    for (int i = 0; i < size; i++)
    {
        sum += array[i];
    }

    return (sum / size);
}