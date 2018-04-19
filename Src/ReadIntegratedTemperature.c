// #include "stm32f4xx.h"
// #include "stm32f4xx_hal_conf.h"
// #include "cmsis_os.h"

// #include "ReadIntegratedTemperature.h"

// #include "Data.h"

// static int READ_INTEGRATED_TEMPERATURE_PERIOD = 1000;

// void readIntegratedTemperatureTask(void const* arg)
// {
//     IntegratedTemperatureData* data = (IntegratedTemperatureData*) arg;
//     uint32_t prevWakeTime = osKernelSysTick();

//     for (;;)
//     {
//         osDelayUntil(&prevWakeTime, READ_INTEGRATED_TEMPERATURE_PERIOD);
//     }
// }
