// #include "stm32f4xx.h"
// #include "stm32f4xx_hal_conf.h"
// #include "cmsis_os.h"

// #include "ReadExternalTemperature.h"

// #include "Data.h"

// static int READ_EXTERNAL_TEMPERATURE_PERIOD = 1000;

// void readExternalTemperatureTask(void const* arg)
// {
//     ExternalTemperatureData* data = (ExternalTemperatureData*) arg;
//     uint32_t prevWakeTime = osKernelSysTick();

//     for (;;)
//     {
//         osDelayUntil(&prevWakeTime, READ_EXTERNAL_TEMPERATURE_PERIOD);
//     }
// }
