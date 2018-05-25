#include "stm32f4xx.h"
#include "stm32f4xx_hal_conf.h"
#include "cmsis_os.h"

#include "ReadGps.h"

#include "Data.h"


static int READ_GPS_PERIOD = 1000;
static int CMD_TIMEOUT = 1500;

static uint8_t header = {0xB5, 0x62};
static uint8_t CFG_ANT[] = {0xB5, 0x62, 0x06, 0x13, 0x04, 0x00, 0x1B, 0x00, 0x8B, 0xA9}; // Set antenna settings
static uint8_t CFG_DAT[] = {0xB5, 0x62, 0x06, 0x06, 0x02, 0x00, 0x00, 0x00}; // Set datum settings
static uint8_t CFG_PRT[];	//TODO: setup configuration for UART
static uint8_t CFG_RATE[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xFA, 0x00, 0x01, 0x00, 0x01, 0x00}; // output rate
static uint8_t CFG_NMEA[]; // not sure if we need to do manually
static uint8_t setNavigationMode[] = {0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 
    								0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    								0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0xDC };


void readGpsTask(void const* arg)
{
    GpsData* data = (GpsData*) arg;
    uint32_t prevWakeTime = osKernelSysTick();
    
    uint16_t latitudeData, longitudeData, altitudeData, epochTimeMsecData;
    uint16_t GpsData[10];

    /* Set antenna settings */
    HAL_GPIO_WritePin(UART_GND_STATION_TX_GPIO_Port, UART_GND_STATION_TX_Pin, GPIO_PIN_RESET);  
    HAL_GPIO_Transmit(&huart1, &CFG_ANT, 10, CMD_TIMEOUT); 
    HAL_GPIO_WritePin(UART_GND_STATION_TX_GPIO_Port, UART_GND_STATION_TX_Pin, GPIO_PIN_SET);

    /* Set datum settings */
    HAL_GPIO_WritePin(UART_GND_STATION_TX_GPIO_Port, UART_GND_STATION_TX_Pin, GPIO_PIN_RESET);  
    HAL_GPIO_Transmit(&huart1, &CFG_DAT, 8, CMD_TIMEOUT); 
    HAL_GPIO_WritePin(UART_GND_STATION_TX_GPIO_Port, UART_GND_STATION_TX_Pin, GPIO_PIN_SET);

    /* Set output rate */
    HAL_GPIO_WritePin(UART_GND_STATION_TX_GPIO_Port, UART_GND_STATION_TX_Pin, GPIO_PIN_RESET);  
    HAL_GPIO_Transmit(&huart1, &CFG_RATE, 12, CMD_TIMEOUT); 
    HAL_GPIO_WritePin(UART_GND_STATION_TX_GPIO_Port, UART_GND_STATION_TX_Pin, GPIO_PIN_SET);

    /* Put in Navigation mode */
    HAL_GPIO_WritePin(UART_GND_STATION_TX_GPIO_Port, UART_GND_STATION_TX_Pin, GPIO_PIN_RESET);  
    HAL_GPIO_Transmit(&huart1, &setNavigationMode, 44, CMD_TIMEOUT); 
    HAL_GPIO_WritePin(UART_GND_STATION_TX_GPIO_Port, UART_GND_STATION_TX_Pin, GPIO_PIN_SET);



    for (;;)
    {
        osDelayUntil(&prevWakeTime, READ_GPS_PERIOD);

        //READ------------------------------------------------------
        // HAL_GPIO_WritePin(UART_GND_STATION_TX_GPIO_Port, UART_GND_STATION_TX_Pin, GPIO_PIN_RESET);
        // HAL_UART_Transmit(&huart1, &GPS_LLA_DOUBLE_PRECISION_CMD_PACKET, 1, CMD_TIMEOUT);
        // HAL_UART_Receive(&huart1, &GpsData[0], 1, CMD_TIMEOUT);
        // HAL_GPIO_WritePin(UART_GND_STATION_TX_GPIO_Port, UART_GND_STATION_TX_Pin, GPIO_PIN_SET);

        // osDelay(3);
    
        // HAL_GPIO_WritePin(UART_GND_STATION_TX_GPIO_Port, UART_GND_STATION_TX_Pin, GPIO_PIN_RESET);
        // HAL_UART_Transmit(&huart1, &GPS_CURR_TIME_CMD_PACKET, 1, CMD_TIMEOUT);
        // HAL_UART_Receive(&huart1, &epochTimeMsecData, 1, CMD_TIMEOUT);
        // HAL_GPIO_WritePin(UART_GND_STATION_TX_GPIO_Port, UART_GND_STATION_TX_Pin, GPIO_PIN_SET);

        // osDelay(3);

        /* Writeback */
        if (osMutexWait(data->mutex_, 0) != osOK)
        {
            continue;
        }

        data -> latitude_ = latitudeData;
    	data -> longitude_ = longitudeData;
    	data -> altitude_ = altitudeData;
    	data -> epochTimeMsec_ = epochTimeMsecData;
    }
}

/* String parser for GPS module */
// void parse_gps( int *lat, int *lon ) {
//   char *string = strstr(txt,"$GPGLL"); // txt -> text string from RX buffer
//   if( string != 0 ) {
//     if( string[7] != ',' ) {
//       	*lat = ( string[7] - 48 )*10 + ( string[8] - 48 );
//       	*lon = ( string[19] - 48 )*100 + ( string[20] - 48 )*10 + ( string[21] - 48 );
          
//         if( string[17] == 'S' )
//             *lat = 0 - latitude;
//         if(string[31] == 'W')
//              lon = 0 - longitude;         }
//  	}
// }