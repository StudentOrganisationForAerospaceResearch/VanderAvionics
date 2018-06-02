#include "stm32f4xx.h"
#include "stm32f4xx_hal_conf.h"
#include "cmsis_os.h"

#include "ReadGps.h"

#include "Data.h"

static int READ_GPS_PERIOD = 1000;
static int CMD_TIMEOUT = 1500;

static const uint8_t GPS_SET_ANTENNA[] = {0xB5, 0x62, 0x06, 0x13, 0x04, 0x00, 0x1B, 0x00, 0x8B, 0xA9}; // CFG-ANT
static const uint8_t GPS_SET_DATUM[] = {0xB5, 0x62, 0x06, 0x06, 0x02, 0x00, 0x00, 0x00}; // CFG-DAT
static const uint8_t GPS_SETUP_UART[];  //TODO: setup configuration for UART [CFG-PRT]
static const uint8_t GPS_SET_OUTPUT_RATE[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xFA, 0x00, 0x01, 0x00, 0x01, 0x00}; // CFG-RATE
static const uint8_t GPS_CONFIG_NMEA[]; // not sure if we need to do manually [CFG_NMEA]
static const uint8_t GPS_SET_NAVIGATION_MODE[] = {0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00,
                                                0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0xDC
                                               };
static const uint8_t GPS_test_packet[] = {0111100101, 0110100101};

void readGpsTask(void const* arg)
{
    GpsData* data = (GpsData*) arg;
    uint32_t prevWakeTime = osKernelSysTick();
    uint16_t latitudeData, longitudeData, altitudeData, epochTimeMsecData;
    uint16_t gpsData[10];

    /* Set antenna settings */
    HAL_GPIO_WritePin(UART_GPS_TX_GPIO_Port, UART_GPS_TX_Pin, GPIO_PIN_RESET);
    HAL_UART_Transmit(&huart1, &GPS_SET_ANTENNA, 10, CMD_TIMEOUT);
    HAL_GPIO_WritePin(UART_GPS_TX_GPIO_Port, UART_GPS_TX_Pin, GPIO_PIN_SET);

    /* Set datum settings */
    HAL_GPIO_WritePin(UART_GPS_TX_GPIO_Port, UART_GPS_TX_Pin, GPIO_PIN_RESET);
    HAL_UART_Transmit(&huart1, &GPS_SET_DATUM, 8, CMD_TIMEOUT);
    HAL_GPIO_WritePin(UART_GPS_TX_GPIO_Port, UART_GPS_TX_Pin, GPIO_PIN_SET);

    /* Set output rate */
    HAL_GPIO_WritePin(UART_GPS_TX_GPIO_Port, UART_GPS_TX_Pin, GPIO_PIN_RESET);
    HAL_UART_Transmit(&huart1, &GPS_SET_OUTPUT_RATE, 12, CMD_TIMEOUT);
    HAL_GPIO_WritePin(UART_GPS_TX_GPIO_Port, UART_GPS_TX_Pin, GPIO_PIN_SET);

    /* Put in Navigation mode */
    HAL_GPIO_WritePin(UART_GPS_TX_GPIO_Port, UART_GPS_TX_Pin, GPIO_PIN_RESET);
    HAL_UART_Transmit(&huart1, &GPS_SET_NAVIGATION_MODE, 44, CMD_TIMEOUT);
    HAL_GPIO_WritePin(UART_GPS_TX_GPIO_Port, UART_GPS_TX_Pin, GPIO_PIN_SET);


    for (;;)
    {
        osDelayUntil(&prevWakeTime, READ_GPS_PERIOD);
        //READ------------------------------------------------------
        // HAL_GPIO_WritePin(UART_GND_STATION_TX_GPIO_Port, UART_GND_STATION_TX_Pin, GPIO_PIN_RESET);
        // HAL_UART_Transmit(&huart1, &GPS_LLA_DOUBLE_PRECISION_CMD_PACKET, 1, CMD_TIMEOUT);
        HAL_UART_Receive(&huart1, &gpsData[0], 32, CMD_TIMEOUT);
        latitudeData = parseData_latitude(gpsData);
        longitudeData = parseData_longitude(gpsData);
        // altitudeData = &gpsData[2];
        // epochTimeMsecData = &gpsData[3];
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
        osMutexRelease(data->mutex_);

    }
}

/* String parser for GPS module */
int parseData_latitude(char * string){
	int lat;
	if( string != 0 ) {
	    if( string[7] != ',' ) {
	       lat = ( string[7] - 48 )*10 + ( string[8] - 48 );
	       // if( string[17] == 'S' )
        //     *lat = 0 - latitude;
	    }
   } else {
   	lat = 0;
   }
   return lat;
}

int parseData_longitude(char * string){
	int lon;
	if( string != 0 ) {
	    if( string[7] != ',' ) {
	       lon = ( string[19] - 48 )*100 + ( string[20] - 48 )*10 + ( string[21] - 48 );
	        // if(string[31] == 'W')
	        //      lon = 0 - longitude;         
	    }
   } else {
   		lon = 0;
   }
   return lon;
}
