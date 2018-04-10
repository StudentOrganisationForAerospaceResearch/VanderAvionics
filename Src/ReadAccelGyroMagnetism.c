#include "stm32f4xx.h"
#include "stm32f4xx_hal_conf.h"
#include "cmsis_os.h"

#include "ReadAccelGyroMagnetism.h"

#include "Data.h"

static int READ_ACCEL_GYRO_MAGNETISM = 1000;

static const int CMD_SIZE = 1;
static const int CMD_TIMEOUT = 150;
static const uint8_t RESET_CMD = 0x1E;
static const uint8_t CTRL6_XL_CMD = 0x20; // set accelerometer and gyro to active
static const uint8_t CTRL1_G_CMD = 0x10 | 0xE0; // turn on both accel and gyro
static const uint8_t CTRL8_CMD = 0x22 | 0x08; // 3 wire mode

static const uint8_t READX_LOW_CMD = 0x28 << 1 | 1;
static const uint8_t READX_HIGH_CMD = 0x29 << 1 | 1;
static const uint8_t READY_LOW_CMD = 0x2A << 1 | 1;
static const uint8_t READY_HIGH_CMD = 0x2B << 1 | 1;
static const uint8_t READZ_LOW_CMD = 0x2C << 1 | 1;
static const uint8_t READZ_HIGH_CMD = 0x2D << 1 | 1;

uint8_t address, dataIn;
uint8_t accelXData[2], accelYData[2], accelZData[2];
uint16_t accelX,accelY,accelZ;
uint8_t gyroX,gyroY,gyroZ;
uint8_t magnetX, magnetY, magnetZ;

void readAccelGyroMagnetismTask(void const* arg)
{
    AccelGyroMagnetismData* data = (AccelGyroMagnetismData*) arg;
    uint32_t prevWakeTime = osKernelSysTick();

    /*HAL_GPIO_WritePin(XL_CS_GPIO_Port, XL_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, &RESET_CMD, CMD_SIZE, CMD_TIMEOUT);
    osDelay(3); 
    HAL_GPIO_WritePin(XL_CS_GPIO_Port, XL_CS_Pin, GPIO_PIN_SET);*/

    /* Accelerometer and gyro active mode on */
    HAL_GPIO_WritePin(XL_CS_GPIO_Port, XL_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, &CTRL6_XL_CMD, CMD_SIZE, CMD_TIMEOUT);
    osDelay(3); 
    HAL_GPIO_WritePin(XL_CS_GPIO_Port, XL_CS_Pin, GPIO_PIN_SET);

    /* Set up control register 1 */
    HAL_GPIO_WritePin(XL_CS_GPIO_Port, XL_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, &CTRL1_G_CMD, CMD_SIZE, CMD_TIMEOUT);
    osDelay(3); 
    HAL_GPIO_WritePin(XL_CS_GPIO_Port, XL_CS_Pin, GPIO_PIN_SET);

    /* Set up control register 8 */
    HAL_GPIO_WritePin(XL_CS_GPIO_Port, XL_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, &CTRL8_CMD, CMD_SIZE, CMD_TIMEOUT);
    osDelay(3); 
    HAL_GPIO_WritePin(XL_CS_GPIO_Port, XL_CS_Pin, GPIO_PIN_SET);


    for (;;)
    {
        osDelayUntil(&prevWakeTime, READ_ACCEL_GYRO_MAGNETISM);

       /* Check XLDA in STATUS_REG (Accelerometer ready bit)
        * Reading OUTX_XL/../ clears XLDA, wait for first sample
        * Read again, discard data
        */

        //address = 0x20;
        //HAL_GPIO_WritePin(XL_CS_GPIO_Port, XL_CS_Pin, GPIO_PIN_RESET);
        //HAL_SPI_Transmit(&hspi1, &address, CMD_SIZE, CMD_TIMEOUT);
        //HAL_GPIO_WritePin(XL_CS_GPIO_Port, XL_CS_Pin, GPIO_PIN_SET);

        //osDelay(2); 

        //HAL_GPIO_WritePin(XL_CS_GPIO_Port, XL_CS_Pin, GPIO_PIN_RESET);
        //HAL_SPI_Transmit(&hspi1, &READZ_CMD, CMD_SIZE, CMD_TIMEOUT);

        //READ------------------------------------------------------
        accelX = 0;
        accelY = 0;
        accelZ = 0;

        /* Get values for X axis */
        HAL_SPI_Transmit(&hspi1, &READX_LOW_CMD, CMD_SIZE, CMD_TIMEOUT);
        HAL_SPI_Receive(&hspi1, &accelXData[0], CMD_SIZE, CMD_TIMEOUT);
        HAL_SPI_Transmit(&hspi1, &READX_HIGH_CMD, CMD_SIZE, CMD_TIMEOUT);
        HAL_SPI_Receive(&hspi1, &accelXData[1], CMD_SIZE, CMD_TIMEOUT);
        accelX += ((int16_t)accelXData[1] << 8) | accelXData[0];  // Turn the MSB and LSB into a signed 16-bit value

        HAL_SPI_Transmit(&hspi1, &READX_LOW_CMD, CMD_SIZE, CMD_TIMEOUT);
        HAL_SPI_Receive(&hspi1, &accelXData[0], CMD_SIZE, CMD_TIMEOUT);
        HAL_SPI_Transmit(&hspi1, &READX_HIGH_CMD, CMD_SIZE, CMD_TIMEOUT);
        HAL_SPI_Receive(&hspi1, &accelXData[1], CMD_SIZE, CMD_TIMEOUT);
        accelX += ((int16_t)accelXData[1] << 8) | accelXData[0];  // Turn the MSB and LSB into a signed 16-bit value

        HAL_SPI_Transmit(&hspi1, &READX_LOW_CMD, CMD_SIZE, CMD_TIMEOUT);
        HAL_SPI_Receive(&hspi1, &accelXData[0], CMD_SIZE, CMD_TIMEOUT);
        HAL_SPI_Transmit(&hspi1, &READX_HIGH_CMD, CMD_SIZE, CMD_TIMEOUT);
        HAL_SPI_Receive(&hspi1, &accelXData[1], CMD_SIZE, CMD_TIMEOUT);
        accelX += ((int16_t)accelXData[1] << 8) | accelXData[0];  // Turn the MSB and LSB into a signed 16-bit value

        /* Get values for Y axis */
        HAL_SPI_Transmit(&hspi1, &READY_LOW_CMD, CMD_SIZE, CMD_TIMEOUT);
        HAL_SPI_Receive(&hspi1, &accelYData[0], CMD_SIZE, CMD_TIMEOUT);
        HAL_SPI_Transmit(&hspi1, &READY_HIGH_CMD, CMD_SIZE, CMD_TIMEOUT);
        HAL_SPI_Receive(&hspi1, &accelYData[1], CMD_SIZE, CMD_TIMEOUT);
        accelY += ((int16_t)accelYData[1] << 8) | accelYData[0];  // Turn the MSB and LSB into a signed 16-bit value

        HAL_SPI_Transmit(&hspi1, &READY_LOW_CMD, CMD_SIZE, CMD_TIMEOUT);
        HAL_SPI_Receive(&hspi1, &accelYData[0], CMD_SIZE, CMD_TIMEOUT);
        HAL_SPI_Transmit(&hspi1, &READY_HIGH_CMD, CMD_SIZE, CMD_TIMEOUT);
        HAL_SPI_Receive(&hspi1, &accelYData[1], CMD_SIZE, CMD_TIMEOUT);
        accelY += ((int16_t)accelYData[1] << 8) | accelYData[0];  // Turn the MSB and LSB into a signed 16-bit value

        HAL_SPI_Transmit(&hspi1, &READY_LOW_CMD, CMD_SIZE, CMD_TIMEOUT);
        HAL_SPI_Receive(&hspi1, &accelYData[0], CMD_SIZE, CMD_TIMEOUT);
        HAL_SPI_Transmit(&hspi1, &READY_HIGH_CMD, CMD_SIZE, CMD_TIMEOUT);
        HAL_SPI_Receive(&hspi1, &accelYData[1], CMD_SIZE, CMD_TIMEOUT);
        accelY += ((int16_t)accelYData[1] << 8) | accelYData[0];  // Turn the MSB and LSB into a signed 16-bit value

        /* Get values for Z axis */
        HAL_SPI_Transmit(&hspi1, &READZ_LOW_CMD, CMD_SIZE, CMD_TIMEOUT);
        HAL_SPI_Receive(&hspi1, &accelZData[0], CMD_SIZE, CMD_TIMEOUT);
        HAL_SPI_Transmit(&hspi1, &READZ_HIGH_CMD, CMD_SIZE, CMD_TIMEOUT);
        HAL_SPI_Receive(&hspi1, &accelZData[1], CMD_SIZE, CMD_TIMEOUT);
        accelZ += ((int16_t)accelZData[1] << 8) | accelZData[0];  // Turn the MSB and LSB into a signed 16-bit value

        HAL_SPI_Transmit(&hspi1, &READZ_LOW_CMD, CMD_SIZE, CMD_TIMEOUT);
        HAL_SPI_Receive(&hspi1, &accelZData[0], CMD_SIZE, CMD_TIMEOUT);
        HAL_SPI_Transmit(&hspi1, &READZ_HIGH_CMD, CMD_SIZE, CMD_TIMEOUT);
        HAL_SPI_Receive(&hspi1, &accelZData[1], CMD_SIZE, CMD_TIMEOUT);
        accelZ += ((int16_t)accelZData[1] << 8) | accelZData[0];  // Turn the MSB and LSB into a signed 16-bit value

        HAL_SPI_Transmit(&hspi1, &READZ_LOW_CMD, CMD_SIZE, CMD_TIMEOUT);
        HAL_SPI_Receive(&hspi1, &accelZData[0], CMD_SIZE, CMD_TIMEOUT);
        HAL_SPI_Transmit(&hspi1, &READZ_HIGH_CMD, CMD_SIZE, CMD_TIMEOUT);
        HAL_SPI_Receive(&hspi1, &accelZData[1], CMD_SIZE, CMD_TIMEOUT);
        accelZ += ((int16_t)accelZData[1] << 8) | accelZData[0];  // Turn the MSB and LSB into a signed 16-bit value

        /* Average the values */
        accelX /= 3;
        accelY /= 3;
        accelZ /= 3; 

        HAL_GPIO_WritePin(XL_CS_GPIO_Port, XL_CS_Pin, GPIO_PIN_SET);

        /* Writeback */
        osMutexWait(data->mutex_, 0);
        data->accelX_ = accelX;
        data->accelY_ = accelY;
      	data->accelZ_ = accelZ;
        // data->gyroX_ = gyroX;
        // data->gyroY_ = gyroY;
        // data->gyroZ_ = gyroZ;
        // data->magnetoX_ = magnetX;
        // data->magnetoY_ = magnetY;
        // data->magnetoZ_ = magnetZ;
      	osMutexRelease(data->mutex_);
    }
}
