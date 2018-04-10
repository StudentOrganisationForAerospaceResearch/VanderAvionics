#include "stm32f4xx.h"
#include "stm32f4xx_hal_conf.h"
#include "cmsis_os.h"

#include "ReadAccelGyroMagnetism.h"

#include "Data.h"

static int READ_ACCEL_GYRO_MAGNETISM = 1000;

static const int READ_CMD_SIZE = 1;
static const int WRITE_CMD_SIZE = 2;
static const int CMD_TIMEOUT = 150;

static const uint8_t READ_CMD = 0x1;
static const uint8_t WRITE_CMD = 0x0;

// Register addresses
static const uint8_t ACCEL_CTRL_REGISTER_6_ADDR = 0x20; // CTRL_REG6_XL (20h)
static const uint8_t GYRO_CTRL_REGISTER_1_ADDR = 0x10; // CTRL_REG1_G (10h)
static const uint8_t CTRL_REGISTER_8_ADDR = 0x22; // CTRL_REG8 (22h)

static const uint8_t ACCEL_X_LOW_REGISTER_ADDR = 0x28;
static const uint8_t ACCEL_X_HIGH_REGISTER_ADDR = 0x29;
static const uint8_t ACCEL_Y_LOW_REGISTER_ADDR = 0x2A;
static const uint8_t ACCEL_Y_HIGH_REGISTER_ADDR = 0x2B;
static const uint8_t ACCEL_Z_LOW_REGISTER_ADDR = 0x2C;
static const uint8_t ACCEL_Z_HIGH_REGISTER_ADDR = 0x2D;

// Full Commands
static const uint8_t ACCEL_ODR_FREQ_SETTING = 0x4; // 0b100, 238Hz
static const uint16_t ACTIVATE_ACCEL_CMD = ACCEL_ODR_FREQ_SETTING << 8 | ACCEL_CTRL_REGISTER_6_ADDR << 1 | WRITE_CMD;
static const uint8_t ACCEL_ODR_FREQ_SETTING = 0x4; // 0b100, 238Hz
static const uint16_t ACTIVATE_GYRO_CMD = ACCEL_ODR_FREQ_SETTING << 8 | GYRO_CTRL_REGISTER_1_ADDR << 1 | WRITE_CMD;
static const uint8_t WIRE_MODE_3_SETTING = 0x8;
static const uint16_t ACTIVATE_3_WIRE_MODE_CMD = WIRE_MODE_3_SETTING << 8 | CTRL_REGISTER_8_ADDR << 1 | WRITE_CMD;

static const uint8_t READ_ACCEL_X_LOW_CMD = ACCEL_X_LOW_REGISTER_ADDR << 1 | READ_CMD;
static const uint8_t READ_ACCEL_X_HIGH_CMD = ACCEL_X_HIGH_REGISTER_ADDR << 1 | READ_CMD;
static const uint8_t READ_ACCEL_Y_LOW_CMD = ACCEL_Y_LOW_REGISTER_ADDR << 1 | READ_CMD;
static const uint8_t READ_ACCEL_Y_HIGH_CMD = ACCEL_Y_HIGH_REGISTER_ADDR << 1 | READ_CMD;
static const uint8_t READ_ACCEL_Z_LOW_CMD = ACCEL_Z_LOW_REGISTER_ADDR << 1 | READ_CMD;
static const uint8_t READ_ACCEL_Z_HIGH_CMD = ACCEL_Z_HIGH_REGISTER_ADDR << 1 | READ_CMD;

uint8_t address, dataIn;
uint8_t accelXData[2], accelYData[2], accelZData[2];
uint16_t accelX, accelY, accelZ;
uint8_t gyroX, gyroY, gyroZ;
uint8_t magnetX, magnetY, magnetZ;

void readAccelGyroMagnetismTask(void const* arg)
{
    AccelGyroMagnetismData* data = (AccelGyroMagnetismData*) arg;
    uint32_t prevWakeTime = osKernelSysTick();

    HAL_GPIO_WritePin(XL_CS_GPIO_Port, XL_CS_Pin, GPIO_PIN_SET);

    /* Accelerometer and gyroscope active mode on */
    HAL_GPIO_WritePin(XL_CS_GPIO_Port, XL_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, &ACTIVATE_ACCEL_CMD, WRITE_CMD_SIZE, CMD_TIMEOUT);
    osDelay(3);
    HAL_GPIO_WritePin(XL_CS_GPIO_Port, XL_CS_Pin, GPIO_PIN_SET);

    HAL_GPIO_WritePin(XL_CS_GPIO_Port, XL_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, &ACTIVATE_GYRO_CMD, WRITE_CMD_SIZE, CMD_TIMEOUT);
    osDelay(3);
    HAL_GPIO_WritePin(XL_CS_GPIO_Port, XL_CS_Pin, GPIO_PIN_SET);

    /* Set up 3 wire mode */
    HAL_GPIO_WritePin(XL_CS_GPIO_Port, XL_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, &ACTIVATE_3_WIRE_MODE_CMD, WRITE_CMD_SIZE, CMD_TIMEOUT);
    osDelay(3);

    for (;;)
    {
        osDelayUntil(&prevWakeTime, READ_ACCEL_GYRO_MAGNETISM);

        /* Check XLDA in STATUS_REG (Accelerometer ready bit)
         * Reading OUTX_XL/../ clears XLDA, wait for first sample
         * Read again, discard data
         */
        //    // Read register to return higher bit data from z-axis
        // // address = 0x80|0x2D;
        //  HAL_GPIO_WritePin(XL_CS_GPIO_Port, XL_CS_Pin, GPIO_PIN_RESET);
        //  HAL_SPI_Transmit(&hspi1,0x20,1,500);
        //  osDelay(2);
        //  //address = 0x00;
        //  //HAL_SPI_Receive(&hspi1,&accelZ,1,500);
        //  HAL_GPIO_WritePin(XL_CS_GPIO_Port, XL_CS_Pin, GPIO_PIN_SET);

        //  //accelZ = *hspi1.pRxBuffPtr;
        //  //accelZ = address;

        //  HAL_GPIO_WritePin(XL_CS_GPIO_Port, XL_CS_Pin, GPIO_PIN_RESET);
        //  HAL_SPI_Transmit(&hspi1, 0x67, 1, 500);
        //  HAL_GPIO_WritePin(XL_CS_GPIO_Port, XL_CS_Pin, GPIO_PIN_SET);

        //  osDelay(2);

        //HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
        //HAL_SPI_Transmit(&hspi1, &address, 1, 500);


        /* Check XLDA in STATUS_REG (Accelerometer ready bit)
         * Reading OUTX_XL/../ clears XLDA, wait for first sample
         * Read again, discard data
         */

        //address = 0x20;
        //HAL_GPIO_WritePin(XL_CS_GPIO_Port, XL_CS_Pin, GPIO_PIN_RESET);
        //HAL_SPI_Transmit(&hspi1, &address, READ_CMD_SIZE, CMD_TIMEOUT);
        //HAL_GPIO_WritePin(XL_CS_GPIO_Port, XL_CS_Pin, GPIO_PIN_SET);

        //osDelay(2);

        //HAL_GPIO_WritePin(XL_CS_GPIO_Port, XL_CS_Pin, GPIO_PIN_RESET);
        //HAL_SPI_Transmit(&hspi1, &READZ_CMD, READ_CMD_SIZE, CMD_TIMEOUT);

        //READ------------------------------------------------------
        accelX = 0;
        accelY = 0;
        accelZ = 0;

        /* Get values for X axis */
        HAL_SPI_Transmit(&hspi1, &READ_ACCEL_X_LOW_CMD, READ_CMD_SIZE, CMD_TIMEOUT);
        HAL_SPI_Receive(&hspi1, &accelXData[0], READ_CMD_SIZE, CMD_TIMEOUT);
        HAL_SPI_Transmit(&hspi1, &READ_ACCEL_X_HIGH_CMD, READ_CMD_SIZE, CMD_TIMEOUT);
        HAL_SPI_Receive(&hspi1, &accelXData[1], READ_CMD_SIZE, CMD_TIMEOUT);
        accelX += ((int16_t)accelXData[1] << 8) | accelXData[0];  // Turn the MSB and LSB into a signed 16-bit value

        HAL_SPI_Transmit(&hspi1, &READ_ACCEL_X_LOW_CMD, READ_CMD_SIZE, CMD_TIMEOUT);
        HAL_SPI_Receive(&hspi1, &accelXData[0], READ_CMD_SIZE, CMD_TIMEOUT);
        HAL_SPI_Transmit(&hspi1, &READ_ACCEL_X_HIGH_CMD, READ_CMD_SIZE, CMD_TIMEOUT);
        HAL_SPI_Receive(&hspi1, &accelXData[1], READ_CMD_SIZE, CMD_TIMEOUT);
        accelX += ((int16_t)accelXData[1] << 8) | accelXData[0];  // Turn the MSB and LSB into a signed 16-bit value

        HAL_SPI_Transmit(&hspi1, &READ_ACCEL_X_LOW_CMD, READ_CMD_SIZE, CMD_TIMEOUT);
        HAL_SPI_Receive(&hspi1, &accelXData[0], READ_CMD_SIZE, CMD_TIMEOUT);
        HAL_SPI_Transmit(&hspi1, &READ_ACCEL_X_HIGH_CMD, READ_CMD_SIZE, CMD_TIMEOUT);
        HAL_SPI_Receive(&hspi1, &accelXData[1], READ_CMD_SIZE, CMD_TIMEOUT);
        accelX += ((int16_t)accelXData[1] << 8) | accelXData[0];  // Turn the MSB and LSB into a signed 16-bit value

        /* Get values for Y axis */
        HAL_SPI_Transmit(&hspi1, &READ_ACCEL_Y_LOW_CMD, READ_CMD_SIZE, CMD_TIMEOUT);
        HAL_SPI_Receive(&hspi1, &accelYData[0], READ_CMD_SIZE, CMD_TIMEOUT);
        HAL_SPI_Transmit(&hspi1, &READ_ACCEL_Y_HIGH_CMD, READ_CMD_SIZE, CMD_TIMEOUT);
        HAL_SPI_Receive(&hspi1, &accelYData[1], READ_CMD_SIZE, CMD_TIMEOUT);
        accelY += ((int16_t)accelYData[1] << 8) | accelYData[0];  // Turn the MSB and LSB into a signed 16-bit value

        HAL_SPI_Transmit(&hspi1, &READ_ACCEL_Y_LOW_CMD, READ_CMD_SIZE, CMD_TIMEOUT);
        HAL_SPI_Receive(&hspi1, &accelYData[0], READ_CMD_SIZE, CMD_TIMEOUT);
        HAL_SPI_Transmit(&hspi1, &READ_ACCEL_Y_HIGH_CMD, READ_CMD_SIZE, CMD_TIMEOUT);
        HAL_SPI_Receive(&hspi1, &accelYData[1], READ_CMD_SIZE, CMD_TIMEOUT);
        accelY += ((int16_t)accelYData[1] << 8) | accelYData[0];  // Turn the MSB and LSB into a signed 16-bit value

        HAL_SPI_Transmit(&hspi1, &READ_ACCEL_Y_LOW_CMD, READ_CMD_SIZE, CMD_TIMEOUT);
        HAL_SPI_Receive(&hspi1, &accelYData[0], READ_CMD_SIZE, CMD_TIMEOUT);
        HAL_SPI_Transmit(&hspi1, &READ_ACCEL_Y_HIGH_CMD, READ_CMD_SIZE, CMD_TIMEOUT);
        HAL_SPI_Receive(&hspi1, &accelYData[1], READ_CMD_SIZE, CMD_TIMEOUT);
        accelY += ((int16_t)accelYData[1] << 8) | accelYData[0];  // Turn the MSB and LSB into a signed 16-bit value

        /* Get values for Z axis */
        HAL_SPI_Transmit(&hspi1, &READ_ACCEL_Z_LOW_CMD, READ_CMD_SIZE, CMD_TIMEOUT);
        HAL_SPI_Receive(&hspi1, &accelZData[0], READ_CMD_SIZE, CMD_TIMEOUT);
        HAL_SPI_Transmit(&hspi1, &READ_ACCEL_Z_HIGH_CMD, READ_CMD_SIZE, CMD_TIMEOUT);
        HAL_SPI_Receive(&hspi1, &accelZData[1], READ_CMD_SIZE, CMD_TIMEOUT);
        accelZ += ((int16_t)accelZData[1] << 8) | accelZData[0];  // Turn the MSB and LSB into a signed 16-bit value

        HAL_SPI_Transmit(&hspi1, &READ_ACCEL_Z_LOW_CMD, READ_CMD_SIZE, CMD_TIMEOUT);
        HAL_SPI_Receive(&hspi1, &accelZData[0], READ_CMD_SIZE, CMD_TIMEOUT);
        HAL_SPI_Transmit(&hspi1, &READ_ACCEL_Z_HIGH_CMD, READ_CMD_SIZE, CMD_TIMEOUT);
        HAL_SPI_Receive(&hspi1, &accelZData[1], READ_CMD_SIZE, CMD_TIMEOUT);
        accelZ += ((int16_t)accelZData[1] << 8) | accelZData[0];  // Turn the MSB and LSB into a signed 16-bit value

        HAL_SPI_Transmit(&hspi1, &READ_ACCEL_Z_LOW_CMD, READ_CMD_SIZE, CMD_TIMEOUT);
        HAL_SPI_Receive(&hspi1, &accelZData[0], READ_CMD_SIZE, CMD_TIMEOUT);
        HAL_SPI_Transmit(&hspi1, &READ_ACCEL_Z_HIGH_CMD, READ_CMD_SIZE, CMD_TIMEOUT);
        HAL_SPI_Receive(&hspi1, &accelZData[1], READ_CMD_SIZE, CMD_TIMEOUT);
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
