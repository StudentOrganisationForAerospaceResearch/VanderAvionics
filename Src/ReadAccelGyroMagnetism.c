#include "stm32f4xx.h"
#include "stm32f4xx_hal_conf.h"
#include "cmsis_os.h"

#include "ReadAccelGyroMagnetism.h"

#include "Data.h"

static int READ_ACCEL_GYRO_MAGNETISM = 1000;

static const int READ_CMD_SIZE = 1;
static const int WRITE_CMD_SIZE = 2;
static const int CMD_TIMEOUT = 150;

#define READ_CMD 0x1;
#define WRITE_CMD 0x0;

// Register addresses
#define ACCEL_CTRL_REGISTER_6_ADDR 0x20 // CTRL_REG6_XL (20h)
#define GYRO_CTRL_REGISTER_1_ADDR 0x10 // CTRL_REG1_G (10h)
#define CTRL_REGISTER_8_ADDR 0x22 // CTRL_REG8 (22h)

#define ACCEL_X_LOW_REGISTER_ADDR 0x28
#define ACCEL_X_HIGH_REGISTER_ADDR 0x29
#define ACCEL_Y_LOW_REGISTER_ADDR 0x2A
#define ACCEL_Y_HIGH_REGISTER_ADDR 0x2B
#define ACCEL_Z_LOW_REGISTER_ADDR 0x2C
#define ACCEL_Z_HIGH_REGISTER_ADDR 0x2D

#define ACCEL_ODR_FREQ_SETTING 0x4 // 0b100, 238Hz
#define GYRO_ODR_FREQ_SETTING 0x4 // 0b100, 238Hz
#define WIRE_MODE_3_SETTING 0x8

// Full Commands
static const uint16_t ACTIVATE_ACCEL_CMD = ACCEL_ODR_FREQ_SETTING << 8 | ACCEL_CTRL_REGISTER_6_ADDR << 1 | WRITE_CMD;
static const uint16_t ACTIVATE_GYRO_CMD = ACCEL_ODR_FREQ_SETTING << 8 | GYRO_CTRL_REGISTER_1_ADDR << 1 | WRITE_CMD;
static const uint16_t ACTIVATE_3_WIRE_MODE_CMD = WIRE_MODE_3_SETTING << 8 | CTRL_REGISTER_8_ADDR << 1 | WRITE_CMD;

static const uint8_t READ_ACCEL_X_LOW_CMD = ACCEL_X_LOW_REGISTER_ADDR << 1 | READ_CMD;
static const uint8_t READ_ACCEL_X_HIGH_CMD = ACCEL_X_HIGH_REGISTER_ADDR << 1 | READ_CMD;
static const uint8_t READ_ACCEL_Y_LOW_CMD = ACCEL_Y_LOW_REGISTER_ADDR << 1 | READ_CMD;
static const uint8_t READ_ACCEL_Y_HIGH_CMD = ACCEL_Y_HIGH_REGISTER_ADDR << 1 | READ_CMD;
static const uint8_t READ_ACCEL_Z_LOW_CMD = ACCEL_Z_LOW_REGISTER_ADDR << 1 | READ_CMD;
static const uint8_t READ_ACCEL_Z_HIGH_CMD = ACCEL_Z_HIGH_REGISTER_ADDR << 1 | READ_CMD;

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

        HAL_GPIO_WritePin(XL_CS_GPIO_Port, XL_CS_Pin, GPIO_PIN_RESET);
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
        // data->gyroX_ = whoami;
        // data->gyroY_ = gyroY;
        // data->gyroZ_ = gyroZ;
        // data->magnetoX_ = magnetX;
        // data->magnetoY_ = magnetY;
        // data->magnetoZ_ = magnetZ;
        osMutexRelease(data->mutex_);
    }
}
