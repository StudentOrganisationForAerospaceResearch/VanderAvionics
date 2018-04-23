#include "stm32f4xx.h"
#include "stm32f4xx_hal_conf.h"
#include "cmsis_os.h"

#include "ReadAccelGyroMagnetism.h"

#include "Data.h"

static int READ_ACCEL_GYRO_MAGNETISM = 200;

static const int CMD_TIMEOUT = 150;

#define READ_CMD 0x80;
#define WRITE_CMD 0x00;

// Register addresses
#define G1_CTRL_REGISTER_ADDR 0x10 // CTRL_REG1_G (10h)
#define XL6_CTRL_REGISTER_ADDR 0x20 // CTRL_REG6_XL (20h)
#define WHOAMI_REGISTER_ADDR 0x0F // CTRL_REG8 (22h)

#define GYRO_X_G_LOW_REGISTER_ADDR 0x18
#define ACCEL_X_LOW_REGISTER_ADDR 0x28

// Full Commands
static const uint8_t ACTIVATE_GYRO_ACCEL_CMD = G1_CTRL_REGISTER_ADDR | WRITE_CMD;
// 011 11 0 00 -> ODR 119, 2000 DPS
static const uint8_t ACTIVATE_GYRO_ACCEL_DATA = 0x68 | WRITE_CMD;

static const uint8_t SET_ACCEL_SCALE_CMD = XL6_CTRL_REGISTER_ADDR | WRITE_CMD;
// 011 01 0 00 -> ODR 119, +/- 16G
static const uint8_t SET_ACCEL_SCALE_DATA = 0x68 | WRITE_CMD;

static const uint8_t READ_GYRO_X_G_LOW_CMD = GYRO_X_G_LOW_REGISTER_ADDR | READ_CMD;
static const uint8_t READ_ACCEL_X_LOW_CMD = ACCEL_X_LOW_REGISTER_ADDR | READ_CMD;
static const uint8_t READ_WHOAMI_CMD = WHOAMI_REGISTER_ADDR | READ_CMD;

void readAccelGyroMagnetismTask(void const* arg)
{
    AccelGyroMagnetismData* data = (AccelGyroMagnetismData*) arg;
    uint32_t prevWakeTime = osKernelSysTick();

    osDelay(1000);

    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, &READ_GYRO_X_G_LOW_CMD, 1, CMD_TIMEOUT);
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);

    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, &ACTIVATE_GYRO_ACCEL_CMD, 1, CMD_TIMEOUT);
    HAL_SPI_Transmit(&hspi1, &ACTIVATE_GYRO_ACCEL_DATA, 1, CMD_TIMEOUT);
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);

    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, &SET_ACCEL_SCALE_CMD, 1, CMD_TIMEOUT);
    HAL_SPI_Transmit(&hspi1, &SET_ACCEL_SCALE_DATA, 1, CMD_TIMEOUT);
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);

    uint8_t whoami;
    /* Read WHO AM I register for verification, should read 104. */
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, &READ_WHOAMI_CMD, 1, CMD_TIMEOUT);
    HAL_SPI_Receive(&hspi1, &whoami, 1, CMD_TIMEOUT);
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);

    uint8_t dataBuffer[6];
    int16_t accelX, accelY, accelZ;
    int16_t gyroX, gyroY, gyroZ;

    for (;;)
    {
        osDelayUntil(&prevWakeTime, READ_ACCEL_GYRO_MAGNETISM);

        //READ------------------------------------------------------
        HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
        HAL_SPI_Transmit(&hspi1, &READ_GYRO_X_G_LOW_CMD, 1, CMD_TIMEOUT);
        HAL_SPI_Receive(&hspi1, &dataBuffer[0], 6, CMD_TIMEOUT);
        HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
        gyroX = (dataBuffer[1] << 8) | (dataBuffer[0]);
        gyroY = (dataBuffer[3] << 8) | (dataBuffer[2]);
        gyroZ = (dataBuffer[5] << 8) | (dataBuffer[4]);

        HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
        HAL_SPI_Transmit(&hspi1, &READ_ACCEL_X_LOW_CMD, 1, CMD_TIMEOUT);
        HAL_SPI_Receive(&hspi1, &dataBuffer[0], 6, CMD_TIMEOUT);
        HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
        accelX = (dataBuffer[1] << 8) | (dataBuffer[0]);
        accelY = (dataBuffer[3] << 8) | (dataBuffer[2]);
        accelZ = (dataBuffer[5] << 8) | (dataBuffer[4]);


        /* Writeback */
        osMutexWait(data->mutex_, 0);
        data->accelX_ = accelX;
        data->accelY_ = accelY;
        data->accelZ_ = accelZ;
        data->gyroX_ = gyroX;
        data->gyroY_ = gyroY;
        data->gyroZ_ = gyroZ;
        data->magnetoX_ = whoami;
        //data->magnetoY_ = magnetY;
        // data->magnetoZ_ = magnetZ;
        osMutexRelease(data->mutex_);
    }
}
