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
#define CTRL_REGISTER_8_ADDR 0x22 // CTRL_REG8 (22h)
#define WHOAMI_REGISTER_ADDR 0x0F // CTRL_REG8 (22h)

#define GYRO_X_G_LOW_REGISTER_ADDR 0x18
#define ACCEL_X_LOW_REGISTER_ADDR 0x28

#define CTRL_REGISTER6_ADDR 0x20 // CTRL_REG6_XL (20h)

// Full Commands
static const uint8_t ACTIVATE_REG8_CMD = CTRL_REGISTER_8_ADDR;
static const uint8_t ACTIVATE_REG8_DATA = 0x8C;

static const uint8_t ACTIVATE_G1_CMD = G1_CTRL_REGISTER_ADDR;
static const uint8_t ACTIVATE_G1_DATA = 0x68;

static const uint8_t ACTIVATE_REG6_CMD = CTRL_REGISTER6_ADDR;
static const uint8_t ACTIVATE_REG6_DATA = 0x00;

static const uint8_t READ_GYRO_X_G_LOW_CMD = GYRO_X_G_LOW_REGISTER_ADDR | READ_CMD;
static const uint8_t READ_ACCEL_X_LOW_CMD = ACCEL_X_LOW_REGISTER_ADDR | READ_CMD;
// static const uint8_t READ_WHOAMI_CMD = WHOAMI_REGISTER_ADDR | READ_CMD;

uint8_t accelData[6], gyroData[6], temp[30];
int16_t accelX, accelY, accelZ;
int16_t gyroX, gyroY, gyroZ;
int16_t magnetX, magnetY, magnetZ;
uint8_t whoami;

void readAccelGyroMagnetismTask(void const* arg)
{
    AccelGyroMagnetismData* data = (AccelGyroMagnetismData*) arg;
    uint32_t prevWakeTime = osKernelSysTick();
    int counter = 0;
    int whoami = 0;

    osDelayUntil(&prevWakeTime, READ_ACCEL_GYRO_MAGNETISM);

    // HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
    // HAL_SPI_Transmit(&hspi1, &ACTIVATE_REG6_CMD, 1, CMD_TIMEOUT);
    // HAL_SPI_Transmit(&hspi1, &ACTIVATE_REG6_DATA, 1, CMD_TIMEOUT);
    // HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);

    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, &READ_GYRO_X_G_LOW_CMD, 1, CMD_TIMEOUT);
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);

    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, &ACTIVATE_G1_CMD, 1, CMD_TIMEOUT);
    HAL_SPI_Transmit(&hspi1, &ACTIVATE_G1_DATA, 1, CMD_TIMEOUT);
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);

    /* Read WHO AM I register for verifcation, should read 104. */
    // HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
    // HAL_SPI_Transmit(&hspi1, &READ_WHOAMI_CMD, 1, CMD_TIMEOUT);
    // HAL_SPI_Receive(&hspi1, &whoami, 1, CMD_TIMEOUT);
    // HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);

    for (;;)
    {
        osDelayUntil(&prevWakeTime, READ_ACCEL_GYRO_MAGNETISM);

        //READ------------------------------------------------------
        HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
        HAL_SPI_Transmit(&hspi1, &READ_GYRO_X_G_LOW_CMD, 1, CMD_TIMEOUT);
        HAL_SPI_Receive(&hspi1, &gyroData[0], 6, CMD_TIMEOUT);
        HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
        gyroX = (gyroData[1] << 8) | (gyroData[0]);
        gyroY = (gyroData[3] << 8) | (gyroData[2]);
        gyroZ = (gyroData[5] << 8) | (gyroData[4]);

        HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
        HAL_SPI_Transmit(&hspi1, &READ_ACCEL_X_LOW_CMD, 1, CMD_TIMEOUT);
        HAL_SPI_Receive(&hspi1, &accelData[0], 6, CMD_TIMEOUT);
        HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
        accelX = (accelData[1] << 8) | (accelData[0]);
        accelY = (accelData[3] << 8) | (accelData[2]);
        accelZ = (accelData[5] << 8) | (accelData[4]);


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
