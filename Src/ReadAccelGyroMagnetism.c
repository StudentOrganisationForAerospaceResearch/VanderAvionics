#include "stm32f4xx.h"
#include "stm32f4xx_hal_conf.h"
#include "cmsis_os.h"

#include "ReadAccelGyroMagnetism.h"

#include "Data.h"

static int READ_ACCEL_GYRO_MAGNETISM = 1000;

static const int READ_CMD_SIZE = 1;
static const int WRITE_CMD_SIZE = 2;
static const int CMD_TIMEOUT = 150;

#define READ_CMD 0x80;
#define WRITE_CMD 0x00;

// Register addresses
#define G1_CTRL_REGISTER_ADDR 0x10 // CTRL_REG1_G (10h)
#define ACCEL_CTRL_REGISTER_5_ADDR 0x1F // CTRL_REG5_XL (1Fh)
#define ACCEL_CTRL_REGISTER_6_ADDR 0x20 // CTRL_REG6_XL (20h)
#define ACCEL_CTRL_REGISTER_7_ADDR 0x21 // CTRL_REG7_XL (21h)
#define GYRO_CTRL_REGISTER_1_ADDR 0x10 // CTRL_REG1_G (10h)
#define CTRL_REGISTER_8_ADDR 0x22 // CTRL_REG8 (22h)
#define WHOAMI_REGISTER_ADDR 0x0F // CTRL_REG8 (22h)

#define ACCELGYRO_X_G_LOW_REGISTER_ADDR 0x18
#define ACCELGYRO_X_G_HIGH_REGISTER_ADDR 0x19

#define ACCEL_X_LOW_REGISTER_ADDR 0x28
#define ACCEL_X_HIGH_REGISTER_ADDR 0x29
#define ACCEL_Y_LOW_REGISTER_ADDR 0x2A
#define ACCEL_Y_HIGH_REGISTER_ADDR 0x2B
#define ACCEL_Z_LOW_REGISTER_ADDR 0x2C
#define ACCEL_Z_HIGH_REGISTER_ADDR 0x2D

#define ACCEL_ODR_FREQ_SETTING 0xC0 // 0b11000000, 238Hz
#define GYRO_ODR_FREQ_SETTING 0x80 // 0b100, 238Hz
#define WIRE_MODE_3_SETTING 0x8

// Full Commands
static const uint8_t ACTIVATE_G1_CMD = G1_CTRL_REGISTER_ADDR;
static const uint8_t ACTIVATE_G1_DATA = 0xC0;

static const uint8_t ACTIVATE_ACCEL5_CMD = ACCEL_CTRL_REGISTER_5_ADDR;
static const uint8_t ACTIVATE_ACCEL5_DATA = 0x34;

static const uint8_t ACTIVATE_ACCEL6_CMD = ACCEL_CTRL_REGISTER_6_ADDR;
static const uint8_t ACTIVATE_ACCEL6_DATA = 0x00;

static const uint8_t ACTIVATE_ACCEL7_CMD = ACCEL_CTRL_REGISTER_7_ADDR;
static const uint8_t ACTIVATE_ACCEL7_DATA = 0x00;

static const uint16_t ACTIVATE_GYRO_CMD = ACCEL_ODR_FREQ_SETTING | GYRO_CTRL_REGISTER_1_ADDR << 8 | WRITE_CMD;

static const uint8_t READ_ACCELGYRO_X_G_LOW_CMD = ACCELGYRO_X_G_LOW_REGISTER_ADDR | READ_CMD;

static const uint8_t READ_ACCEL_X_LOW_CMD = ACCEL_X_LOW_REGISTER_ADDR | READ_CMD;
static const uint8_t READ_ACCEL_X_HIGH_CMD = ACCEL_X_HIGH_REGISTER_ADDR | READ_CMD;
static const uint8_t READ_ACCEL_Y_LOW_CMD = ACCEL_Y_LOW_REGISTER_ADDR | READ_CMD;
static const uint8_t READ_ACCEL_Y_HIGH_CMD = ACCEL_Y_HIGH_REGISTER_ADDR | READ_CMD;
static const uint8_t READ_ACCEL_Z_LOW_CMD = ACCEL_Z_LOW_REGISTER_ADDR | READ_CMD;
static const uint8_t READ_ACCEL_Z_HIGH_CMD = ACCEL_Z_HIGH_REGISTER_ADDR | READ_CMD;
static const uint8_t READ_WHOAMI_CMD = WHOAMI_REGISTER_ADDR | READ_CMD;

uint8_t accelData[12];
uint16_t accelX, accelY, accelZ;
uint8_t gyroX, gyroY, gyroZ;
uint8_t magnetX, magnetY, magnetZ;
uint8_t whoami;

void readAccelGyroMagnetismTask(void const* arg)
{
    AccelGyroMagnetismData* data = (AccelGyroMagnetismData*) arg;
    uint32_t prevWakeTime = osKernelSysTick();

    /* Accelerometer and gyroscope active mode on */
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, &ACTIVATE_G1_CMD, 1, CMD_TIMEOUT);
    HAL_SPI_Transmit(&hspi1, &ACTIVATE_G1_DATA, 1, CMD_TIMEOUT);
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);

    // HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
    // HAL_SPI_Transmit(&hspi1, &ACTIVATE_ACCEL6_CMD, 1, CMD_TIMEOUT);
    // HAL_SPI_Transmit(&hspi1, &ACTIVATE_ACCEL6_DATA, 1, CMD_TIMEOUT);
    // HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);

    // HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
    // HAL_SPI_Transmit(&hspi1, &ACTIVATE_ACCEL7_CMD, 1, CMD_TIMEOUT);
    // HAL_SPI_Transmit(&hspi1, &ACTIVATE_ACCEL7_DATA, 1, CMD_TIMEOUT);
    // HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);

    // HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
    // HAL_SPI_Transmit(&hspi1, &ACTIVATE_GYRO_CMD, WRITE_CMD_SIZE, CMD_TIMEOUT);
    // osDelay(3);
    // HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);

    /* Set up 3 wire mode */
    // HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
    // HAL_SPI_Transmit(&hspi1, &ACTIVATE_3_WIRE_MODE_CMD, WRITE_CMD_SIZE, CMD_TIMEOUT);
    // osDelay(3);

    for (;;)
    {
        osDelayUntil(&prevWakeTime, READ_ACCEL_GYRO_MAGNETISM);

        //READ------------------------------------------------------
        accelX = 0;
        accelY = 0;
        accelZ = 0;

        /* Get values for X axis */
        HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
        HAL_SPI_Transmit(&hspi1, &READ_ACCELGYRO_X_G_LOW_CMD, READ_CMD_SIZE, CMD_TIMEOUT);
        HAL_SPI_Receive(&hspi1, accelData, 12, CMD_TIMEOUT);
        HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
        gyroX += ((uint16_t)(accelData[1]) << 8) | (accelData[0]); 
        gyroY += ((uint16_t)(accelData[3] << 8)) | ((uint16_t)accelData[2]); 
        gyroZ += ((uint16_t)(accelData[5] << 8)) | ((uint16_t)accelData[4]); 

        accelX += ((uint16_t)(accelData[7] << 8)) | ((uint16_t)accelData[6]);  // Turn the MSB and LSB into a signed 16-bit value
        accelY += ((uint16_t)(accelData[9] << 8)) | ((uint16_t)accelData[8]);
        accelZ += ((uint16_t)(accelData[11] << 8)) | ((uint16_t)accelData[10]);  

        // HAL_SPI_Transmit(&hspi1, &READ_ACCEL_X_LOW_CMD, READ_CMD_SIZE, CMD_TIMEOUT);
        // HAL_SPI_Receive(&hspi1, &accelXData[0], READ_CMD_SIZE, CMD_TIMEOUT);
        // HAL_SPI_Transmit(&hspi1, &READ_ACCEL_X_HIGH_CMD, READ_CMD_SIZE, CMD_TIMEOUT);
        // HAL_SPI_Receive(&hspi1, &accelXData[1], READ_CMD_SIZE, CMD_TIMEOUT);
        // accelX += ((int16_t)accelXData[1] << 8) | accelXData[0];  // Turn the MSB and LSB into a signed 16-bit value

        // HAL_SPI_Transmit(&hspi1, &READ_ACCEL_X_LOW_CMD, READ_CMD_SIZE, CMD_TIMEOUT);
        // HAL_SPI_Receive(&hspi1, &accelXData[0], READ_CMD_SIZE, CMD_TIMEOUT);
        // HAL_SPI_Transmit(&hspi1, &READ_ACCEL_X_HIGH_CMD, READ_CMD_SIZE, CMD_TIMEOUT);
        // HAL_SPI_Receive(&hspi1, &accelXData[1], READ_CMD_SIZE, CMD_TIMEOUT);
        // accelX += ((int16_t)accelXData[1] << 8) | accelXData[0];  // Turn the MSB and LSB into a signed 16-bit value

        // /* Get values for Y axis */
        // HAL_SPI_Transmit(&hspi1, &READ_ACCEL_Y_LOW_CMD, READ_CMD_SIZE, CMD_TIMEOUT);
        // HAL_SPI_Receive(&hspi1, &accelYData[0], READ_CMD_SIZE, CMD_TIMEOUT);
        // HAL_SPI_Transmit(&hspi1, &READ_ACCEL_Y_HIGH_CMD, READ_CMD_SIZE, CMD_TIMEOUT);
        // HAL_SPI_Receive(&hspi1, &accelYData[1], READ_CMD_SIZE, CMD_TIMEOUT);
        // accelY += ((int16_t)accelYData[1] << 8) | accelYData[0];  // Turn the MSB and LSB into a signed 16-bit value

        // HAL_SPI_Transmit(&hspi1, &READ_ACCEL_Y_LOW_CMD, READ_CMD_SIZE, CMD_TIMEOUT);
        // HAL_SPI_Receive(&hspi1, &accelYData[0], READ_CMD_SIZE, CMD_TIMEOUT);
        // HAL_SPI_Transmit(&hspi1, &READ_ACCEL_Y_HIGH_CMD, READ_CMD_SIZE, CMD_TIMEOUT);
        // HAL_SPI_Receive(&hspi1, &accelYData[1], READ_CMD_SIZE, CMD_TIMEOUT);
        // accelY += ((int16_t)accelYData[1] << 8) | accelYData[0];  // Turn the MSB and LSB into a signed 16-bit value

        // HAL_SPI_Transmit(&hspi1, &READ_ACCEL_Y_LOW_CMD, READ_CMD_SIZE, CMD_TIMEOUT);
        // HAL_SPI_Receive(&hspi1, &accelYData[0], READ_CMD_SIZE, CMD_TIMEOUT);
        // HAL_SPI_Transmit(&hspi1, &READ_ACCEL_Y_HIGH_CMD, READ_CMD_SIZE, CMD_TIMEOUT);
        // HAL_SPI_Receive(&hspi1, &accelYData[1], READ_CMD_SIZE, CMD_TIMEOUT);
        // accelY += ((int16_t)accelYData[1] << 8) | accelYData[0];  // Turn the MSB and LSB into a signed 16-bit value

        // /* Get values for Z axis */
        // HAL_SPI_Transmit(&hspi1, &READ_ACCEL_Z_LOW_CMD, READ_CMD_SIZE, CMD_TIMEOUT);
        // HAL_SPI_Receive(&hspi1, &accelZData[0], READ_CMD_SIZE, CMD_TIMEOUT);
        // HAL_SPI_Transmit(&hspi1, &READ_ACCEL_Z_HIGH_CMD, READ_CMD_SIZE, CMD_TIMEOUT);
        // HAL_SPI_Receive(&hspi1, &accelZData[1], READ_CMD_SIZE, CMD_TIMEOUT);
        // accelZ += ((int16_t)accelZData[1] << 8) | accelZData[0];  // Turn the MSB and LSB into a signed 16-bit value

        // HAL_SPI_Transmit(&hspi1, &READ_ACCEL_Z_LOW_CMD, READ_CMD_SIZE, CMD_TIMEOUT);
        // HAL_SPI_Receive(&hspi1, &accelZData[0], READ_CMD_SIZE, CMD_TIMEOUT);
        // HAL_SPI_Transmit(&hspi1, &READ_ACCEL_Z_HIGH_CMD, READ_CMD_SIZE, CMD_TIMEOUT);
        // HAL_SPI_Receive(&hspi1, &accelZData[1], READ_CMD_SIZE, CMD_TIMEOUT);
        // accelZ += ((int16_t)accelZData[1] << 8) | accelZData[0];  // Turn the MSB and LSB into a signed 16-bit value

        // HAL_SPI_Transmit(&hspi1, &READ_ACCEL_Z_LOW_CMD, READ_CMD_SIZE, CMD_TIMEOUT);
        // HAL_SPI_Receive(&hspi1, &accelZData[0], READ_CMD_SIZE, CMD_TIMEOUT);
        // HAL_SPI_Transmit(&hspi1, &READ_ACCEL_Z_HIGH_CMD, READ_CMD_SIZE, CMD_TIMEOUT);
        // HAL_SPI_Receive(&hspi1, &accelZData[1], READ_CMD_SIZE, CMD_TIMEOUT);
        // accelZ += ((int16_t)accelZData[1] << 8) | accelZData[0];  // Turn the MSB and LSB into a signed 16-bit value

        /* Average the values */
        // accelX /= 3;
        // accelY /= 3;
        // accelZ /= 3;

        HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
        HAL_SPI_Transmit(&hspi1, &READ_WHOAMI_CMD, READ_CMD_SIZE, CMD_TIMEOUT);
        HAL_SPI_Receive(&hspi1, &whoami, READ_CMD_SIZE, CMD_TIMEOUT);
        HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);

        /* Writeback */
        osMutexWait(data->mutex_, 0);
        data->accelX_ = accelX;
        data->accelY_ = accelY;
        data->accelZ_ = accelZ;
        data->gyroX_ = gyroX;
        data->gyroY_ = gyroY;
        data->gyroZ_ = gyroZ;
        data->magnetoX_ = whoami;
        // data->magnetoY_ = magnetY;
        // data->magnetoZ_ = magnetZ;
        osMutexRelease(data->mutex_);
    }
}
