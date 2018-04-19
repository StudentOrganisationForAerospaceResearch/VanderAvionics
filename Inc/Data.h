#pragma once

/* Structs containing data primitives */

typedef struct
{
    osMutexId mutex_;
    int32_t accelX_;
    int32_t accelY_;
    int32_t accelZ_;
    int32_t gyroX_;
    int32_t gyroY_;
    int32_t gyroZ_;
    int32_t magnetoX_;
    int32_t magnetoY_;
    int32_t magnetoZ_;
} AccelGyroMagnetismData;


typedef struct
{
    osMutexId mutex_;
    int32_t externalPressure_;
    int32_t externalTemperature_;
} ExternalPressureTemperatureData;

typedef struct
{
    osMutexId mutex_;
    int32_t latitude_;
    int32_t longitude_;
    int32_t altitude_;
    unsigned int epochTimeMsec_;
} GpsData;


typedef struct
{
    osMutexId mutex_;
    float pressure_;
    float temperature_;
} OxidizerTankConditionsData;


/* Data Containers */

typedef struct
{
    AccelGyroMagnetismData* accelGyroMagnetismData_;
    ExternalPressureTemperatureData* externalPressureTemperatureData_;
    GpsData* gpsData_;
    OxidizerTankConditionsData* oxidizerTankConditionsData_;
} AllData;

typedef struct
{
    AccelGyroMagnetismData* accelGyroMagnetismData_;
    ExternalPressureTemperatureData* externalPressureTemperatureData_;
} ParachutesControlData;
