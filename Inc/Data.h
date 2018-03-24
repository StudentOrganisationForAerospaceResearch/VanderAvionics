#pragma once

/* Structs containing data primitives */

typedef struct
{
    osMutexId mutex_;
    float accelX_;
    float accelY_;
    float accelZ_;
    float gyroX_;
    float gyroY_;
    float gyroZ_;
    float magnetoX_;
    float magnetoY_;
    float magnetoZ_;
} AccelGyroMagnetismData;


typedef struct
{
    osMutexId mutex_;
    int externalPressure_;
} ExternalPressureData;


typedef struct
{
    osMutexId mutex_;
    int externalTemperature_;
} ExternalTemperatureData;


typedef struct
{
    osMutexId mutex_;
    int integratedTemperature_;
} IntegratedTemperatureData;


typedef struct
{
    osMutexId mutex_;
    int latitude_;
    int longitude_;
    int altitude_;
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
    ExternalPressureData* externalPressureData_;
    ExternalTemperatureData* externalTemperatureData_;
    GpsData* gpsData_;
    IntegratedTemperatureData* integratedTemperatureData_;
    OxidizerTankConditionsData* oxidizerTankConditionsData_;
} AllData;

typedef struct
{
    AccelGyroMagnetismData* accelGyroMagnetismData_;
    ExternalPressureData* externalPressureData_;
} ParachutesControlData;
