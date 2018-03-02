#pragma once

/* Structs containing data primitives */

typedef struct
{
    float accelX_;
    float accelY_;
    float accelZ_;
    float gyroX_;
    float gyroY_;
    float gyroZ_;
    float magnetoX_;
    float magnetoY_;
    float magnetoZ_;

    osMutexId mutex_;
} AccelGyroMagnetismData;

typedef struct
{
    int externalPressure_;
    osMutexId mutex_;
} ExternalPressureData;

typedef struct
{
    int externalTemperature_;
    osMutexId mutex_;
} ExternalTemperatureData;

typedef struct
{
    int integratedTemperature_;
    osMutexId mutex_;
} IntegratedTemperatureData;

typedef struct
{
    int latitude_;
    int longitude_;
    int altitude_;
    unsigned int epochTimeMsec_;
    osMutexId mutex_;
} GpsData;

typedef struct
{
    float pressure_;
    float temperature_;
    osMutexId mutex_;
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
} MonitorForParachuteData;
