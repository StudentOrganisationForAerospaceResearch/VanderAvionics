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
} AccelGyroMagnetismData;
extern osMutexId accelGyroMagnetismDataMutex;

typedef struct
{
    int externalPressure_;
} ExternalPressureData;
extern osMutexId externalPressureDataMutex;

typedef struct
{
    int externalTemperature_;
} ExternalTemperatureData;
extern osMutexId externalTemperatureDataMutex;

typedef struct
{
    int integratedTemperature_;
} IntegratedTemperatureData;
extern osMutexId gpsDataMutex;

typedef struct
{
    int latitude_;
    int longitude_;
    int altitude_;
    unsigned int epochTimeMsec_;
} GpsData;
extern osMutexId integratedTemperatureDataMutex;

typedef struct
{
    float pressure_;
    float temperature_;
} OxidizerTankConditionsData;
extern osMutexId oxidizerTankConditionsDataMutex;

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
