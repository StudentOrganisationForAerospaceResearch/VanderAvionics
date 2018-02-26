#pragma once

/* Structs containing data primitives */

typedef struct {
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

typedef struct {
    int data_;
    osMutexId mutex_;
} PressureData;

typedef struct {
    int data_;
    osMutexId mutex_;
} ExternalTemperatureData;

typedef struct {
    int data_;
    osMutexId mutex_;
} IntegratedTemperatureData;

typedef struct {
    int latitude_;
    int longitude_;
    int altitude_;
    int timeSeconds_;
    osMutexId mutex_;
} GpsData;

typedef struct {
    float pressure_;
    float temperature_;
    osMutexId mutex_;
} OxidizerTankData;

/* Data Containers */

typedef struct {
    AccelGyroMagnetismData* accelGyroMagnetismData_;
    PressureData* pressureData_;
    ExternalTemperatureData* externalTemperatureData_;
    IntegratedTemperatureData* integratedTemperatureData_;
    GpsData* gpsData_;
    OxidizerTankData* oxidizerTankData_;
} AllData;

typedef struct {
    AccelGyroMagnetismData* accelGyroMagnetismData_;
    PressureData* pressureData_;
} MonitorForParachuteData;
