#pragma once

extern int32_t counter;

struct KalmanStateVector
{
    double altitude;
    double velocity;
    double acceleration;
};

struct KalmanStateVector filterSensors(
    struct KalmanStateVector oldState,
    int32_t currentAccel,
    int32_t currentPressure,
    double dtMillis
);

void parachutesControlTask(void const* arg);

