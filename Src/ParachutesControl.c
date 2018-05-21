#include <math.h>

#include "stm32f4xx.h"
#include "stm32f4xx_hal_conf.h"
#include "cmsis_os.h"

#include "ParachutesControl.h"
#include "Data.h"
#include "main.h"

static int MONITOR_FOR_PARACHUTES_PERIOD = 1000;
static double velocity_buffer = new double[6];


struct Vector {
    double altitude;
    double velocity;
    double acceleration;
}

int32_t readAccel(AccelGyroMagnetismData* data)
{
    if (osMutexWait(data->mutex_, 0) != osOK)
    {
        return -1;
    }

    int32_t accelX = data->accelX_;
    int32_t accelY = data->accelY_;
    int32_t accelZ = data->accelZ_;
    osMutexRelease(data->mutex_);

    int32_t accelMagnitude =
        sqrt(
            accelX * accelX +
            accelY * accelY +
            accelZ * accelZ
        );

    return accelMagnitude;
}

int32_t readPressure(BarometerData* data)
{
    if (osMutexWait(data->mutex_, 0) != osOK)
    {
        return -1;
    }

    int32_t pressure = data->pressure_;
    osMutexRelease(data->mutex_);

    return (int32_t)pressure;
}

void filterSensors(int32_t current_accel, int32_t current_pressure, int32_t positionVector[3])
{
    // TODO
    positionVector[0] = 0;
    positionVector[1] = 0;
    positionVector[2] = 0;
}

/*
  Takes an old state vector and current state measurements and 
  converts them into a prediction of the rocket's current state.
  
  Params:
    old_state - (Vector) Past position, velocity and acceleration
    accel_in - (double) Measured acceleration
    alt_in - (double) Measured altitude
    dt - (double) Time since last step
  
  Returns:
    new_state - (Vector) Current position, velocity and acceleration
*/
struct Vector filterSensors(struct Vector old_state, int32_t current_accel, int32_t current_pressure, double dt) {
    struct Vector new_state;
    
    // TODO: Need to find the conversion factor for these.
    double accel_in = (double) current_accel;
    double alt_in = (double) current_pressure;
    
    // Propogate old state using simple kinematics equations
    new_state.altitude = old_state.position + old_state.velocity*dt + 0.5*dt*dt*old_state.acceleration;
    new_state.velocity = old_state.velocity + old_state.acceleration*dt;
    new_state.acceleration = old_state.acceleration;
    
    // Calculate the difference between the new state and the measurements
    double baro_difference = alt_in - new_state.position
    double accel_difference = accel_in - new_state.acceleration
    
    // Minimize the chi2 error by means of the Kalman gain matrix
    new_state.altitude = new_state.altitude + k[0][0]*baro_difference + k[0][1]*baro_difference;
    new_state.velocity = new_state.velocity + k[1][0]*baro_difference + k[1][1]*baro_difference;
    new_state.acceleration = new_state.velocity + k[2][0]*baro_difference + k[2][1]*baro_difference;
    
    return new_state
}

// Warning: the states there are total BS for the moment. They're only placeholders.
bool detectApogee(int32_t positionVector[3])
{
    double velocity = (double) positionVector[1]; // Convert this!

    if (currentFlightPhase == PRELAUNCH) {

        if (positionVector[1] > 100)
            currentFlightPhase = BURNING;
        
        suffleBuffer(velocity);
        
    } else if (currentFlightPhase == BURNING) {
        
        if (((velocity_buffer[0] + velocity_buffer[1] + velocity_buffer[2]) - 
            (velocity_buffer[3] + velocity_buffer[4] + velocity_buffer[5])) > 1)
            currentFlightPhase = AFTER_BURN;
            
        suffleBuffer(velocity);
        
    } else if (currentFlightPhase == AFTER_BURN) {
        
        if (positionVector[1] > 100){
            currentFlightPhase = DROGUE_DESCENT;
            return 1;
        }
            
    }

    return 0;
}

void suffleBuffer(double velocity) {
    velocity_buffer[0] = velocity_buffer[1];
    velocity_buffer[1] = velocity_buffer[2];
    velocity_buffer[2] = velocity_buffer[3];
    velocity_buffer[3] = velocity_buffer[4];
    velocity_buffer[4] = velocity_buffer[5];
    velocity_buffer[5] = positionVector[1];
}

void ejectDrogueParachute()
{
    // TODO
}

void ejectMainParachute()
{
    // TODO
}

/**
 * This routine just waits for the currentFlightPhase to get out of PRELAUNCH
 */
void parachutesControlPrelaunchRoutine()
{
    uint32_t prevWakeTime = osKernelSysTick();

    for (;;)
    {
        osDelayUntil(&prevWakeTime, MONITOR_FOR_PARACHUTES_PERIOD);

        if (currentFlightPhase > PRELAUNCH)
        {
            // Ascent has begun
            return;
        }
    }
}


/**
 * This routine monitors for apogee.
 * Once apogee has been detected,
 * eject the drogue parachute and update the currentFlightPhase.
 */
void parachutesControlAscentRoutine(
    AccelGyroMagnetismData* accelGyroMagnetismData,
    BarometerData* barometerData
)
{
    uint32_t prevWakeTime = osKernelSysTick();
    int32_t positionVector[3];

    for (;;)
    {
        osDelayUntil(&prevWakeTime, MONITOR_FOR_PARACHUTES_PERIOD);

        int32_t currentAccel = readAccel(accelGyroMagnetismData);
        int32_t currentPressure = readPressure(barometerData);

        if (currentAccel == -1 || currentPressure == -1)
        {
            // failed to read values
            continue;
        }

        filterSensors(currentAccel, currentPressure, positionVector);

        if (detectApogee(positionVector))
        {
            ejectDrogueParachute();
            currentFlightPhase = DROGUE_DESCENT;
            return;
        }
    }
}

/**
 * This routine detects reaching a certain altitude
 * Once that altitude has been reached, eject the main parachute
 * and update the currentFlightPhase.
 */
void parachutesControlDrogueDescentRoutine()
{
    uint32_t prevWakeTime = osKernelSysTick();

    for (;;)
    {
        osDelayUntil(&prevWakeTime, MONITOR_FOR_PARACHUTES_PERIOD);

        // TODO
        // detect 4600 ft above sea level and eject main parachute
        if (0)
        {

            ejectMainParachute();
            currentFlightPhase = MAIN_DESCENT;
            return;
        }
    }
}

/**
 * This routine does nothing since there is nothing left to do
 * after the main parachute has been launched.
 */
void parachutesControlMainDescentRoutine()
{
    for (;;)
    {
        osThreadSuspend(osThreadGetId());
    }
}

void parachutesControlTask(void const* arg)
{
    ParachutesControlData* data = (ParachutesControlData*) arg;

    for (;;)
    {
        switch (currentFlightPhase)
        {
            case PRELAUNCH:
                parachutesControlPrelaunchRoutine();
                break;

            case BURN: // fall through
            case COAST:
                parachutesControlAscentRoutine(
                    data->accelGyroMagnetismData_,
                    data->barometerData_
                );
                break;

            case DROGUE_DESCENT:
                parachutesControlDrogueDescentRoutine();
                break;

            case MAIN_DESCENT:
                parachutesControlMainDescentRoutine();
                break;

            default:
                break;
        }
    }
}
