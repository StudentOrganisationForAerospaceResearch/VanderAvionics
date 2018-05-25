#include <math.h>

#include "stm32f4xx.h"
#include "stm32f4xx_hal_conf.h"
#include "cmsis_os.h"

#include "ParachutesControl.h"
#include "Data.h"
#include "main.h"

static const int MAIN_DEPLOYMENT_ALTITUDE = 1000; //TODO: FIND OUT WHAT THIS IS SUPPOSED TO BE!!! Units in meters.
static int MONITOR_FOR_PARACHUTES_PERIOD = 1000;


struct KalmanStateVector {
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

void filterSensors(int32_t currentAccel, int32_t currentAltitude, int32_t positionVector[3])
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
    oldState - (KalmanStateVector) Past position, velocity and acceleration
    currentAccel - (double) Measured acceleration
    currentAltitude - (double) Measured altitude
    dt - (double) Time since last step
  
  Returns:
    newState - (KalmanStateVector) Current position, velocity and acceleration
*/
struct KalmanStateVector filterSensors(struct KalmanStateVector oldState, int32_t currentAccel, int32_t currentAltitude, double dt) {
    struct KalmanStateVector newState;
    
    // TODO: Need to find the conversion factor for these.
    double accelIn = (double) currentAccel;
    double altIn = (double) currentAltitude; //This should be altitude instead of pressure.
    
    // Propogate old state using simple kinematics equations
    newState.altitude = oldState.position + oldState.velocity*dt + 0.5*dt*dt*oldState.acceleration;
    newState.velocity = oldState.velocity + oldState.acceleration*dt;
    newState.acceleration = oldState.acceleration;
    
    // Calculate the difference between the new state and the measurements
    double baroDifference = altIn - newState.position
    double accelDifference = accelIn - newState.acceleration
    
    // Minimize the chi2 error by means of the Kalman gain matrix
    newState.altitude = newState.altitude + k[0][0]*baroDifference + k[0][1]*accelDifference;
    newState.velocity = newState.velocity + k[1][0]*baroDifference + k[1][1]*accelDifference;
    newState.acceleration = newState.velocity + k[2][0]*baroDifference + k[2][1]*accelDifference;
    
    return newState
}

/*
  Takes an old state vector and current state measurements and 
  converts them into a prediction of the rocket's current state.
  
  Params:
    state - (KalmanStateVector) Current state returned by the Kalman Filter
  
  Returns:
    - (Bool) True if either parachute is deployed.
*/
bool detectApogee(struct KalmanStateVector state)

    if (currentFlightPhase == COAST) {
        // Monitor for when to deploy drogue chute. Simple velocity tolerance, looking for a minimum.
        
        if (state.velocity < 25){
            ejectDrogueParachute();
            currentFlightPhase = DROGUE_DESCENT;
            return 1;
        }
            
    } else if (currentFlightPhase == DROGUE_DESCENT) {
        // Monitor for when to deploy main chute. Simply look for less than desired altitude.
        
        if (positionVector[0] < MAIN_DEPLOYMENT_ALTITUDE) {
            ejectMainParachute();
            currentFlightPhase = MAIN_DESCENT;
            return 1;
        }
            
    }

    return 0;
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
