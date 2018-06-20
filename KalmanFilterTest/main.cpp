#include <fstream>
#include <iostream>
#include <math.h>

#include "../Inc/ParachutesControl.h"

static const int SEA_LEVEL_PRESSURE = 101421.93903699999; //TODO: THIS NEEDS TO BE UPDATED AND RECORDED ON LAUNCH DAY
static const int MAIN_DEPLOYMENT_ALTITUDE = 457 + 1401; // Units in meters. Equivalent of 1500 ft + altitude of spaceport america.
static const double KALMAN_GAIN[][2] =
{
    {0.105553059, 0.109271566},
    {0.0361533034, 0.0661198847},
    {0.000273178915, 0.618030079}
};


struct KalmanStateVector
{
    double altitude;
    double velocity;
    double acceleration;
};

/**
 * Takes an old state vector and current state measurements and
 * converts them into a prediction of the rocket's current state.
 *
 * Params:
 *   oldState - (KalmanStateVector) Past altitude, velocity and acceleration
 *   currentAccel - (double) Measured acceleration
 *   currentAltitude - (double) Measured altitude
 *   dt - (double) Time since last step. In ms.
 *
 * Returns:
 *   newState - (KalmanStateVector) Current altitude, velocity and acceleration
 */
struct KalmanStateVector filterSensors(
    struct KalmanStateVector oldState,
    int32_t currentAccel,
    int32_t currentPressure,
    double dtMillis
)
{
    struct KalmanStateVector newState;

    double accelIn = (double) currentAccel / 1000 * 9.8; // Milli-g -> g -> m/s

    // Convert from 100*millibars to m. This may or may not be right, depending on where you look. Needs testing
    double altIn = (double) 44307.69396 * (1 - pow(currentPressure / SEA_LEVEL_PRESSURE, 0.190284));

    // Convert from ms to s
    double dt = dtMillis / 1000;


    // Propagate old state using simple kinematics equations
    newState.altitude = oldState.altitude + oldState.velocity * dt + 0.5 * dt * dt * oldState.acceleration;
    newState.velocity = oldState.velocity + oldState.acceleration * dt;
    newState.acceleration = oldState.acceleration;

    // Calculate the difference between the new state and the measurements
    double baroDifference = altIn - newState.altitude;
    double accelDifference = accelIn - newState.acceleration;

    // Minimize the chi2 error by means of the Kalman gain matrix
    newState.altitude = newState.altitude + KALMAN_GAIN[0][0] * baroDifference + KALMAN_GAIN[0][1] * accelDifference;
    newState.velocity = newState.velocity + KALMAN_GAIN[1][0] * baroDifference + KALMAN_GAIN[1][1] * accelDifference;
    newState.acceleration = newState.velocity + KALMAN_GAIN[2][0] * baroDifference + KALMAN_GAIN[2][1] * accelDifference;

    return newState;
}

int main(int argc, char const *argv[])
{
    struct KalmanStateVector state;
	std::ifstream infile("thefile.txt");

	int accel, pressure;
	while (infile >> accel >> pressure)
	{
	    // process pair (a,b)
	}

	std::ofstream myfile;
	myfile.open ("example.txt");
	myfile << "Writing this to a file.\n";
	myfile.close();

	return 0;
}