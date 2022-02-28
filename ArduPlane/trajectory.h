#ifndef _RALPHIE_TRAJECTORY_H_
#define _RALPHIE_TRAJECTORY_H_

#include <AP_Math/AP_Math.h>
#include <stdio.h>
#include <cstdio>
#include <math.h>
#include <AP_AHRS/AP_AHRS.h> 
#include "state_types_ralphie.h"


#define WARIO_TRAJECTORY_SIZE   (100)
#define PI (3.14159265)
#define WIND_BUFFER_SIZE (20)


typedef struct {

	float lat;
	float lon;
	float rad;  //convert to gps degrees by /111111. 

	float maxAlt;
	float minAlt;

    float initialAngle;
 
    float targetVelocity;

} warioInput_t;


class RalphieTrajectory {

    /**
     * @brief The current estimation of wind vector
     * 
     */
    Vector3f currentWindEstimate;

    /**
     * @brief The current estimation of wind direction 
     * 
     */
    float currentWindAngleEstimate;

    /**
     * @brief Index to keep track of wind averaging operations
     * 
     */
    uint16_t windBufferIndex=0;

    /**
     * @brief Array of aircraft states representing the trajectory
     * 
     */
    aircraftState_t waypoints[WARIO_TRAJECTORY_SIZE];

    /**
     * @brief Array of wind vectors
     * 
     */
    Vector3f windBuffer[WIND_BUFFER_SIZE];

    /**
     * @brief Number of measurements in the wind buffer 
     * 
     */
    uint16_t windSamples;

    /**
     * @brief Average the current contents of the wind buffer
     * 
     */
    void averageWind();

public:

    /**
     * @brief Generate the default circular trajectory 
     * 
     */
    void init(warioInput_t parameters);

    /**
     * @brief Update the trajectory based on the current wind estimate
     * 
     */
    void update(warioInput_t parameters);

    /**
     * @brief Update the rolling average for the wind estimate
     * 
     * @param newMeasurement 
     */
    void updateAverageWind(Vector3f newMeasurement);

    /**
     * @brief Write over all values in the wind buffer
     * 
     */
    void resetWindAverage();

};


#endif /* _RALPHIE_TRAJECTORY_H_ */
