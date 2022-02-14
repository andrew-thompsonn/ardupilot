#ifndef _RALPHIE_TRAJECTORY_H_
#define _RALPHIE_TRAJECTORY_H_

#include "state_task.h"
#include <AP_Math/AP_Math.h>
#include <stdio.h>
#include <cstdio>
#include <math.h>
#include <AP_AHRS/AP_AHRS.h> 
#include "state_types_ralphie.h"


#define WARIO_TRAJECTORY_SIZE   (100)
#define PI (3.14159265)
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
     * @brief The current estimation of wind direction
     * 
     */
    Vector3f currentWindEstimate;

    /**
     * @brief Array of aircraft states representing the trajectory
     * 
     */
    aircraftState_t waypoints[WARIO_TRAJECTORY_SIZE];



public:

    /**76
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
     * @brief Set the current wind estimate
     * 
     * @param windEstimate 
     */
    void setCurrentWind(Vector3f windEstimate);

};


#endif /* _RALPHIE_TRAJECTORY_H_ */
