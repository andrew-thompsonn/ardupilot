#ifndef _RALPHIE_TRAJECTORY_H_
#define _RALPHIE_TRAJECTORY_H_



#include <AP_Math/AP_Math.h>
#include <stdio.h>
#include <cstdio>
#include <math.h>
#include <AP_AHRS/AP_AHRS.h> 
#include "state_types_ralphie.h"


#define WARIO_TRAJECTORY_SIZE   (100)
#define wpOnFirstQuarterCircle  (5)
#define wpOnSecondQuarterCircle  (5)
#define wpOnFirstStraightLine (40)
#define wpOnSecondStraightLine  (40)
#define wpOnHalfCircle  (10)


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

    /**
     * @brief Array of aircraft states representing the trajectory
     * 
     */
    aircraftState_t waypointsSquircle[WARIO_TRAJECTORY_SIZE];

     /**
     * @brief Array of aircraft states representing the trajectory of the transition
     * 
     */
    aircraftState_t waypointsTransition[WARIO_TRAJECTORY_SIZE];

    /**
     * @brief Array of aircraft states representing the trajectory of the transition
     * 
     */
    aircraftState_t waypointsRotated[WARIO_TRAJECTORY_SIZE];





public:

    /**
     * @brief Generate the default circular trajectory 
     * 
     */
    void initCircle(warioInput_t parameters);

        /**
     * @brief Generate the default circular trajectory 
     * 
     */
    void initSquircle(warioInput_t parameters);

    /**
     * @brief Update the trajectory based on the current wind estimate
     * 
     */
    void updatePath(warioInput_t parameters, Vector3f windEstimate);

     /**
     * @brief Update the transition based on wind direction and previous wind direction
     * 
     */
    void updateTransition(warioInput_t parameters, Vector3f windEstimate, Vector3f pastWindEstimate);

    /**
     * @brief Set the current wind estimate
     * 
     * @param windEstimate 
     */
    void setCurrentWind(Vector3f windEstimate);


    // Define init circle variables and arrays. 

    float deltaAngle;
    float angles[WARIO_TRAJECTORY_SIZE];


    //write vectors of positions over path
    float xPos[WARIO_TRAJECTORY_SIZE];
    float  yPos[WARIO_TRAJECTORY_SIZE];
    float cRoll;

    // Define init Squircle variables and arrays. 
    float radiusFactor;
    float majorAxis;
    float minorAxis;

    //calculate total distance of shape. 
    float circleDistance;
    float straightDistance;
    float totalDistance;

    //Distance between waypoints
    float distanceBetweenWaypoints;
    //printf("\nfloor test\n");

    //calculate percentage of waypoints per section. 

    //calculate range of angles vector
    float angleFirstQuarterCircle[wpOnFirstQuarterCircle];
    float angleSecondQuarterCircle[wpOnSecondQuarterCircle];
    float angleHalfCircle[wpOnHalfCircle];


    float xFirstQuarterCircle[wpOnFirstQuarterCircle];
    float xSecondQuarterCircle[wpOnSecondQuarterCircle];
    float xHalfCircle[wpOnHalfCircle];
    float xFirstStraightLine[wpOnFirstStraightLine];
    float xSecondStraightLine[wpOnSecondStraightLine];

    float yFirstQuarterCircle[wpOnFirstQuarterCircle];
    float ySecondQuarterCircle[wpOnSecondQuarterCircle];
    float yHalfCircle[wpOnHalfCircle];
    float yFirstStraightLine[wpOnFirstStraightLine];
    float ySecondStraightLine[wpOnSecondStraightLine];

    //calculate delta range of angles needed. 
    float deltaAngleFirstQuarterCircle;
    float deltaAngleSecondQuarterCircle;
    float deltaAngleHalfCircle;

    float xPosition[WARIO_TRAJECTORY_SIZE];
    float yPosition[WARIO_TRAJECTORY_SIZE];
    float yDeltaFirstStraight;
    float yDeltaSecondStraight;
    float cRollS;

    //Define squircle rotation parameters
    float rotationAngle;

    //Define transition path parameters
    float startAngle;
    float finalAngle;
    float transitionSize;
    float currentAngle;

};


#endif /* _RALPHIE_TRAJECTORY_H_ */
