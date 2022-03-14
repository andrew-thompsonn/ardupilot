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
#define WIND_BUFFER_SIZE (20)
#define DegreeAdjustment (111139)

#define ROTATION_THRESHOLD (15)*PI/180 /* deg */


typedef struct {

	float lat;
	float lon;
	float rad;  //convert to gps degrees by /111111. 

	float maxAlt;
	float minAlt;

    float initialAngle;
 
    float targetVelocity;

} warioInput_t;



typedef enum {

    STATE_CIRCLING,
    STATE_NORMAL,
    STATE_TRANSITIONING
} warioEngineState_t;


class RalphieTrajectory {
    public:
    /*
    *Initial Parameter creation for use in trajectory functions
    */
    warioInput_t parameters;

    //set initial location home
    Location TrajectoryHome;

    /**
     *  The current estimation of wind vector
     * 
     * 
     */
    Vector3f currentWindEstimate;

    /**
     *  The current estimation of wind direction 
     * 
     */
    float currentWindAngleEstimate;

    /**
     * Index to keep track of wind averaging operations
     * 
     */
    uint16_t windBufferIndex=0;

    /**
     * Array of aircraft states representing the trajectory
     * 
     */
    aircraftState_t waypoints[WARIO_TRAJECTORY_SIZE];

     /**
     * Array of aircraft states representing the trajectory in location class
     * 
     */
    Location circleWaypointsLoc[WARIO_TRAJECTORY_SIZE];

    /**
     * Array of aircraft states representing the trajectory
     * 
     */
    aircraftState_t waypointsSquircle[WARIO_TRAJECTORY_SIZE];

    /**
     * Array of aircraft states representing the trajectory
     * non-rotated squircle array in Location class. 
     */
    Location waypointsSquircleLoc[WARIO_TRAJECTORY_SIZE];

     /**
     * Array of aircraft states representing the trajectory of the transition
     * 
     */
    aircraftState_t waypointsTransition[WARIO_TRAJECTORY_SIZE];

     /**
     * Array of aircraft states representing the trajectory of the transition
     * transition waypoints in location array
     */
    Location waypointsTransitionLoc[WARIO_TRAJECTORY_SIZE];

    /**
     * Array of aircraft states representing the trajectory of the transition
     * 
     */
    aircraftState_t waypointsRotated[WARIO_TRAJECTORY_SIZE];

    /**
     * Array of aircraft states representing the trajectory of the transition
     * Rotated squircle waypoints in location form
     */
    Location waypointsRotatedLoc[WARIO_TRAJECTORY_SIZE];


     // array of wind buffer values
    Vector3f windBuffer[WIND_BUFFER_SIZE];

    /**
     *  Number of measurements in the wind buffer 
     * 
     */
    uint16_t windSamples;

    /**
     *  Average the current contents of the wind buffer
     * 
     */
    void averageWind();

    /**
     * @brief array of location waypoints for integration with auto controller. 
     * 
     */

    flightPhase_t phases[WARIO_TRAJECTORY_SIZE];


    /**
     *  Generate the default circular trajectory 
     * 
     */
    void initCircle();

        /**
     * Generate the default circular trajectory 
     * 
     */
    void initSquircle();

    /**
     * @brief Update the trajectory based on the current wind estimate
     * // TODO: convert parameters to member variable (store in class)
     */
    void updatePath();

     /**
     * @brief Update the transition based on wind direction and previous wind direction
     * // TODO: convert parameters to member variable (store in class)
     */
    void updateTransition();

  
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

     /**
     * @brief function used to convert calculated trajectories into arrays of data in Location class form for wp allocation. 
     * 
     */
    void convertWaypointsToLocations(Location locations[], aircraftState_t states[]);

    void update();

    flightPhase_t fillNextWaypoint(Location &prev_WP_loc, Location current_loc, Location &next_WP_loc);

    uint8_t currentWaypointIndex;

    void init(Location home);

    bool needToTransition;
    bool transitioning;

    void printState();
    void getFirstWaypoint(Location &location);

    warioEngineState_t state;




    // Define init circle variables and arrays. 

    float deltaAngle;
    float angles[WARIO_TRAJECTORY_SIZE];


    //write vectors of positions over path
    float xPos[WARIO_TRAJECTORY_SIZE];
    float yPos[WARIO_TRAJECTORY_SIZE];
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

  

    //Define transition path parameters
    float startAngle;
    float finalAngle;
    float transitionSize;
    float currentAngle;

    uint8_t transitionPathSize;

    float currentPathDirection; /* Angle in radians CCW from north */

};


#endif /* _RALPHIE_TRAJECTORY_H_ */
