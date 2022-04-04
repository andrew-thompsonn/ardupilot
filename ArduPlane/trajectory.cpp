#include "trajectory.h"


void RalphieTrajectory::init(Location home) {
    TrajectoryHome = home;
    finalAngle = 0;
    startAngle = 0;
    currentWaypointIndex = 0;
    transitionPathSize = 0;
    needToTransition = false;
    transitioning = false;
    parameters.rad = 500;
    parameters.maxAlt = 100;
    parameters.minAlt = 100; 
    parameters.initialAngle = 0;
    parameters.lat = 0;
    parameters.lon = 0;
    parameters.targetVelocity = 20; 

    state = STATE_CIRCLING;

    initCircle();
    initSquircle();
}


void RalphieTrajectory::getFirstWaypoint(Location &location) {

    location = circleWaypointsLoc[currentWaypointIndex++];
}

void RalphieTrajectory::initCircle() {
    //generate initial circular trajectory based off of given radius and central location. 
    angles[0] = parameters.initialAngle;
    cRoll = atan2f(powf(parameters.targetVelocity,2),(parameters.rad * 9.81));
    deltaAngle = (2*PI)/WARIO_TRAJECTORY_SIZE;
    //generate vector of all angles
    for (int i = 1; i < WARIO_TRAJECTORY_SIZE; i++){
       angles[i] = angles[i-1] + deltaAngle;
      // printf("Current Angle: %.3f", angles[i]);
    }
    
    for (int i = 0; i < WARIO_TRAJECTORY_SIZE; i++)
    {
        xPos[i] = (parameters.rad * -sinf(angles[i])) + parameters.lat;
        if (angles[i]<(PI/2)){
           yPos[i] = (parameters.lon + sqrtf((-(powf(parameters.lat,2)) + (powf(parameters.rad,2)) + 2*parameters.lat*xPos[i] - powf(xPos[i],2) )));
        }else if (angles[i]<(PI) && angles[i] >= (PI/2)){
           yPos[i] = parameters.lon - sqrtf((-(powf(parameters.lat,2)) + (powf(parameters.rad,2)) + 2*parameters.lat*xPos[i] - powf(xPos[i],2) ));
        }else if (angles[i]<(3*(PI/2)) && angles[i] >= PI){
           yPos[i] = parameters.lon - sqrtf((-(powf(parameters.lat,2)) + (powf(parameters.rad,2)) + 2*parameters.lat*xPos[i] - powf(xPos[i],2) ));
        }else if (angles[i]<(2*PI) && angles[i] >= (3*PI/2)){
           yPos[i] = parameters.lon + sqrtf((-(powf(parameters.lat,2)) + (powf(parameters.rad,2)) + 2*parameters.lat*xPos[i] - powf(xPos[i],2) ));
        }
     
    }
  
   //write positions into vector3f for input into aircraft_statet struct.

   for (int i = 0; i < WARIO_TRAJECTORY_SIZE; i++)
   {
       waypoints[i].position.x = xPos[i];
       waypoints[i].position.y = yPos[i];
       waypoints[i].position.z = parameters.maxAlt;
   // printf("Angle: %.3f, Position: %.3f, %.3f\n", angles[i],waypoints[i].position.x,waypoints[i].position.y );
   //velocities into array of aircraft_statet

       waypoints[i].velocity.x = parameters.targetVelocity;
       waypoints[i].velocity.y = 0.0;
       waypoints[i].velocity.z = 0.0;
 
   //angular rates into aircraftstate array
   
       waypoints[i].angularVelocity.x = 0.0;
       waypoints[i].angularVelocity.y = 0.0;
       waypoints[i].angularVelocity.z = 0.0;

   //euler angles into aircraftstate array
       waypoints[i].roll = cRoll;
       waypoints[i].pitch= 0.0;
       waypoints[i].yaw= 0.0;

    //set phase type of each waypoint
        waypoints[i].phase = FLIGHT_PHASE_CIRCLE;

    //printf("X loc: %.7f, Y loc: %.7f, index: %.3d\n", waypoints[i].position.x,waypoints[i].position.y,i);
   }
 
  
    convertWaypointsToLocations(circleWaypointsLoc, waypoints);
    //printf("\nInnit generated circle\n");
}


void RalphieTrajectory::initSquircle() {
    //printf("\nStart of initSquircle\n");
    majorAxis = parameters.rad;
    radiusFactor = 4.0;
    minorAxis = majorAxis/radiusFactor;
    straightDistance = 4 * (majorAxis - minorAxis);
    circleDistance = 2 * PI * minorAxis;
    totalDistance = straightDistance + circleDistance;
    distanceBetweenWaypoints = totalDistance / WARIO_TRAJECTORY_SIZE;

    deltaAngleFirstQuarterCircle = ((PI/2) / wpOnFirstQuarterCircle);
    deltaAngleSecondQuarterCircle = ((PI/2) / wpOnSecondQuarterCircle);
    deltaAngleHalfCircle = ((PI) / wpOnHalfCircle);
    yDeltaFirstStraight = ((minorAxis - majorAxis) - (majorAxis - minorAxis))/wpOnFirstStraightLine;
    yDeltaSecondStraight = ((majorAxis - minorAxis) - (minorAxis - majorAxis))/wpOnSecondStraightLine;


    angleFirstQuarterCircle[0] = 0.0;
    

    // TODO: wario
    //Pull updated wind data from currentWindEstimate
    //setCurrentWind(currentWindEstimate);
    // printf("\nStart of update\n");
    radiusFactor = 7.0;
    majorAxis = parameters.rad;
    minorAxis = majorAxis / radiusFactor;


    //calculate total distance of shape. 
    circleDistance = 2 * PI * minorAxis;
    straightDistance = 4 * (majorAxis - minorAxis);
    totalDistance = circleDistance + straightDistance;

    //Distance between waypoints
    distanceBetweenWaypoints = totalDistance / WARIO_TRAJECTORY_SIZE;
    //printf("\nfloor test\n");

    //calculate percentage of waypoints per section. 
    //int wpOnFirstQuarterCircle = floorf(circleDistance / (4 * distanceBetweenWaypoints));
    //int wpOnFirstStraightLine = floorf(straightDistance / (2 * distanceBetweenWaypoints));
    //int wpOnHalfCircle = floorf(circleDistance / (2 * distanceBetweenWaypoints));
    //int wpOnSecondStraightLine = floorf(straightDistance / (2 * distanceBetweenWaypoints));
    //int wpOnSecondQuarterCircle = wpOnFirstQuarterCircle;

    //calculate delta range of angles needed. 
    deltaAngleFirstQuarterCircle = ((PI/2) / wpOnFirstQuarterCircle);
    deltaAngleSecondQuarterCircle = ((PI/2) / wpOnSecondQuarterCircle);
    deltaAngleHalfCircle = ((PI) / wpOnHalfCircle);

    //calculate range of angles vector
    //angleFirstQuarterCircle[wpOnFirstQuarterCircle];
    //angleSecondQuarterCircle[wpOnSecondQuarterCircle];
    //angleHalfCircle[wpOnHalfCircle];

    angleFirstQuarterCircle[0] = (PI/2);
    
    for (int i = 1; i < wpOnFirstQuarterCircle; i++){
        angleFirstQuarterCircle[i] = angleFirstQuarterCircle[i-1] + deltaAngleFirstQuarterCircle; 
    }

    angleSecondQuarterCircle[0] = 0.0;
    for (int i = 1; i < wpOnSecondQuarterCircle; i++){
        angleSecondQuarterCircle[i] = angleSecondQuarterCircle[i-1] + deltaAngleSecondQuarterCircle; 
    }

    angleHalfCircle[0] = PI;
     for (int i = 1; i < wpOnHalfCircle; i++){
        angleHalfCircle[i] = angleHalfCircle[i-1] + deltaAngleHalfCircle; 
    }
  

    for (int i = 0; i < wpOnFirstQuarterCircle; i++){
        xFirstQuarterCircle[i] = parameters.lat + (minorAxis * (cosf(angleFirstQuarterCircle[i]))); 
        yFirstQuarterCircle[i] = parameters.lon + sqrtf(-(powf(parameters.lat,2) ) + powf(minorAxis,2) + (2 * parameters.lat * xFirstQuarterCircle[i]) - (powf(xFirstQuarterCircle[i],2)) ) + majorAxis - minorAxis;
    }
     for (int i = 0; i < wpOnSecondQuarterCircle; i++){
        xSecondQuarterCircle[i] = parameters.lat + (minorAxis * cosf(angleSecondQuarterCircle[i]));
        ySecondQuarterCircle[i] = parameters.lon + sqrtf(-(powf(parameters.lat,2) ) + powf(minorAxis,2) + (2 * parameters.lat * xSecondQuarterCircle[i]) - (powf(xSecondQuarterCircle[i],2)) ) + majorAxis - minorAxis; 
    }
     for (int i = 0; i < wpOnHalfCircle; i++){
        xHalfCircle[i] = parameters.lat + (minorAxis * cosf(angleHalfCircle[i])); 
        yHalfCircle[i] = parameters.lon - sqrtf(-(powf(parameters.lat,2) ) + powf(minorAxis,2) + (2 * parameters.lat * xHalfCircle[i]) - (powf(xHalfCircle[i],2)) ) - majorAxis + minorAxis;
    }
     for (int i = 0; i < wpOnFirstStraightLine; i++){
        xFirstStraightLine[i] = parameters.lat - minorAxis;
    }
     for (int i = 0; i < wpOnSecondStraightLine; i++){
        xSecondStraightLine[i] = parameters.lat + minorAxis;
    }
    //printf("\nX vector generated\n");


    for (int i = 0; i < wpOnFirstQuarterCircle; i++){
        yFirstQuarterCircle[i] = parameters.lon + sqrtf(-(powf(parameters.lat,2) ) + powf(minorAxis,2) + (2 * parameters.lat * xFirstQuarterCircle[i]) - (powf(xFirstQuarterCircle[i],2)) ) + majorAxis - minorAxis;
    }
     for (int i = 0; i < wpOnSecondQuarterCircle; i++){
        ySecondQuarterCircle[i] = parameters.lon + sqrtf(-(powf(parameters.lat,2) ) + powf(minorAxis,2) + (2 * parameters.lat * xSecondQuarterCircle[i]) - (powf(xSecondQuarterCircle[i],2)) ) + majorAxis - minorAxis;
    }
    for (int i = 0; i < wpOnHalfCircle; i++){
        yHalfCircle[i] = parameters.lon - sqrtf(-(powf(parameters.lat,2) ) + powf(minorAxis,2) + (2 * parameters.lat * xHalfCircle[i]) - (powf(xHalfCircle[i],2)) ) - majorAxis + minorAxis;
    }

    yFirstStraightLine[0] = parameters.lon + majorAxis - minorAxis;
    ySecondStraightLine[0] = parameters.lon + minorAxis - majorAxis;

    for (int i = 1; i < wpOnFirstStraightLine; i++){
        yFirstStraightLine[i] = yFirstStraightLine[i-1] + yDeltaFirstStraight;
    } 
    for (int i = 1; i < wpOnSecondStraightLine; i++){
        ySecondStraightLine[i] = ySecondStraightLine[i-1] + yDeltaSecondStraight;
    } 
    //printf("\ny vector generated\n");

    for (int i = 0; i < wpOnFirstQuarterCircle; i++){
        xPosition[i] = xFirstQuarterCircle[i];
        yPosition[i] = yFirstQuarterCircle[i];
        //printf("First quarter circ: X loc: %.3f, Y loc: %.3f, index: %.3d\n", xPosition[i],yPosition[i],i);
    }
    for (int i = wpOnFirstQuarterCircle; i < wpOnFirstQuarterCircle + wpOnFirstStraightLine; i++){
        xPosition[i] = xFirstStraightLine[i - wpOnFirstQuarterCircle];
        yPosition[i] = yFirstStraightLine[i - wpOnFirstQuarterCircle];
        //printf("First straight: X loc: %.3f, Y loc: %.3f, index: %.3d\n", xPosition[i],yPosition[i],i);
    }
    for (int i = wpOnFirstQuarterCircle + wpOnFirstStraightLine; i < wpOnFirstQuarterCircle + wpOnFirstStraightLine + wpOnHalfCircle; i++){
        xPosition[i] = xHalfCircle[i - wpOnFirstQuarterCircle - wpOnFirstStraightLine];
        yPosition[i] = yHalfCircle[i - wpOnFirstQuarterCircle - wpOnFirstStraightLine];
        //printf("First half circ: X loc: %.3f, Y loc: %.3f, index: %.3d\n", xPosition[i],yPosition[i],i);
    }
    for (int i = wpOnFirstQuarterCircle + wpOnFirstStraightLine + wpOnHalfCircle; i < wpOnFirstQuarterCircle + wpOnFirstStraightLine + wpOnHalfCircle + wpOnSecondStraightLine; i++){
        xPosition[i] = xSecondStraightLine[i - wpOnFirstQuarterCircle - wpOnFirstStraightLine - wpOnHalfCircle];
        yPosition[i] = ySecondStraightLine[i - wpOnFirstQuarterCircle - wpOnFirstStraightLine - wpOnHalfCircle];
        //printf("Second straight: X loc: %.3f, Y loc: %.3f, index: %.3d\n", xPosition[i],yPosition[i],i);    
    }
    for (int i = wpOnFirstQuarterCircle + wpOnFirstStraightLine + wpOnHalfCircle + wpOnSecondStraightLine; i < wpOnFirstQuarterCircle + wpOnFirstStraightLine + wpOnHalfCircle + wpOnSecondStraightLine + wpOnSecondQuarterCircle; i++){
        xPosition[i] = xSecondQuarterCircle[i - wpOnFirstQuarterCircle - wpOnFirstStraightLine - wpOnHalfCircle - wpOnSecondStraightLine];
        yPosition[i] = ySecondQuarterCircle[i - wpOnFirstQuarterCircle - wpOnFirstStraightLine - wpOnHalfCircle - wpOnSecondStraightLine];
        //printf("Second quarter: X loc: %.3f, Y loc: %.3f, index: %.3d\n", xPosition[i],yPosition[i],i);
    }


   for (int i = 0; i < WARIO_TRAJECTORY_SIZE; i++)
   {
       waypointsSquircle[i].position.x = xPosition[i];
       waypointsSquircle[i].position.y = yPosition[i];
       waypointsSquircle[i].position.z = parameters.maxAlt;
       //printf("X: %.3f, Y: %.3f\n", waypointsSquircle[i].position.x, waypointsSquircle[i].position.y);

   //velocities into array of aircraft_statet

       waypointsSquircle[i].velocity.x = parameters.targetVelocity;
       waypointsSquircle[i].velocity.y = 0.0;
       waypointsSquircle[i].velocity.z = 0.0;
 
   //angular rates into aircraftstate array
   
       waypointsSquircle[i].angularVelocity.x = 0.0;
       waypointsSquircle[i].angularVelocity.y = 0.0;
       waypointsSquircle[i].angularVelocity.z = 0.0;

    

   }
   //euler angles into aircraftstate array

    cRollS = atan2f(powf(parameters.targetVelocity,2),(minorAxis * 9.81));
    for (int i = 0; i < wpOnFirstQuarterCircle; i++){
        waypointsSquircle[i].roll = cRollS;
        waypointsSquircle[i].pitch= 0.0;
        waypointsSquircle[i].yaw= 0.0;
        waypointsSquircle[i].phase = FLIGHT_PHASE_SEMI_CIRCLE;
    }
    for (int i = wpOnFirstQuarterCircle; i < wpOnFirstQuarterCircle + wpOnFirstStraightLine; i++){
        waypointsSquircle[i].roll = 0.0;
        waypointsSquircle[i].pitch= 0.0;
        waypointsSquircle[i].yaw= 0.0;
        waypointsSquircle[i].phase = FLIGHT_PHASE_STRAIGHT;
    }
    for (int i = wpOnFirstQuarterCircle + wpOnFirstStraightLine; i < wpOnFirstQuarterCircle + wpOnFirstStraightLine + wpOnHalfCircle; i++){
        waypointsSquircle[i].roll = cRollS;
        waypointsSquircle[i].pitch= 0.0;
        waypointsSquircle[i].yaw= 0.0;
        waypointsSquircle[i].phase = FLIGHT_PHASE_SEMI_CIRCLE;
    }
     for (int i = wpOnFirstQuarterCircle + wpOnFirstStraightLine + wpOnHalfCircle; i < wpOnFirstQuarterCircle + wpOnFirstStraightLine + wpOnHalfCircle + wpOnSecondStraightLine; i++){
        waypointsSquircle[i].roll = 0.0;
        waypointsSquircle[i].pitch= 0.0;
        waypointsSquircle[i].yaw= 0.0;
        waypointsSquircle[i].phase = FLIGHT_PHASE_STRAIGHT;
    }
    for (int i = wpOnFirstQuarterCircle + wpOnFirstStraightLine + wpOnHalfCircle + wpOnSecondStraightLine; i < wpOnFirstQuarterCircle + wpOnFirstStraightLine + wpOnHalfCircle + wpOnSecondStraightLine + wpOnSecondQuarterCircle; i++){
        waypointsSquircle[i].roll = cRollS;
        waypointsSquircle[i].pitch= 0.0;
        waypointsSquircle[i].yaw= 0.0;
        waypointsSquircle[i].phase = FLIGHT_PHASE_SEMI_CIRCLE;
    }
    //printf("\nEnd of initSquircle\n");
    // convertWaypointsToLocations(FLIGHT_PHASE_STRAIGHT);
}

 void RalphieTrajectory::updateTransition(){

    // TODO: Base rotation off of 'finalAngle'

    // printf("\nStart of updateTransition\n");
    //rotation angle defined as angle of rotation counterclockwise from 0  to final starting angle to next squircle. 
    //rotationAngle = tanf(windEstimate.x/windEstimate.y)
    // float transitionAngle = PI ;
    float transitionAngle = finalAngle;

    //generate transition path 
    // startAngle = PI/3;  //previous wind/squircle direction
    
    deltaAngle = (2*PI)/WARIO_TRAJECTORY_SIZE;
     if (finalAngle < startAngle){
        transitionPathSize = ceilf(((2*PI - startAngle)+ transitionAngle)/deltaAngle);
    }else{
        transitionPathSize = ceilf((finalAngle-startAngle)/deltaAngle);
    }
    //printf("initial Angle: %.3f, final Angle: %.3f, transition size: %.3f\n", startAngle, finalAngle, transitionSize); 
    cRoll = atan2f(powf(parameters.targetVelocity,2),(parameters.rad * 9.81));
    for (int i = 0; i < transitionPathSize; i++)
    {   
        if ((startAngle + (i * deltaAngle)) < (2*PI)){
            currentAngle = (startAngle + (i * deltaAngle));
        }else{
            currentAngle = (startAngle + (i * deltaAngle) - (2*PI));
        }
        waypointsTransition[i].position.x = (parameters.rad * -sinf(currentAngle)) + parameters.lat;
        if (currentAngle < (PI/2)){
           waypointsTransition[i].position.y = parameters.lon + sqrtf((-(powf(parameters.lat,2)) + (powf(parameters.rad,2)) + 2*parameters.lat*(waypointsTransition[i].position.x) - powf(waypointsTransition[i].position.x,2) ));
        }else if (currentAngle<(PI) && currentAngle >= (PI/2) ){
           waypointsTransition[i].position.y = parameters.lon - sqrtf((-(powf(parameters.lat,2)) + (powf(parameters.rad,2)) + 2*parameters.lat*(waypointsTransition[i].position.x) - powf(waypointsTransition[i].position.x,2) ));
        }else if (currentAngle<(3*(PI/2)) && currentAngle >= (PI) ){
           waypointsTransition[i].position.y = parameters.lon - sqrtf((-(powf(parameters.lat,2)) + (powf(parameters.rad,2)) + 2*parameters.lat*(waypointsTransition[i].position.x) - powf(waypointsTransition[i].position.x,2) ));
        }else if (currentAngle<(2*PI) && currentAngle >= (3*(PI/2)) ){
           waypointsTransition[i].position.y = parameters.lon + sqrtf((-(powf(parameters.lat,2)) + (powf(parameters.rad,2)) + 2*parameters.lat*(waypointsTransition[i].position.x) - powf(waypointsTransition[i].position.x,2) ));
        }
    // printf("Angle: %.3f, Position: %.3f, %.3f\n",currentAngle, waypointsTransition[i].position.x, waypointsTransition[i].position.y);

        waypointsTransition[i].position.z = parameters.maxAlt;

   //velocities into array of aircraft_statet

       waypointsTransition[i].velocity.x = parameters.targetVelocity;
       waypointsTransition[i].velocity.y = 0.0;
       waypointsTransition[i].velocity.z = 0.0;
   //angular rates into aircraftstate array
       waypointsTransition[i].angularVelocity.x = 0.0;
       waypointsTransition[i].angularVelocity.y = 0.0;
       waypointsTransition[i].angularVelocity.z = 0.0;
   //euler angles into aircraftstate array

       waypointsTransition[i].roll = cRoll;
       waypointsTransition[i].pitch= 0.0;
       waypointsTransition[i].yaw= 0.0;

    //set phase type of each waypoint
       waypointsTransition[i].phase = FLIGHT_PHASE_TRANSITION;

   }
convertWaypointsToLocations(waypointsTransitionLoc, waypointsTransition);
 }
void RalphieTrajectory::updatePath() {

    // TODO: Base rotation off of 'finalAngle'
  
    // float rotationAngle = tanf(windEstimate.x/windEstimate.y); 
    // float rotationAngle = PI/8;
    float rotationAngle = finalAngle;

    for (int i = 0; i < WARIO_TRAJECTORY_SIZE; i++){
        waypointsRotated[i].position.x = (cosf(rotationAngle)*waypointsSquircle[i].position.x - sinf(rotationAngle)*waypointsSquircle[i].position.y);
        waypointsRotated[i].position.y = (sinf(rotationAngle)*waypointsSquircle[i].position.x + cosf(rotationAngle)*waypointsSquircle[i].position.y);
        //printf("X loc: %.3f, Y loc: %.3f, index: %.3d\n", waypointsRotated[i].position.x,waypointsRotated[i].position.y,i);

        waypointsRotated[i].position.z = waypointsSquircle[i].position.z;
        waypointsRotated[i].velocity.x = waypointsSquircle[i].velocity.x;
        waypointsRotated[i].velocity.y = waypointsSquircle[i].velocity.y;
        waypointsRotated[i].velocity.z = waypointsSquircle[i].velocity.z;
        waypointsRotated[i].angularVelocity.x = waypointsSquircle[i].angularVelocity.x;
        waypointsRotated[i].angularVelocity.y = waypointsSquircle[i].angularVelocity.y;
        waypointsRotated[i].angularVelocity.z = waypointsSquircle[i].angularVelocity.z;
        waypointsRotated[i].roll = waypointsSquircle[i].roll;
        waypointsRotated[i].pitch = waypointsSquircle[i].pitch;
        waypointsRotated[i].yaw= waypointsSquircle[i].yaw;
        waypointsRotated[i].phase = waypointsSquircle[i].phase;
    }
    convertWaypointsToLocations(waypointsRotatedLoc, waypointsRotated);
    
}


void RalphieTrajectory::updateAverageWind(Vector3f measurement) {

    /* If we haven't filled the buffer with wind samples yet, fill the buffer */
    if (windSamples < WIND_BUFFER_SIZE) 
        memcpy(&windBuffer[windSamples++], &measurement, sizeof(Vector3f));

    /* If the buffer is already full, replace the oldest sample with the new sample */
    else
        memcpy(&windBuffer[windBufferIndex++], &measurement, sizeof(Vector3f));

    /* Roll index over */
    if (WIND_BUFFER_SIZE == windBufferIndex) 
        windBufferIndex = 0;

    /* Average the contents of the buffer */
    averageWind(); 
}


void RalphieTrajectory::averageWind() {

    /* Sum every vector in the buffer */
    Vector3f sum;
    for (uint8_t index = 0; index < windSamples; index++) 
        sum += windBuffer[index];

    /* Use the sum to compute the average vector and angle */
    currentWindEstimate = sum / WIND_BUFFER_SIZE;
    currentWindAngleEstimate = atanF(currentWindEstimate.y / currentWindEstimate.x);
}


void RalphieTrajectory::resetWindAverage() {

    /* Clear the wind buffer */
    windSamples = 0;
}



void RalphieTrajectory::convertWaypointsToLocations(Location locations[], aircraftState_t states[]) {

    /* Convert aircraft states to locations */
    for (uint8_t index = 0; index < WARIO_TRAJECTORY_SIZE; index++) {

        /* Altitude is measured in centimeters (hence the multiplier of 100) */
        locations[index].alt = TrajectoryHome.alt + (int32_t)states[index].position.z * 100;
        locations[index].lat = ((TrajectoryHome.lat * LATLON_TO_M) + states[index].position.x) * LATLON_TO_M_INV;
        locations[index].lng = ((TrajectoryHome.lng * LATLON_TO_M) + states[index].position.y) * LATLON_TO_M_INV;
    }
}


void RalphieTrajectory::update() {

    /* If we are already transitioning or circling, we don't want to touch anything */ 
    if ( (STATE_TRANSITIONING == state) || (STATE_CIRCLING == state) )
        return;

    /* Check if we need to cancel a previous transition */
    if (fabsf(currentWindAngleEstimate - startAngle) < ROTATION_THRESHOLD) {
        needToTransition = false;
        return;
    }

    /* If the difference between current wind and current path is greater than the threshold, rotate the paths */
    if (fabsf(currentWindAngleEstimate - startAngle) > ROTATION_THRESHOLD) {
        finalAngle = currentWindAngleEstimate;
        updateTransition();
        needToTransition = true;
    }
}


flightPhase_t RalphieTrajectory::fillNextWaypoint(Location &prev_WP_loc, Location current_loc, Location &next_WP_loc) {

    /* If we have passed the current waypoint being tracked */
    if (current_loc.past_interval_finish_line(prev_WP_loc, next_WP_loc)) {
        prev_WP_loc = next_WP_loc;
        flightPhase_t phase;

        /* WARIO State Machine */
        switch (state) {

            /* Aircraft is in circling pattern */
            case STATE_CIRCLING:

                /* If we have reaced the end of the circle buffer */
                if (currentWaypointIndex == WARIO_TRAJECTORY_SIZE) {

                    /* Document the current wind estimate, update the trajectory */
                    startAngle = finalAngle;
                    finalAngle = currentWindAngleEstimate;
                    updateTransition();
                    updatePath();

                    /* Reset the index, copy the first waypoint from the transition array */ 
                    currentWaypointIndex = 0;
                    next_WP_loc = waypointsTransitionLoc[currentWaypointIndex];
                    state = STATE_TRANSITIONING;
                    phase = FLIGHT_PHASE_TRANSITION;
                    break;
                }

                /* Copy next waypoint from CIRCLE array */
                next_WP_loc = circleWaypointsLoc[currentWaypointIndex];
                phase = FLIGHT_PHASE_CIRCLE;
                break;

            /* Aircraft is in squircle/racetrack pattern */ 
            case STATE_NORMAL:  

                /* If the we have reached the end of a squircle and need a trajectory */ 
                if (currentWaypointIndex == WARIO_TRAJECTORY_SIZE && needToTransition) {

                    /* Update the path based on the angle used to rotate the transition trajectory */
                    updatePath();
                    startAngle = finalAngle;
                    currentWaypointIndex = 0;

                    /* Copy the first waypoint from the transition array */
                    next_WP_loc = waypointsTransitionLoc[currentWaypointIndex];
                    needToTransition = false;
                    state = STATE_TRANSITIONING;
                    phase = FLIGHT_PHASE_TRANSITION;
                    break;
                }

                /* If we finish the current loop, but no rotation is needed, reset the index */
                if (currentWaypointIndex == WARIO_TRAJECTORY_SIZE && !needToTransition)
                    currentWaypointIndex = 0;

                /* Copy the next waypoint in the SQUIRCLE to the next_WP_loc */ 
                next_WP_loc = waypointsRotatedLoc[currentWaypointIndex];
                phase = phases[currentWaypointIndex];
                break;

            /* Aircraft is transitioning between squircles/racetracks */
            case STATE_TRANSITIONING:

                /* If we have reached the end of a transition path */
                if (currentWaypointIndex == transitionPathSize) {

                    /* Reset the index, and copy the first waypoint from the squircle array */
                    currentWaypointIndex = 0;
                    next_WP_loc = waypointsRotatedLoc[currentWaypointIndex];
                    state = STATE_NORMAL;
                    phase = phases[currentWaypointIndex];
                    currentPathDirection = finalAngle;
                    break;
                }

                /* Copy the next waypoint in the TRANSITION to the next_WP_loc */ 
                next_WP_loc = waypointsTransitionLoc[currentWaypointIndex];
                phase = FLIGHT_PHASE_TRANSITION;
                break; 

            default:
                /* Something went wrong */
                phase = FLIGHT_PHASE_STRAIGHT; // TODO: wtf should be returned here
                break;
        }

        /* Increment the index of the current waypoint, return the flight phase of waypoint */
        currentWaypointIndex++;
        return phase;
    }

    /* We didn't pass a waypoint, but still need to return the correct flight phase */
    if (state == STATE_CIRCLING)
        return FLIGHT_PHASE_CIRCLE;
    else if (state == STATE_NORMAL)
        return phases[currentWaypointIndex];
    else
        return FLIGHT_PHASE_CIRCLE;
    return phases[currentWaypointIndex];
}



void RalphieTrajectory::printState() {

    switch (state) {

        case STATE_CIRCLING:
            printf("STATE: CIRCLING %d/%d\n", currentWaypointIndex, WARIO_TRAJECTORY_SIZE);
            break;
        case STATE_TRANSITIONING:
            printf("STATE: TRANSITIONING %d/%d\n", currentWaypointIndex, transitionPathSize);    
            break;
        case STATE_NORMAL:
            printf("STATE: NORMAL %d/%d\n", currentWaypointIndex, WARIO_TRAJECTORY_SIZE);
            break;
    }
}
