#include "trajectory.h"

void RalphieTrajectory::initCircle(warioInput_t parameters) {
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
 
  

    //printf("\nInnit generated circle\n");
}


void RalphieTrajectory::initSquircle(warioInput_t parameters) {
    //printf("\nStart of initSquircle\n");
    majorAxis = parameters.rad;
    radiusFactor = 7.0;
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
        xFirstQuarterCircle[i] = parameters.lat + (minorAxis * -(sinf(angleFirstQuarterCircle[i]))); 
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
}

 void RalphieTrajectory::updateTransition(warioInput_t parameters, Vector3f windEstimate, Vector3f pastWindEstimate){
    // printf("\nStart of updateTransition\n");
    //rotation angle defined as angle of rotation counterclockwise from 0  to final starting angle to next squircle. 
    //rotationAngle = tanf(windEstimate.x/windEstimate.y)
    rotationAngle = PI ;

    //generate transition path 
    startAngle = PI/2;  //previous wind/squircle direction
    finalAngle = rotationAngle; 
    deltaAngle = (2*PI)/WARIO_TRAJECTORY_SIZE;
     if (finalAngle < startAngle){
        transitionSize = ceilf(((2*PI - startAngle)+ rotationAngle)/deltaAngle);
    }else{
        transitionSize = ceilf((finalAngle-startAngle)/deltaAngle);
    }
    //printf("initial Angle: %.3f, final Angle: %.3f, transition size: %.3f\n", startAngle, finalAngle, transitionSize); 
    cRoll = atan2f(powf(parameters.targetVelocity,2),(parameters.rad * 9.81));
    for (int i = 0; i < transitionSize; i++)
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
       waypoints[i].phase = FLIGHT_PHASE_TRANSITION;

   }

 }
void RalphieTrajectory::updatePath(warioInput_t parameters, Vector3f windEstimate) {
  
    // float rotationAngle = tanf(windEstimate.x/windEstimate.y); 
    rotationAngle = PI/2;

    for (int i = 0; i < WARIO_TRAJECTORY_SIZE; i++){
        waypointsRotated[i].position.x = (cosf(rotationAngle)*waypointsSquircle[i].position.x - sinf(rotationAngle)*waypointsSquircle[i].position.y);
        waypointsRotated[i].position.y = (sinf(rotationAngle)*waypointsSquircle[i].position.x + cosf(rotationAngle)*waypointsSquircle[i].position.y);
        printf("X loc: %.3f, Y loc: %.3f, index: %.3d\n", waypointsRotated[i].position.x,waypointsRotated[i].position.y,i);

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

    
}


void RalphieTrajectory::setCurrentWind(Vector3f windEstimate) {

 
	
    memcpy(&currentWindEstimate, &windEstimate, sizeof(Vector3f));
}
