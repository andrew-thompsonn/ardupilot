#include "trajectory.h"

void RalphieTrajectory::init(warioInput_t parameters) {
// TODO: circle
 
   //write vector of angles for circle.
  
   float deltaAngle = (2*PI)/WARIO_TRAJECTORY_SIZE;
   float angles[WARIO_TRAJECTORY_SIZE];
   float initialAngle = 0.0;
  // angles[0] = parameters.initialAngle;
   angles[0] = initialAngle;
   for (int i = 1; i < WARIO_TRAJECTORY_SIZE; i++){
       angles[i] = angles[i-1] + deltaAngle;
      // printf("Current Angle: %.3f", angles[i]);
   }
  
   //write vectors of positions over path
   float xPos[WARIO_TRAJECTORY_SIZE];
   float  yPos[WARIO_TRAJECTORY_SIZE];
   for (int i = 0; i < WARIO_TRAJECTORY_SIZE; i++)
   {
       xPos[i] = (parameters.rad * cosf(angles[i])) + parameters.lat;
       if (i < WARIO_TRAJECTORY_SIZE/2){
           yPos[i] = parameters.lon + sqrtf((-(powf(parameters.lat,2)) + (powf(parameters.rad,2)) + 2*parameters.lat*xPos[i] - powf(xPos[i],2) ));
      }else{
           yPos[i] = parameters.lon - sqrtf((-(powf(parameters.lat,2)) + (powf(parameters.rad,2)) + 2*parameters.lat*xPos[i] - powf(xPos[i],2) ));
       }
      // printf("Angle: %.3f, Position: %.3f, %.3f\n", angles[i],xPos[i],yPos[i]);
   }
  
 
   //create array of aircraftStates
   //aircraftState_t circlePoints[WARIO_TRAJECTORY_SIZE];
 
    float cRoll;
   //write positions into vector3f for input into aircraft_statet struct.

   for (int i = 0; i < WARIO_TRAJECTORY_SIZE; i++)
   {
       waypoints[i].position.x = xPos[i];
       waypoints[i].position.y = yPos[i];
       waypoints[i].position.z = parameters.maxAlt;

   //velocities into array of aircraft_statet

       waypoints[i].velocity.x = parameters.targetVelocity;
       waypoints[i].velocity.y = 0.0;
       waypoints[i].velocity.z = 0.0;
 
   //angular rates into aircraftstate array
   
       waypoints[i].angularVelocity.x = 0.0;
       waypoints[i].angularVelocity.y = 0.0;
       waypoints[i].angularVelocity.z = 0.0;

   //euler angles into aircraftstate array

       cRoll = atan2f(powf(parameters.targetVelocity,2),(parameters.rad * 9.81));
       waypoints[i].roll = cRoll;
       waypoints[i].pitch= 0.0;
       waypoints[i].yaw= 0.0;

      // printf("X loc: %.7f, Y loc: %.7f, index: %.3d\n", waypoints[i].position.x,waypoints[i].position.y,i);
   }
 
   //waypoints = circlePoints;
 
   // (x - xc)^2 + (y - yc)^2 = r^2   
  

    printf("\nInnit generated circle\n");
}


void RalphieTrajectory::update(warioInput_t parameters) {
    // TODO: wario
    //Pull updated wind data from currentWindEstimate
    //setCurrentWind(currentWindEstimate);
    // printf("\nStart of update\n");
    float radiusFactor = 7.0;
    float majorAxis = parameters.rad;
    float minorAxis = majorAxis / radiusFactor;
    return;

    //calculate total distance of shape. 
    float circleDistance = 2 * PI * minorAxis;
    float straightDistance = 4 * (majorAxis - minorAxis);
    float totalDistance = circleDistance + straightDistance;

    //Distance between waypoints
    float distanceBetweenWaypoints = totalDistance / WARIO_TRAJECTORY_SIZE;
    //printf("\nfloor test\n");

    //calculate percentage of waypoints per section. 
    int wpOnFirstQuarterCircle = floorf(circleDistance / (4 * distanceBetweenWaypoints));
    int wpOnFirstStraightLine = floorf(straightDistance / (2 * distanceBetweenWaypoints));
    int wpOnHalfCircle = floorf(circleDistance / (2 * distanceBetweenWaypoints));
    int wpOnSecondStraightLine = floorf(straightDistance / (2 * distanceBetweenWaypoints));
    int wpOnSecondQuarterCircle = wpOnFirstQuarterCircle;

    //calculate delta range of angles needed. 
    float deltaAngleFirstQuarterCircle = ((PI/2) / wpOnFirstQuarterCircle);
    float deltaAngleSecondQuarterCircle = ((PI/2) / wpOnSecondQuarterCircle);
    float deltaAngleHalfCircle = ((PI) / wpOnHalfCircle);

    //calculate range of angles vector
    float angleFirstQuarterCircle[wpOnFirstQuarterCircle];
    float angleSecondQuarterCircle[wpOnSecondQuarterCircle];
    float angleHalfCircle[wpOnHalfCircle];

    angleFirstQuarterCircle[0] = (PI/2);
    for (int i = 1; i < wpOnFirstQuarterCircle; i++){
        angleFirstQuarterCircle[i] = angleFirstQuarterCircle[i-1] + deltaAngleFirstQuarterCircle; 
    }

    angleSecondQuarterCircle[0] = deltaAngleSecondQuarterCircle;
    for (int i = 1; i < wpOnSecondQuarterCircle; i++){
        angleSecondQuarterCircle[i] = angleSecondQuarterCircle[i-1] + deltaAngleSecondQuarterCircle; 
    }

    angleHalfCircle[0] = PI + deltaAngleHalfCircle;
     for (int i = 1; i < wpOnHalfCircle; i++){
        angleHalfCircle[i] = angleHalfCircle[i-1] + deltaAngleHalfCircle; 
    }
    //printf("\nangle vector generated\n");
    //solve x values
    float xFirstQuarterCircle[wpOnFirstQuarterCircle];
    float xSecondQuarterCircle[wpOnSecondQuarterCircle];
    float xHalfCircle[wpOnHalfCircle];
    float xFirstStraightLine[wpOnFirstStraightLine];
    float xSecondStraightLine[wpOnSecondStraightLine];

    for (int i = 0; i < wpOnFirstQuarterCircle; i++){
        xFirstQuarterCircle[i] = parameters.lat + (minorAxis * cosf(angleFirstQuarterCircle[i])); 
    }
     for (int i = 0; i < wpOnSecondQuarterCircle; i++){
        xSecondQuarterCircle[i] = parameters.lat + (minorAxis * cosf(angleSecondQuarterCircle[i])); 
    }
     for (int i = 0; i < wpOnHalfCircle; i++){
        xHalfCircle[i] = parameters.lat + (minorAxis * cosf(angleHalfCircle[i])); 
    }
     for (int i = 0; i < wpOnFirstStraightLine; i++){
        xFirstStraightLine[i] = parameters.lat - minorAxis;
    }
     for (int i = 0; i < wpOnSecondStraightLine; i++){
        xSecondStraightLine[i] = parameters.lat + minorAxis;
    }
    //printf("\nX vector generated\n");

    //solve y values
    float yFirstQuarterCircle[wpOnFirstQuarterCircle];
    float ySecondQuarterCircle[wpOnSecondQuarterCircle];
    float yHalfCircle[wpOnHalfCircle];
    float yFirstStraightLine[wpOnFirstStraightLine];
    float ySecondStraightLine[wpOnSecondStraightLine];

    for (int i = 0; i < wpOnFirstQuarterCircle; i++){
        yFirstQuarterCircle[i] = parameters.lon + sqrtf(-(powf(parameters.lat,2) ) + powf(minorAxis,2) + (2 * parameters.lat * xFirstQuarterCircle[i]) - (powf(xFirstQuarterCircle[i],2)) ) + majorAxis - minorAxis;
    }
     for (int i = 0; i < wpOnSecondQuarterCircle; i++){
        ySecondQuarterCircle[i] = parameters.lon + sqrtf(-(powf(parameters.lat,2) ) + powf(minorAxis,2) + (2 * parameters.lat * xSecondQuarterCircle[i]) - (powf(xSecondQuarterCircle[i],2)) ) + majorAxis - minorAxis;
    }
    for (int i = 0; i < wpOnHalfCircle; i++){
        yHalfCircle[i] = parameters.lon - sqrtf(-(powf(parameters.lat,2) ) + powf(minorAxis,2) + (2 * parameters.lat * xHalfCircle[i]) - (powf(xHalfCircle[i],2)) ) - majorAxis + minorAxis;
    }

    float yDeltaFirstStraight = ((minorAxis - majorAxis) - (majorAxis - minorAxis))/wpOnFirstStraightLine;
    float yDeltaSecondStraight = ((majorAxis - minorAxis) - (minorAxis - majorAxis))/wpOnSecondStraightLine;
    yFirstStraightLine[0] = parameters.lon + majorAxis - minorAxis;
    ySecondStraightLine[0] = parameters.lon + minorAxis - majorAxis;

    for (int i = 1; i < wpOnFirstStraightLine; i++){
        yFirstStraightLine[i] = yFirstStraightLine[i-1] + yDeltaFirstStraight;
    } 
    for (int i = 1; i < wpOnSecondStraightLine; i++){
        ySecondStraightLine[i] = ySecondStraightLine[i-1] + yDeltaSecondStraight;
    } 
    //printf("\ny vector generated\n");

    //create full position vectors
    float xPosition[WARIO_TRAJECTORY_SIZE];
    float yPosition[WARIO_TRAJECTORY_SIZE];

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

    //print out squircle data points
   // for (int i = 0; i < WARIO_TRAJECTORY_SIZE; i++){
   //  printf("X loc: %.3f, Y loc: %.3f, index: %.3d\n", xPosition[i],yPosition[i],i);
   // }


    //Calculate Aircraft State;
    float cRollS;
   //write positions into vector3f for input into aircraft_statet struct.

   for (int i = 0; i < WARIO_TRAJECTORY_SIZE; i++)
   {
       waypoints[i].position.x = xPosition[i];
       waypoints[i].position.y = yPosition[i];
       waypoints[i].position.z = parameters.maxAlt;

   //velocities into array of aircraft_statet

       waypoints[i].velocity.x = parameters.targetVelocity;
       waypoints[i].velocity.y = 0.0;
       waypoints[i].velocity.z = 0.0;
 
   //angular rates into aircraftstate array
   
       waypoints[i].angularVelocity.x = 0.0;
       waypoints[i].angularVelocity.y = 0.0;
       waypoints[i].angularVelocity.z = 0.0;

   }
   //euler angles into aircraftstate array

    cRollS = atan2f(powf(parameters.targetVelocity,2),(minorAxis * 9.81));
    for (int i = 0; i < wpOnFirstQuarterCircle; i++){
        waypoints[i].roll = cRollS;
        waypoints[i].pitch= 0.0;
        waypoints[i].yaw= 0.0;
    }
    for (int i = wpOnFirstQuarterCircle; i < wpOnFirstQuarterCircle + wpOnFirstStraightLine; i++){
        waypoints[i].roll = 0.0;
        waypoints[i].pitch= 0.0;
        waypoints[i].yaw= 0.0;
    }
    for (int i = wpOnFirstQuarterCircle + wpOnFirstStraightLine; i < wpOnFirstQuarterCircle + wpOnFirstStraightLine + wpOnHalfCircle; i++){
        waypoints[i].roll = cRollS;
        waypoints[i].pitch= 0.0;
        waypoints[i].yaw= 0.0;
    }
     for (int i = wpOnFirstQuarterCircle + wpOnFirstStraightLine + wpOnHalfCircle; i < wpOnFirstQuarterCircle + wpOnFirstStraightLine + wpOnHalfCircle + wpOnSecondStraightLine; i++){
        waypoints[i].roll = 0.0;
        waypoints[i].pitch= 0.0;
        waypoints[i].yaw= 0.0;
    }
    for (int i = wpOnFirstQuarterCircle + wpOnFirstStraightLine + wpOnHalfCircle + wpOnSecondStraightLine; i < wpOnFirstQuarterCircle + wpOnFirstStraightLine + wpOnHalfCircle + wpOnSecondStraightLine + wpOnSecondQuarterCircle; i++){
        waypoints[i].roll = cRollS;
        waypoints[i].pitch= 0.0;
        waypoints[i].yaw= 0.0;
    }
    
    //rotate default squircle trajectory into a target direction around center location. 
    //rotation angle defined as angle of rotation counterclockwise from 0  to final starting angle to next squircle. 
    float rotationAngle = PI ;

    //generate transition path 
    float initialAngle = 0;  //previous wind/squircle direction
    float finalAngle = rotationAngle; 
    float deltaAngle = (2*PI)/WARIO_TRAJECTORY_SIZE;
    int transitionSize = floorf((finalAngle-initialAngle)/deltaAngle);
     if (rotationAngle < initialAngle){
        transitionSize = floorf((2*PI - initialAngle)+ rotationAngle);
    }
     printf("initial Angle: %.3f, final Angle: %.3f, transition size: %.3d\n", initialAngle, finalAngle, transitionSize); 
    float xTransition[transitionSize];
    float yTransition[transitionSize];
   for (int i = 0; i < transitionSize; i++)
   {
       xTransition[i] = (parameters.rad * cosf((initialAngle + (i * deltaAngle)))) + parameters.lat;
       if ((initialAngle + (i * deltaAngle)) < (PI)){
           yTransition[i] = parameters.lon + sqrtf((-(powf(parameters.lat,2)) + (powf(parameters.rad,2)) + 2*parameters.lat*xTransition[i] - powf(xTransition[i],2) ));
      }else{
           yTransition[i] = parameters.lon - sqrtf((-(powf(parameters.lat,2)) + (powf(parameters.rad,2)) + 2*parameters.lat*xTransition[i] - powf(xTransition[i],2) ));
       }
     printf("Angle: %.3f, Position: %.3f, %.3f\n",(initialAngle + (i * deltaAngle)) ,xTransition[i],yTransition[i]);
   }



    int lengthS = 98;
    
     //create full position vectors
    float xPositionRotated[lengthS];
    float yPositionRotated[lengthS];
    for (int i = 0; i < lengthS; i++){
        xPositionRotated[i] = cosf(rotationAngle)*xPosition[i] + sinf(rotationAngle)*yPosition[i];
        yPositionRotated[i] = (-1) * sinf(rotationAngle)*xPosition[i] + cosf(rotationAngle)*yPosition[i];
       printf("X loc: %.3f, Y loc: %.3f, index: %.3d\n", xPositionRotated[i],yPositionRotated[i],i);
    }



 
    
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
    currentWindAngleEstimate = RAD_TO_DEG * atanF(currentWindEstimate.y / currentWindEstimate.x);

    printf("Estimated wind direction: %.3f\n", currentWindAngleEstimate);
}


void RalphieTrajectory::resetWindAverage() {

    windSamples = 0;
}



