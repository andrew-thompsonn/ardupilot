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
       printf("Current Angle: %.3f", angles[i]);
   }
  
   //write vectors of positions over path
   float xPos[WARIO_TRAJECTORY_SIZE];
   float  yPos[WARIO_TRAJECTORY_SIZE];
   for (int i = 0; i < WARIO_TRAJECTORY_SIZE; i++)
   {
       xPos[i] = (parameters.rad * cos(angles[i])) + parameters.x;
       if (i < WARIO_TRAJECTORY_SIZE/2){
           yPos[i] = parameters.y + sqrt((-(pow(parameters.x,2)) + (pow(parameters.rad,2)) + 2*parameters.x*xPos[i] - pow(xPos[i],2) ));
      }else{
           yPos[i] = parameters.y - sqrt((-(pow(parameters.x,2)) + (pow(parameters.rad,2)) + 2*parameters.x*xPos[i] - pow(xPos[i],2) ));
       }
       printf("Angle: %.3f, Position: %.3f, %.3f\n", angles[i],xPos[i],yPos[i]);
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

       cRoll = atan2f(pow(parameters.targetVelocity,2),(parameters.rad * 9.81));
       waypoints[i].roll = cRoll;
       waypoints[i].pitch= 0.0;
       waypoints[i].yaw= 0.0;
   }
 
   //waypoints = circlePoints;
 
   // (x - xc)^2 + (y - yc)^2 = r^2   
  

	
}


void RalphieTrajectory::update() {
    // TODO: wario
}


void RalphieTrajectory::setCurrentWind(Vector3f windEstimate) {

    memcpy(&currentWindEstimate, &windEstimate, sizeof(Vector3f));
}
