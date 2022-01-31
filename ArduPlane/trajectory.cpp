#include "trajectory.h"


void RalphieTrajectory::init(warioInput_t parameters) {

    // TODO: circle

	//write vector of angles for circle. 
    
    float deltaAngle = (2*PI)/WARIO_TRAJECTORY_SIZE;
    float angles[WARIO_TRAJECTORY_SIZE];

    angles[0] = parameters.initialAngle; 
    for (int i = 0; i < WARIO_TRAJECTORY_SIZE; i++){
        angles[i+1] = angles[i] + deltaAngle; 
    }
	
    //write vectors of positions over path
   // float xPos[WARIO_TRAJECTORY_SIZE];
    //float  yPos[WARIO_TRAJECTORY_SIZE];
   // for (int i = 0; i < WARIO_TRAJECTORY_SIZE; i++)
  //  {
       // xPos[i] = (parameters.rad * cos(angles[i])) + parameters.x;
       // if (i < WARIO_TRAJECTORY_SIZE/2){
        //    yPos[i] = parameters.y + sqrt((-(pow(parameters.x,2)) + (pow(parameters.rad,2)) + 2*parameters.x*xPos[i] - pow(xPos[i],2) ));
       //}else{
       //     yPos[i] = parameters.y - sqrt((-(pow(parameters.x,2)) + (pow(parameters.rad,2)) + 2*parameters.x*xPos[i] - pow(xPos[i],2) ));
       // }
        //printf("Angle: %.3f, Position: %.3f, %.3f\n", angles[i],xPos[i],yPos[i]);
  //  }
    

	// (x - xc)^2 + (y - yc)^2 = r^2	
	
	
}


void RalphieTrajectory::update() {
    // TODO: wario
}


void RalphieTrajectory::setCurrentWind(Vector3f windEstimate) {

    memcpy(&currentWindEstimate, &windEstimate, sizeof(Vector3f));
}
