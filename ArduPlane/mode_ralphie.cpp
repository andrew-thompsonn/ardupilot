
#include "mode.h"
#include "plane.h"
#include "LQT_Constants.h"

//call mode 26 from console to switch to RALPHIE mode//
void ModeRalphie::run() {

    /* For control system -> called from Plane::stablize() in Attitude.cpp line 503 */
    switch (desiredState.phase) {
     
        case FLIGHT_PHASE_CIRCLE:
            break;
      
        case FLIGHT_PHASE_STRAIGHT:
            break;
       
        case FLIGHT_PHASE_SEMI_CIRCLE:
            break;
        
        case FLIGHT_PHASE_TRANSITION:
            break;
    }

    //crashThePlane();
    //controllerLQT();
}

void ModeRalphie::crashThePlane() {

    printf("crashing\n");
    SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 0.0);
    SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, 100.0);
    SRV_Channels::set_output_scaled(SRV_Channel::k_rudder, 100.0);
    SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, 100.0);
}

void ModeRalphie::controllerLQT() {

    float latInput[2];
    //float lonInput[2];
    float latError[6] = {1,5,66,85,2,20};
    //float lonError[6] = {1,5,66,85,2,20};

    matrixMathFuncs matrixTestObject;
    matrixTestObject.LQTMult(GAINS_LAT_LINE,latError,latInput);
    
    for (int i = 0; i < 2; i++) {
        printf("Computed control input in index %d is %f\n",i,latInput[i]);
    }

}


bool ModeRalphie::_enter() {
    
    /* Enters the mode, perform tasks that only need to happen on initialization */
    // trajectory.init();
    printState();

    return true;
}


void ModeRalphie::update() {
    
    /* Called at 400 Hz from scheduler, other miscellaneous items can happen here */

    currentState.roll = plane.ahrs.get_roll();
    currentState.pitch = plane.ahrs.get_pitch();
    currentState.yaw = plane.ahrs.get_yaw();

    plane.ahrs.get_relative_position_NED_origin(currentState.position);
    plane.ahrs.get_velocity_NED(currentState.velocity);
    currentState.angularVelocity = plane.ahrs.get_gyro();

    // printState();
}


void ModeRalphie::navigate() {
    
    /* For trajectory and navigation -> called from Plane::navigate in ArduPlane.cpp line 109 */
    if (navigation == INACTIVE)
        return;
    
    trajectory.update();

    plane.current_loc = plane.next_WP_loc;
    plane.next_WP_loc = plane.prev_WP_loc;

    /* Update plane.next_WP_loc, plane.current_loc, plane.prev_WP_loc etc */
    //printf("RALPHIE NAVIGATING\n");
}


void ModeRalphie::printState() {

	printf("Position: %.3f, %.3f, %.3f\n", currentState.position.x, 
                                           currentState.position.y, 
                                           currentState.position.z);

	printf("Velocity: %.3f, %.3f, %.3f\n", currentState.velocity.x, 
                                           currentState.velocity.y, 
                                           currentState.velocity.z);

	printf("Angles:   %.3f, %.3f, %.3f\n", currentState.roll*RAD_TO_DEG, 
                                           currentState.pitch*RAD_TO_DEG, 
                                           currentState.yaw*RAD_TO_DEG);

	printf("Omega:    %.3f, %.3f, %.3f\n\n", currentState.angularVelocity.x, 
                                             currentState.angularVelocity.y, 
                                             currentState.angularVelocity.z);

    float ans[2];
    float xError[6] = {1,5,66,85,2,20};

    matrixMathFuncs matrixTestObject;
    matrixTestObject.LQTMult(GAINS_LAT_LINE,xError,ans);
    
    for (int i = 0; i < 2; i++) {
        printf("Computed control input in index %d is %f\n",i,ans[i]);
    }

}




