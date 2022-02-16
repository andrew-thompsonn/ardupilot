
#include "mode.h"
#include "plane.h"

//call mode 27 from console to switch to RALPHIE mode//
void ModeLQT::run() {

    /* For control system -> called from Plane::stablize() in Attitude.cpp line 503 */
    switch (desiredState.phase) {
     
        case FLIGHT_PHASE_CIRCLE:
            //controllerLQT(GAINS_LAT_CIRCLE,GAINS_LON_CIRCLE);
            break;
      
        case FLIGHT_PHASE_STRAIGHT:
            //controllerLQT(GAINS_LAT_LINE,GAINS_LON_LINE);
            break;
       
        case FLIGHT_PHASE_SEMI_CIRCLE:
            //controllerLQT(GAINS_LAT_CURVE,GAINS_LON_CURVE);
            break;
        
        case FLIGHT_PHASE_TRANSITION:
            //controllerLQT(GAINS_LAT_CIRCLE,GAINS_LON_CIRCLE);
            break;
    }

    //crashThePlane();
    //controllerLQT(GAINS_LAT_LINE,GAINS_LON_LINE);
}

void ModeLQT::crashThePlane() {

    printf("crashing\n");
    SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 0.0);
    SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, 100.0);
    SRV_Channels::set_output_scaled(SRV_Channel::k_rudder, 100.0);
    SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, 100.0);
}

void ModeLQT::controllerLQT(float gainsLat[][6], float gainsLon[][6]) {

    // Function takes in the gain values based on case in switch statement

    float latInput[2];
    float lonInput[2];
    float latError[6];
    float lonError[6];

    // declaring the actual state of the aircraft //
    //v p r phi psi y
    float latState[6] = {currentState.velocity.y,currentState.angularVelocity.x,currentState.angularVelocity.z,currentState.roll,currentState.yaw,currentState.position.y};
    //u w q theta x z
    float lonState[6] = {currentState.velocity.x,currentState.velocity.z,currentState.angularVelocity.y,currentState.pitch,currentState.position.x,currentState.position.z};
    
    // Desired state determined by WARIO algorithm //
    //for testing, use next_wp_loc() to update desired state, probably only going to be able to compare x and y location

    //Treating desired state as movement only in the y direction so everything is held constant except our y position
    float latStateDesired[6] = {currentState.velocity.y,0,0,0,0,currentState.position.y + 1};
    float lonStateDesired[6] = {0,0,0,currentState.pitch,currentState.position.x,currentState.position.z};

    for (int i = 0; i < 6; i++) {
        latError[i] = latState[i] - latStateDesired[i];
        lonError[i] = lonState[i] - lonStateDesired[i];
    }

    matrixMathFuncs matrixTestObject;
    matrixTestObject.LQTMult(gainsLat,latError,latInput);
    matrixTestObject.LQTMult(gainsLon,lonError,lonInput);
    
    /*for (int i = 0; i < 2; i++) {
        printf("LATERAL: Computed control input in index %d is %f\n",i,latInput[i]);
    }
    printf("\n");
    for (int i = 0; i < 2; i++) {
        printf("LONGITUDINAL: Computed control input in index %d is %f\n",i,lonInput[i]);
    }*/

    // can set out own min, max, and trim values for our servos, could be useful in limitting the LQT
    SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, lonInput[1]/1000);
    SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, latInput[0]/1000);
    SRV_Channels::set_output_scaled(SRV_Channel::k_rudder, latInput[1]/1000);
    SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, lonInput[0]/1000);

}


bool ModeLQT::_enter() {
    
    /* Enters the mode, perform tasks that only need to happen on initialization */
    // trajectory.init();
    // printState();

    Location test(currentState.position.y,currentState.position.x,currentState.position.z,Location::AltFrame::ABSOLUTE);
    loc = test;

    /*
    controllerLQT(GAINS_LAT_LINE,GAINS_LON_LINE);
    controllerLQT(GAINS_LAT_CIRCLE,GAINS_LON_CIRCLE);
    */

    return true;
}


void ModeLQT::update() {
    
    /* Called at 400 Hz from scheduler, other miscellaneous items can happen here */

    currentState.roll = plane.ahrs.get_roll();
    currentState.pitch = plane.ahrs.get_pitch();
    currentState.yaw = plane.ahrs.get_yaw();

    plane.ahrs.get_relative_position_NED_origin(currentState.position);
    plane.ahrs.get_velocity_NED(currentState.velocity);
    currentState.angularVelocity = plane.ahrs.get_gyro();

    controllerLQT(GAINS_LAT_LINE,GAINS_LON_LINE);

    //printf("printing current waypoint data: %d \n",plane.current_loc.alt);

    // printState();
}


void ModeLQT::navigate() {
    
    /* For trajectory and navigation -> called from Plane::navigate in ArduPlane.cpp line 109 */
    if (navigation == INACTIVE)
        return;
    
    trajectory.update();

    plane.current_loc = plane.next_WP_loc;
    plane.next_WP_loc = plane.prev_WP_loc;

    /* Update plane.next_WP_loc, plane.current_loc, plane.prev_WP_loc etc */
    //printf("RALPHIE NAVIGATING\n");
}


void ModeLQT::printState() {

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


}




