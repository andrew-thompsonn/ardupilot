
#include "mode.h"
#include "plane.h"
#include "LQT_Constants.h"

//call mode 26 from console to switch to RALPHIE mode//
void ModeRalphie::run() {

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

void ModeRalphie::crashThePlane() {

    printf("crashing\n");
    SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 0.0);
    SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, 100.0);
    SRV_Channels::set_output_scaled(SRV_Channel::k_rudder, 100.0);
    SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, 100.0);
}


void ModeRalphie::controllerLQT(float gainsLat[][6], float gainsLon[][6]) {

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
    //NEED TO CHANGE TO WARIO VALUES
    float latStateDesired[6] = {1,5,66,85,2,20};
    float lonStateDesired[6] = {1,5,66,85,2,20};

    for (int i = 0; i < 6; i++) {
        latError[i] = latState[i] - latStateDesired[i];
        lonError[i] = lonState[i] - lonStateDesired[i];
    }

    matrixMathFuncs matrixTestObject;
    matrixTestObject.LQTMult(gainsLat,latError,latInput);
    matrixTestObject.LQTMult(gainsLon,lonError,lonInput);
    
    for (int i = 0; i < 2; i++) {
        printf("LATERAL: Computed control input in index %d is %f\n",i,latInput[i]);
    }
    printf("\n");
    for (int i = 0; i < 2; i++) {
        printf("LONGITUDINAL: Computed control input in index %d is %f\n",i,lonInput[i]);
    }
}


bool ModeRalphie::_enter() {

    /* Initialize the trajectory based on the takeoff location */ 
    trajectory.init(plane.home);

    /* Get the first waypoint of the trajectory */
    plane.prev_WP_loc = plane.next_WP_loc;
    trajectory.getFirstWaypoint(plane.next_WP_loc);

    /* TODO: what conditions should this fail? */
    return true;
}


void ModeRalphie::update() {
    
    /* Retrieve the current state of the aircraft */
    currentState.roll = plane.ahrs.get_roll();
    currentState.pitch = plane.ahrs.get_pitch();
    currentState.yaw = plane.ahrs.get_yaw();
    plane.ahrs.get_relative_position_NED_origin(currentState.position);
    plane.ahrs.get_velocity_NED(currentState.velocity);
    currentState.angularVelocity = plane.ahrs.get_gyro();
}


void ModeRalphie::navigate() {
    
    /* For trajectory and navigation -> called from Plane::navigate in ArduPlane.cpp line 109 */
    if (navigation == INACTIVE)
        return;
    
    /* Add to the wind rolling average */
    Vector3f windEstimate = plane.ahrs.wind_estimate();
    trajectory.updateAverageWind(windEstimate);

    /* Update the trajectory */
    trajectory.update();

    /* Update the active waypoint */
    nextWpPhase = trajectory.fillNextWaypoint(plane.prev_WP_loc, plane.current_loc, plane.next_WP_loc);

    /* Update navigation controller to track trajectory */
    plane.nav_controller->update_waypoint(plane.prev_WP_loc, plane.next_WP_loc);
    plane.calc_nav_roll();
    plane.calc_nav_pitch();
    plane.calc_throttle();

    if (executionCounter++ % 10 == 0) {
        trajectory.printState();
        printf("CURRENT POSITION: %.3f, %.3f, %.3f\n", (plane.current_loc.lat - plane.home.lat)*LATLON_TO_M, (plane.current_loc.lng - plane.home.lng)*LATLON_TO_M, (double)plane.current_loc.alt/100.0);
        printf("TRACKING POSITION: %.3f, %.3f, %.3f\n", (plane.next_WP_loc.lat - plane.home.lat)*LATLON_TO_M, (plane.next_WP_loc.lng - plane.home.lng)*LATLON_TO_M, (double)plane.next_WP_loc.alt/100.0);
        printf("WIND ESTIMATE: %.3f\n", trajectory.currentWindAngleEstimate*RAD_TO_DEG);
        float yaw = plane.ahrs.get_yaw();
        printf("TRAJECTORY PHASE: %d\n", nextWpPhase);
        printf("HEADING: %.3f\n\n", yaw*RAD_TO_DEG);
    }

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


}



