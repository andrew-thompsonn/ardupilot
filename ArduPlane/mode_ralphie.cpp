
#include "mode.h"
#include "plane.h"
#include "LQT_Constants.h"


void ModeRalphie::run() {

    /* Run the LQT, calculate desired roll from navigation controller */
    controllerLQT(GAINS_LAT_LINE,GAINS_LON_LINE);
    plane.calc_nav_roll();
}


void ModeRalphie::crashThePlane() {

    /* Crash the plane, used for debugging only */
    SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 0.0);
    SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, 100.0);
    SRV_Channels::set_output_scaled(SRV_Channel::k_rudder, 100.0);
    SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, 100.0);
}


void ModeRalphie::controllerLQT(float gainsLat[][6], float gainsLon[][6]) {

    float latInput[2];
    float lonInput[2];
    float latError[6];
    float lonError[6];

    const AP_Navigation *nav_controller = plane.nav_controller;
    float latState[6] = {currentState.velocity.y, currentState.angularVelocity.x, currentState.angularVelocity.z, currentState.roll * float(RAD_TO_DEG), currentState.yaw * float(RAD_TO_DEG), currentState.position.y};
    float lonState[6] = {currentState.velocity.x, currentState.velocity.z, currentState.angularVelocity.y, currentState.pitch * float(RAD_TO_DEG), currentState.position.x, currentState.position.z};

    float latStateDesired[6] = {11, 0, 0, 0, float(nav_controller->nav_bearing_cd() * 0.01), float(plane.next_WP_loc.lng * LATLON_TO_M - plane.home.lng * LATLON_TO_M)};
    float lonStateDesired[6] = {0, 0, 0, WP_ten_pitch, float(plane.next_WP_loc.lat * LATLON_TO_M - plane.home.lat * LATLON_TO_M), -float(plane.next_WP_loc.alt - plane.home.alt) / 100};

    for (int i = 0; i < 6; i++) {
        latError[i] = latStateDesired[i] - latState[i];
        lonError[i] = lonStateDesired[i] - lonState[i];
    }
    lonError[5] = 0;

    matrixMathFuncs matrixTestObject;
    matrixTestObject.LQTMult(gainsLat, latError, latInput);
    matrixTestObject.LQTMult(gainsLon, lonError, lonInput);

    // can set out own min, max, and trim values for our servos, could be useful in limitting the LQT
    // look into set_output_scaled to see what units should be passed into the channels
    lonInput[1] = constrain_float(lonInput[1] * 100.0f, 0, 100);

    SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, lonInput[1]);
    SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, latInput[0]);
    SRV_Channels::set_output_scaled(SRV_Channel::k_rudder, latInput[1]);
    SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, lonInput[0]);
}

bool ModeRalphie::_enter() {

    /* Initialize the trajectory based on the takeoff location */ 
    trajectory.init(plane.home);

    /* Set the center of the mission area */
    int32_t latCenter = (int32_t)40.084712E7; //plane.home.lat;
    int32_t lngCenter = (int32_t)-105.231767E7; //plane.home.lng + trajectory.parameters.rad*LATLON_TO_M_INV;
    trajectory.setPointOfInterest(latCenter, lngCenter);

    /* Get the first waypoint of the trajectory */
    plane.prev_WP_loc = plane.next_WP_loc;
    trajectory.getFirstWaypoint(plane.next_WP_loc);

    /* TODO: what conditions should this fail? */
    return true;
}


void ModeRalphie::update() {
    
    /* Euler angles */
    currentState.roll = plane.ahrs.get_roll();
    currentState.pitch = plane.ahrs.get_pitch();
    currentState.yaw = plane.ahrs.get_yaw();

    /* Intertial position/velocity */
    plane.ahrs.get_relative_position_NED_origin(currentState.position);
    plane.ahrs.get_velocity_NED(currentState.velocity);

    /* Angular velocity */
    currentState.angularVelocity = plane.ahrs.get_gyro();

    /* Altitude */ 
    float alt = (float)plane.current_loc.alt /100.0;
    
    /* Update the active waypoint and previous waypoint */
    nextWpPhase = trajectory.fillNextWaypoint(plane.prev_WP_loc, plane.current_loc, plane.next_WP_loc);

    /* Update navigation controller to track trajectory between previous and active waypoint */
    plane.nav_controller->update_waypoint(plane.prev_WP_loc, plane.next_WP_loc);

    /* Check if we we have reached the first squircle of the flight */
    if (!readyForLQT) {
        if (nextWpPhase == FLIGHT_PHASE_STRAIGHT) {
            readyForLQT = true;
        }
        return;
    }
    /* If we have reached the first squircle and are in straight flight */
    if (nextWpPhase != FLIGHT_PHASE_STRAIGHT) {
        controls = false;
        return;
    }
    /* Altitude conditions for activating LQT */
    if (!controls) {
        if (alt > (0.01)*plane.next_WP_loc.alt - 5   &&  alt < (0.01)*plane.next_WP_loc.alt + 5) {
            controls = true;
        }
    }
    /* Altitude conditions for disabling LQT */
    else {
        WP_ten_pitch = float(plane.nav_pitch_cd * 0.01);
        if (alt < (0.01)*plane.next_WP_loc.alt - 50  ||  alt > (0.01)*plane.next_WP_loc.alt + 50) {
            controls = false;
        }
    }
}


void ModeRalphie::navigate() {
    
    /* For debugging, if WARIO navigation is disabled, return early */
    if (navigation == INACTIVE)
        return;
    
    /* Add to the wind rolling average */
    Vector3f windEstimate = plane.ahrs.wind_estimate();
    trajectory.updateAverageWind(windEstimate);

    /* Update the trajectory */
    trajectory.update();

    /* If the LQT is inactive, hand control to the navigation L1 controller */
    if (!controls) {
        plane.calc_nav_roll();
        plane.calc_nav_pitch();
        plane.calc_throttle();
    }

    /* 1 Hz print statements for debugging */
    if (executionCounter++ % 10 == 0) {
        trajectory.printState();
        float yaw = plane.ahrs.get_yaw();

        printf("CURRENT POSITION: %.3f, %.3f, %.3f\n", (plane.current_loc.lat - plane.home.lat)*LATLON_TO_M, (plane.current_loc.lng - plane.home.lng)*LATLON_TO_M, (double)plane.current_loc.alt/100.0);
        printf("TRACKING POSITION: %.3f, %.3f, %.3f\n", (plane.next_WP_loc.lat - plane.home.lat)*LATLON_TO_M, (plane.next_WP_loc.lng - plane.home.lng)*LATLON_TO_M, (double)plane.next_WP_loc.alt/100.0);
        printf("WIND ESTIMATE: %.3f\n", trajectory.currentWindAngleEstimate*RAD_TO_DEG);
        printf("TRAJECTORY PHASE: %d\n", nextWpPhase);
        printf("HEADING: %.3f\n\n", yaw*RAD_TO_DEG);

        if (!controls) 
            printf("LQT is DISABLED\n");
        else 
            printf("LQT is ENABLED\n");

        printf("Lower bound: %.3f\n", (0.01)*plane.next_WP_loc.alt - 5);
        printf("Upper bound: %.3f\n", (0.01)*plane.next_WP_loc.alt + 5);
        printf("Home: %d, %d\n", plane.home.lat, plane.home.lng);
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



