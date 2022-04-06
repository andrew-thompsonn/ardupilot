
#include "mode.h"
#include "plane.h"
#include "LQT_Constants.h"

//call mode 26 from console to switch to RALPHIE mode//
void ModeRalphie::run() {

    /* For control system -> called from Plane::stablize() in Attitude.cpp line 503 */
    // switch (desiredState.phase) {
     
    //     case FLIGHT_PHASE_CIRCLE:
    //         controllerLQT(GAINS_LAT_CIRCLE,GAINS_LON_CIRCLE);
    //         break;
      
    //     case FLIGHT_PHASE_STRAIGHT:
    //         controllerLQT(GAINS_LAT_LINE,GAINS_LON_LINE);
    //         break;
       
    //     case FLIGHT_PHASE_SEMI_CIRCLE:
    //         controllerLQT(GAINS_LAT_CURVE,GAINS_LON_CURVE);
    //         break;
        
    //     case FLIGHT_PHASE_TRANSITION:
    //         controllerLQT(GAINS_LAT_CIRCLE,GAINS_LON_CIRCLE);
    //         break;
    // }

    controllerLQT(GAINS_LAT_LINE,GAINS_LON_LINE);
    plane.calc_nav_roll();
}

void ModeRalphie::crashThePlane() {

    printf("crashing\n");
    SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 0.0);
    SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, 100.0);
    SRV_Channels::set_output_scaled(SRV_Channel::k_rudder, 100.0);
    SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, 100.0);
}

void ModeRalphie::controllerLQT(float gainsLat[][6], float gainsLon[][6])
{
    // comparison between actual and desired is being done correctly, issues could be with tuning or actual weights
    float latInput[2];
    float lonInput[2];
    float latError[6];
    float lonError[6];

    const AP_Navigation *nav_controller = plane.nav_controller;
    // const AP_TECS *tecs_controller = plane.AP_TECS;

    // declaring the actual state of the aircraft // units -> meters, m/s, deg, deg/s
    // v p r phi psi y
    float latState[6] = {currentState.velocity.y, currentState.angularVelocity.x, currentState.angularVelocity.z, currentState.roll * float(RAD_TO_DEG), currentState.yaw * float(RAD_TO_DEG), currentState.position.y};
    // u w q theta x z
    float lonState[6] = {currentState.velocity.x, currentState.velocity.z, currentState.angularVelocity.y, currentState.pitch * float(RAD_TO_DEG), currentState.position.x, currentState.position.z};

    // printState();

    // Desired state determined by WARIO algorithm //
    // for testing, use next_wp_loc() to update desired state, probably only going to be able to compare x and y location

    // can use AP_TECS and AP_L1_Control functions to determine desired roll and pitch angles
    // can offset using plane.home.lat*LATLON_TO_M
    // lat is x component, lng is y component
    // float latStateDesired[6] = {currentState.velocity.y,0,0,float(plane.nav_roll_cd * 0.01),float(nav_controller->nav_bearing_cd() * 0.01),float(plane.next_WP_loc.lng*LATLON_TO_M - plane.home.lng*LATLON_TO_M)};
    // float lonStateDesired[6] = {currentState.velocity.x,currentState.velocity.z,0,float(plane.nav_pitch_cd * 0.01),float(plane.next_WP_loc.lat*LATLON_TO_M - plane.home.lat*LATLON_TO_M),float(plane.next_WP_loc.alt*LATLON_TO_M - plane.home.alt*LATLON_TO_M)};
    float latStateDesired[6] = {12, 0, 0, 0, float(nav_controller->nav_bearing_cd() * 0.01), float(plane.next_WP_loc.lng * LATLON_TO_M - plane.home.lng * LATLON_TO_M)};
    float lonStateDesired[6] = {0, 0, 0, WP_ten_pitch, float(plane.next_WP_loc.lat * LATLON_TO_M - plane.home.lat * LATLON_TO_M), -float(plane.next_WP_loc.alt - plane.home.alt) / 100};
    // printf("wanted: %f, actual: %f\n\n",-float(plane.next_WP_loc.alt - plane.home.alt)/100,currentState.position.z);
    // printVector(latStateDesired); printVector(lonStateDesired); printf("\n");

    // CHECK TO MAKE SURE UNITS OF DESIRED AND ACTUAL ARE THE SAME
    // printf("difference value:\nlat = %f\nlon = %f\nalt = %f\n",float(plane.next_WP_loc.lat*LATLON_TO_M - plane.home.lat*LATLON_TO_M),float(plane.next_WP_loc.lng*LATLON_TO_M - plane.home.lng*LATLON_TO_M),float(plane.next_WP_loc.alt*LATLON_TO_M - plane.home.alt*LATLON_TO_M));

    for (int i = 0; i < 6; i++)
    {
        latError[i] = latStateDesired[i] - latState[i];
        lonError[i] = lonStateDesired[i] - lonState[i];
        // latError[i] = 0; lonError[i] = 0;
    }
    lonError[5] = 0;
    // latError[5] = 0; lonError[4] = 0; lonError[5] = 0;
    // printf("error values for the x y and z: %f %f %f\n",latError[5],lonError[4],lonError[5]);
    // printState(); printf("actual alt in non-relative frame: %f\n",currentState.position.z*float(LATLON_TO_M_INV) + float(plane.home.alt*LATLON_TO_M_INV));
    // printDesired(lonStateDesired,latStateDesired);
    // printDesired(lonError,latError);

    matrixMathFuncs matrixTestObject;
    matrixTestObject.LQTMult(gainsLat, latError, latInput);
    matrixTestObject.LQTMult(gainsLon, lonError, lonInput);

    /*for (int i = 0; i < 2; i++) {
        printf("LATERAL: Computed control input in index %d is %f\n",i,latInput[i]);
    }
    for (int i = 0; i < 2; i++) {
        printf("LONGITUDINAL: Computed control input in index %d is %f\n",i,lonInput[i]);
    }
    printf("\n");*/

    // can set out own min, max, and trim values for our servos, could be useful in limitting the LQT
    // look into set_output_scaled to see what units should be passed into the channels

    // printf("throttle: %f\n",lonInput[1]*100.0f);
    // printf("elevator: %f\n",lonInput[0]);
    // printf("airleron: %f\n",latInput[0]);
    // printf("rudder: %f\n",latInput[1]);
    lonInput[1] = constrain_float(lonInput[1] * 100.0f, 0, 75);
    SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, lonInput[1]);
    // SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, tecs_controller->get_throttle_demand()); //--> cant use get_throttle_demand() to determine the desired throttle, same function is used in calc_throttle()
    // printf("throttle: %f\n",SRV_Channels::get_output_scaled(SRV_Channel::k_throttle));
    SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, latInput[0]);
    // printf("aileron: %f\n",SRV_Channels::get_output_scaled(SRV_Channel::k_aileron)/100);
    SRV_Channels::set_output_scaled(SRV_Channel::k_rudder, latInput[1]);
    // printf("rudder: %f\n",SRV_Channels::get_output_scaled(SRV_Channel::k_rudder)/100);
    SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, lonInput[0]);
    // printf("elevator: %f\n\n",SRV_Channels::get_output_scaled(SRV_Channel::k_elevator)/100);

    // printf("Actual y: %.3f\nDesired y: %.3f\n",currentState.position.y,float(plane.next_WP_loc.lng*LATLON_TO_M - plane.home.lng*LATLON_TO_M));
    // printf("Error between them: %.3f\n\n",latError[5]);
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
    
    float alt = (float)plane.current_loc.alt /100.0;

    if (!readyForLQT) {
        if (nextWpPhase == FLIGHT_PHASE_STRAIGHT) {
            readyForLQT = true;
        }
        return;
    }

    if (nextWpPhase == FLIGHT_PHASE_TRANSITION) {
        controls = false;
        return;
    }

    if (!controls)
    {
        if (alt > (0.01)*plane.next_WP_loc.alt - 5   &&  alt < (0.01)*plane.next_WP_loc.alt + 5) {
            controls = true;
        }
    }
    else {
        WP_ten_pitch = float(plane.nav_pitch_cd * 0.01);
        if (alt < (0.01)*plane.next_WP_loc.alt - 50  ||  alt > (0.01)*plane.next_WP_loc.alt + 50) {
            controls = false;
        }
    }
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

    if (!controls) {
        plane.calc_nav_roll();
        plane.calc_nav_pitch();
        plane.calc_throttle();
    }

    if (executionCounter++ % 10 == 0) {
        trajectory.printState();
        printf("CURRENT POSITION: %.3f, %.3f, %.3f\n", (plane.current_loc.lat - plane.home.lat)*LATLON_TO_M, (plane.current_loc.lng - plane.home.lng)*LATLON_TO_M, (double)plane.current_loc.alt/100.0);
        printf("TRACKING POSITION: %.3f, %.3f, %.3f\n", (plane.next_WP_loc.lat - plane.home.lat)*LATLON_TO_M, (plane.next_WP_loc.lng - plane.home.lng)*LATLON_TO_M, (double)plane.next_WP_loc.alt/100.0);
        printf("WIND ESTIMATE: %.3f\n", trajectory.currentWindAngleEstimate*RAD_TO_DEG);
        float yaw = plane.ahrs.get_yaw();
        printf("TRAJECTORY PHASE: %d\n", nextWpPhase);
        printf("HEADING: %.3f\n\n", yaw*RAD_TO_DEG);

        if (!controls) 
            printf("LQT is DISABLED\n");
        else 
            printf("LQT is ENABLED\n");

        printf("Lower bound: %.3f\n", (0.01)*plane.next_WP_loc.alt - 5);
        printf("Upper bound: %.3f\n", (0.01)*plane.next_WP_loc.alt + 5);
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



