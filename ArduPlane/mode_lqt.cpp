
#include "mode.h"
#include "plane.h"
//#include <AP_Mission/AP_Mission.h>

//call mode 27 from console to switch to RALPHIE mode//
void ModeLQT::run() {

    // For control system -> called from Plane::stablize() in Attitude.cpp line 503 //
    //Can use "past_interval_finish_line" from location library to check if we have passed a certain waypoint
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
    // NEED TO FIGURE OUT HOW TO KEEP MISSION UPDATING WHILE IN THIS MODE

    // Function takes in the gain values based on case in switch statement
    AP_Mission::Mission_Command myCmd;
    //plane.mission.read_cmd_from_storage(0,myCmd);
    plane.mission.get_next_nav_cmd(0,myCmd);
    //printf("next command id is: %d\n",myCmd.id);
    uint16_t nav_cmd_id = plane.mission.get_current_nav_cmd().id;
    
    //printf("mission state %d\n",plane.mission.state());
    //printf("Current command: %d\n", nav_cmd_id);
    //printf("Position of x y z: %f %f %f\n",plane.next_WP_loc.lat*LATLON_TO_M,plane.next_WP_loc.lng*LATLON_TO_M,plane.next_WP_loc.alt*LATLON_TO_M);
    //printf("altitude %d\n",myCmd.content.location.alt);
    //printf("is running? %d\n",plane.mission.get_current_nav_cmd().id);

    if (nav_cmd_id == MAV_CMD_NAV_TAKEOFF ||
        (nav_cmd_id == MAV_CMD_NAV_LAND && plane.flight_stage == AP_Vehicle::FixedWing::FLIGHT_ABORT_LAND)) {
        plane.takeoff_calc_roll();
        plane.takeoff_calc_pitch();
        plane.calc_throttle();
    } else if (nav_cmd_id == MAV_CMD_NAV_LAND) {
        plane.calc_nav_roll();
        plane.calc_nav_pitch();

        // allow landing to restrict the roll limits
        plane.nav_roll_cd = plane.landing.constrain_roll(plane.nav_roll_cd, plane.g.level_roll_limit*100UL);

        plane.calc_throttle();
    } else {
        // we are doing normal AUTO flight, the special cases
        // are for takeoff and landing
        if (nav_cmd_id != MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT) {
            plane.steer_state.hold_course_cd = -1;
        }
        plane.calc_nav_roll();
        plane.calc_nav_pitch();
        plane.calc_throttle();
    }

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
    
    //****************WAYPOINT NOTES:*******************
    //hal.util->persistent_data.waypoint_num can be used to access the current waypoint index number, this number is updated inside AP_Mission and is declared in Util.h
    //_nav_cmd.index looks like it governs whether the misison continues or not
    //line 2452 in AP_Mission.cpp continously updates the previous waypoint
    //value of MAV_CMD_NAV_WAYPOINT == 16

    //can use AP_TECS and AP_L1_Control functions to determine desired roll and pitch angles
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
    //look into set_output_scaled to see what units should be passed into the channels
    /*SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, lonInput[1]);
    SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, latInput[0]/1000);
    SRV_Channels::set_output_scaled(SRV_Channel::k_rudder, latInput[1]/1000);
    SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, lonInput[0]/1000);*/

}


bool ModeLQT::_enter()
{
    plane.next_WP_loc = plane.prev_WP_loc = plane.current_loc;
    // start or resume the mission, based on MIS_AUTORESET
    plane.mission.start_or_resume();

    if (hal.util->was_watchdog_armed()) {
        if (hal.util->persistent_data.waypoint_num != 0) {
            gcs().send_text(MAV_SEVERITY_INFO, "Watchdog: resume WP %u", hal.util->persistent_data.waypoint_num);
            plane.mission.set_current_cmd(hal.util->persistent_data.waypoint_num);
            hal.util->persistent_data.waypoint_num = 0;
        }
    }

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

    //controllerLQT(GAINS_LAT_LINE,GAINS_LON_LINE);
    controllerLQT(GAINS_LAT_CIRCLE,GAINS_LON_CIRCLE);

    //printf("printing current waypoint data: %d \n",plane.current_loc.alt);

    //printState();
}


//void ModeLQT::navigate() {
    
    /* For trajectory and navigation -> called from Plane::navigate in ArduPlane.cpp line 109 
    if (navigation == INACTIVE)
        return;
    
    trajectory.update();

    plane.current_loc = plane.next_WP_loc;
    plane.next_WP_loc = plane.prev_WP_loc;

    /* Update plane.next_WP_loc, plane.current_loc, plane.prev_WP_loc etc */
    //printf("RALPHIE NAVIGATING\n");
//}

void ModeLQT::navigate()
{
    //printf("in nav\n");
    if (AP::ahrs().home_is_set()) {
        plane.mission.update();
    }
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




