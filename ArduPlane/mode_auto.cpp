#include "mode.h"
#include "Plane.h"

bool ModeAuto::_enter()
{
#if HAL_QUADPLANE_ENABLED
    if (plane.quadplane.available() && plane.quadplane.enable == 2) {
        plane.auto_state.vtol_mode = true;
    } else {
        plane.auto_state.vtol_mode = false;
    }
#else
    plane.auto_state.vtol_mode = false;
#endif
    plane.next_WP_loc = plane.prev_WP_loc = plane.current_loc;
    // start or resume the mission, based on MIS_AUTORESET
    //printf("State num1 auto: %d \n",plane.mission.state());
    plane.mission.start_or_resume();
    //printf("State num2 auto: %d \n",plane.mission.state());

    if (hal.util->was_watchdog_armed()) {
        if (hal.util->persistent_data.waypoint_num != 0) {
            gcs().send_text(MAV_SEVERITY_INFO, "Watchdog: resume WP %u", hal.util->persistent_data.waypoint_num);
            plane.mission.set_current_cmd(hal.util->persistent_data.waypoint_num);
            hal.util->persistent_data.waypoint_num = 0;
        }
    }

#if HAL_SOARING_ENABLED
    plane.g2.soaring_controller.init_cruising();
#endif

    return true;
}

void ModeAuto::_exit()
{
    if (plane.mission.state() == AP_Mission::MISSION_RUNNING) {
        //plane.mission.stop();

        bool restart = plane.mission.get_current_nav_cmd().id == MAV_CMD_NAV_LAND;
#if HAL_QUADPLANE_ENABLED
        if (plane.quadplane.is_vtol_land(plane.mission.get_current_nav_cmd().id)) {
            restart = false;
        }
#endif
        if (restart) {
            plane.landing.restart_landing_sequence();
        }
    }
    plane.auto_state.started_flying_in_auto_ms = 0;
    plane.lqtID = Plane::LQT_DISABLED;
}

void ModeAuto::update()
{
    //printf("Waypoint num: %d \n",hal.util->persistent_data.waypoint_num);
    if (plane.mission.state() != AP_Mission::MISSION_RUNNING) {
        //printf("a\n");
        // this could happen if AP_Landing::restart_landing_sequence() returns false which would only happen if:
        // restart_landing_sequence() is called when not executing a NAV_LAND or there is no previous nav point
        plane.set_mode(plane.mode_rtl, ModeReason::MISSION_END);
        gcs().send_text(MAV_SEVERITY_INFO, "Aircraft in auto without a running mission");
        return;
    }

    uint16_t nav_cmd_id = plane.mission.get_current_nav_cmd().id;
    if (hal.util->persistent_data.waypoint_num == 10) plane.lqtID = Plane::LQT_STRAIGHT;

#if HAL_QUADPLANE_ENABLED
    if (plane.quadplane.in_vtol_auto()) {
        //printf("b\n");
        plane.quadplane.control_auto();
        return;
    }
#endif

    if (nav_cmd_id == MAV_CMD_NAV_TAKEOFF ||
        (nav_cmd_id == MAV_CMD_NAV_LAND && plane.flight_stage == AP_Vehicle::FixedWing::FLIGHT_ABORT_LAND)) {
            //printf("c\n");
        plane.takeoff_calc_roll();
        plane.takeoff_calc_pitch();
        plane.calc_throttle();
    } else if (nav_cmd_id == MAV_CMD_NAV_LAND) {
        //printf("d\n");
        plane.calc_nav_roll();
        plane.calc_nav_pitch();

        // allow landing to restrict the roll limits
        plane.nav_roll_cd = plane.landing.constrain_roll(plane.nav_roll_cd, plane.g.level_roll_limit*100UL);

        if (plane.landing.is_throttle_suppressed()) {
            //printf("e\n");
            // if landing is considered complete throttle is never allowed, regardless of landing type
            SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 0.0);
        } else {
            //printf("f\n");
            plane.calc_throttle();
        }
#if AP_SCRIPTING_ENABLED
    } else if (nav_cmd_id == MAV_CMD_NAV_SCRIPT_TIME) {
        //printf("g\n");
        // NAV_SCRIPTING has a desired roll and pitch rate and desired throttle
        plane.nav_roll_cd = plane.ahrs.roll_sensor;
        plane.nav_pitch_cd = plane.ahrs.pitch_sensor;
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, plane.nav_scripting.throttle_pct);
#endif
    } else if (plane.lqtID != Plane::LQT_DISABLED) {
        //printf("IN fake lqt mode\n");
        currentState.roll = plane.ahrs.get_roll();
        currentState.pitch = plane.ahrs.get_pitch();
        currentState.yaw = plane.ahrs.get_yaw();

        plane.ahrs.get_relative_position_NED_origin(currentState.position);
        plane.ahrs.get_velocity_NED(currentState.velocity);
        currentState.angularVelocity = plane.ahrs.get_gyro();

        if (plane.lqtID == Plane::LQT_STRAIGHT){
            controllerLQT(GAINS_LAT_LINE,GAINS_LON_LINE);
        }
        else{
            controllerLQT(GAINS_LAT_CURVE,GAINS_LON_CURVE);
        }
        
    } else {
        ///printf("h\n");
        // we are doing normal AUTO flight, the special cases
        // are for takeoff and landing
        if (nav_cmd_id != MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT) {
            //printf("i\n");
            plane.steer_state.hold_course_cd = -1;
        }
        plane.calc_nav_roll();
        plane.calc_nav_pitch();
        plane.calc_throttle();
    }
}

void ModeAuto::navigate()
{
    if (AP::ahrs().home_is_set()) {
        plane.mission.update();
    }
}

void ModeAuto::controllerLQT(float gainsLat[][6], float gainsLon[][6]) {
    float latInput[2];
    float lonInput[2];
    float latError[6];
    float lonError[6];

    // declaring the actual state of the aircraft // units -> meters, m/s, deg, deg/s
    //v p r phi psi y
    float latState[6] = {currentState.velocity.y,currentState.angularVelocity.x,currentState.angularVelocity.z,currentState.roll,currentState.yaw,currentState.position.y};
    //u w q theta x z
    float lonState[6] = {currentState.velocity.x,currentState.velocity.z,currentState.angularVelocity.y,currentState.pitch,currentState.position.x,currentState.position.z};
    //printState();
    
    // Desired state determined by WARIO algorithm //
    //for testing, use next_wp_loc() to update desired state, probably only going to be able to compare x and y location

    //can use AP_TECS and AP_L1_Control functions to determine desired roll and pitch angles
    const AP_Navigation *nav_controller = plane.nav_controller;
    float latStateDesired[6] = {currentState.velocity.y,0,0,float(plane.nav_roll_cd * 0.01),float(nav_controller->nav_bearing_cd() * 0.01),float(plane.next_WP_loc.lat*LATLON_TO_M - plane.home.lat*LATLON_TO_M)};
    float lonStateDesired[6] = {0,0,0,float(plane.nav_pitch_cd * 0.01),float(plane.next_WP_loc.lng*LATLON_TO_M - plane.home.lng*LATLON_TO_M),float(plane.next_WP_loc.alt*LATLON_TO_M - plane.home.alt*LATLON_TO_M)};

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
    SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, lonInput[1]);
    SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, latInput[0]);
    SRV_Channels::set_output_scaled(SRV_Channel::k_rudder, latInput[1]);
    SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, lonInput[0]);

}

void ModeAuto::printState() {

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

