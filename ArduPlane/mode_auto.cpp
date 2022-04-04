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
    plane.lqtID = Plane::LQT_DISABLED;
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

    /*printf("throttle auto: %f\n",SRV_Channels::get_output_scaled(SRV_Channel::k_throttle));
    printf("aileron auto: %f\n",SRV_Channels::get_output_scaled(SRV_Channel::k_aileron)/100);
    printf("rudder auto: %f\n",SRV_Channels::get_output_scaled(SRV_Channel::k_rudder)/100);
    printf("elevator auto: %f\n\n",SRV_Channels::get_output_scaled(SRV_Channel::k_elevator)/100);*/

    currentState.roll = plane.ahrs.get_roll();
    currentState.pitch = plane.ahrs.get_pitch();
    currentState.yaw = plane.ahrs.get_yaw();

    plane.ahrs.get_relative_position_NED_origin(currentState.position);
    plane.ahrs.get_velocity_NED(currentState.velocity);
    currentState.angularVelocity = plane.ahrs.get_gyro();

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
    /*if (hal.util->persistent_data.waypoint_num == 12) {
        //printf("Changing lqtID from: %d ",plane.lqtID);
        plane.lqtID = Plane::LQT_STRAIGHT;
        WP_ten_pitch = float(plane.nav_pitch_cd * 0.01);
        //printf("to %d\n",plane.lqtID);
    }*/

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
        //printf("Current lqtID value is %d\n",plane.lqtID);

        if (plane.lqtID == Plane::LQT_STRAIGHT){
            //printf("Using Straight line gains\n");
            controllerLQT(GAINS_LAT_LINE,GAINS_LON_LINE);
            plane.calc_nav_roll();
            //plane.calc_nav_roll(); plane.calc_nav_pitch(); plane.calc_throttle();
            if (hal.util->persistent_data.waypoint_num == 45 || hal.util->persistent_data.waypoint_num == 93){
                //plane.lqtID = Plane::LQT_CURVED;
            } 
        }
        else{
            //wp 45 start, 57 end -- 93 start
            controllerLQT(GAINS_LAT_CURVE,GAINS_LON_CURVE);
            if (hal.util->persistent_data.waypoint_num == 57){
                plane.lqtID = Plane::LQT_STRAIGHT;
            } 
        }
        
    } else {
        //printf("h\n");
        // we are doing normal AUTO flight, the special cases
        // are for takeoff and landing
        printf("\n\nSWITCHING TO RALPHIE\n\n");
        plane.set_mode(plane.mode_ralphie, ModeReason::INITIALISED);
        if (nav_cmd_id != MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT) {
            //printf("i\n");
            plane.steer_state.hold_course_cd = -1;
        }
        plane.calc_nav_roll();
        plane.calc_nav_pitch();
        plane.calc_throttle();
    }

    if (currentState.position.z > -50 || currentState.position.z < -300){
        plane.lqtID = Plane::LQT_DISABLED;
        printf("LQT is DISABLED\n");
        flag_bounds = false;
        if(currentState.position.z > -250){
            flag_bounds = true;
        }
    }
    else if (currentState.position.z < -50 && flag_bounds){
        plane.lqtID = Plane::LQT_STRAIGHT;
        WP_ten_pitch = float(plane.nav_pitch_cd * 0.01);
        printf("LQT is ENABLED\n");
    }

}

void ModeAuto::navigate()
{
    if (AP::ahrs().home_is_set()) {
        plane.mission.update();
    }
}

void ModeAuto::controllerLQT(float gainsLat[][6], float gainsLon[][6]) {
    //comparison between actual and desired is being done correctly, issues could be with tuning or actual weights
    float latInput[2];
    float lonInput[2];
    float latError[6];
    float lonError[6];

    const AP_Navigation *nav_controller = plane.nav_controller;
    //const AP_TECS *tecs_controller = plane.AP_TECS;

    // declaring the actual state of the aircraft // units -> meters, m/s, deg, deg/s
    //v p r phi psi y
    float latState[6] = {currentState.velocity.y,currentState.angularVelocity.x,currentState.angularVelocity.z,currentState.roll*float(RAD_TO_DEG),currentState.yaw*float(RAD_TO_DEG),currentState.position.y};
    //u w q theta x z
    float lonState[6] = {currentState.velocity.x,currentState.velocity.z,currentState.angularVelocity.y,currentState.pitch*float(RAD_TO_DEG),currentState.position.x,currentState.position.z};
    
    //printState();
    
    // Desired state determined by WARIO algorithm //
    //for testing, use next_wp_loc() to update desired state, probably only going to be able to compare x and y location

    //can use AP_TECS and AP_L1_Control functions to determine desired roll and pitch angles
    //can offset using plane.home.lat*LATLON_TO_M
    //lat is x component, lng is y component
    //float latStateDesired[6] = {currentState.velocity.y,0,0,float(plane.nav_roll_cd * 0.01),float(nav_controller->nav_bearing_cd() * 0.01),float(plane.next_WP_loc.lng*LATLON_TO_M - plane.home.lng*LATLON_TO_M)};
    //float lonStateDesired[6] = {currentState.velocity.x,currentState.velocity.z,0,float(plane.nav_pitch_cd * 0.01),float(plane.next_WP_loc.lat*LATLON_TO_M - plane.home.lat*LATLON_TO_M),float(plane.next_WP_loc.alt*LATLON_TO_M - plane.home.alt*LATLON_TO_M)};
    float latStateDesired[6] = {22,0,0,0,float(nav_controller->nav_bearing_cd() * 0.01),float(plane.next_WP_loc.lng*LATLON_TO_M - plane.home.lng*LATLON_TO_M)};
    float lonStateDesired[6] = {0,0,0,WP_ten_pitch,float(plane.next_WP_loc.lat*LATLON_TO_M - plane.home.lat*LATLON_TO_M),-float(plane.next_WP_loc.alt - plane.home.alt)/100};
    //printf("wanted: %f, actual: %f\n\n",-float(plane.next_WP_loc.alt - plane.home.alt)/100,currentState.position.z);
    //printVector(latStateDesired); printVector(lonStateDesired); printf("\n");

    // CHECK TO MAKE SURE UNITS OF DESIRED AND ACTUAL ARE THE SAME
    //printf("difference value:\nlat = %f\nlon = %f\nalt = %f\n",float(plane.next_WP_loc.lat*LATLON_TO_M - plane.home.lat*LATLON_TO_M),float(plane.next_WP_loc.lng*LATLON_TO_M - plane.home.lng*LATLON_TO_M),float(plane.next_WP_loc.alt*LATLON_TO_M - plane.home.alt*LATLON_TO_M));

    for (int i = 0; i < 6; i++) {
        latError[i] = latStateDesired[i] - latState[i];
        lonError[i] = lonStateDesired[i] - lonState[i];
        //latError[i] = 0; lonError[i] = 0;
    }
    lonError[5] = 0;
    //latError[5] = 0; lonError[4] = 0; lonError[5] = 0;
    //printf("error values for the x y and z: %f %f %f\n",latError[5],lonError[4],lonError[5]);
    //printState(); printf("actual alt in non-relative frame: %f\n",currentState.position.z*float(LATLON_TO_M_INV) + float(plane.home.alt*LATLON_TO_M_INV));
    //printDesired(lonStateDesired,latStateDesired);
    //printDesired(lonError,latError);

    matrixMathFuncs matrixTestObject;
    matrixTestObject.LQTMult(gainsLat,latError,latInput);
    matrixTestObject.LQTMult(gainsLon,lonError,lonInput);
    
    /*for (int i = 0; i < 2; i++) {
        printf("LATERAL: Computed control input in index %d is %f\n",i,latInput[i]);
    }
    for (int i = 0; i < 2; i++) {
        printf("LONGITUDINAL: Computed control input in index %d is %f\n",i,lonInput[i]);
    }
    printf("\n");*/

    // can set out own min, max, and trim values for our servos, could be useful in limitting the LQT
    //look into set_output_scaled to see what units should be passed into the channels

    //printf("throttle: %f\n",lonInput[1]*100.0f);
    //printf("elevator: %f\n",lonInput[0]);
    //printf("airleron: %f\n",latInput[0]);
    //printf("rudder: %f\n",latInput[1]);
    lonInput[1] = constrain_float(lonInput[1]*100.0f,0,100);
    SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, lonInput[1]);
    //SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, tecs_controller->get_throttle_demand()); //--> cant use get_throttle_demand() to determine the desired throttle, same function is used in calc_throttle()
    //printf("throttle: %f\n",SRV_Channels::get_output_scaled(SRV_Channel::k_throttle));
    SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, latInput[0]);
    //printf("aileron: %f\n",SRV_Channels::get_output_scaled(SRV_Channel::k_aileron)/100);
    SRV_Channels::set_output_scaled(SRV_Channel::k_rudder, latInput[1]);
    //printf("rudder: %f\n",SRV_Channels::get_output_scaled(SRV_Channel::k_rudder)/100);
    SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, lonInput[0]);
    //printf("elevator: %f\n\n",SRV_Channels::get_output_scaled(SRV_Channel::k_elevator)/100);

    //printf("Actual y: %.3f\nDesired y: %.3f\n",currentState.position.y,float(plane.next_WP_loc.lng*LATLON_TO_M - plane.home.lng*LATLON_TO_M));
    //printf("Error between them: %.3f\n\n",latError[5]);

}

void ModeAuto::printState() {
    printf("ACTUAL:\n");
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

void ModeAuto::printVector(float vector[6]){
    for (int i = 0; i < 6; i++) {
        printf("Vector index %d, %f\n",i,vector[i]);
        
    }

}

void ModeAuto::printDesired(float lonVector[6], float latVector[6]){
    //v p r phi psi y - lat
    //u w q theta x z - lng
    printf("DESIRED:\n");
    printf("Position: %.3f, %.3f, %.3f\n", lonVector[4], 
                                           latVector[5], 
                                           lonVector[5]);

	printf("Velocity: %.3f, %.3f, %.3f\n", lonVector[0], 
                                           latVector[0], 
                                           lonVector[1]);

	printf("Angles:   %.3f, %.3f, %.3f\n", latVector[3], 
                                           lonVector[3], 
                                           latVector[4]);

	printf("Omega:    %.3f, %.3f, %.3f\n\n", latVector[1], 
                                             lonVector[2], 
                                             latVector[2]);

}

