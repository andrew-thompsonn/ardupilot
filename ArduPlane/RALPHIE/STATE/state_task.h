#pragma once


#include "../../Plane.h"


typedef struct {

    Vector3f position;
    Vector3f velocity;
    Vector3f angularVelocity;

    float roll;
    float pitch;
    float yaw;
 
} aircraftState_t;

typedef struct {
    //wind direction for squircle rotation
    Vector3f windDirection;  

    //current aircraft state
    aircraftState_t currentState;

    //radius of initial orbit (m);
    float radiusOrbit; 

    //center point of radius (gps coordinates) and flight altitude
    Location centerPoint;   




} warioInput;

