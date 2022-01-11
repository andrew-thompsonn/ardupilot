#ifndef _AIRCRAFT_STATE_H_
#define _AIRCRAFT_STATE_H_

#include "../../../libraries/AP_Math/vector3.h"
#include <string.h>
#include <stdint.h>

#define DEFAULT_FLIGHT_PHASE FLIGHT_PHASE_CIRCLE;


typedef enum {

    FLIGHT_PHASE_CRUISE,        /* Straight */
    FLIGHT_PHASE_TURN,          /* Semi-circle */
    FLIGHT_PHASE_TRANSITION,    /* Transition circle */
    FLIGHT_PHASE_CIRCLE         /* Full default circle */
} trajectoryPhase_t;


class AircraftState {

    /* 3x1 vectors for full 12x1 state */
    Vector3f inertialPosition;
    Vector3f inertialVelocity;
    Vector3f eulerAngles;
    Vector3f angularVelocity;

    /* Phase of trajectory */
    trajectoryPhase_t flightState;

public: /* TODO: add get/set for trajectory phase */

    /* Constructor */
    AircraftState(Vector3f positionIn, Vector3f  velocityIn, Vector3f eulerIn, Vector3f angularIn);

    /* Get/Set inertial position */
    void setInertialPosition(Vector3f positionIn);
    void getInertialPosition(Vector3f *positionOut); 

    void setInertialVelocity(Vector3f velocityIn);
    void getInertialVelocity(Vector3f *velocityOut);

    /* Get/Set euler angles */
    void setEulerAngles(Vector3f eulerIn);
    void getEulerAngles(Vector3f *eulerOut);

    /* Get/Set angular velocity vector */
    void setAngularVelocity(Vector3f angularIn);
    void getAngularVelocity(Vector3f *angularOut);

    /* Get/Set trajectory phase */
    void setFlightPhase(trajectoryPhase_t phase);
    trajectoryPhase_t getFlightPhase();
};


#endif /* _AIRCRAFT_STATE_H_ */