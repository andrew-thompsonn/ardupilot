#include "aircraft_state.h"


AircraftState::AircraftState(Vector3f positionIn, Vector3f  velocityIn, Vector3f eulerIn, 
                             Vector3f angularIn) {

    memcpy(&inertialPosition, &positionIn, sizeof(Vector3f));
    memcpy(&inertialVelocity, &velocityIn, sizeof(Vector3f));
    memcpy(&eulerAngles, &eulerIn, sizeof(Vector3f));
    memcpy(&angularVelocity, &angularIn, sizeof(Vector3f));

    flightState = DEFAULT_FLIGHT_PHASE;
}

void AircraftState::setInertialPosition(Vector3f positionIn) {
    memcpy(&inertialPosition, &positionIn, sizeof(Vector3f));
}

void AircraftState::getInertialPosition(Vector3f *positionOut) {
    memcpy(positionOut, &inertialPosition, sizeof(Vector3f));
}

void AircraftState::setInertialVelocity(Vector3f velocityIn) {
    memcpy(&inertialVelocity, &velocityIn, sizeof(Vector3f));
}

void AircraftState::getInertialVelocity(Vector3f *velocityOut) {
    memcpy(velocityOut, &inertialPosition, sizeof(Vector3f));
}

void AircraftState::setEulerAngles(Vector3f eulerIn) {
    memcpy(&eulerAngles, &eulerIn, sizeof(Vector3f));
}

void AircraftState::getEulerAngles(Vector3f *eulerOut) {
    memcpy(eulerOut, &eulerAngles, sizeof(Vector3f));
}

void AircraftState::setAngularVelocity(Vector3f velocityIn) {
    memcpy(&angularVelocity, &velocityIn, sizeof(Vector3f));
}

void AircraftState::getAngularVelocity(Vector3f *velocityOut) {
    memcpy(velocityOut, &angularVelocity, sizeof(Vector3f));
}

void AircraftState::setFlightPhase(trajectoryPhase_t phase) {
    flightState = phase;
}

trajectoryPhase_t AircraftState::getFlightPhase() {
    return flightState;
}