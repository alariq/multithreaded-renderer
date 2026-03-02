#ifndef WINGVIZ_WING_CONTROLLER_H
#define WINGVIZ_WING_CONTROLLER_H

#include <cstddef>
#include <deque>

typedef float r32;

#include "engine/utils/vec.h"
#include "engine/utils/math_utils.h"

struct WingControllerConfig {
    r32 vyMax;
    r32 pitchAuthorityGain;
    r32 verticalSpeedTimeConstant;
    r32 horizontalSpeedDistanceGain;
    r32 nonFinalWaypointMinSpeed;
    r32 desiredSpeedSmoothingTimeConstant;
    r32 headingErrorRollGain;
    r32 lateralPositionAccelGain;
    r32 lateralSpeedAccelDamping;
    r32 approachSlowdownDistance;
    r32 brakeAltitudeWindow;
    r32 brakeDistanceMargin;
    r32 altitudeErrorPitchGain;
    r32 waypointDirectionPitchGain;
    r32 verticalSpeedPitchDamping;
    r32 thrustSpeedGain;
    r32 turnSpeedLimitSafetyFactor;
    r32 turnSpeedLimitLookaheadDistanceFactor;
    r32 headingSpeedCapStartDeg;
    r32 headingSpeedCapFullDeg;
    r32 climbSpeedLimitSafetyFactor;

    WingControllerConfig();
};

struct WingControllerDebugState {
    r32 headingErrorRad;
    r32 desiredHorSpeedRaw;
    r32 desiredHorSpeedFiltered;
    r32 turnSpeedLimit;
    r32 climbSpeedLimit;

    WingControllerDebugState()
        : headingErrorRad(0.0),
          desiredHorSpeedRaw(0.0),
          desiredHorSpeedFiltered(0.0),
          turnSpeedLimit(0.0),
          climbSpeedLimit(0.0) {}
};

struct AircraftState {
    vec3 position;
    vec3 velocity;
    vec3 acceleration;
    vec3 forward;
    vec3 up;
    vec3 right;
    r32 speed;
    r32 thrust;

    AircraftState();

    static AircraftState interp(const AircraftState& prev, const AircraftState& curr, double t) {
        t = saturate(t);
        AircraftState out = curr;

        out.position = lerp(prev.position, curr.position, t);
        out.velocity = lerp(prev.velocity, curr.velocity, t);
        out.acceleration = lerp(prev.acceleration, curr.acceleration, t);

        out.forward = lerp(prev.forward, curr.forward, t);
        out.up = lerp(prev.up, curr.up, t);
        out.right = lerp(prev.right, curr.right, t);
        orthonormalize_basis(out.forward, out.up, out.right);

        out.speed = lerp(prev.speed, curr.speed, t);
        out.thrust = lerp(prev.thrust, curr.thrust, t);

        return out;
    }
};

struct Waypoint {
    vec3 position;
    r32 reachRadius;
    bool isFinal;

    Waypoint(const vec3& pos = vec3(0, 0, 0), r32 radius = 20.0, bool finalFlag = true)
        : position(pos), reachRadius(radius), isFinal(finalFlag) {}
};

class AircraftPhysics {
public:
    void update(const vec4& controlInputs, WingControllerConfig config, double dt);
    r32 getEffectiveHeading() const;
    vec3 calculateForwardVector() const;
    vec3 calculateUpVector() const;
    vec3 calculateRightVector() const;

    const AircraftState& getState() const;
    AircraftState& getState();

    const WingControllerConfig& getConfig() const;
    WingControllerConfig& mutableConfig();
    void setConfig(const WingControllerConfig& newConfig);

private:
    AircraftState state;
    //r32 dt;
    //WingControllerConfig config;

    r32 calculateLift(r32 airspeed) const;
    r32 calculateDrag(r32 airspeed) const;
    vec3 calculateThrust() const;
};

class WingOnlyAIController {
public:
    WingOnlyAIController();

    void setTargetPosition(const vec3& position);
    void addWaypoint(const vec3& position, r32 reachRadius, bool isFinal);
    void addWaypoint(const Waypoint& waypoint);
    void clearWaypoints();
    void setTargetSpeed(r32 speed);

    //vec3 getControlInputs();
    vec4 getControlInputs(const WingControllerConfig& cfg, const AircraftState state, const double dt);

    void evade(const vec3& threatPosition, r32 threatRadius, const AircraftState state);
    bool isAtTarget() const;
    void resetTracking();
    bool isTracking() const;
    const Waypoint* getActiveWaypoint() const;
    std::size_t getQueuedWaypointCount() const;

    //const WingControllerConfig& getConfig() const;
    //WingControllerConfig& mutableConfig();
    //void setConfig(const WingControllerConfig& newConfig);
    WingControllerDebugState getDebugState() const;

private:
    //AircraftPhysics& aircraft;
    std::deque<Waypoint> waypointQueue;
    Waypoint activeWaypoint;
    bool hasActiveWaypoint;
    bool finalWaypointReached;
    r32 targetSpeed;
    bool trackingEnabled;
    bool desiredSpeedFilterInitialized;
    r32 desiredHorSpeedFiltered;
    WingControllerDebugState debugState;
};

#endif
