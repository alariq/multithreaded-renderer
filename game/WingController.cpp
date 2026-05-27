#include "WingController.h"

#include "engine/utils/vec.h"

#include <algorithm>
#include <cmath>
#include <cassert>
#include <cstdio>

// Plane config
static const r32 MASS = 1000.0;
static const r32 MAX_THRUST = 1000.0;
static const r32 MAX_SPEED = 300.0;
static const r32 MAX_BRAKE_FORCE = 7000.0;
static const r32 MAX_BANK_ANGLE = 45.0 * M_PI / 180.0;
static const r32 MAX_ROLL_RATE = 1.5;
static const r32 LIFT_REFERENCE_PITCH = 45.0 * M_PI / 180.0;
static const r32 FORWARD_ALIGN_RATE = 8.0;

// Environment config
static const r32 GRAVITY = 9.81;
static const r32 AIR_DENSITY = 1.225;

// Aerodynamics config
static const r32 WING_AREA = 15.0;
static const r32 AERODYNAMIC_PRESSURE_FACTOR = 0.5;
static const r32 DRAG_COEFFICIENT = 0.02;
static const r32 LIFT_BASE_COEFFICIENT = 0.50;
static const r32 LIFT_PITCH_GAIN = 0.25;
static const r32 LIFT_MIN_COEFFICIENT = 0.20;
static const r32 LIFT_MAX_COEFFICIENT = 0.90;
static const r32 LIFT_MAX_G_FACTOR = 1.25;

// Generic control constants
static const r32 CONTROL_INPUT_MIN = -1.0;
static const r32 CONTROL_INPUT_MAX = 1.0;
static const r32 TARGET_REACHED_DISTANCE = 20.0;
static const r32 EVASION_OFFSET_DISTANCE = 100.0;

// Numeric config
static const r32 EPS = 1e-6;

static r32 magnitude(const vec3& v) {
    return static_cast<r32>(length(v));
}

static vec3 rotateAroundAxis(const vec3& v, const vec3& axis, r32 angle) {
    // Rodrigues (Euler) formula
    const vec3 axisUnit = normalizeSafe(axis);
    const float c = static_cast<float>(std::cos(angle));
    const float s = static_cast<float>(std::sin(angle));
    return v * c + cross(axisUnit, v) * s + axisUnit * dot(axisUnit, v) * (1.0f - c);
}

static void orthonormalizeBasis(vec3& forward, vec3& up, vec3& right) {
    forward = normalizeSafe(forward);
    vec3 rightCandidate = cross(forward, up);
    if (length(rightCandidate) <= EPS) {
        vec3 fallbackUp(0.0f, 1.0f, 0.0f);
        if (std::abs(dot(forward, fallbackUp)) > 0.98) {
            fallbackUp = vec3(0.0f, 0.0f, 1.0f);
        }
        rightCandidate = cross(forward, fallbackUp);
    }
    right = normalizeSafe(rightCandidate);
    up = normalizeSafe(cross(right, forward));
}

static r32 clampSignedUnit(r32 value) {
    return clamp(value, -1.0f, 1.0f);
}

static r32 sanitizeReachRadius(r32 reachRadius) {
    return std::max(reachRadius, EPS);
}

static r32 getPitchFromForward(const vec3& forward) {
    return std::asin(clampSignedUnit(forward.y));
}

static vec3 getLeveledUpReference(const vec3& forward) {
    const vec3 worldUp(0.0f, 1.0f, 0.0f);
    const vec3 projected = worldUp - forward * static_cast<float>(dot(worldUp, forward));
    if (length(projected) <= EPS) {
        vec3 fallbackAxis(1.0f, 0.0f, 0.0f);
        if (std::abs(dot(forward, fallbackAxis)) > 0.98) {
            fallbackAxis = vec3(0.0f, 0.0f, 1.0f);
        }
        const vec3 right = normalizeSafe(cross(forward, fallbackAxis));
        return normalizeSafe(cross(right, forward));
    }
    return normalizeSafe(projected);
}

static r32 signedAngleAroundAxis(const vec3& fromVec, const vec3& toVec, const vec3& axis) {
    const vec3 axisUnit = normalizeSafe(axis);
    const vec3 fromProjected = normalizeSafe(fromVec - axisUnit * static_cast<float>(dot(fromVec, axisUnit)));
    const vec3 toProjected = normalizeSafe(toVec - axisUnit * static_cast<float>(dot(toVec, axisUnit)));
    const r32 sinTerm = dot(cross(fromProjected, toProjected), axisUnit);
    const r32 cosTerm = dot(fromProjected, toProjected);
    return std::atan2(sinTerm, cosTerm);
}

static WingControllerConfig sanitizeConfig(const WingControllerConfig& in) {
    WingControllerConfig out = in;
    out.vyMax = max(in.vyMax, r32(0.0));
    out.pitchAuthorityGain = max(in.pitchAuthorityGain, r32(0.0));
    out.verticalSpeedTimeConstant = max(in.verticalSpeedTimeConstant, r32(0.01));
    out.nonFinalWaypointMinSpeed = max(in.nonFinalWaypointMinSpeed, r32(0.0));
    out.desiredSpeedSmoothingTimeConstant = max(in.desiredSpeedSmoothingTimeConstant, r32(0.01));
    out.headingErrorRollGain = max(in.headingErrorRollGain, 0.0f);
    out.lateralPositionAccelGain = max(in.lateralPositionAccelGain, r32(0.0));
    out.lateralSpeedAccelDamping = max(in.lateralSpeedAccelDamping, r32(0.0));
    out.approachSlowdownDistance = max(in.approachSlowdownDistance, r32(1.0));
    out.brakeAltitudeWindow = max(in.brakeAltitudeWindow, r32(0.1));
    out.brakeDistanceMargin = max(in.brakeDistanceMargin, r32(0.1));
    out.altitudeErrorPitchGain = max(in.altitudeErrorPitchGain, r32(0));
    out.waypointDirectionPitchGain = max(in.waypointDirectionPitchGain, r32(0.0));
    out.verticalSpeedPitchDamping = max(in.verticalSpeedPitchDamping, r32(0.0));
    out.thrustSpeedGain = max(in.thrustSpeedGain, r32(0.0));
    out.turnSpeedLimitSafetyFactor = clamp(in.turnSpeedLimitSafetyFactor, r32(0.05), r32(1.0));
    out.headingSpeedCapStartDeg = clamp(in.headingSpeedCapStartDeg, r32(0.0), r32(179.0));
    out.headingSpeedCapFullDeg = clamp(in.headingSpeedCapFullDeg, r32(0.0), r32(179.0)); out.headingSpeedCapFullDeg = max(out.headingSpeedCapFullDeg, out.headingSpeedCapStartDeg + r32(1e-3));
    out.climbSpeedLimitSafetyFactor = clamp(in.climbSpeedLimitSafetyFactor, r32(0.05), r32(1.0));
    return out;
}

WingControllerConfig::WingControllerConfig()
    : vyMax(6.0),
      pitchAuthorityGain(1.0),
      verticalSpeedTimeConstant(0.5),
      //nonFinalWaypointMinSpeed(30.0),
      nonFinalWaypointMinSpeed(5.0), // m/s
      desiredSpeedSmoothingTimeConstant(0.50),
      headingErrorRollGain(0.25),
      lateralPositionAccelGain(0.030),
      lateralSpeedAccelDamping(0.55),
      approachSlowdownDistance(0.0),
      //brakeAltitudeWindow(35.0),
      brakeAltitudeWindow(3.5),
      //brakeDistanceMargin(120.0),
      brakeDistanceMargin(1.2),
      altitudeErrorPitchGain(0.06),
      waypointDirectionPitchGain(0.55),
      verticalSpeedPitchDamping(0.15),
      thrustSpeedGain(20.0),
      turnSpeedLimitSafetyFactor(0.55),
      headingSpeedCapStartDeg(15.0),
      headingSpeedCapFullDeg(120.0),
      climbSpeedLimitSafetyFactor(0.90) {}

AircraftState::AircraftState()
    : position(0, 0, 0),
      velocity(0, 0, 0),
      acceleration(0, 0, 0),
      forward(1, 0, 0),
      up(0, 1, 0),
      right(0, 0, 1),
      speed(0),
      thrust(0) {}

void AircraftPhysics::update(const vec4& controlInputs, WingControllerConfig cfg, double dt) {

    // Extract control inputs (roll, pitch, brake)
    const r32 rollInput = clamp(controlInputs.x, CONTROL_INPUT_MIN, CONTROL_INPUT_MAX);
    const r32 pitchInput = clamp(controlInputs.y, CONTROL_INPUT_MIN, CONTROL_INPUT_MAX);
    const r32 brakeInput = saturate(controlInputs.z);
    state.thrust = clamp(controlInputs.w, 0.0f, MAX_THRUST);

    // Roll input commands desired bank angle around forward.
    const r32 desiredBank = rollInput * MAX_BANK_ANGLE;
    const vec3 leveledUp = getLeveledUpReference(state.forward);
    const vec3 desiredUp = rotateAroundAxis(leveledUp, state.forward, desiredBank);
    const r32 bankError = signedAngleAroundAxis(state.up, desiredUp, state.forward);
    const r32 maxRollStep = MAX_ROLL_RATE * dt;
    const r32 rollAngleStep = clamp(bankError, -maxRollStep, maxRollStep);

    state.up = rotateAroundAxis(state.up, state.forward, rollAngleStep);
    state.right = rotateAroundAxis(state.right, state.forward, rollAngleStep);
    orthonormalizeBasis(state.forward, state.up, state.right);

    const vec3 airspeed = state.velocity;
    const r32 airspeedlength = length(airspeed);

    const r32 drag = calculateDrag(airspeedlength);
    const vec3 thrust = calculateThrust();
    const vec3 gravity(0.0f, static_cast<float>(-GRAVITY * MASS), 0.0f);

    vec3 dragForce(0.0f, 0.0f, 0.0f);
    if (airspeedlength > EPS) {
        dragForce = normalizeSafe(airspeed) * static_cast<float>(drag);
    }

    // Arcade vertical model: cancel baseline gravity, then command desired vertical speed.
    const r32 desiredVerticalSpeed = cfg.vyMax * cfg.pitchAuthorityGain * pitchInput;
    const r32 accY = (desiredVerticalSpeed - state.velocity.y) / cfg.verticalSpeedTimeConstant;
    const vec3 verticalControl(0.0f, static_cast<float>(GRAVITY * MASS + MASS * accY), 0.0f);

    vec3 totalForce = gravity + verticalControl + thrust - dragForce;
    if (airspeedlength > EPS && brakeInput > 0.0) {
        const vec3 brakeDirection = normalizeSafe(state.velocity);
        totalForce = totalForce - brakeDirection * static_cast<float>(MAX_BRAKE_FORCE * brakeInput);
    }

    // Roll creates lateral acceleration to steer horizontal velocity.
    const vec3 horizontalVelocity(state.velocity.x, 0.0f, state.velocity.z);
    const r32 horizontalSpeed = length(horizontalVelocity);
    if (horizontalSpeed > EPS) {
        const vec3 rightDir(
            static_cast<float>(-horizontalVelocity.z / horizontalSpeed),
            0.0f,
            static_cast<float>(horizontalVelocity.x / horizontalSpeed));
        // Arcade behavior: roll-to-turn without speed-based authority scaling.
        const r32 turnAccel = GRAVITY * std::tan(desiredBank);
        totalForce = totalForce + rightDir * static_cast<float>(MASS * turnAccel);
    }

    state.acceleration = totalForce / static_cast<float>(MASS);

    state.velocity = state.velocity + state.acceleration * static_cast<float>(dt);
    r32 newSpeed = length(state.velocity);
    if (newSpeed > MAX_SPEED) {
        state.velocity = normalizeSafe(state.velocity) * static_cast<float>(MAX_SPEED);
        newSpeed = MAX_SPEED;
    }

    if (newSpeed > EPS) {
        // Align nose with full velocity vector so climb/descent shows as pitch.
        const vec3 desiredForward = normalizeSafe(state.velocity);
        const r32 alignAlpha = clamp(FORWARD_ALIGN_RATE * dt, 0.0, 1.0);
        state.forward = normalizeSafe(
            state.forward * static_cast<float>(1.0 - alignAlpha) +
            desiredForward * static_cast<float>(alignAlpha));
        orthonormalizeBasis(state.forward, state.up, state.right);
    }

    state.position = state.position + state.velocity * static_cast<float>(dt);
    state.speed = length(state.velocity);
}

r32 AircraftPhysics::getEffectiveHeading() const {
    const vec3 horizontalVelocity(state.velocity.x, 0.0f, state.velocity.z);
    if (length(horizontalVelocity) > EPS) {
        return std::atan2(horizontalVelocity.z, horizontalVelocity.x);
    }
    const vec3 forward = calculateForwardVector();
    return std::atan2(forward.z, forward.x);
}

vec3 AircraftPhysics::calculateForwardVector() const {
    return normalizeSafe(state.forward);
}

vec3 AircraftPhysics::calculateUpVector() const {
    return normalizeSafe(state.up);
}

vec3 AircraftPhysics::calculateRightVector() const {
    return normalizeSafe(state.right);
}

const AircraftState& AircraftPhysics::getState() const {
    return state;
}

AircraftState& AircraftPhysics::getState() {
    return state;
}

r32 AircraftPhysics::calculateLift(r32 airspeed) const {
    const r32 pitchNorm = getPitchFromForward(state.forward) / LIFT_REFERENCE_PITCH;
    const r32 liftCoefficient = clamp(
        LIFT_BASE_COEFFICIENT + LIFT_PITCH_GAIN * pitchNorm,
        LIFT_MIN_COEFFICIENT,
        LIFT_MAX_COEFFICIENT);
    const r32 rawLift = AERODYNAMIC_PRESSURE_FACTOR * AIR_DENSITY * airspeed * airspeed * WING_AREA * liftCoefficient;
    const r32 maxLift = MASS * GRAVITY * LIFT_MAX_G_FACTOR;
    return clamp(rawLift, 0.0f, maxLift);
}

r32 AircraftPhysics::calculateDrag(r32 airspeed) const {
    return AERODYNAMIC_PRESSURE_FACTOR * AIR_DENSITY * airspeed * airspeed * WING_AREA * DRAG_COEFFICIENT;
}

vec3 AircraftPhysics::calculateThrust() const {
    const vec3 forward = calculateForwardVector();
    return forward * static_cast<float>(state.thrust);
}

WingOnlyAIController::WingOnlyAIController()
    :  waypointQueue(),
      activeWaypoint(),
      hasActiveWaypoint(false),
      finalWaypointReached(false),
      targetSpeed(0.0),
      trackingEnabled(false),
      desiredSpeedFilterInitialized(false),
      desiredHorSpeedFiltered(0.0),
      debugState() {}

void WingOnlyAIController::setTargetPosition(const vec3& position) {
    clearWaypoints();
    addWaypoint(position, TARGET_REACHED_DISTANCE, true);
}

void WingOnlyAIController::addWaypoint(const vec3& position, r32 reachRadius, bool isFinal) {
    addWaypoint(Waypoint(position, sanitizeReachRadius(reachRadius), isFinal));
}

void WingOnlyAIController::addWaypoint(const Waypoint& waypoint) {
    waypointQueue.push_back(Waypoint(
        waypoint.position,
        sanitizeReachRadius(waypoint.reachRadius),
        waypoint.isFinal));

    if (!hasActiveWaypoint) {
        activeWaypoint = waypointQueue.front();
        waypointQueue.pop_front();
        hasActiveWaypoint = true;
    }

    finalWaypointReached = false;
    trackingEnabled = true;
}

void WingOnlyAIController::clearWaypoints() {
    waypointQueue.clear();
    hasActiveWaypoint = false;
    finalWaypointReached = false;
    trackingEnabled = false;
    desiredSpeedFilterInitialized = false;
    desiredHorSpeedFiltered = 0.0;
    debugState = WingControllerDebugState();
}

void WingOnlyAIController::setTargetSpeed(r32 speed) {
    targetSpeed = clamp(speed, 0.0f, MAX_SPEED);
}

vec4 WingOnlyAIController::getControlInputs(const WingControllerConfig& cfg, const AircraftState state, const double dt) {
    vec4 controlInputs(0.0f);
    debugState.headingErrorRad = 0.0;
    debugState.desiredHorSpeedRaw = 0.0;
    debugState.desiredHorSpeedFiltered = 0.0;
    debugState.turnSpeedLimit = targetSpeed;
    debugState.climbSpeedLimit = targetSpeed;

    auto activateNextWaypoint = [this]() -> bool {
        if (!waypointQueue.empty()) {
            activeWaypoint = waypointQueue.front();
            waypointQueue.pop_front();
            hasActiveWaypoint = true;
            return true;
        }
        hasActiveWaypoint = false;
        trackingEnabled = false;
        return false;
    };

    if (!trackingEnabled) {
        return controlInputs;
    }
    if (!hasActiveWaypoint && !activateNextWaypoint()) {
        return controlInputs;
    }

    vec3 dirToTarget = activeWaypoint.position - state.position;
    r32 distanceToTarget = length(dirToTarget);

    while (hasActiveWaypoint && distanceToTarget <= activeWaypoint.reachRadius) {
        if (activeWaypoint.isFinal) {
            finalWaypointReached = true;
            hasActiveWaypoint = false;
            trackingEnabled = false;
            controlInputs.w = 0; //aircraft.setThrust(0.0);
            return controlInputs;
        }
        if (!activateNextWaypoint()) {
            return controlInputs;
        }
        dirToTarget = activeWaypoint.position - state.position;
        distanceToTarget = length(dirToTarget);
    }

    const r32 altitudeError = activeWaypoint.position.y - state.position.y;

    const vec3 horizontalToTarget(dirToTarget.x, 0.0f, dirToTarget.z);
    const r32 horDist = length(horizontalToTarget);

    const vec3 horizontalVelocity(state.velocity.x, 0.0f, state.velocity.z);
    const r32 horizontalSpeed = length(horizontalVelocity);
    vec3 forwardDirection = horizontalSpeed > EPS ? horizontalVelocity / horizontalSpeed : vec3(state.forward.x, 0.0f, state.forward.z);

    if (length(forwardDirection) <= EPS) {
        assert(0 && "only happens when we go straight up or down (forward=(0,+/-1, 0)), could be ok");
        forwardDirection = vec3(1.0f, 0.0f, 0.0f);
    } else {
        forwardDirection = normalizeSafe(forwardDirection);
    }

    // When directly under/over waypoint, keep current forward heading instead of
    // snapping to an arbitrary world axis (which causes yaw spikes/loiter).
    const vec3 targetHorDir = horDist > EPS ? horizontalToTarget / horDist : forwardDirection;
    const vec3 dirToTargetUnit = distanceToTarget > EPS ? dirToTarget / distanceToTarget : vec3(0.0f, 0.0f, 0.0f);

    const vec3 rightDirection(-forwardDirection.z, 0.0f, forwardDirection.x);

    const r32 targetHeading = std::atan2(targetHorDir.z, targetHorDir.x);
    const r32 currentHeading = std::atan2(forwardDirection.z, forwardDirection.x);
    r32 headingError = targetHeading - currentHeading;
    while (headingError > M_PI) headingError -= 2 * M_PI;
    while (headingError < -M_PI) headingError += 2 * M_PI;
    debugState.headingErrorRad = headingError;

    // 0 - no slowdown at WP
    const r32 approachFactor = cfg.approachSlowdownDistance > 0 ? saturate(horDist / cfg.approachSlowdownDistance) : 1.0f;

    // Slow down for all waypoints, but keep a non-zero pass-through floor
    // on intermediate points to avoid low-speed handoff yaw snaps.
    const r32 minSpeed = std::min(activeWaypoint.isFinal ? 0 : cfg.nonFinalWaypointMinSpeed, targetSpeed);
    r32 desiredHorSpeedRaw = lerp(minSpeed, targetSpeed, approachFactor);
    debugState.desiredHorSpeedRaw = desiredHorSpeedRaw;

    // Turn-feasibility limiter:
    // cap speed when current heading error cannot be resolved within the
    // available horizontal distance at max bank. Use a bounded lookahead so
    // handoff to a far next waypoint still slows down for a large heading step.
    r32 turnSpeedLimit = targetSpeed;
    const r32 absHeadErr = abs(headingError);
    if (horDist > EPS && absHeadErr > 1e-3) {
        const r32 maxTurnAccel = GRAVITY * std::tan(MAX_BANK_ANGLE);

        // calculate imaginary trajectory radius which plane will have if it would turn to current WP
        // chord = lookaheadDistance 
        // requiredTurnRadius - radius of a circle with imaginary center and arc part which goes from plane position to WP
        // current plane dir is tangential to this arc. we use hord formula
        // hord = 2 * R * sin(0.5 * alpha). absHeadErr is actually half alpha, but in case our direction is away from WP
        // this will produce wrong results (e.g. if absHeadErr goes from 90 to 180 this will actually allow more speed
        // e.g. alpha = 180, hord = 2*R*sin(0.5*(2*180)) => R is huge (as if we are headed straight to WP)
        // so to not handle these cases, I just clamp it at 180 
        const r32 chordAngle = clamp(2.0f*absHeadErr, 0.0f, M_PIf);
        const r32 lookaheadDistance = std::min(horDist, std::max(activeWaypoint.reachRadius, cfg.approachSlowdownDistance));
        const r32 denom = 2.0 * std::sin(0.5 * chordAngle);
        const r32 requiredTurnRadius = max(lookaheadDistance / denom, r32(1.0));

        // now using centripetal accleration formula a = V^2 / R we get our max allowable speed
        const r32 feasibleTurnSpeed = sqrt(max(maxTurnAccel * requiredTurnRadius, r32(0.0)));

        // if we are headed away from WP then even more reduce speed limit
        // TODO: come up with better solution
        const r32 slowdownK = saturate((absHeadErr - M_PI_2f)/M_PI_2f); // 0..180 -> -1..1 clamped to 0..1
        turnSpeedLimit = feasibleTurnSpeed * lerp(1.0f, 0.5f, slowdownK) * cfg.turnSpeedLimitSafetyFactor;
        desiredHorSpeedRaw = min(desiredHorSpeedRaw, turnSpeedLimit);
    }
    debugState.turnSpeedLimit = turnSpeedLimit;

    // This might not bee needed anymore after slowdownK
    // TODO: check with different waypoints
    
    // Additional heading-based cap:
    // large heading errors should proactively reduce speed, even when the
    // waypoint is far enough that pure radius math looks permissive.
    const r32 headSpeedCapStartRad = cfg.headingSpeedCapStartDeg * M_PI / 180.0;
    const r32 headSpeedCapFullRad = cfg.headingSpeedCapFullDeg * M_PI / 180.0;
    assert(headSpeedCapFullRad - headSpeedCapStartRad > EPS);
    if (absHeadErr > headSpeedCapStartRad) {
        const r32 headingCapFactor = 1.0 - saturate((absHeadErr - headSpeedCapStartRad) / (headSpeedCapFullRad - headSpeedCapStartRad));
        desiredHorSpeedRaw = min(desiredHorSpeedRaw, targetSpeed * headingCapFactor);
    }

    // Vertical-feasibility limiter:
    // with capped vertical-speed authority, high horizontal speed can make the
    // required climb/descent angle unreachable and cause loiter around waypoint.
#if 0
    if (distanceToTarget > EPS && horDist > EPS) {
        const r32 maxVerticalSpeed = cfg.vyMax * cfg.pitchAuthorityGain;
        if (maxVerticalSpeed > EPS) {
            const r32 directionVertical = std::abs(dot(dirToTargetUnit, vec3(0.0f, 1.0f, 0.0f)));
            const r32 directionHorizontal = std::sqrt(std::max(1.0 - directionVertical * directionVertical, 0.0));
            if (directionHorizontal > 1e-3) {
                const r32 requiredSlope = directionVertical / directionHorizontal;
                const r32 climbLimitedHorSpeed =
                    (maxVerticalSpeed / max(requiredSlope, r32(1e-6))) * cfg.climbSpeedLimitSafetyFactor;
                debugState.climbSpeedLimit = climbLimitedHorSpeed;
                desiredHorSpeedRaw = std::min(desiredHorSpeedRaw, climbLimitedHorSpeed);
            }
        }
    }
#endif
    debugState.desiredHorSpeedRaw = desiredHorSpeedRaw;

    if (!desiredSpeedFilterInitialized) {
        desiredHorSpeedFiltered = clamp(state.speed, 0.0f, targetSpeed);
        desiredSpeedFilterInitialized = true;
    }
    const r32 tau = max(cfg.desiredSpeedSmoothingTimeConstant, r32(dt));
    const r32 speedFilterAlphaBase = saturate(dt/ tau);
    r32 speedFilterAlpha = speedFilterAlphaBase;
    if (desiredHorSpeedRaw < desiredHorSpeedFiltered) {
        // Reduce target speed faster than we increase it, to avoid loitering
        // when a sharp turn suddenly requires deceleration.
        speedFilterAlpha = saturate(speedFilterAlphaBase * r32(4.0));
    }
    desiredHorSpeedFiltered += (desiredHorSpeedRaw - desiredHorSpeedFiltered) * speedFilterAlpha;
    //TODO: clamp(x, minSpeed, targetSpeed) ?
    desiredHorSpeedFiltered = clamp(desiredHorSpeedFiltered, 0.0f, targetSpeed);
    const r32 desiredHorSpeed = desiredHorSpeedFiltered;
    debugState.desiredHorSpeedFiltered = desiredHorSpeedFiltered;

    const r32 lateralPositionError = dot(horizontalToTarget, rightDirection);
    const r32 lateralSpeed = dot(horizontalVelocity, rightDirection);
    const r32 lateralAccelCmd = lateralPositionError * cfg.lateralPositionAccelGain - lateralSpeed * cfg.lateralSpeedAccelDamping;
    const r32 desiredBankFromPd = std::atan2(lateralAccelCmd, GRAVITY);
    const r32 headingBankBias = headingError * cfg.headingErrorRollGain * approachFactor;
    const r32 desiredBank = clamp(desiredBankFromPd + headingBankBias, -MAX_BANK_ANGLE, MAX_BANK_ANGLE);
    const r32 rollControl = clamp(desiredBank / MAX_BANK_ANGLE, CONTROL_INPUT_MIN, CONTROL_INPUT_MAX);

    const r32 desiredPitchAngle = std::asin(clampSignedUnit(dirToTargetUnit.y));
    const r32 currentPitchAngle = state.speed > EPS
        ? std::asin(clampSignedUnit(state.velocity.y / state.speed))
        : 0.0;
    const r32 flightPitchError = desiredPitchAngle - currentPitchAngle;

    const r32 pitchControl = clamp(
        altitudeError * cfg.altitudeErrorPitchGain +
        flightPitchError * cfg.waypointDirectionPitchGain -
        state.velocity.y * cfg.verticalSpeedPitchDamping,
        CONTROL_INPUT_MIN,
        CONTROL_INPUT_MAX);

    const r32 desiredVerticalSpeed = cfg.vyMax * pitchControl;
    const vec3 desiredVelocity = targetHorDir * desiredHorSpeed + vec3(0.0f, desiredVerticalSpeed, 0.0f);
    const r32 desiredSpeed = length(desiredVelocity);
    const r32 speedError = desiredSpeed - state.speed;
    const r32 thrustControl = speedError * cfg.thrustSpeedGain;

    controlInputs.x = rollControl;
    controlInputs.y = pitchControl;

    const r32 maxBrakeDecel = MAX_BRAKE_FORCE / MASS;
    const r32 stoppingDistance = state.speed * state.speed / (2.0 * std::max(maxBrakeDecel, EPS));
    const r32 predictiveBrakeFactor = saturate((stoppingDistance - distanceToTarget) / cfg.brakeDistanceMargin);
    const r32 speedBrakeDenominator = max(desiredSpeed, r32(1.0));
    const r32 speedBrakeFactor = saturate((state.speed - desiredSpeed) / speedBrakeDenominator);
    const r32 brakeAltitudeFactor = saturate((cfg.brakeAltitudeWindow - std::abs(altitudeError)) / cfg.brakeAltitudeWindow);
    controlInputs.z = static_cast<float>(max(predictiveBrakeFactor, speedBrakeFactor) * brakeAltitudeFactor);

    // Additional braking for turn infeasibility. Unlike altitude-window brake,
    // this should remain active even with large altitude error.
    if (turnSpeedLimit < targetSpeed - EPS) {
        const r32 turnSpeedDenominator = max(turnSpeedLimit, r32(1.0));
        const r32 turnSpeedExcess = (state.speed - turnSpeedLimit) / turnSpeedDenominator;
        const r32 headingBrakeWeight = saturate(absHeadErr / (20.0 * M_PI / 180.0));
        const r32 turnBrakeFactor = saturate(turnSpeedExcess) * headingBrakeWeight;
        controlInputs.z = max(controlInputs.z, static_cast<float>(turnBrakeFactor));
    }

    const r32 newThrust = clamp(thrustControl, 0.0f, MAX_THRUST);
    controlInputs.w = newThrust; //aircraft.setThrust(newThrust);

    return controlInputs;
}

void WingOnlyAIController::evade(const vec3& threatPosition, r32 threatRadius, const AircraftState state) {
    const vec3 threatDirection = state.position - threatPosition;
    const r32 distance = length(threatDirection);

    if (distance < threatRadius) {
        const vec3 evasionDirection = normalizeSafe(threatDirection);
        const vec3 evasionTarget = state.position + evasionDirection * static_cast<float>(EVASION_OFFSET_DISTANCE);
        setTargetPosition(evasionTarget);
    }
}

bool WingOnlyAIController::isAtTarget() const {
    if(finalWaypointReached) {
        static bool printed = false;
        if(!printed) {
            printf("at target");
            printed = true;
        }
    }
    return finalWaypointReached;
}

void WingOnlyAIController::resetTracking() {
    clearWaypoints();
}

bool WingOnlyAIController::isTracking() const {
    return trackingEnabled;
}

const Waypoint* WingOnlyAIController::getActiveWaypoint() const {
    return hasActiveWaypoint ? &activeWaypoint : nullptr;
}

std::size_t WingOnlyAIController::getQueuedWaypointCount() const {
    return waypointQueue.size();
}

WingControllerDebugState WingOnlyAIController::getDebugState() const {
    return debugState;
}
