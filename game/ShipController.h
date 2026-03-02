#include "engine/utils/vec.h"
#include "engine/utils/quaternion.h"
#include "engine/utils/math_utils.h"

struct PlaneState {
    vec3 position;
    vec3 velocity;
    vec3 forward;  // Current facing direction
    vec3 up;       // Up direction
    vec3 right;    // Right direction
    quaternion q;
};

struct AIControls {
    float pitch;      // -1 to 1 (nose down to nose up)
    float yaw;        // -1 to 1 (turn left to turn right)
    float roll;       // -1 to 1 (roll left to roll right)
    float throttle;   // 0 to 1
};

class PlaneAI {
private:
    float turnRate;
    float arrivalRadius;
    float bankAngleMax;  // Maximum bank angle in radians (e.g., 60 degrees)
    
public:
    PlaneAI() : turnRate(20.0f), arrivalRadius(5.0f), bankAngleMax(3*1.047f) {} // ~60 degrees
    
    // Main update function - call this each frame
    AIControls update(const PlaneState& plane, const vec3& targetPos, float deltaTime) {
        AIControls controls = {0};
        
        vec3 toTarget = targetPos - plane.position;
        float distToTarget = length(toTarget);
        vec3 desiredDir = normalize(toTarget);
        // Calculate how aligned we are with target
        float alignment = dot(plane.forward, desiredDir);
        
#if 1 
        // For yaw calculation, project onto horizontal plane (world space)
        vec3 worldUp = vec3(0, 1, 0); // Adjust if your world up is different
        vec3 forwardHorizontal = normalize(plane.forward - worldUp * dot(plane.forward, worldUp));
        vec3 desiredDirHorizontal = normalize(desiredDir - worldUp * dot(desiredDir, worldUp));
        
        // Calculate turn direction in horizontal plane
        vec3 worldRight = cross(worldUp, forwardHorizontal);
        float horizontalTurnDot = dot(worldRight, desiredDirHorizontal);
        
        // For roll/pitch, use local space calculations
        vec3 right = plane.right;
        float rightDot = dot(right, desiredDir);
        float upDot = dot(plane.up, desiredDir);
        
        // Bank-and-yank: coordinated turn
        // When turning, we bank the plane AND pull up to turn efficiently
        float desiredRoll = -rightDot * bankAngleMax; // Bank into the turn
        float turnIntensity = abs(rightDot); // How hard we're turning
        
        // Add roll stabilization to prevent inverted flight
        float currentRoll = dot(plane.right, worldUp); // How tilted we are
        float rollCorrection = -currentRoll * 2.0f; // Correction to level out
        (void)rollCorrection;
#else
    // For yaw calculation, project onto horizontal plane (world space)
        vec3 worldUp = vec3(0, 1, 0); // Adjust if your world up is different
        vec3 forwardHorizontal = normalize(plane.forward - worldUp * dot(plane.forward, worldUp));
        vec3 desiredDirHorizontal = normalize(desiredDir - worldUp * dot(desiredDir, worldUp));
        
        // Calculate turn direction in horizontal plane
        vec3 worldRight = cross(worldUp, forwardHorizontal);
        float horizontalTurnDot = dot(worldRight, desiredDirHorizontal);
        
        // For roll/pitch, use local space calculations
        vec3 right = plane.right;
        float rightDot = dot(right, desiredDir);
        float upDot = dot(plane.up, desiredDir);
        
        // Bank-and-yank: coordinated turn
        // When turning, we bank the plane AND pull up to turn efficiently
        float desiredRoll = -rightDot * bankAngleMax; // Bank into the turn
        float turnIntensity = abs(rightDot); // How hard we're turning
        
        // Add roll stabilization to prevent inverted flight
        float uprightness = dot(plane.up, worldUp); // 1 = upright, -1 = inverted
        
        // If inverted or heavily banked, add strong correction
        float rollCorrection = 0.0f; (void)rollCorrection;
        if (uprightness < 0.7f) { // Not sufficiently upright
            // Determine which way to roll to get upright faster
            vec3 rightHorizontal = cross(worldUp, plane.forward);
            if (length(rightHorizontal) > 0.01f) {
                rightHorizontal = normalize(rightHorizontal);
                float rollDirection = dot(plane.right, rightHorizontal);
                rollCorrection = -rollDirection * 3.0f; // Strong correction
            }
        }
#endif
        
        // Combine turn banking with roll stabilization
        float finalRoll = desiredRoll;// + rollCorrection;
        
        // Roll control - bank into turns
        controls.roll = clamp(finalRoll * turnRate, -1.0f, 1.0f);
        
        // Pitch control - pull up during turns to maintain altitude
        // When banked, pulling up creates the turn
        if (turnIntensity > 0.2f) {
            // Pull up harder when banked to create coordinated turn
            controls.pitch = clamp((0.5f + turnIntensity * 0.5f) * turnRate, -1.0f, 1.0f);
        } else {
            // Normal pitch to match target elevation
            controls.pitch = clamp(upDot * turnRate, -1.0f, 1.0f);
        }
        controls.pitch *= -1;

        //controls.pitch *= 0;
        
        // Yaw control - rudder for coordinated flight (world space horizontal)
        // Small yaw input to coordinate with roll and turn in world space
        controls.yaw = clamp(horizontalTurnDot * turnRate * 0.3f, -1.0f, 1.0f);
        controls.yaw = 0;
        
        // Speed control
        if (distToTarget < arrivalRadius) {
            controls.throttle = 0.3f; // Slow down when close
        } else if (alignment > 0.8f) {
            controls.throttle = 1.0f; // Full speed when facing target
        } else {
            controls.throttle = 0.7f; // Moderate speed while turning
        }
        
        return controls;
    }

    AIControls update2(PlaneState& plane, const vec3& targetPos, float deltaTime) {
        AIControls controls = {0};
        
        vec3 toTarget = targetPos - plane.position;
        //float distToTarget = length(toTarget);
        vec3 desiredDir = normalize(toTarget);
        float alignment = dot(plane.forward, desiredDir);

        // up/right plane
        vec4 upRightPlaneAt0 = make_plane(plane.forward, vec3(0));
        vec3 targetUp = project_vector_on_plane(desiredDir, upRightPlaneAt0);
        //const float rollStrength = length(targetUp);
        if(length(targetUp) < 0.01f) {
            targetUp = plane.up; // avoid degenerate case
        } else {
            targetUp = normalize(targetUp);
        }

        alignment = 0.5*alignment + 0.5f;
        alignment = (alignment - 0.98f)/0.02f; // remap so that 0.98..1.0 -> 0..1.0
        printf("Alignment: %f\n", alignment);
        targetUp = lerp(targetUp, vec3(0,1,0), clamp(alignment, 0, 1)); // prefer world up when aligned

        quaternion qr = quat_from_two_axes(plane.up, targetUp);
        qr = quat_lerp(quaternion::identity(), qr, 0.75f); // smooth rotation

        vec3 newForward = quat_rotate(qr, plane.forward);
        vec3 newUp = quat_rotate(qr, plane.up);
        vec3 newRight = quat_rotate(qr, plane.right);

        vec4 fwdUpPlane = make_plane(newRight, vec3(0));
        vec3 targetFwd = project_vector_on_plane(desiredDir, fwdUpPlane);
        if(length(targetFwd) < 0.01f) {
            targetFwd = newForward; // avoid degenerate case
        } else {
            targetFwd = normalize(targetFwd);
        }
        
        quaternion qp = quat_from_two_axes(newForward, targetFwd);
        qp = quat_lerp(quaternion::identity(), qp, 0.95f); // smooth rotation

        newForward = quat_rotate(qp, newForward);
        newUp = quat_rotate(qp, newUp);
        newRight = quat_rotate(qp, newRight);

        // update plane state for next frame
        plane.forward = normalize(newForward);
        plane.up = normalize(newUp);
        plane.right = normalize(newRight);
        plane.q = qp * qr * plane.q;

        //printf("TargetUp: %f %f %f\n", targetUp.x, targetUp.y, targetUp.z);
        //printf("TargetFwd: %f %f %f\n", targetFwd.x, targetFwd.y, targetFwd.z);
       
#if 0
        float rightDot = dot(plane.right, targetUp); // we take plane.right and not plane.up because we want larger dot() produce larger corrections (basically right is 90deg from up)
        float upDot = dot(plane.up, targetFwd);
        printf("upDot: %f rightDot: %f rollStr: %f\n", upDot, rightDot, rollStrength);


        float pitchIntensity = 1;//abs(upDot);
        controls.roll = clamp(-rightDot * bankAngleMax * rollStrength * pitchIntensity, -1.0f, 1.0f);
        //controls.roll = 0;
        
        controls.pitch = clamp(-upDot * bankAngleMax * 4, -1.0f, 1.0f);
        //controls.pitch *= 0;


        if(rollStrength < 0.2f) {
            // try to satbilize roll
            controls.roll = clamp(-dot(plane.right, worldUp) * 2.0f, -1.0f, 1.0f);
        }
        
        // Yaw control - rudder for coordinated flight (world space horizontal)
        // Small yaw input to coordinate with roll and turn in world space
        controls.yaw = clamp(horizontalTurnDot * 0.3f, -1.0f, 1.0f);
        controls.yaw = 0;
        
        // Speed control
        if (distToTarget < arrivalRadius){
            controls.throttle = 0.3f; // Slow down when close
        } else if (alignment > 0.8f) {
            controls.throttle = 1.0f; // Full speed when facing target
        } else {
            controls.throttle = 0.7f; // Moderate speed while turning
        }
#endif    
        return controls;
    }

    float WrapToRange(float value, float min, float max) {
        float range = max - min;
        while (value < min) value += range;
        while (value > max) value -= range;
        return value;
    }

    float DegreesToRadians(float degrees) {
        return degrees * (3.14159265f / 180.0f);
    }

    float RadiansToDegrees(float radians) {
        return radians * (180.0f / 3.14159265f);
    }

    AIControls update3(PlaneState& plane, const vec3& targetPos, float deltaTime) {
        AIControls controls = {0};

        float mControlBankAnglePerHeadingChange = 0.5f;
        float mControlMaxBankAngle = 70.0f; // degrees
        float mControlPitchControlPerRollAngle = 0.01f;
        float mControlRollControlPerRollAngle = 0.01f;
        float mControlMaxRollControl = 1.0f;
        float mControlMaxPitchControl = 1.0f;

        float mPoweredControlMaxSlopeDown = 0.1f;
        float mPoweredControlMaxSlopeUp = 0.5f;
        float mPoweredControlPitchControlPerSlope = 10.0f;
        float mPoweredControlThrottleControlPerAltitude = 0.05f;
        float mPoweredControlThrottleControlPerSpeed = 0.1f;
        float mPoweredControlCruiseSpeed  = 10.0f;

        //--------------------------------------------------------

        vec3 deltaToTarget = targetPos - plane.position;
        vec3 horDeltaToTarget(deltaToTarget.x, 0.0f, deltaToTarget.z);
        const float horDistToTarget = length(horDeltaToTarget);

        vec3 vel = plane.velocity; // temporarily use forward as heading //plane.velocity;
        const vec3 windVel = vec3(0,0,0); // TODO: get wind velocity from environment
        const vec3 velRelWind = vel - windVel;
        const float fwdSpeedRelAir = dot(velRelWind, plane.forward);

        const float heading = atan2f(vel.x, vel.z);
        const float desiredHeading = atan2f(horDeltaToTarget.x, horDeltaToTarget.z);

        // Make sure we always turn into wind
        const float headingChange = WrapToRange(desiredHeading - heading, -PI, PI);

        float desiredRoll = clamp(headingChange * mControlBankAnglePerHeadingChange, 
                -DegreesToRadians(mControlMaxBankAngle), DegreesToRadians(mControlMaxBankAngle));

        // in PicaSim rowZ is +ve up
        // Airflow is relative to the aerofoil so +ve X is forward, +ve Y is left, +ve Z is up
        
        const float currentRoll = asinf(-plane.right.y);
        float rollControl = clamp(RadiansToDegrees(desiredRoll - currentRoll) * mControlRollControlPerRollAngle, -1.0f, 1.0f);
        rollControl *= mControlMaxRollControl;

        // Control pitch. Note that +ve is down
        float pitchControl = RadiansToDegrees(fabsf(currentRoll)) * -mControlPitchControlPerRollAngle;

        // Also adjust depending on air speed.
        float currentSlope = plane.forward.y;//velRelWind.y / fwdSpeedRelAir; 
        printf("velRelWind: %f %f %f fwdSpeedRelAir: %f\n", velRelWind.x, velRelWind.y, velRelWind.z, fwdSpeedRelAir);

        float desiredAltitudeChange = targetPos.y - plane.position.y;
        float targetSlope = desiredAltitudeChange / horDistToTarget;
        targetSlope = clamp(targetSlope, -mPoweredControlMaxSlopeDown, mPoweredControlMaxSlopeUp);

        pitchControl -= (targetSlope - currentSlope) * mPoweredControlPitchControlPerSlope;
        pitchControl = clamp(pitchControl, -1.0f, 1.0f);
        pitchControl *= mControlMaxPitchControl;
        printf("currentSlope: %f targetSlope: %f pitchControl: %f\n", currentSlope, targetSlope, pitchControl);

        float throttleControl = desiredAltitudeChange * mPoweredControlThrottleControlPerAltitude;
        throttleControl += (mPoweredControlCruiseSpeed - fwdSpeedRelAir) * mPoweredControlThrottleControlPerSpeed;
        throttleControl = clamp(throttleControl, 0.0f, 1.0f);

        controls.roll = -rollControl;
        controls.pitch = pitchControl;
        controls.throttle = throttleControl;
        return controls;
    }
    
    void setTurnRate(float rate) { turnRate = rate; }
    void setArrivalRadius(float radius) { arrivalRadius = radius; }
    void setBankAngleMax(float angle) { bankAngleMax = angle; }
    
private:
    float clamp(float value, float min, float max) const {
        if (value < min) return min;
        if (value > max) return max;
        return value;
    }
};

// Example usage:
/*
PlaneAI ai;
PlaneState myPlane;
vec3 targetPosition(100, 50, 200);

// Set max bank angle (optional, default is ~60 degrees)
ai.setBankAngleMax(1.047f); // 60 degrees in radians

// In your game loop:
AIControls controls = ai.update(myPlane, targetPosition, deltaTime);

// Apply controls to your plane physics:
// The controls now work together for coordinated turns:
// - Roll banks the plane
// - Pitch pulls up (which turns the plane when banked)
// - Yaw coordinates the turn

mat4 rotation = mat4::identity();
rotation = rotate(rotation, controls.pitch * turnRate * deltaTime, myPlane.right);
rotation = rotate(rotation, controls.yaw * turnRate * deltaTime, myPlane.up);
rotation = rotate(rotation, controls.roll * turnRate * deltaTime, myPlane.forward);

vec3 newForward = (rotation * vec4(myPlane.forward, 0.0f)).xyz();
vec3 newUp = (rotation * vec4(myPlane.up, 0.0f)).xyz();
vec3 newRight = (rotation * vec4(myPlane.right, 0.0f)).xyz();

myPlane.forward = normalize(newForward);
myPlane.up = normalize(newUp);
myPlane.right = normalize(newRight);
myPlane.velocity = myPlane.forward * (controls.throttle * maxSpeed);
myPlane.position = myPlane.position + myPlane.velocity * deltaTime;
*/
