#include "game/Text.h"
#include "game/Enemy.h"
#include "game/MainShip.h"
#include "game/Path.h"

#include "engine/utils/vec.h"
#include "engine/utils/spline.h"
#include "engine/profiler/profiler.h"

MainShip* MainShip::Create(const char* res) {

    static size_t obj_num = 0;
    MainShip *obj = new MainShip();

    obj->name_ = res;
    obj->name_ += std::to_string(obj_num++);

    auto tr = obj->AddComponent<TransformComponent>();

    obj->mesh_comp_ = MeshComponent::Create(res);
    obj->AddComponent(obj->mesh_comp_);
    obj->mesh_comp_->SetParent(tr);
    // rotate mesh to point forward in case of fighter1
    //obj->mesh_comp_->SetRotation(quaternion(vec3(0,1,0), M_PI/2.0f));

    auto* TextComp = obj->AddComponent<GameTextComp>();
    TextComp->SetParent(tr);

    return obj;
}

void MainShip::Initialize(const std::vector<CheckPoint>& cps, GameObject* intarget, const Curve<vec3>* master_curve) 
{
    target_ = intarget;
    levelCurve = master_curve;
    repar = new ReparameterizeByArclength<vec3>(*master_curve);

    radius = 10.0f;
    origin_pos = GetComponent<TransformComponent>()->GetPosition();
    quaternion q = GetComponent<TransformComponent>()->GetRotation();
    total_time = 0.0f;
    check_points_ = cps;
    cur_check_point_idx_ = 0;
    plane_state_.position = origin_pos;
    plane_state_.velocity = vec3(0, 0, 0.01f);
    plane_state_.right = q.axis0();
    plane_state_.up = q.axis1();
    plane_state_.forward = q.axis2();
    plane_state_.q = q;

    mat3 m3 = q.to_mat3(); vec3 right = m3.getRow(0); vec3 up = m3.getRow(1);
    vec3 fwd = m3.getRow(2);
    printf("Ship init right: %f %f %f axis0: %f %f %f\n", right.x, right.y, right.z, q.axis0().x, q.axis0().y, q.axis0().z);
    printf("Ship init up: %f %f %f axis1: %f %f %f\n", up.x, up.y, up.z, q.axis1().x, q.axis1().y, q.axis1().z);
    printf("Ship init fwd: %f %f %f axis2: %f %f %f\n", fwd.x, fwd.y, fwd.z, q.axis2().x, q.axis2().y, q.axis2().z);

    ai_controller.setTargetSpeed(targetSpeed);
    statePrev = physics.getState();
    stateCur = statePrev;
    renderState = stateCur;

    // add dummy first point of a spline
    //curve.addPoint(check_points_[0].position + 0.25f*(check_points_[0].position - check_points_[1].position));

    const int CPCount = (int)check_points_.size();
    for(int i=0; i< CPCount;++i) {
        bool b_is_final = i == CPCount-1;
        printf("add wp, final:%d\n", b_is_final);
        const CheckPoint& cp = check_points_[i];
        ai_controller.addWaypoint(Waypoint(cp.position, cp.radius, b_is_final));
        //curve.addPoint(cp.position);
    }

    // add dummy last point of a spline
    //curve.addPoint(check_points_[CPCount-1].position + 0.25f*(check_points_[CPCount-1].position - check_points_[CPCount-2].position));


    accumulator = 0;

    AircraftState& state = physics.getState();
    state.position = origin_pos;
    state.right = q.axis0();
    state.up = q.axis1();
    state.forward = q.axis2();
    state.acceleration = vec3(0);
    state.speed = 0;
    state.velocity = vec3(0);

}

#if 1
void MainShip::Update2(float dt) {

        TransformComponent* tc = GetComponent<TransformComponent>();

        mat4 rotm;
        rotm = mat4::rotationY(total_time*2);
        total_time += dt;

        quaternion cur_q = tc->GetRotation();
        plane_state_.right = cur_q.axis0();
        plane_state_.up = cur_q.axis1();
        plane_state_.forward = cur_q.axis2();

        vec3 targetPosition = target_ ? 
            target_->GetComponent<TransformComponent>()->GetPosition() : 
            check_points_[cur_check_point_idx_].position;

        printf("Target pos: %f %f %f\n", targetPosition.x, targetPosition.y, targetPosition.z);
        AIControls controls = ship_ai_.update3(plane_state_, targetPosition, dt);
        printf("AI controls: pitch: %f yaw: %f roll: %f throttle: %f\n", controls.pitch, controls.yaw, controls.roll, controls.throttle);
#if 1
        float pitchAngle = (controls.pitch + (plane_state_.forward.y > 0 ? -0.2f : 0.2f)) * turnRate * dt;
        //float yawAngle = controls.yaw * turnRate * dt;
        float rollAngle = controls.roll * turnRate * dt;

        quaternion qr = quaternion(plane_state_.forward, rollAngle);
        quaternion qp = quaternion(plane_state_.right, pitchAngle);
        quaternion q = qp * qr;
        
        // transform by roll
        vec3 newForward = quat_rotate(qr, plane_state_.forward);
        vec3 newUp = quat_rotate(qr, plane_state_.up);
        vec3 newRight = quat_rotate(qr, plane_state_.right);

        quaternion new_q = q * cur_q;
#if 0
        // check that transformed axes are the same as quaternion axes
        printf("Ship new right: %f %f %f axis0: %f %f %f\n", newRight.x, newRight.y, newRight.z, new_q.axis0().x, new_q.axis0().y, new_q.axis0().z);
        printf("Ship new up: %f %f %f axis1: %f %f %f\n", newUp.y, newUp.y, newUp.z, new_q.axis1().x, new_q.axis1().y, new_q.axis1().z);
        printf("Ship new fwd: %f %f %f axis2: %f %f %f\n", newForward.x, newForward.y, newForward.z, new_q.axis2().x, new_q.axis2().y, new_q.axis2().z);
        assert(lengthSqr(newRight - new_q.axis0()) < 0.001f);
        assert(lengthSqr(newUp - new_q.axis1()) < 0.001f);
        assert(lengthSqr(newForward - new_q.axis2()) < 0.001f);
#endif

        plane_state_.forward = normalize(newForward);
        plane_state_.up = normalize(newUp);
        plane_state_.right = normalize(newRight);
        plane_state_.velocity = plane_state_.forward * (controls.throttle * maxSpeed);
        plane_state_.position = plane_state_.position + plane_state_.velocity * dt;
        plane_state_.q = new_q;
#endif

        tc->SetPosition(plane_state_.position);
        tc->SetRotation(plane_state_.q);

        if(length(plane_state_.position.xz() - targetPosition.xz()) < check_points_[cur_check_point_idx_].radius) {
            printf("Checkpoint %d reached!\n", cur_check_point_idx_);
            cur_check_point_idx_ = (cur_check_point_idx_ + 1) % check_points_.size();
        }

        //extern void update_ship_cam(MainShip* ship);
        //update_ship_cam(this);
    }
#endif

void MainShip::pushTrailPoint(const vec3& point) {
    if (!trail.empty()) {
        const float spacing = length(point - trail.back());
        if (spacing < 0.5f) {
            return;
        }
    }

    trail.push_back(point);
    const size_t maxPoints = 1024;
    while (trail.size() > maxPoints) {
        trail.pop_front();
    }
}


void MainShip::simulateFixedStep(double dt) {

    if (ai_controller.isAtTarget()) {
        return;
    }

    statePrev = stateCur;
    const vec4 controls = ai_controller.getControlInputs(
            controller_config, physics.getState(), dt);
    physics.update(controls, controller_config, dt);
    stateCur = physics.getState();

    pushTrailPoint(stateCur.position);
    //updateDebugWaypointState();
}


void MainShip::Update(float dt) {

    TransformComponent* tc = GetComponent<TransformComponent>();

    const double fixedDt = 0.016;
    const int maxSubSteps = 64;
    const int simSpeed = 1;
    const double maxAccumulated = fixedDt * maxSubSteps;
    accumulator = min(accumulator + dt*simSpeed, maxAccumulated);

    ai_controller.setTargetSpeed(targetSpeed);

    if(ai_controller.isAtTarget()) {
        return;
    }

    int steps = 0;
    while (accumulator >= fixedDt && steps < maxSubSteps) {
        simulateFixedStep(fixedDt);
        accumulator -= fixedDt;
        steps++;
    }
    assert(accumulator <= fixedDt);

    const double alpha = saturate(accumulator / fixedDt);
    renderState = AircraftState::interp(statePrev, stateCur, alpha);
    // actually we calculate left, so need to flip
    renderState.right = -renderState.right;

    mat3 m = mat3::fromBasis(renderState.right, renderState.up, renderState.forward);

    const quaternion qnew = mat3_to_quat(m); // expects column major, so transpose
    tc->SetPosition(renderState.position);
    tc->SetRotation(qnew);
}

void MainShip::AddRenderPackets(struct RenderFrameContext* rfc) const {

    return;

    vec3 pos = renderState.position;
    vec3 up = renderState.up;
    vec3 right = renderState.right;
    vec3 fwd = renderState.forward;

    RenderList* rl = rfc->rl_;
    rl->addDebugLine(pos, pos + 3*right, vec4(1, 0,0, 1));
    rl->addDebugLine(pos, pos + 3*up, vec4(0, 1,0, 1));
    rl->addDebugLine(pos, pos + 3*fwd, vec4(0, 0,1, 1));


    const size_t n = trail.size();
    if (n >= 2) {
        vec3* dbg_lines = new vec3[2*(n-1)];
        vec4* colours = new vec4[n-1];
        for (size_t i = 0; i < n-1; ++i) {
            //const float t = (float)i / (n - 1);
            const float alpha = 1;//0.35f + 0.65f * t;
            const float brightness = 1;//0.25f + 0.75f * t;
            vec4 colour(0.15f * brightness, 0.9f * brightness, 1.0f * brightness, alpha);
            const vec3 p = trail[i];
            //rl->addDebugLine(p, trail[i+1], colour);
            dbg_lines[2*i + 0] = p;
            dbg_lines[2*i + 1] = trail[i+1];
            colours[i] = colour;
        }
        rl->addDebugLines(dbg_lines, colours, vec4(1), n-1);
        delete[] dbg_lines;
        delete[] colours;
    }

    //TODO: actually move to curve component
	SCOPED_ZONE_NAMED(DebugDrawRemappedNodes, 0);
    constexpr int num_intervals = 25;
    vec3* pts = new vec3[num_intervals];
    float tot_len = levelCurve->getTotalLength();
    for(int i=0;i<num_intervals;++i) {
        float s = tot_len * (float)i / num_intervals;
        {
            SCOPED_ZONE_NAMED(GetT, 0);
            ReparameterizeByArclength<vec3>::Output o = repar->GetT(s, true);
            SCOPED_ZONE_NAMED(GetAt, 0);
            pts[i] = levelCurve->getAt(o.t);
        }
    }
    rl->addDebugPoints(pts, num_intervals, vec4(1, 0, 0, 1), 10, true);
    delete[] pts;
}


MainShip::~MainShip() {
    delete repar;
}


PROPERTY_LIST_BEGIN_DERIVED(MainShip, GameObject)
    PROPERTY_READONLY_TEXT("Name", [](const MainShip& o) { return o.GetName(); });
PROPERTY_LIST_END()
