#include "game/Text.h"
#include "game/Enemy.h"
#include "game/Path.h"

#include <string.h>

EnemyState BasicEnemyAIController::update(const EnemyState& s, float dt) {
    const float t = cur_t;
    EnemyState next_state;

    if(movePath) {
        next_state.position = movePath->Get(t);
        next_state.forward = movePath->GetDerivative(t);


        next_state.up = cross(next_state.forward, s.forward);
        if(length(next_state.up) < 0.0001f) {
            next_state.up = vec3(0, 1, 0); 
        } else {
            next_state.up = normalize(next_state.up);
        }

        next_state.right = cross(next_state.up, next_state.forward);

        next_state.velocity = next_state.position - s.position;
        next_state.acceleration = next_state.velocity - s.velocity;

        next_state.speed = length(next_state.velocity);
        next_state.thrust = 0;
    } else {
        next_state = s;
    }

    cur_t += dt;

    return next_state;
}


Enemy* Enemy::Create(const char* res) {

    static size_t obj_num = 0;
    Enemy* obj = new Enemy();

    obj->name_ = res;
    obj->name_ += std::to_string(obj_num++);

    auto tr = obj->AddComponent<TransformComponent>();

    obj->mesh_comp_ = MeshComponent::Create(res);
    obj->AddComponent(obj->mesh_comp_);
    obj->mesh_comp_->SetParent(tr);

    auto* TextComp = obj->AddComponent<EnemyTextComp>();
    TextComp->SetParent(tr);

    return obj;
}

void Enemy::Initialize(GameObject* intarget) 
{
    target_ = intarget;

    origin_pos = GetComponent<TransformComponent>()->GetPosition();
    quaternion q = GetComponent<TransformComponent>()->GetRotation();
    total_time = 0.0f;

    statePrev = statePrev;
    stateCur = statePrev;
    renderState = stateCur;

    accumulator = 0;

    stateCur.position = origin_pos;
    stateCur.velocity = vec3(0);
    stateCur.acceleration = vec3(0);
    stateCur.right = q.axis0();
    stateCur.up = q.axis1();
    stateCur.forward = q.axis2();
    stateCur.speed = 0;
    stateCur.thrust = 0;

}

void Enemy::pushTrailPoint(const vec3& point) {
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

void Enemy::simulateFixedStep(double dt) {

    //if (controller_.isAtTarget()) {
    //    return;
    //}

    statePrev = stateCur;
    stateCur = controller_.update(stateCur, dt);

    pushTrailPoint(stateCur.position);
}

void Enemy::Update(float dt) {

    TransformComponent* tc = GetComponent<TransformComponent>();

    const double fixedDt = 0.016;
    const int maxSubSteps = 64;
    const int simSpeed = 1;
    const double maxAccumulated = fixedDt * maxSubSteps;
    accumulator = min(accumulator + dt*simSpeed, maxAccumulated);

    //if(controller_.isAtTarget()) {
    //    return;
    //}

    int steps = 0;
    while (accumulator >= fixedDt && steps < maxSubSteps) {
        simulateFixedStep(fixedDt);
        accumulator -= fixedDt;
        steps++;
    }
    assert(accumulator <= fixedDt);

    const double alpha = saturate(accumulator / fixedDt);
    renderState = EnemyState::interp(statePrev, stateCur, alpha);
    // actually we calculate left, so need to flip
    renderState.right = -renderState.right;

    mat4 m = identity4();
    m.setRow(0, vec4(renderState.right, 0));
    m.setRow(1, vec4(renderState.up, 0));
    m.setRow(2, vec4(renderState.forward, 0));
    // also mat to quat provides rotation in the opposite direction than matrix, so invert
    // TODO: fix!!!
    const quaternion qnew = normalize(inverse(mat4_to_quat(m)));
    tc->SetPosition(renderState.position);
    tc->SetRotation(qnew);

}

void Enemy::AddRenderPackets(struct RenderFrameContext* rfc) const {

    vec3 pos = renderState.position;
    vec3 up = renderState.up;
    vec3 right = renderState.right;
    vec3 fwd = renderState.forward;

    RenderList* rl = rfc->rl_;
    rl->addDebugLine(pos, pos + 3*right, vec4(1, 0,0, 1));
    rl->addDebugLine(pos, pos + 3*up, vec4(0, 1,0, 1));
    rl->addDebugLine(pos, pos + 3*fwd, vec4(0, 0,1, 1));

    if (trail.size() >= 2) {
        const size_t n = trail.size();
        for (size_t i = 0; i < n-1; ++i) {
            //const float t = (float)i / (n - 1);
            const float alpha = 1;//0.35f + 0.65f * t;
            const float brightness = 1;//0.25f + 0.75f * t;
            vec4 colour(0.15f * brightness, 0.9f * brightness, 1.0f * brightness, alpha);
            const vec3 p = trail[i];
            rl->addDebugLine(p, trail[i+1], colour);
        }
    }
#if 0
    const int nseg = curve.getNSegments();
    vec3 p_prev = curve.getAt(0);
    if(1)
        for(int s = 0; s<nseg; s++) {
            for (size_t i = 0; i < 128; ++i) {
                const float t = (float)i / 127;
                const float alpha = 1;
                const float brightness = 1;//0.35f + 0.65f * t;
                vec4 colour(0.8f * brightness, 0.8f * brightness, .1f * brightness, alpha);
                const vec3 p = curve.getAt((float)s + t);
                const vec3 dp = curve.getDerivativeAt((float)s + t);
                const float vmag = length(dp);
                const vec3 vdir = dp/vmag;
                const vec4 vc = saturate(vec4(0.5f*vdir + vec3(0.5f), 1) * vec4(0.05f*vmag, 0.05f*vmag, 0.05f*vmag, 1.0f));
                rl->addDebugLine(p_prev, p, colour);
                rl->addDebugLine(p, p + 0.1f*dp, vc);
                p_prev = p;
            }
        }
#endif
}

void Enemy::SetPath(const class Path* path) {

    myPath = path;
    controller_.SetPath(myPath);
}



