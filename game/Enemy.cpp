#include "game/Text.h"
#include "game/Enemy.h"
#include "game/Path.h"
#include "game/Time.h"
#include "engine/utils/imgui_property_list.h"

#include <string.h>



EnemyState BasicEnemyAIController::update(const EnemyState& s, float dt, const EnemyTargetCtx& target_ctx) {
    EnemyState next_state;
    memcpy(&next_state, &s, sizeof(next_state));

    float speed = 0.5f;
    float du = speed * dt;

    const float umax = movePath ? movePath->GetCurve()->getTMax() : 0;
    const bool b_wrapped = cur_t + du > umax;
    const float t = std::fmod(cur_t + du, umax);
    cur_t = t;

    if(1) {
        assert(movePath);

        next_state.position = movePath->Get(t);
        vec3 pos = next_state.position;
        mat4 m = mat4(
                target_ctx.right.x, target_ctx.up.x, target_ctx.fwd.x, target_ctx.path_pos.x,
                target_ctx.right.y, target_ctx.up.y, target_ctx.fwd.y, target_ctx.path_pos.y,
                target_ctx.right.z, target_ctx.up.z, target_ctx.fwd.z, target_ctx.path_pos.z, 
                0, 0, 0, 1);

        // TODO: quat?
        next_state.position = (m * vec4(pos,1)).xyz() + target_ctx.fwd*2;
        next_state.forward = normalize(target_ctx.pos - next_state.position);
        calculate_basis(next_state.forward, next_state.right, next_state.up);

    } else if(movePath) {

        next_state.position = movePath->Get(t);
        next_state.forward = normalize(movePath->GetDerivative(t));

        next_state.up = cross(next_state.forward, s.forward);
        if(length(next_state.up) < 0.0001f) {
            next_state.up = vec3(0, 1, 0); 
        } else {
            next_state.up = normalize(next_state.up);
        }

        next_state.right = cross(next_state.up, next_state.forward);

    } else {
        next_state = s;
    }

    next_state.velocity = (next_state.position - s.position)/dt;
    next_state.acceleration = (next_state.velocity - s.velocity)/dt;
    next_state.speed = length(next_state.velocity);
    next_state.thrust = 0;
    next_state.num_cycles += b_wrapped ? 1 : 0;

    return next_state;
}


Enemy* Enemy::Create(const char* res) {

    static size_t obj_num = 0;
    Enemy* obj = new Enemy();

    obj->name_ = res;
    obj->name_ += std::to_string(obj_num++);

    auto tr = scene_create_component<TransformComponent>(obj);

    obj->mesh_comp_ = MeshComponent::Create(res, obj);
    obj->mesh_comp_->SetScale(vec3(0.2f));
    obj->mesh_comp_->SetParent(tr);

    obj->curve_comp_ = scene_create_component<C_Curve>(obj);

    auto TextComp = scene_create_component<EnemyTextComp>(obj);
    TextComp->SetParent(tr);
    TextComp->Initialize();

    obj->Initialize(nullptr);

    return obj;
}

static const char* const gs_labels[] = {
    "Hello", "Catch me", "I am faster", "Game Over", "LLM was here", "Too fast for you"
};


void Enemy::Initialize(GameObject* intarget) 
{
    origin_pos = GetComponent<TransformComponent>()->GetPosition();
    quaternion q = GetComponent<TransformComponent>()->GetRotation();
    total_time = 0.0f;

    memset(&statePrev, 0, sizeof(statePrev));
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

    // generate random curve in local space for enemy to follow
    vec2 off = random_vec(vec3(-2, -2, 0), vec3(2,2,0)).xy();
    float z_back = 5;
    float z_front = 4;
    vec3 pt0 = vec3(off.x,off.y,-z_back);
    vec3 pt1 = vec3(off.y,off.y,z_front-1);

    Curve<vec3>& curve = *curve_comp_->GetCurve();

    curve.addPoint(pt0 + 0.25f*(pt0 - pt1));

    curve.addPoint(pt0);
    curve.addPoint(pt1);

    // generate a dstorted circle
    const int num_circle_pts = 12;
    vec3 last_pt = pt1;
    float rmin = 3;
    float rmax = 5;
    for(int i=0;i<num_circle_pts; i++) {
        float t = i * 2*M_PIf / num_circle_pts;
        float r = random(rmin, rmax);
        float x = r*cos(t);
        float y = r*sin(t);
        vec3 pos = vec3(x, y, z_front);
        curve.addPoint(pos);
        last_pt  = pos;
    }

    curve.addPoint(vec3(last_pt.x, last_pt.y, z_front-1));
    curve.addPoint(vec3(last_pt.x, last_pt.y, -z_back));

    curve.addPoint(curve[curve.count()-1] + 0.25f*(curve[curve.count()-1] - curve[curve.count()-2]));

    controller_.SetPath(curve_comp_);

    int num_labels = COUNTOF(gs_labels);
    EnemyTextComp* txt_comp = GetComponent<EnemyTextComp>();
    int r = random(0, num_labels);
    txt_comp->SetText(gs_labels[r]);

}

void Enemy::pushTrailPoint(const vec3& point) {
    if (!trail.empty()) {
        const float spacing = length(point - trail.back());
        if (spacing < 0.05f) {
            return;
        }
    }

    trail.push_back(point);
    const size_t maxPoints = 32;
    while (trail.size() > maxPoints) {
        trail.pop_front();
    }
}

void Enemy::simulateFixedStep(double dt) {

    //if (controller_.isAtTarget()) {
    //    return;
    //}

    statePrev = stateCur;

    if(b_has_valid_target_) 
        stateCur = controller_.update(stateCur, dt, target_ctx_);

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

    mat3 m = mat3::fromBasis(renderState.right, renderState.up, renderState.forward);
    const quaternion qnew = mat3_to_quat(m); 
    tc->SetPosition(renderState.position);
    tc->SetRotation(qnew);

    GetComponent<EnemyTextComp>()->SetColour(b_is_active_target_ ? 0xFFAAAA00 : 0);
}

void Enemy::AddRenderPackets(struct RenderFrameContext* rfc) const {

    RenderList* rl = rfc->rl_;

#if 0
    vec3 pos = renderState.position;
    vec3 up = renderState.up;
    vec3 right = renderState.right;
    vec3 fwd = renderState.forward;

    rl->addDebugLine(pos, pos + 3*right, vec4(1, 0,0, 1));
    rl->addDebugLine(pos, pos + 3*up, vec4(0, 1,0, 1));
    rl->addDebugLine(pos, pos + 3*fwd, vec4(0, 0,1, 1));
#endif

    const size_t n = trail.size();
    if (n >= 2) {
        vec3* dbg_lines = new vec3[2*(n-1)];
        vec4* colours = new vec4[n-1];
        for (size_t i = 0; i < n-1; ++i) {
            const float t = (float)i / (n - 1);
            const float alpha = 1;//0.35f + 0.65f * t;
            const float brightness = 0.25f + 0.75f * t;
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

    if(0) {
        // transpose rotation, because we want to rotate by it and not transform to this coord system
        mat4 m = mat4(
                target_ctx_.right.x, target_ctx_.up.x, target_ctx_.fwd.x, target_ctx_.path_pos.x,
                target_ctx_.right.y, target_ctx_.up.y, target_ctx_.fwd.y, target_ctx_.path_pos.y,
                target_ctx_.right.z, target_ctx_.up.z, target_ctx_.fwd.z, target_ctx_.path_pos.z, 
                0, 0, 0, 1);
        auto curve = curve_comp_->GetCurve();
        CurveDebugDraw(*curve, 20, false, &m, rl);

        TransformComponent* tc = GetComponent<TransformComponent>();

        vec3 closest_pos = spline_get_closest_point(*curve, tc->GetPosition()).point;
        rl->addDebugPoints(&closest_pos, 1, vec4(0,0,1,1), 10, true);
    }

}

PROPERTY_LIST_BEGIN_DERIVED(Enemy, GameObject)
    PROPERTY_READONLY_TEXT("Name", [](const Enemy& e) { return e.GetName(); });
    PROPERTY_READONLY_TEXT("Text", [](const Enemy& e) { return e.text_label; });
PROPERTY_LIST_END()


EnemySpawner* EnemySpawner::Create() {
    EnemySpawner* obj = new EnemySpawner();
    obj->last_time_spawned_ = TimerGetGameTime();
    return obj;
}

void EnemySpawner::Update(float dt) {


}

