#pragma once

#include "obj_model.h"
#include "engine/utils/vec.h"
#include "engine/utils/math_utils.h"
#include "engine/utils/spline.h"
#include "engine/utils/imgui_property_list.h"

#include <deque>
#include <string>

struct EnemyTargetCtx {
    vec3 pos; // target location
    vec3 dir; // target direction

    // NOTE: could just pass "t" and set main path, main path might bee needed anyway
    vec3 path_pos; // target closest pos on path
    vec3 right, up, fwd; // path basis
    //...
};

struct EnemyState {
#if 0
    vec3 position = vec3(0);
    vec3 velocity = vec3(0);
    vec3 acceleration = vec3(0);
    vec3 right = vec3(1,0,0);
    vec3 up = vec3(0,1,0);
    vec3 forward = vec3(0,0,1);
    float speed = 0;
    float thrust = 0;
    int num_cycles = 0;
#else
    vec3 position;
    vec3 velocity;
    vec3 acceleration;
    vec3 right;
    vec3 up;
    vec3 forward;
    float speed;
    float thrust;
    int num_cycles;
#endif

    static EnemyState interp(const EnemyState& prev, const EnemyState& curr, double t) {
        t = saturate(t);
        EnemyState out = curr;

        out.position = lerp(prev.position, curr.position, t);
        out.velocity = lerp(prev.velocity, curr.velocity, t);
        out.acceleration = lerp(prev.acceleration, curr.acceleration, t);

        out.forward = lerp(prev.forward, curr.forward, t);
        out.up = lerp(prev.up, curr.up, t);
        out.right = lerp(prev.right, curr.right, t);
        orthonormalize_basis(out.forward, out.up, out.right);

        out.speed = lerp(prev.speed, curr.speed, t);
        out.thrust = lerp(prev.thrust, curr.thrust, t);

        // no sense to interpolate
        out.num_cycles = curr.num_cycles;

        return out;
    }
};


// maybe have a separate path follower component to just follow path
// and seprate for other enemy related things
class BasicEnemyAIController {
    const class C_Curve* movePath;
    float cur_t;
    
    public:
        EnemyState update(const EnemyState& s, float dt, const EnemyTargetCtx& ctx);

        void SetPath(const class C_Curve* path) {
            cur_t = 0;
            movePath = path;
        }
};

class Enemy: public GameObject {

    std::string name_;
    class MeshComponent* mesh_comp_;
    class C_Curve* curve_comp_;
    
    vec3 origin_pos;
    float total_time;

    BasicEnemyAIController controller_;
    EnemyState statePrev, stateCur, renderState;
    double accumulator;

    std::deque<vec3> trail;

    char text_label[16];
    bool b_is_active_target_ = false;

    bool b_has_valid_target_ = false;
    EnemyTargetCtx target_ctx_;

    void simulateFixedStep(double dt);
    void pushTrailPoint(const vec3& point);

  public:
    PROPERTY_SUPPORT(Enemy);
    PROPERTY_POLYMORPHIC_DRAW_IMPL(Enemy)

	virtual const char* GetName() const override { return name_.c_str(); } 

    static Enemy* Create(const char* res);

    void Initialize(GameObject* intarget);
    virtual void Update(float dt) override;

    virtual void AddRenderPackets(struct RenderFrameContext* rfc) const override;

    void UpdateTargetCtx(const EnemyTargetCtx& ctx) { target_ctx_ = ctx; b_has_valid_target_ = true; }
    const EnemyState& GetState() const { return stateCur; }
    void SetActiveTarget(bool b_is_active) { b_is_active_target_ = b_is_active; }
};

PROPERTY_LIST_DECLARE_DERIVED(Enemy, GameObject)

class EnemySpawner: GameObject {

    uint64_t last_time_spawned_;

	virtual const char* GetName() const override { return "enemy_spawner"; } 

    static EnemySpawner* Create();

    virtual void Update(float dt) override;
    virtual void AddRenderPackets(struct RenderFrameContext* rfc) const override {};
    virtual int IsSelectable() const override { return false; }

};
