#pragma once

#include "obj_model.h"
#include "engine/utils/vec.h"
#include "engine/utils/math_utils.h"

#include <deque>
#include <string>

struct EnemyState {
    vec3 position;
    vec3 velocity;
    vec3 acceleration;
    vec3 right;
    vec3 up;
    vec3 forward;
    float speed;
    float thrust;

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

        return out;
    }
};


// maybe have a separate path follower component to just follow path
// and seprate for other enemy related things
class BasicEnemyAIController {
    const class Path* movePath;
    float cur_t;
    
    public:
        EnemyState update(const EnemyState& s, float dt);

        void SetPath(const class Path* path) {
            cur_t = 0;
            movePath = path;
        }
};

class Enemy: public GameObject {

    std::string name_;
    class MeshComponent* mesh_comp_;
    

    vec3 origin_pos;
    float total_time;

    BasicEnemyAIController controller_;
    EnemyState statePrev, stateCur, renderState;
    double accumulator;

    std::deque<vec3> trail;
    const class Path* myPath = nullptr;
    //Curve<vec3> curve;

    GameObject* target_ = nullptr;

    void simulateFixedStep(double dt);
    void pushTrailPoint(const vec3& point);

  public:
	virtual const char* GetName() const override { return name_.c_str(); } 

    static Enemy* Create(const char* res);

    void Initialize(GameObject* intarget);
    virtual void Update(float dt) override;

    virtual void AddRenderPackets(struct RenderFrameContext* rfc) const override;

    void SetPath(const Path* path);
};
