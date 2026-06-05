#pragma once

#include "obj_model.h"
#include "engine/utils/spline.h"
#include "engine/utils/imgui_property_list.h"

#include "game/ShipController.h"
#include "game/WingController.h"
#include "game/Path.h"

struct CheckPoint {
    vec3 position;
    float radius;
};

class MainShip: public GameObject {

    std::string name_;
    MeshComponent* mesh_comp_;

    vec3 origin_pos;
    float radius;
    float total_time;

    PlaneState plane_state_;
    PlaneAI ship_ai_;

    AircraftPhysics physics;
    WingOnlyAIController ai_controller;
    WingControllerConfig controller_config;
    AircraftState statePrev, stateCur, renderState;
    double accumulator;

    float turnRate = 2.5f;
    float maxSpeed = 3.0f;
    float targetSpeed = 35.0f;

    std::deque<vec3> trail;
    const ReparameterizeByArclength<vec3>* repar;
    const Curve<vec3>* levelCurve;

    std::vector<CheckPoint> check_points_;
    int cur_check_point_idx_;

    GameObject* target_ = nullptr;

    void pushTrailPoint(const vec3& point);
    void simulateFixedStep(double dt);
    virtual void Update2(float dt);

    public:

    PROPERTY_SUPPORT(MainShip);
    PROPERTY_POLYMORPHIC_DRAW_IMPL(MainShip)

    virtual ~MainShip();

    virtual const char* GetName() const override { return name_.c_str(); } 

    static MainShip* Create(const char* res);
    void Initialize(const std::vector<CheckPoint>& cps, GameObject* intarget, const Curve<vec3>* master_curve);

    virtual void Update(float dt) override;
    virtual void AddRenderPackets(struct RenderFrameContext* rfc) const override;
};

PROPERTY_LIST_DECLARE_DERIVED(MainShip, GameObject)

