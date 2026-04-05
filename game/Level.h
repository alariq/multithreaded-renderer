#pragma once

#include "Path.h"
#include "MainShip.h"
#include "Enemy.h"

#include "engine/utils/spline.h"
#include "engine/utils/myarray.h"

#include <string>

class Level: public GameObject {

    std::string name_;

    Curve<vec3> mainCurve_;
    const Path* mainPath = nullptr;
    MainShip* mainShip = nullptr;

    BufferT<Enemy*, int> enemies_;
    uint64_t last_time_spawned_;
    uint64_t spawn_interval_ms_;
    int total_spawned_;

    EnemyTargetCtx ctx_;
    Enemy* text_target_ = nullptr;


    void simulateFixedStep(double dt);

    //TODP: move fixed step update to higher level, so go classes should not take care of it
    double accumulator = 0;

    void UpdateTextInput();

  public:
    PROPERTY_SUPPORT(Level);
    PROPERTY_POLYMORPHIC_DRAW_IMPL(Level)

	virtual const char* GetName() const override { return name_.c_str(); } 
    static Level* Create(const char* res);

    void BeginGame();
    void EngGame();
    virtual void Update(float dt) override;

    virtual void AddRenderPackets(struct RenderFrameContext* rfc) const override;

};
PROPERTY_LIST_DECLARE_DERIVED(Level, GameObject)
