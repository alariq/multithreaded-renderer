#pragma once

#include "obj_model.h"
#include "utils/vec.h"
#include <atomic>
#include <vector>

void initialize_particle_positions(class SPHSceneObject* o);

class PBDTestObject: public GameObject {
    TransformComponent* transform_ = nullptr;

    class PBDUnifiedSimulation* sim_;
    vec2 sim_dim_;
    vec2 sim_origin_;

    RenderMesh* sphere_mesh_ = nullptr; 
    HGOSVERTEXDECLARATION vdecl_;
    std::vector<HGOSBUFFER> inst_vb_;
    mutable int cur_inst_vb_ = 0;
    
    HGOSRENDERMATERIAL  mat_;

    std::atomic_bool b_initalized_rendering_resources;

public:
    static PBDTestObject* Create();

    virtual void Update(float /*dt*/) override;

    virtual const char* GetName() const override { return "PBD Scene"; };
    virtual RenderMesh* GetMesh() const override { return nullptr; }
    virtual ~PBDTestObject();

    virtual void InitRenderResources() override;
    virtual void DeinitRenderResources() override;

    // IRenderable
    virtual void AddRenderPackets(struct RenderFrameContext* rfc) const override;
};

