#pragma once

#include "obj_model.h"
#include "utils/vec.h"
#include <atomic>

void initialize_particle_positions(class SPHSceneObject* o);

class SPHSceneObject : public GameObject {
    struct SPHParticle2D* particles_;
    uint32_t* part_flags_;
    int* part_indices_;
    int num_particles_;
    vec2 view_dim_;
    float radius_;
    struct SPHGrid* grid_;
    struct SPHFluidModel* fluid_;
    class SPHBoundaryModel* boundary_;
    class SPHSurfaceMesh* surface_;
    // cached transform component
    TransformComponent* transform_ = nullptr;

    RenderMesh* sphere_mesh_ = nullptr; 
    HGOSVERTEXDECLARATION vdecl_;
    std::vector<HGOSBUFFER> inst_vb_;
    mutable int cur_inst_vb_ = 0;
    
    HGOSRENDERMATERIAL  mat_;
    HGOSRENDERMATERIAL  sdf_mat_;
    DWORD surface_grid_tex_;
    DWORD sdf_tex_;

    std::atomic_bool b_initalized_rendering_resources;

public:
    static SPHSceneObject* Create(const vec2& view_dim, int num_particles, const vec3& pos);

    virtual void Update(float /*dt*/) override;

    virtual const char* GetName() const override { return "SPH Scene"; };
    virtual RenderMesh* GetMesh() const override { return nullptr; }
    virtual ~SPHSceneObject();

	float GetRadius() { return radius_; }

    const vec2& GetBounds() { return view_dim_; }

	SPHParticle2D* GetParticles() { return particles_; }
	int GetParticlesCount() { return num_particles_; }

    virtual void InitRenderResources() override;
    void DeinitRenderResources();

    // IRenderable
    virtual void AddRenderPackets(struct RenderFrameContext* rfc) const override;
};

