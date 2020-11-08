#pragma once

#include "obj_model.h"
#include "utils/vec.h"

struct SPHParticle2D {
	vec2 pos;
	vec2 vel;
	vec2 force;
	float density;
	float pressure;
};

void sph_update(SPHParticle2D *particles, int count, vec2 view_dim);

class SPHSceneObject : public GameObject {
    SPHParticle2D* particles_;
    int num_particles_;
    vec2 view_dim_;
    float radius_;

    RenderMesh*         sphere_mesh_;
    HGOSBUFFER			inst_vb_;
    HGOSRENDERMATERIAL  mat_;
public:
    static SPHSceneObject* Create(const vec2& view_dim, int num_particles, const vec3& pos);

    virtual void Update(float /*dt*/) override;

    virtual const char* GetName() const override { return "SPH Scene"; };
    virtual RenderMesh* GetMesh() const override { return nullptr; }
    virtual ~SPHSceneObject();

	void SetRadius(float r) { radius_ = r; }
	float GetRadius() { return radius_; }

    const vec2& GetBounds() { return view_dim_; }

	SPHParticle2D* GetParticles() { return particles_; }
	int GetParticlesCount() { return num_particles_; }

    virtual void InitRenderResources() override;
    void DeinitRenderResources();

    // Renderable
    virtual void AddRenderPackets(class RenderList* rl) const override;
};

void initialize_particle_positions(SPHSceneObject* o);
