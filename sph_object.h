#pragma once

#include "obj_model.h"
#include "utils/vec.h"
#include <atomic>
#include <vector>

class SPHBoundaryComponent : public TransformComponent {
	class SPHBoundaryModel *boundary_ = nullptr;
    virtual ~SPHBoundaryComponent() {};

	static void on_transformed(TransformComponent* );
public:
	static const ComponentType type_ = ComponentType::kSPHBoundary;
	virtual ComponentType GetType() const override { return type_; }
    SPHBoundaryComponent() {};

    class SPHBoundaryModel* getBoundary() const { return boundary_; }
    class SPHBoundaryModel* getBoundary() { return boundary_; }

    virtual void UpdateComponent(float dt) override;

    static SPHBoundaryComponent* Create(const class SPHSimulation *sim, const vec2 &dim, bool b_invert, bool b_dynamic);
    static void Destroy(SPHBoundaryComponent* comp);
};

class SPHBoundaryObject: public GameObject {

    struct BoundaryTuple {
        TransformComponent* tr_;
        SPHBoundaryComponent* bm_;
    };
    BoundaryTuple Tuple_;
public:
    SPHBoundaryObject(const SPHSimulation* sim, const vec2 &dim, bool b_invert, bool b_dynamic);
    ~SPHBoundaryObject();

    // IEditorObject
    virtual int GetIconID() const override { return 1; }
    //

    void addToSimulation(SPHSimulation* sim);
    void removeFromSimulation(SPHSimulation* sim);

    virtual const char* GetName() const override { return "boundary"; };
    virtual void Update(float dt) override;
    virtual RenderMesh* GetMesh() const override { return nullptr; }

    // IRenderable
    virtual void InitRenderResources() override;
    virtual void DeinitRenderResources() override;
    virtual void AddRenderPackets(struct RenderFrameContext *rfc) const override;
};

void initialize_particle_positions(class SPHSceneObject* o);

class SPHSceneObject : public GameObject {
    uint32_t* part_flags_;
    int* part_indices_;
    vec2 view_dim_;
    float radius_;
    struct SPHGrid* grid_;
    struct SPHFluidModel* fluid_;
    std::vector<class SPHBoundaryModel*> boundaries_;
    std::vector<struct SPHEmitter*> emitters_;
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

    virtual void InitRenderResources() override;
    virtual void DeinitRenderResources() override;

    // IRenderable
    virtual void AddRenderPackets(struct RenderFrameContext* rfc) const override;
};

