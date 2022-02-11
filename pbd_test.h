#pragma once

#include "obj_model.h"
#include "utils/vec.h"
#include <atomic>
#include <vector>

void unified_pbd_init();
void unified_pbd_deinit();
void unified_pbd_update(float dt);
struct PBDUnifiedSimulation* unified_pbd_get();

void initialize_particle_positions(class SPHSceneObject* o);

class StaticCollisionComponent: public TransformComponent {
	int id_ = -1;
    struct CollisionWorld* world_ = nullptr;
    virtual ~StaticCollisionComponent() {};

	static void on_transformed(TransformComponent* );
	static const ComponentType type_ = ComponentType::kPBDStaticCollision;
public:
	virtual ComponentType GetType() const override { return type_; }
    StaticCollisionComponent() {};

    static StaticCollisionComponent* Create(const struct PBDUnifiedSimulation*sim, const vec2 &dim);
    static void Destroy(StaticCollisionComponent* comp);

	virtual void Initialize() override { state_ = kInitialized; }
	virtual void Deinitialize() override { state_ = kUninitialized; }
};

class PBDStaticCollisionObj: public GameObject {
    struct Tuple {
        TransformComponent* tr_;
        StaticCollisionComponent* coll_;
    };
    Tuple Tuple_;
public:
    PBDStaticCollisionObj(const PBDUnifiedSimulation* sim, const vec2 &dim);
    ~PBDStaticCollisionObj();

    // IEditorObject
    virtual int GetIconID() const override { return 1; }
    //

    void addToSimulation(PBDUnifiedSimulation* sim);
    void removeFromSimulation(PBDUnifiedSimulation* sim);

    virtual const char* GetName() const override { return "unified pbd collision"; };
    virtual void Update(float dt) override {};
};

class PBDVisComponent : public Component, public IRenderable {

	RenderMesh* sphere_mesh_ = nullptr;
	HGOSVERTEXDECLARATION vdecl_;
	std::vector<HGOSBUFFER> inst_vb_;
	mutable int cur_inst_vb_ = 0;

	HGOSRENDERMATERIAL mat_;

	std::atomic_bool b_initalized_rendering_resources;

  public:
    virtual IRenderable* getRenderableInterface() override { return this; }
	virtual ComponentType GetType() const override { return ComponentType::kPBDVisComponent; }
	// IRenderable
	virtual void InitRenderResources() override;
	virtual void DeinitRenderResources() override;
	virtual void AddRenderPackets(struct RenderFrameContext* rfc) const override;

    // this will be updated by Init/Deinit Render Resoueces
	virtual void Initialize() override {}
	virtual void Deinitialize() override {}
};

class PBDTestObject: public GameObject {
public:
    typedef struct {
        uint32_t contacts: 1;
        uint32_t friction: 1;
        // rotation of soft body particle
        uint32_t sb_part_rot: 1;
        uint32_t distance_constraints: 1;
    } DDFlags;
    DDFlags dbg_flags_;

private:
    vec2 sim_dim_;

public:
    struct PBDUnifiedSimulation* sim_;

    static PBDTestObject* Create();
    virtual void Update(float /*dt*/) override;
    virtual const char* GetName() const override { return "PBD Scene"; };
    virtual ~PBDTestObject();

    const DDFlags& getDbgFlags() { return dbg_flags_; }

};

