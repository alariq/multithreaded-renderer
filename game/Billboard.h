#pragma once
#include "obj_model.h"

template<> inline constexpr ComponentType 
get_component_type<class C_Billboard>() { return ComponentType::kBillboard; }
class C_Billboard : public TransformComponent, public IRenderable {
	RenderMesh *mesh_;
    DWORD texture_id_;
    std::string texture_name_;
    mutable bool b_update_texture;

	mutable std::string pending_mesh_name_;
	mutable std::atomic<void *> pending_mesh_;

	mutable std::string pending_texture_name_;
	mutable std::atomic<DWORD> pending_texture_id_;


  public:
    PROPERTY_SUPPORT(C_Billboard)
    PROPERTY_POLYMORPHIC_DRAW_IMPL(C_Billboard)

	C_Billboard() : mesh_(nullptr), texture_id_(0), 
    b_update_texture(false), pending_mesh_(nullptr), pending_texture_id_(0) {}

    //int getState() const { return (int)initState.load(); }
    virtual bool IsSelectable() { return true; }
    virtual IRenderable* getRenderableInterface() override { return this; }
	virtual ComponentType GetType() const override { return get_component_type<C_Billboard>(); }
	static C_Billboard *Create(const char *res);
	virtual void InitRenderResources() override;
	virtual void DeinitRenderResources() override;
	virtual void AddRenderPackets(struct RenderFrameContext *) const override;
	virtual void UpdateComponent(float dt) override;
	void SetTexure(const char* texture_name);
    const AABB& GetAABB() const { return mesh_->aabb_; }
    // this will be updated by Init/Deinit Render Resoueces
	virtual void Initialize() override {}
	virtual void Deinitialize() override {}

};

PROPERTY_LIST_DECLARE_DERIVED(C_Billboard, TransformComponent)


class O_Billboard: public GameObject {
    std::string name_;
public:
    PROPERTY_SUPPORT(O_Billboard)
    PROPERTY_POLYMORPHIC_DRAW_IMPL(O_Billboard)

   static O_Billboard* Create(const char* res);
   virtual const char* GetName() const override { return name_.c_str(); } 

    virtual const TransformComponent* GetTransformInterface() const override { return GetComponent<C_Billboard>(); };
    virtual TransformComponent* GetTransformInterface() override { return GetComponent<C_Billboard>(); };
};

PROPERTY_LIST_DECLARE_DERIVED(O_Billboard, GameObject)
