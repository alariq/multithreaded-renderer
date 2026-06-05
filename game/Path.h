#pragma once

#include <cstdint>
#include "obj_model.h"
#include "engine/utils/spline.h"
#include "engine/utils/vec.h"

template<> inline constexpr ComponentType 
get_component_type<class C_Curve>() { return ComponentType::kCurve; }
class C_Curve: public TransformComponent, public IRenderable {
    Curve<vec3> curve_;
    BufferT<int, int> gizmo_node_ids_;

public:

    static C_Curve* Create(const char* name) {
        return new C_Curve();
    }

    const Curve<vec3>* GetCurve() const { return &curve_; }
    Curve<vec3>* GetCurve() { return &curve_; }
    int32_t GetNumNodes() const;
    vec3 Get(float t) const;
    vec3 GetDerivative(float t) const;

    // AddPoint(vec3)
    // RemovePoint(int idx)

    PROPERTY_SUPPORT(C_Curve);
    PROPERTY_POLYMORPHIC_DRAW_IMPL(C_Curve)

    virtual void Initialize() override;
    virtual void UpdateComponent(float dt) override;

	virtual void InitRenderResources() override {
        state_ = Component::kInitialized;
    }
	virtual void DeinitRenderResources() override {
        state_ = Component::kUninitialized;
    }

    virtual bool IsSelectable() { return true; }
    virtual IRenderable* getRenderableInterface() override { return this; }
	virtual ComponentType GetType() const override { return get_component_type<C_Curve>(); }
	virtual void AddRenderPackets(struct RenderFrameContext *) const override;

    bool IsValid() { return true; }
    void SetPosition(vec3 p, void* userdata) override;
    vec3 GetPosition(void* userdata) const override;
    void SetRotation(quaternion q, void* userdata) override { }
    quaternion GetRotation(void* userdata) const override { return quaternion::identity(); }
    void SetScale(vec3 s, void* userdata) override { }
    vec3 GetScale(void* userdata) const override { return vec3(1); }
    vec3 GetWorldSpaceScale(void* userdata) const override { return vec3(1); }
    void SetWorldSpaceScale(const vec3 ws, void* userdata) override { }

    virtual bool HasMove() const override { return true; }
    virtual bool HasRotate() const override { return false; }
    virtual bool HasScale() const override { return false; }
};

PROPERTY_LIST_DECLARE_DERIVED(C_Curve, TransformComponent)


class O_Path: public GameObject {
    std::string name_;
    C_Curve* curve_comp_ = nullptr;
    BufferT<MeshComponent*, int> checkpoint_meshes_;

    O_Path() {}

public:
    PROPERTY_SUPPORT(O_Path)
    PROPERTY_POLYMORPHIC_DRAW_IMPL(O_Path)

    static O_Path* Create(const char* res);
    virtual const char* GetName() const override { return name_.c_str(); } 

    virtual void Update(float dt) override;

    virtual const TransformComponent* GetTransformInterface() const override { return GetComponent<C_Curve>(); };
    virtual TransformComponent* GetTransformInterface() override { return GetComponent<C_Curve>(); };

    const Curve<vec3>* GetCurve() const { assert(curve_comp_); return curve_comp_->GetCurve(); }
    Curve<vec3>* GetCurve() { assert(curve_comp_); return curve_comp_->GetCurve(); }
};

PROPERTY_LIST_DECLARE_DERIVED(O_Path, GameObject)


void CurveDebugDraw(const Curve<vec3>& curve, int num_pts_per_segment, bool b_draw_basis, const struct mat4* transform, class RenderList* rl);
