#pragma once
#include "obj_model.h"

template<> inline constexpr ComponentType 
get_component_type<class C_Billboard>() { return ComponentType::kBillboard; }
class C_Billboard : public MeshComponent {
    bool b_horizontal_only_;
  public:
    PROPERTY_SUPPORT(C_Billboard)
    PROPERTY_POLYMORPHIC_DRAW_IMPL(C_Billboard)

	virtual ComponentType GetType() const override { return get_component_type<C_Billboard>(); }
	virtual void UpdateComponent(float dt) override;
    virtual IRenderProxy* CreateRenderProxy() override;

};

PROPERTY_LIST_DECLARE_DERIVED(C_Billboard, MeshComponent)


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
