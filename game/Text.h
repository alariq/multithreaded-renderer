#pragma once

#include "../obj_model.h"

template<> inline constexpr ComponentType 
get_component_type<class GameTextComp>() { return ComponentType::kGameText; }
template<> inline constexpr ComponentType 
get_component_type<class EnemyTextComp>() { return ComponentType::kEnemyText; }


// Game related dirty hardcoded crap goes here
class GameTextComp: public TransformComponent {
    std::string font_name_;
    std::string slug_font_name_;
    float intensity_;
    int text_size_;
    float frame_time_ms_;
    char text[128];
    class FontRenderProxy* proxy_;

  public:
    PROPERTY_SUPPORT(GameTextComp)
    PROPERTY_POLYMORPHIC_DRAW_IMPL(GameTextComp)

	virtual ComponentType GetType() const override { return get_component_type<GameTextComp>(); }

	virtual void Initialize() override;

    virtual IRenderProxy* CreateRenderProxy() override;
    virtual void DestroyRenderProxy(struct RenderFrameContext* rfc) override;
	virtual IRenderProxy* GetRenderProxy() override;

    virtual void UpdateComponent(float dt) override;
	virtual void RenderUpdateComponent(struct RenderFrameContext *) override;

};
PROPERTY_LIST_DECLARE_DERIVED(GameTextComp, TransformComponent)

class EnemyTextComp: public TransformComponent {
    HGOSFONT3D font_handle_;
    float intensity_;
    uint32_t colour_;
    char text[32];
    int text_size_;
    bool b_is_active_;

    class FontRenderProxy* proxy_;
  public:
    PROPERTY_SUPPORT(EnemyTextComp);
    PROPERTY_POLYMORPHIC_DRAW_IMPL(EnemyTextComp)

	virtual ComponentType GetType() const override { return get_component_type<EnemyTextComp>(); }

	virtual void Initialize() override;

    IRenderProxy* CreateRenderProxy() override;
    void DestroyRenderProxy(struct RenderFrameContext* rfc) override;
    IRenderProxy* GetRenderProxy() override;

    virtual void UpdateComponent(float dt) override;
	virtual void RenderUpdateComponent(struct RenderFrameContext *) override;

    //----------------------------------------------

    void SetText(const char* t);
    const char* GetText() const { return text; }
    void SetColour(uint32_t c) { colour_ = c; }
};
PROPERTY_LIST_DECLARE_DERIVED(EnemyTextComp, TransformComponent)
