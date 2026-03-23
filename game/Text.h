#pragma once

#include "../obj_model.h"

template<> inline constexpr ComponentType 
get_component_type<class GameTextComp>() { return ComponentType::kGameText; }
template<> inline constexpr ComponentType 
get_component_type<class EnemyTextComp>() { return ComponentType::kEnemyText; }


// Game related dirty hardcoded crap goes here
class GameTextComp: public TransformComponent, public IRenderable {
    HGOSFONT3D font_handle_;
    HGOSFONT3D small_font_handle_;
    float intensity_;
    float frame_time_ms_;
    char text[128];
  public:
	virtual ComponentType GetType() const override { return get_component_type<GameTextComp>(); }

	virtual void InitRenderResources() override;
	virtual void DeinitRenderResources() override;

    virtual IRenderable* getRenderableInterface() override { return this; }
	virtual void AddRenderPackets(struct RenderFrameContext *) const override;

    // this will be updated by Init/Deinit Render Resoueces
	virtual void Initialize() override;
	virtual void Deinitialize() override;

    virtual void UpdateComponent(float dt) override;
};

class EnemyTextComp: public TransformComponent, public IRenderable {
    HGOSFONT3D font_handle_;
    float intensity_;
    uint32_t colour_;
    char text[32];
    bool b_is_active_;
  public:
	virtual ComponentType GetType() const override { return get_component_type<EnemyTextComp>(); }

	virtual void InitRenderResources() override;
	virtual void DeinitRenderResources() override;

    virtual IRenderable* getRenderableInterface() override { return this; }
	virtual void AddRenderPackets(struct RenderFrameContext *) const override;

    // this will be updated by Init/Deinit Render Resoueces
	virtual void Initialize() override;
	virtual void Deinitialize() override;

    virtual void UpdateComponent(float dt) override;

    //----------------------------------------------

    void SetText(const char* t);
    const char* GetText() const { return text; }
    void SetColour(uint32_t c) { colour_ = c; }
};
