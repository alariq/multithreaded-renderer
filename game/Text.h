#pragma once

#include "../obj_model.h"

// Game related dirty hardcoded crap goes here
class GameTextComp: public TransformComponent, public IRenderable {
    HGOSFONT3D font_handle_;
    HGOSFONT3D small_font_handle_;
    float intensity_;
    float frame_time_ms_;
    char text[128];
  public:
	virtual ComponentType GetType() const override { return ComponentType::kGameText; }

	virtual void InitRenderResources() override;
	virtual void DeinitRenderResources() override;

    virtual IRenderable* getRenderableInterface() override { return this; }
	virtual void AddRenderPackets(struct RenderFrameContext *) const override;

    // this will be updated by Init/Deinit Render Resoueces
	virtual void Initialize() override;
	virtual void Deinitialize() override;

    virtual void UpdateComponent(float dt) override;
};

