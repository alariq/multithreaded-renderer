#pragma once

void initialize_scene(const class camera* cam, class RenderFrameContext* rfc);
void finalize_scene();

void scene_update(const class camera* cam, const float dt);
void scene_render_update(class RenderFrameContext*);
