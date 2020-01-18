#pragma once

void initialize_editor();
void finalize_editor();

void editor_update(class camera* cam, const float dt);
void editor_render_update(struct RenderFrameContext *rfc);
