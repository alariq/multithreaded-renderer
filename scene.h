#pragma once

#include "engine/utils/vec.h"
#include <vector>
class GameObject;

void initialize_scene(const class camera* cam, struct RenderFrameContext* rfc);
void finalize_scene();

void scene_update(const class camera* cam, const float dt);
void scene_render_update(struct RenderFrameContext*);

void scene_get_intersected_objects(
    const vec3& ws_orig, const vec3& ws_dir,
    std::vector<std::pair<float, GameObject *>>& out_obj);

class GameObject* scene_get_object_under_cursor(int x, int y);

