#pragma once

#include "engine/utils/vec.h"
#include <vector>
class GameObject;
typedef uint32_t GameObjectId;

namespace scene {
	constexpr uint32_t kInvalidObjectId = 0xffffffff;
}

void initialize_scene(const struct camera* cam, struct RenderFrameContext* rfc);
void finalize_scene();

void scene_update(const struct camera* cam, const float dt);
void scene_render_update(struct RenderFrameContext*);

void scene_get_intersected_objects(
    const vec3& ws_orig, const vec3& ws_dir,
    std::vector<std::pair<float, GameObject *>>& out_obj);

GameObject* scene_get_object_by_id(GameObjectId id);

void scene_set_object_id_under_cursor(uint32_t obj_id);
uint32_t scene_get_object_id_under_cursor();

// thi sfunction immediately returns object under cursor, uses simple AABB intersection
class GameObject* scene_get_object_under_cursor(int x, int y);

