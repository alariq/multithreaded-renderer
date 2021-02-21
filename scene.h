#pragma once

#include "engine/utils/vec.h"
#include <vector>
#include <stdint.h>
#include "renderer.h"

namespace ReservedObjIds {
	enum : uint32_t {
		kFirst = 1,
		kGizmoFirst = kFirst,
		kGizmoMoveX = kGizmoFirst,
		kGizmoMoveY,
		kGizmoMoveZ,

		kGizmoMoveXZ,
		kGizmoMoveYX,
		kGizmoMoveYZ,

		kGizmoRotateX,
		kGizmoRotateY,
		kGizmoRotateZ,
		kGizmoRotateXYZ,

		kGizmoScaleX,
		kGizmoScaleY,
		kGizmoScaleZ,

		kGizmoScaleXZ,
		kGizmoScaleYX,
		kGizmoScaleYZ,
		kGizmoScaleXYZ,
		kGizmoLast,
		// here can go other ids
		kLast,
	};
}

class GameObject;
typedef uint32_t GameObjectId;

namespace scene {
	constexpr uint32_t kInvalidObjectId = 0xffffffff;
	constexpr uint32_t kFirstGameObjectId = ReservedObjIds::kLast;
}

void initialize_scene(const struct camera* cam, struct RenderFrameContext* rfc);
void finalize_scene();

void scene_update(const struct camera* cam, const bool b_update_simulation, const float dt);
void scene_render_update(struct RenderFrameContext*, bool is_in_editor_mode);

void scene_get_intersected_objects(
    const vec3& ws_orig, const vec3& ws_dir,
    std::vector<std::pair<float, GameObject *>>& out_obj);

void scene_add_game_object(GameObject* go);
void scene_delete_game_object(GameObject* go);

const std::vector<PointLight>& scene_get_light_list();

GameObject* scene_get_object_by_id(GameObjectId id);

void scene_set_object_id_under_cursor(uint32_t obj_id);
uint32_t scene_get_object_id_under_cursor();

// thi sfunction immediately returns object under cursor, uses simple AABB intersection
class GameObject* scene_get_object_under_cursor(int x, int y);

