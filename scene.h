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
        kFirstEditorObjectId,
        kFirstGameObjectId = 0x100,
		//kLast,
	};
}

struct SceneViewInfo {
    mat4 view_mat_;
    mat4 inv_view_mat_;
    mat4 proj_mat_;
    float fov_;
    float aspect_;
};

class GameObject;
class Component;
typedef uint32_t GameObjectId;

namespace scene {
	constexpr uint32_t kInvalidObjectId = 0xffffffff;
	constexpr uint32_t kFirstGameObjectId = ReservedObjIds::kFirstGameObjectId;
}

void initialize_scene();
void finalize_scene();

void scene_update(const struct camera* cam, const bool b_update_simulation, const float dt);
void scene_render_update(struct RenderFrameContext *rfc, bool is_in_editor_mode, bool b_exclusive_3dview);

void scene_get_intersected_objects(
    const vec3& ws_orig, const vec3& ws_dir,
    std::vector<std::pair<float, GameObject *>>& out_obj);

void scene_add_game_object(GameObject* go);

// using non-type version of enable_if_t with second parameter
template <typename T, std::enable_if_t<std::is_base_of_v<GameObject, T>, int> = 0>
T* scene_create_game_object() {
    T* go = new T();
    scene_add_game_object(go);
    return go;
}

void scene_attach_component(GameObject* go, Component* comp);
void scene_detach_component(GameObject* go, Component* comp);

void scene_delete_game_object(GameObject* go);

void scene_add_component__(Component* comp);

// TODO: do we really need any args? maybe only standard base ones
template <typename T, typename... Params, std::enable_if_t<std::is_base_of_v<Component, T>, int> = 0>
T* scene_create_component(GameObject* go, Params && ... args) {
    T* comp = new T(std::forward<Params>(args)...);
    scene_add_component__(comp);
    if(go)
        scene_attach_component(go, comp);
    return comp;
}

void scene_delete_component(Component* comp);

const SceneViewInfo& scene_get_view_info();

const std::vector<PointLight>& scene_get_light_list();

GameObject* scene_get_object_by_id(GameObjectId id);

void scene_set_object_id_under_cursor(uint32_t obj_id);
uint32_t scene_get_object_id_under_cursor();

// thi sfunction immediately returns object under cursor, uses simple AABB intersection
class GameObject* scene_get_object_under_cursor(int x, int y);

void scene_set_camera_controller(class ICameraController* cc);
class ICameraController* scene_get_camera_controller();

