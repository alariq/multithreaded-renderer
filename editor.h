#pragma once
#include "engine/utils/vec.h"
#include "engine/utils/quaternion.h"
#include <stdint.h>

struct UserEditorInterface {
    typedef class GameObject* (*update_t)(struct camera* cam, float dt, class GameObject* sel_go);
    typedef void (*render_update_t)(struct RenderFrameContext *rfc);
    typedef bool (*wants_activate_t)();
    typedef const char* (*name_t)();

    update_t update;
    render_update_t render_update;
    wants_activate_t wants_activate;
    name_t name;
};

struct ITransformInterface {
    virtual void SetPosition(vec3, void* userdata) = 0;
    virtual vec3 GetPosition(void* userdata) const = 0;
    virtual void SetRotation(quaternion, void* userdata) = 0;
    virtual quaternion GetRotation(void* userdata) const = 0;

    virtual void SetScale(vec3 s, void* userdata) = 0;
    virtual vec3 GetScale(void* userdata) const = 0;
    virtual vec3 GetWorldSpaceScale(void* userdata) const = 0;
    virtual void SetWorldSpaceScale(const vec3 ws, void* userdata) = 0;

    virtual bool HasMove() const = 0;
    virtual bool HasRotate() const = 0;
    virtual bool HasScale() const = 0;
};

void initialize_editor();
void initialize_render_editor();
void finalize_editor();

void editor_update(struct camera* cam, const float dt);
void editor_render_update(struct RenderFrameContext *rfc, bool b_editor_mode, bool b_exclusive_3dview);

int editor_register_user_editor(UserEditorInterface ue_interface);
void editor_unregister_user_editor(int id);

// returns selection buffer object index
int editor_add_gizmo(struct ITransformInterface*, void* userdata);

void editor_set_selected_obj(class GameObject* go);
class GameObject* editor_get_selected_obj();


#if 1
bool editor_get_3dview_hovered();
ivec4 editor_get_3dview_rect();
ivec4 editor_calc_3dview(bool b_full, intptr_t scene_colour);
#endif

class ICameraController* editor_get_cam_controller();
// a bit of a hack to not expose editor cam controller in .h
void editor_cam_controller_set_transform(const vec3& pos, float rotX);
