#pragma once

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

void initialize_editor();
void finalize_editor();

void editor_update(struct camera* cam, const float dt);
void editor_render_update(struct RenderFrameContext *rfc);

int editor_register_user_editor(UserEditorInterface ue_interface);
void editor_unregister_user_editor(int id);
