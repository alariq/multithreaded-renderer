#include "editor.h"
#include "scene.h"
#include "obj_model.h"
#include "res_man.h"
#include "renderer.h"
#include "engine/gameos.hpp"
#include "engine/utils/camera.h"

#include <algorithm>
#include <vector>
#include <cassert>

static MeshObject* g_sel_obj_marker = nullptr;
void initialize_editor()
{
    g_sel_obj_marker = MeshObject::Create("cube");
    g_sel_obj_marker->SetPosition(vec3(0, 0, 0));
    g_sel_obj_marker->SetScale(vec3(1.1f));
}

void finalize_editor()
{
    delete g_sel_obj_marker;
}

// x,y = [-1,1] screen space range
// z - distance from camera pos
vec3 get_cursor_pos(const camera *cam, const float x, const float y,
                    const float dist_from_cam) {
    vec4 view_pos = cam->inv_proj_ * vec4(x, y, 0.5f, 1.0f);
    vec3 view_pos_n = dist_from_cam * normalize(view_pos.xyz());
    vec3 wpos = (cam->inv_view_ * vec4(view_pos_n, 1.0f)).xyz();
    return wpos;
}

// x,y = [-1,1] screen space range
void select_object_under_cursor(const camera *cam, float x, float y) {

    const vec3 ws_cursor_pos = get_cursor_pos(cam, x, y, 10.0f);
    // go->SetPosition(wpos);

    const vec3 ws_cam_pos = (cam->inv_view_ * vec4(0, 0, 0, 1)).xyz();
    const vec3 ws_dir = normalize(ws_cursor_pos - ws_cam_pos);
    using el_t = std::pair<float, GameObject *>;
    std::vector<el_t> int_obj;
    scene_get_intersected_objects(ws_cam_pos, ws_dir, int_obj);

    std::sort(
        int_obj.begin(), int_obj.end(),
        [](const el_t &a, const el_t &b) -> bool { return a.first < b.first; });

    if (int_obj.empty())
        return;

    const GameObject *closest_obj = int_obj[0].second;
    const vec3 pos = closest_obj->GetTransform().getTranslation();
    g_sel_obj_marker->SetPosition(pos);
    printf("int name: %s\n", closest_obj->GetName());
    for (auto &obj : int_obj) {
        printf("%.3f : %s\n", obj.first, obj.second->GetName());
    }
}

void editor_update(camera* cam, const float dt) {
    if (gos_GetKeyStatus(KEY_LMOUSE) == KEY_PRESSED) {
        // get at screen center
        select_object_under_cursor(cam, 0.0f, 0.0f);
    }
}

void editor_render_update(struct RenderFrameContext *rfc)
{
    RenderList *frame_render_list = rfc->rl_;
    frame_render_list->ReservePackets(1);

    RenderMesh *sphere = res_man_load_mesh("sphere");
    assert(sphere);
    extern camera g_camera;

    const vec3 cursor_pos = get_cursor_pos(&g_camera, 0.0f, 0.0f, 10.0f);
    RenderPacket *rp = frame_render_list->AddPacket();
    rp->mesh_ = *sphere;
    rp->m_ = mat4::translation(cursor_pos) * mat4::scale(vec3(0.05f));
    rp->debug_color = vec4(1,1,1, 0.8f);
    rp->is_debug_pass = 1;
    rp->is_opaque_pass = 0;
    rp->is_render_to_shadow = 0;
    rp->is_transparent_pass = 0;
}

