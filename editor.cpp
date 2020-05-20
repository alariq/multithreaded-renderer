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

static GameObject* g_sel_obj = nullptr; 
void initialize_editor()
{
}

void finalize_editor()
{
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
GameObject* select_object_under_cursor(const camera *cam, float x, float y) {

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
        return nullptr;

    GameObject *closest_obj = int_obj[0].second;
    printf("int name: %s\n", closest_obj->GetName());
    for (auto &obj : int_obj) {
        printf("%.3f : %s\n", obj.first, obj.second->GetName());
    }
    return closest_obj;
}

static bool drag_started = false;
static vec2 drag_start_pos;
static vec3 drag_start_obj_pos;
//static vec3 drag_obj_x_axis;

void editor_update(camera* cam, const float dt) {

    int XDelta, YDelta, WheelDelta;
    float XPos, YPos;
    DWORD buttonsPressed;
    gos_GetMouseInfo(&XPos, &YPos, &XDelta, &YDelta, &WheelDelta, &buttonsPressed);

    if(gos_GetKeyStatus(KEY_ESCAPE) == KEY_RELEASED)
        gos_TerminateApplication();

    if (gos_GetKeyStatus(KEY_LMOUSE) == KEY_PRESSED) {
        // get at screen center
        g_sel_obj = select_object_under_cursor(cam, 0.0f, 0.0f);
        if (g_sel_obj) {
            const auto *tc = g_sel_obj->GetComponent<TransformComponent>();
            if (tc) {
                drag_start_obj_pos = tc->GetPosition();
                drag_start_pos = vec2(XPos, YPos);
                // drag_obj_x_axis =
                drag_started = true;
            }
        }
        printf("mouse x:%f y:%f\n", XPos, YPos);
    }
    if(drag_started && gos_GetKeyStatus(KEY_LMOUSE) == KEY_HELD && g_sel_obj)
    {
        const vec2 cur_pos = vec2(XPos, YPos);
        const float len = length(cur_pos - drag_start_pos);
        auto* tc = g_sel_obj->GetComponent<TransformComponent>();
        tc->SetPosition(drag_start_obj_pos + vec3(len*10.0f,0,0));
        printf("dragging \n");
    }
    if(gos_GetKeyStatus(KEY_LMOUSE) == KEY_RELEASED) {
        drag_started = false;
        printf("drag stop\n");
    }
}

void editor_render_update(struct RenderFrameContext *rfc)
{
    RenderList *frame_render_list = rfc->rl_;
    frame_render_list->ReservePackets(2);

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

    // draw axes at the center of a selected object
    if(g_sel_obj) {
        const auto* tc = g_sel_obj->GetComponent<TransformComponent>();
        if(tc) {
           const vec3 pos = tc->GetPosition();

            RenderPacket *rp = frame_render_list->AddPacket();
            rp->mesh_ = *res_man_load_mesh("axes");
            rp->m_ = mat4::translation(pos) * mat4::scale(vec3(1.0f));
            rp->debug_color = vec4(1,1,1, 0.8f);
            rp->is_debug_pass = 1;
            rp->is_opaque_pass = 0;
            rp->is_render_to_shadow = 0;
            rp->is_transparent_pass = 0;
        }
    }
}

