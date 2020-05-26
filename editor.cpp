#include "editor.h"
#include "scene.h"
#include "obj_model.h"
#include "res_man.h"
#include "renderer.h"
#include "engine/gameos.hpp"
#include "engine/utils/camera.h"
#include "utils/logging.h"

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

// converts screen position to world space at a given distance from camera
// x,y = [-1,1] screen space range
// z - distance from camera pos (in world space)
// returns: world position at a given distance from camera
static vec3 get_cursor_pos(const mat4* inv_proj, const mat4* inv_view, const float x, const float y,
                    const float dist_from_cam) {
    vec4 view_pos = *inv_proj * vec4(x, y, 0.5f, 1.0f);
    vec3 view_pos_n = dist_from_cam * normalize(view_pos.xyz());
    vec3 wpos = (*inv_view * vec4(view_pos_n, 1.0f)).xyz();
    return wpos;
}

static vec3 screen2world(const camera *cam, const vec2 screen_pos,
                    const float dist_from_cam) {
    vec4 view_pos = cam->get_inv_projection() * vec4(screen_pos, 0.0f, 1.0f);
    vec3 view_pos_n = dist_from_cam * normalize(view_pos.xyz());
    vec3 wpos = (cam->get_inv_view() * vec4(view_pos_n, 1.0f)).xyz();
    return wpos;
}

static vec3 screen2world_dir(const camera *cam, const vec2 screen_pos) {
    vec4 view_pos = cam->get_inv_projection() * vec4(screen_pos, 0.0f, 1.0f);
    vec3 view_pos_n = normalize(view_pos.xyz());
	return normalize((cam->get_inv_view() * vec4(view_pos_n, 0.0f)).xyz());
}


// x,y = [-1,1] screen space range
GameObject* select_object_under_cursor(const camera *cam, float x, float y) {

	const mat4& inv_view = cam->get_inv_view();
    const vec3 ws_cursor_pos =
		get_cursor_pos(&inv_view, &cam->get_inv_projection(), x, y, 10.0f);

    const vec3 ws_cam_pos = (inv_view * vec4(0, 0, 0, 1)).xyz();
    const vec3 ws_dir = normalize(ws_cursor_pos - ws_cam_pos);
    using el_t = std::pair<float, GameObject *>;
    std::vector<el_t> int_obj;
    scene_get_intersected_objects(ws_cam_pos, ws_dir, int_obj);

	std::sort(int_obj.begin(), int_obj.end(),
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
static vec3 drag_start_mouse_world_pos;
static vec3 drag_cur_mouse_world_pos;
static vec3 drag_start_obj_pos;
static float drag_obj_view_dist;

static vec3 project_on_x_axis(const vec3 ray_dir, const vec3 ray_origin)
{
	// intersect with xy plane at origin
	float t = - ray_origin.z / ray_dir.z;
	vec3 int_pt = ray_origin + t * ray_dir;
	// project to x axis
	float int_dot = dot(int_pt, vec3(1, 0, 0));
	return int_dot * vec3(1, 0, 0);
}

void editor_update(camera *cam, const float /*dt*/) {

	int XDelta, YDelta, WheelDelta;
	float XPos, YPos;
	DWORD buttonsPressed;
	gos_GetMouseInfo(&XPos, &YPos, &XDelta, &YDelta, &WheelDelta, &buttonsPressed);
	vec2 cur_mouse_proj_pos = 2 * vec2(XPos, 1 - YPos) - 1;

	GameObject *go_under_cursor = scene_get_object_by_id(scene_get_object_id_under_cursor());

	if (gos_GetKeyStatus(KEY_ESCAPE) == KEY_RELEASED)
		gos_TerminateApplication();

	if (gos_GetKeyStatus(KEY_LMOUSE) == KEY_PRESSED) {
		// get at screen center
		g_sel_obj = go_under_cursor;
		if (g_sel_obj) {
			const auto *tc = g_sel_obj->GetComponent<TransformComponent>();
			if (tc) {
				drag_start_obj_pos = tc->GetPosition();
				mat4 vm = cam->get_view();
				vec3 vp = (vm * vec4(drag_start_obj_pos, 1.0f)).getXYZ();
				drag_obj_view_dist = vp.z;
				drag_start_mouse_world_pos =
					screen2world(cam, cur_mouse_proj_pos, drag_obj_view_dist);

				drag_started = true;
			}
		}
		log_debug("mouse x:%f y:%f\n", XPos, YPos);
	}
	if (drag_started && gos_GetKeyStatus(KEY_LMOUSE) == KEY_HELD && g_sel_obj) {

		drag_cur_mouse_world_pos = screen2world(cam, cur_mouse_proj_pos, drag_obj_view_dist);

		vec3 ray_dir = screen2world_dir(cam, cur_mouse_proj_pos);
		vec3 pr_start = project_on_x_axis(ray_dir, drag_start_mouse_world_pos);
		vec3 pr_end = project_on_x_axis(ray_dir, drag_cur_mouse_world_pos);

		auto *tc = g_sel_obj->GetComponent<TransformComponent>();
		vec3 upd_pos = drag_start_obj_pos + (pr_end - pr_start);
		tc->SetPosition(upd_pos);
	}
	if (gos_GetKeyStatus(KEY_LMOUSE) == KEY_RELEASED) {
		drag_started = false;
		log_debug("drag stop\n");
	}
}

static void add_debug_mesh(struct RenderFrameContext *rfc, RenderMesh* mesh, const mat4& mat, const vec4& color)
{
    RenderList *frame_render_list = rfc->rl_;
    frame_render_list->ReservePackets(1);
    RenderPacket *rp = frame_render_list->AddPacket();
    rp->mesh_ = *mesh;
	rp->m_ = mat;
	rp->debug_color = color;
    rp->is_debug_pass = 1;
    rp->is_opaque_pass = 0;
    rp->is_render_to_shadow = 0;
    rp->is_transparent_pass = 0;
}

static void add_debug_sphere_constant_size(struct RenderFrameContext *rfc, const vec3& pos, const vec4& color) {

	const float oo_no_scale_distance = 1.0f / 100.0f;
	const float cam_z = (rfc->view_ * vec4(pos, 1)).z;
	const mat4 tr = mat4::translation(pos) * mat4::scale(vec3(cam_z * oo_no_scale_distance));
	add_debug_mesh(rfc, res_man_load_mesh("sphere"), tr, color);
}

static void add_debug_mesh_constant_size(struct RenderFrameContext *rfc, RenderMesh* mesh, const vec3& pos, const vec4& color) {

	const float oo_no_scale_distance = 1.0f / 100.0f;
	const float cam_z = (rfc->view_ * vec4(pos, 1)).z;
	const mat4 tr = mat4::translation(pos) * mat4::scale(vec3(cam_z * oo_no_scale_distance));
	add_debug_mesh(rfc, mesh, tr, color);
}


void editor_render_update(struct RenderFrameContext *rfc)
{
    RenderList *frame_render_list = rfc->rl_;
    frame_render_list->ReservePackets(4);

    const vec3 screen_center = get_cursor_pos(&rfc->inv_view_, &rfc->inv_proj_, 0.0f, 0.0f, 10.0f);
	add_debug_sphere_constant_size(rfc, screen_center, vec4(1, 1, 1, 0.8f));
	add_debug_sphere_constant_size(rfc, drag_cur_mouse_world_pos, vec4(1, 0, 0, 1.0f));

    if(g_sel_obj) {
        auto* tc = g_sel_obj->GetComponent<TransformComponent>();
        if(tc) {
			const vec3 pos = tc->GetPosition();
			add_debug_mesh_constant_size(rfc, res_man_load_mesh("axes"), pos, vec4(1,1,1, 0.8f));
		}
    }
}

