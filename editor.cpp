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

enum class EditorOpMode {
	kMove,
	kRotate,
	kScale
};

enum class GizmoMode {
	kMove,
	kRotate,
	kScale
};

// TODOL move all variables in a single Editor state
static EditorOpMode s_editor_mode = EditorOpMode::kMove;
static GizmoMode s_gizmo_mode = GizmoMode::kMove;

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

static vec2 proj2screen(vec2 proj, float w, float h) {
	return vec2((0.5f*proj.x + 0.5f)*w, (0.5f*(1.0f - proj.y) + 0.5f)*h);
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

// drag related variables
static bool drag_started = false;
static vec3 drag_start_mouse_world_pos;
static vec2 drag_start_mouse_proj_pos;
static vec3 drag_start_obj_pos;
static vec3 drag_start_obj_scale;

static vec3 drag_cur_mouse_world_pos;
static float drag_obj_view_dist;
static int drag_type;

static vec3 ray_plane_intersect(const vec3 ray_dir, const vec3 ray_origin, const vec4 plane) {
	const vec3 n = plane.xyz(); // plane normal
	const vec3 p0 = n * plane.w; // point on plane
	// (r0 + rd*t - p0) ^ n = 0;
	// t = (p0 - r0)^n / rd^n;
	return ray_origin + ray_dir * dot((p0 - ray_origin), n) / dot(ray_dir, n);
}

static vec3 project_on_axis(const vec3 ray_dir, const vec3 ray_origin, int axis) {
	static const vec3 axes[3] = {vec3(1.0f, 0.0f, 0.0f), vec3(0.0f, 1.0f, 0.0f),
								 vec3(0.0f, 0.0f, 1.0f)};

	// for X or Y axis: intersect with xy plane at origin
	// for Z axis: intersect with xy plane at origin
	float t = axis == 2 ? -ray_origin.x / ray_dir.x : -ray_origin.z / ray_dir.z;
	vec3 int_pt = ray_origin + t * ray_dir;
	// project to x axis
	float int_dot = dot(int_pt, axes[axis]);
	return int_dot * axes[axis];
}

static EditorOpMode undate_input_mode(const EditorOpMode ed_mode) {

	 if(gos_GetKeyStatus(KEY_Q) == KEY_PRESSED)
        return EditorOpMode::kMove;
	 if(gos_GetKeyStatus(KEY_W) == KEY_PRESSED)
        return EditorOpMode::kRotate;
	 if(gos_GetKeyStatus(KEY_E) == KEY_PRESSED)
        return EditorOpMode::kScale;

	 return ed_mode;
}

static GizmoMode update_gizmo_mode(const EditorOpMode ed_mode) {

	 if(EditorOpMode::kMove == ed_mode)
        return GizmoMode::kMove;
	 else if(EditorOpMode::kRotate == ed_mode)
        return GizmoMode::kRotate;
	 else if(EditorOpMode::kScale == ed_mode)
        return GizmoMode::kScale;
	 else
        return GizmoMode::kMove;
}

void editor_update(camera *cam, const float /*dt*/) {

	int XDelta, YDelta, WheelDelta;
	float XPos, YPos;
	DWORD buttonsPressed;
	gos_GetMouseInfo(&XPos, &YPos, &XDelta, &YDelta, &WheelDelta, &buttonsPressed);
	vec2 cur_mouse_proj_pos = 2 * vec2(XPos, 1 - YPos) - 1;
	const float screen_width = (float)Environment.drawableWidth;
	const float screen_height = (float)Environment.drawableHeight;

	if (gos_GetKeyStatus(KEY_ESCAPE) == KEY_RELEASED)
		gos_TerminateApplication();

	s_editor_mode = undate_input_mode(s_editor_mode);
	s_gizmo_mode = update_gizmo_mode(s_editor_mode);

	const uint32_t sel_id = scene_get_object_id_under_cursor();
	GameObject *go_under_cursor = nullptr;
	if (sel_id >= scene::kFirstGameObjectId) {
		go_under_cursor = scene_get_object_by_id(sel_id);
	}

	if (gos_GetKeyStatus(KEY_LMOUSE) == KEY_PRESSED) {
		// get at screen center
		if (go_under_cursor) {
			g_sel_obj = go_under_cursor;
		}
		else if (sel_id >= ReservedObjIds::kGizmoFirst && sel_id < ReservedObjIds::kGizmoLast) {
			gosASSERT(g_sel_obj);
			drag_type = sel_id;
			const auto *tc = g_sel_obj->GetComponent<TransformComponent>();
			if (tc) {
				drag_start_obj_pos = tc->GetPosition();
				drag_start_obj_scale = tc->GetScale();
				mat4 vm = cam->get_view();
				vec3 vp = (vm * vec4(drag_start_obj_pos, 1.0f)).getXYZ();
				drag_obj_view_dist = vp.z;
				drag_start_mouse_proj_pos = cur_mouse_proj_pos;
				drag_start_mouse_world_pos =
					screen2world(cam, cur_mouse_proj_pos, drag_obj_view_dist);

				drag_started = true;
			}
		}
	}

	if (drag_started && gos_GetKeyStatus(KEY_LMOUSE) == KEY_HELD) {
		gosASSERT(g_sel_obj);
		drag_cur_mouse_world_pos = screen2world(cam, cur_mouse_proj_pos, drag_obj_view_dist);

		auto *tc = g_sel_obj->GetComponent<TransformComponent>();
		vec3 ray_origin = (cam->get_inv_view() * vec4(0, 0, 0, 1)).xyz();
		vec3 ray_dir = normalize(drag_cur_mouse_world_pos - ray_origin);
		vec3 ray_dir_start = normalize(drag_start_mouse_world_pos - ray_origin);
		switch (drag_type) {
			case ReservedObjIds::kGizmoMoveX:
			case ReservedObjIds::kGizmoMoveY:
			case ReservedObjIds::kGizmoMoveZ:
			{
				int axis = drag_type - ReservedObjIds::kGizmoMoveX;
				vec3 pr_start = project_on_axis(ray_dir, drag_start_mouse_world_pos, axis);
				vec3 pr_end = project_on_axis(ray_dir, drag_cur_mouse_world_pos, axis);
				vec3 upd_pos = drag_start_obj_pos + (pr_end - pr_start);
				tc->SetPosition(upd_pos);
				break;
			}
			case ReservedObjIds::kGizmoMoveXZ:
			case ReservedObjIds::kGizmoMoveYX:
			case ReservedObjIds::kGizmoMoveYZ:
			{
				vec3 cur_pos = tc->GetPosition();
				const vec4 planes[3] = {vec4(0.0f, 1.0f, 0.0f, cur_pos.y),
										vec4(0.0f, 0.0f, 1.0f, cur_pos.z),
										vec4(1.0f, 0.0f, 0.0f, cur_pos.x)};
				const int plane_idx = drag_type - ReservedObjIds::kGizmoMoveXZ;
				vec3 start_pos = ray_plane_intersect(ray_dir_start, ray_origin, planes[plane_idx]);
				vec3 offset = start_pos - drag_start_obj_pos;
				vec3 upd_pos = ray_plane_intersect(ray_dir, ray_origin, planes[plane_idx]);
				tc->SetPosition(upd_pos - offset);
				break;
			}
			case ReservedObjIds::kGizmoScaleX:
			case ReservedObjIds::kGizmoScaleY:
			case ReservedObjIds::kGizmoScaleZ:
			case ReservedObjIds::kGizmoScaleXZ:
			case ReservedObjIds::kGizmoScaleYX:
			case ReservedObjIds::kGizmoScaleYZ:
			{
				const int axis_idx = drag_type - ReservedObjIds::kGizmoScaleX;
				const vec3 axes[6] = { vec3(1,0,0), vec3(0,1,0), vec3(0,0,1), vec3(1,0,1), vec3(1,1,0), vec3(0,1,1) };
				const vec2 screen_delta =
					proj2screen(cur_mouse_proj_pos, screen_width, screen_height) -
					proj2screen(drag_start_mouse_proj_pos, screen_width, screen_height);
				// TODO: use distance to the object as an additional multiplier for better UX?
				const float scale = screen_delta.x * 0.025f; 
				const vec3 scale_axis = scale * axes[axis_idx] + vec3(1,1,1);
				tc->SetScale(drag_start_obj_scale * scale_axis);
				break;
			}
		}

	}
	if (gos_GetKeyStatus(KEY_LMOUSE) == KEY_RELEASED) {
		drag_started = false;
		log_debug("drag stop\n");
	}
}

static void add_debug_mesh(struct RenderFrameContext *rfc, RenderMesh *mesh, const mat4 &mat,
						   const vec4 &color, uint32_t selection_id = 0) {
    RenderList *frame_render_list = rfc->rl_;
    frame_render_list->ReservePackets(1);
    RenderPacket *rp = frame_render_list->AddPacket();
    rp->mesh_ = *mesh;
	rp->m_ = mat;
	rp->id_ = selection_id;
	rp->debug_color = color;

    rp->is_debug_pass = 1;
	rp->is_selection_pass = selection_id ? 1 : 0;

    rp->is_opaque_pass = 0;
    rp->is_render_to_shadow = 0;
    rp->is_transparent_pass = 0;
}
#if 0
static void add_debug_sphere_constant_size(struct RenderFrameContext *rfc, const vec3& pos, const vec4& color) {

	const float oo_no_scale_distance = 1.0f / 100.0f;
	const float cam_z = (rfc->view_ * vec4(pos, 1)).z;
	const mat4 tr = mat4::translation(pos) * mat4::scale(vec3(cam_z * oo_no_scale_distance));
	add_debug_mesh(rfc, res_man_load_mesh("sphere"), tr, color);
}

static void add_debug_mesh_constant_size(struct RenderFrameContext *rfc, RenderMesh *mesh,
										 const vec4 &color, const vec3 &pos,
										 const vec3 &scale = vec3(1)) {

	const float oo_no_scale_distance = 1.0f / 100.0f;
	const float cam_z = (rfc->view_ * vec4(pos, 1)).z;
	const mat4 tr = mat4::translation(pos) * mat4::scale(vec3(cam_z * oo_no_scale_distance) * scale);
	add_debug_mesh(rfc, mesh, tr, color);
}
#endif
static void add_debug_mesh_constant_size(struct RenderFrameContext *rfc, RenderMesh *mesh,
										 const vec4 &color, const mat4 &tr_m,
										 const vec3 &scale = vec3(1)) {

	const float oo_no_scale_distance = 1.0f / 100.0f;
	const float cam_z = (rfc->view_ * tr_m.getTranslationPoint()).z;
	const mat4 tr = tr_m * mat4::scale(vec3(cam_z * oo_no_scale_distance) * scale);
	add_debug_mesh(rfc, mesh, tr, color);
}

static void add_debug_axes_constant_size(struct RenderFrameContext *rfc, const vec3& pos, const GizmoMode mode) {

	RenderMesh* cube = res_man_load_mesh("cube");
	const float no_scale_distance = 100.0f;
	const float cam_z = (rfc->view_ * vec4(pos, 1)).z;
	const float scaler = cam_z / no_scale_distance;

	const mat4 tr_x = mat4::translation(pos) * mat4::translation(vec3(2.0f * scaler, 0.0f, 0.0f)) * mat4::scale(vec3(2.0f, 0.1f, 0.1f) * scaler);
	const mat4 tr_y = mat4::translation(pos) * mat4::translation(vec3(0.0f, 2.0f* scaler, 0.0f)) * mat4::scale(vec3(0.1f, 2.0f, 0.1f) * scaler);
	const mat4 tr_z = mat4::translation(pos) * mat4::translation(vec3(0.0f, 0.0f, 2.0f * scaler)) * mat4::scale(vec3(0.1f, 0.1f, 2.0f) * scaler);

	uint32_t axis_x_id = GizmoMode::kMove == mode ? ReservedObjIds::kGizmoMoveX : 0;
	uint32_t axis_y_id = GizmoMode::kMove == mode ? ReservedObjIds::kGizmoMoveY : 0;
	uint32_t axis_z_id = GizmoMode::kMove == mode ? ReservedObjIds::kGizmoMoveZ : 0;
	add_debug_mesh(rfc, cube, tr_x, vec4(1.0f, 0.15f, 0.15f, 1.0f), axis_x_id);
	add_debug_mesh(rfc, cube, tr_y, vec4(0.15f, 1.0f, 0.15f, 1.0f), axis_y_id);
	add_debug_mesh(rfc, cube, tr_z, vec4(0.15f, 0.15f, 1.0f, 1.0f), axis_z_id);

	if (GizmoMode::kScale == mode) {
		const mat4 scale_cube_scale = mat4::scale(vec3(.3f, 0.3f, 0.3f) * scaler);
		const mat4 tr_sx =
			mat4::translation(pos) * mat4::translation(vec3(4.0f * scaler, 0.0f, 0.0f)) * scale_cube_scale;
		const mat4 tr_sy =
			mat4::translation(pos) * mat4::translation(vec3(0.0f, 4.0f * scaler, 0.0f)) * scale_cube_scale;
		const mat4 tr_sz =
			mat4::translation(pos) * mat4::translation(vec3(0.0f, 0.0f, 4.0f * scaler)) * scale_cube_scale;

		add_debug_mesh(rfc, cube, tr_sx, vec4(1.0f, 0.15f, 0.15f, 1.0f), ReservedObjIds::kGizmoScaleX);
		add_debug_mesh(rfc, cube, tr_sy, vec4(0.15f, 1.0f, 0.15f, 1.0f), ReservedObjIds::kGizmoScaleY);
		add_debug_mesh(rfc, cube, tr_sz, vec4(0.15f, 0.15f, 1.0f, 1.0f), ReservedObjIds::kGizmoScaleZ);
	}


	const mat4 tr_xz = mat4::translation(pos) * mat4::scale(vec3(1.0f, 0.01f, 1.0f) * scaler) *
					  mat4::translation(vec3(2.0f, 0.0f, 2.0f));
	const mat4 tr_yx = mat4::translation(pos) * mat4::scale(vec3(1.0f, 1.0f, 0.01f) * scaler) *
					  mat4::translation(vec3(2.0f, 2.0f, 0.0f));
	const mat4 tr_yz = mat4::translation(pos) * mat4::scale(vec3(0.01f, 1.0f, 1.0f) * scaler) *
					  mat4::translation(vec3(0.0f, 2.0f, 2.0f));

	if (GizmoMode::kRotate != mode) {
		uint32_t plane_xz_id = GizmoMode::kMove == mode ? ReservedObjIds::kGizmoMoveXZ : ReservedObjIds::kGizmoScaleXZ;
		uint32_t plane_yx_id = GizmoMode::kMove == mode ? ReservedObjIds::kGizmoMoveYX : ReservedObjIds::kGizmoScaleYX;
		uint32_t plane_yz_id = GizmoMode::kMove == mode ? ReservedObjIds::kGizmoMoveYZ : ReservedObjIds::kGizmoScaleYZ;
		add_debug_mesh(rfc, cube, tr_xz, vec4(1.0f, 0.0f, 1.0f, .25f), plane_xz_id);
		add_debug_mesh(rfc, cube, tr_yx, vec4(1.0f, 1.0f, 0.0f, .25f), plane_yx_id);
		add_debug_mesh(rfc, cube, tr_yz, vec4(0.0f, 1.0f, 1.0f, .25f), plane_yz_id);
	}
}

void editor_render_update(struct RenderFrameContext *rfc)
{
    if(g_sel_obj) {
        auto* tc = g_sel_obj->GetComponent<TransformComponent>();
        if(tc) {
			const vec3 pos = tc->GetPosition();
			add_debug_axes_constant_size(rfc, pos, s_gizmo_mode);
		}
    }

    RenderMesh *sphere = res_man_load_mesh("sphere");
    assert(sphere);
    {
		auto& light_list = scene_get_light_list();
        rfc->rl_->ReservePackets(light_list.size());

        // add lights to debug render pass
		for (auto &l : light_list) {
			vec4 c(l.color_.getXYZ(), 0.5f);
			add_debug_mesh_constant_size(rfc, sphere, c, l.transform_, vec3(0.1f));
		}
    }
}
