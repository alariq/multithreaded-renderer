#include "editor.h"
#include "scene.h"
#include "obj_model.h"
#include "res_man.h"
#include "renderer.h"
#include "render_utils.h"
#include "engine/gameos.hpp"
#include "engine/utils/camera.h"
#include "utils/logging.h"
#include "utils/vec.h"
#include "utils/quaternion.h"
#include "utils/logging.h"
#include "utils/math_utils.h"
#include "utils/camera_utils.h"

#include <algorithm>
#include <vector>
#include <cassert>
#include <cfloat> // FLT_MAX

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


class Gizmo {
	static const float kNoScaleDistance;
	static const float kAxisLength;
	static const float kRotSphereRadius;
	static const float kScaleCubesScale;

	GizmoMode mode_ = GizmoMode::kMove;
	vec3 pos_ = vec3(0,0,0);
	mat4 rot_ = mat4::identity();
	bool bWorldSpace = false;

  public:
	// controlled object position
	void set_position(const vec3 &pos) { pos_ = pos; }
	// controlled object rotation
	void set_rotation(const quaternion& q) { rot_ = quat_to_mat4(q); }
	mat4 get_rotation() { return rot_; }
	void set_world_space(bool ws) { bWorldSpace = ws; }
	bool get_world_space() { return bWorldSpace; }
	void update_mode(const EditorOpMode ed_mode) {
		if (EditorOpMode::kMove == ed_mode)
			mode_ = GizmoMode::kMove;
		else if (EditorOpMode::kRotate == ed_mode)
			mode_ = GizmoMode::kRotate;
		else if (EditorOpMode::kScale == ed_mode)
			mode_ = GizmoMode::kScale;
		else
			mode_ = GizmoMode::kMove;
	}

	float get_rotation_sphere_radius(const camera* cam) const {
		return kRotSphereRadius * (cam->get_view() * vec4(pos_, 1.0f)).z / kNoScaleDistance;
	}

	void draw(struct RenderFrameContext* rfc) {
		const vec3 pos = pos_;
		GizmoMode mode = mode_;
		const mat4 rot = bWorldSpace ? mat4::identity() : rot_;

		RenderMesh *cube = res_man_load_mesh("cube");
		const float cam_z = (rfc->view_ * vec4(pos, 1)).z;
		const float scaler = cam_z / kNoScaleDistance;
		const float al = kAxisLength;

		const mat4 tr_x = mat4::translation(pos) * rot * mat4::translation(vec3(al * scaler, 0.0f, 0.0f)) * 
						  mat4::scale(vec3(al, 0.1f, 0.1f) * scaler);
		const mat4 tr_y = mat4::translation(pos) * rot * mat4::translation(vec3(0.0f, al *scaler, 0.0f)) * 
						  mat4::scale(vec3(0.1f, al, 0.1f) * scaler);
		const mat4 tr_z = mat4::translation(pos) * rot * mat4::translation(vec3(0.0f, 0.0f, al * scaler)) * 
						  mat4::scale(vec3(0.1f, 0.1f, al) * scaler);

		uint32_t axis_x_id = GizmoMode::kMove == mode ? ReservedObjIds::kGizmoMoveX : 0;
		uint32_t axis_y_id = GizmoMode::kMove == mode ? ReservedObjIds::kGizmoMoveY : 0;
		uint32_t axis_z_id = GizmoMode::kMove == mode ? ReservedObjIds::kGizmoMoveZ : 0;
		add_debug_mesh(rfc, cube, tr_x, vec4(1.0f, 0.15f, 0.15f, 1.0f), axis_x_id);
		add_debug_mesh(rfc, cube, tr_y, vec4(0.15f, 1.0f, 0.15f, 1.0f), axis_y_id);
		add_debug_mesh(rfc, cube, tr_z, vec4(0.15f, 0.15f, 1.0f, 1.0f), axis_z_id);

		if (GizmoMode::kScale == mode) {
			const float cl = kScaleCubesScale;
			const mat4 scale_cube_scale = mat4::scale(vec3(cl, cl, cl) * scaler);
			const mat4 tr_sx =
				mat4::translation(pos) * rot * mat4::translation(vec3(2.0f * al * scaler, 0.0f, 0.0f)) * scale_cube_scale ;
			const mat4 tr_sy =
				mat4::translation(pos) * rot * mat4::translation(vec3(0.0f, 2.0f * al * scaler, 0.0f)) * scale_cube_scale ;
			const mat4 tr_sz =
				mat4::translation(pos) * rot * mat4::translation(vec3(0.0f, 0.0f, 2.0f * al * scaler)) * scale_cube_scale ;

			add_debug_mesh(rfc, cube, tr_sx, vec4(1.0f, 0.15f, 0.15f, 1.0f),
						   ReservedObjIds::kGizmoScaleX);
			add_debug_mesh(rfc, cube, tr_sy, vec4(0.15f, 1.0f, 0.15f, 1.0f),
						   ReservedObjIds::kGizmoScaleY);
			add_debug_mesh(rfc, cube, tr_sz, vec4(0.15f, 0.15f, 1.0f, 1.0f),
						   ReservedObjIds::kGizmoScaleZ);

			const mat4 tr_sxyz = mat4::translation(pos) * scale_cube_scale;
			add_debug_mesh(rfc, cube, tr_sxyz, vec4(0.15f, 0.15f, 1.0f, 1.0f),
						   ReservedObjIds::kGizmoScaleXYZ);
		}

		const mat4 tr_xz = mat4::translation(pos) * rot * mat4::scale(vec3(1.0f, 0.01f, 1.0f) * scaler) *
						   mat4::translation(vec3(2.0f, 0.0f, 2.0f));
		const mat4 tr_yx = mat4::translation(pos) * rot * mat4::scale(vec3(1.0f, 1.0f, 0.01f) * scaler) *
						   mat4::translation(vec3(2.0f, 2.0f, 0.0f));
		const mat4 tr_yz = mat4::translation(pos) * rot * mat4::scale(vec3(0.01f, 1.0f, 1.0f) * scaler) *
						   mat4::translation(vec3(0.0f, 2.0f, 2.0f));

		if (GizmoMode::kRotate != mode) {
			uint32_t plane_xz_id = GizmoMode::kMove == mode ? ReservedObjIds::kGizmoMoveXZ
															: ReservedObjIds::kGizmoScaleXZ;
			uint32_t plane_yx_id = GizmoMode::kMove == mode ? ReservedObjIds::kGizmoMoveYX
															: ReservedObjIds::kGizmoScaleYX;
			uint32_t plane_yz_id = GizmoMode::kMove == mode ? ReservedObjIds::kGizmoMoveYZ
															: ReservedObjIds::kGizmoScaleYZ;
			add_debug_mesh(rfc, cube, tr_xz, vec4(1.0f, 0.0f, 1.0f, .25f), plane_xz_id);
			add_debug_mesh(rfc, cube, tr_yx, vec4(1.0f, 1.0f, 0.0f, .25f), plane_yx_id);
			add_debug_mesh(rfc, cube, tr_yz, vec4(0.0f, 1.0f, 1.0f, .25f), plane_yz_id);
		} else {
			RenderMesh *torus = res_man_load_mesh("torus");
			RenderMesh *sphere = res_man_load_mesh("sphere");
			const float sr = kRotSphereRadius;

			const mat4 tr_rx = mat4::translation(pos) * rot * mat4::rotationY(90 * M_PI / 180.0f) *
							   mat4::scale(vec3(sr, sr, 0.01f) * scaler);
			const mat4 tr_ry = mat4::translation(pos) * rot * mat4::rotationX(90 * M_PI / 180.0f) *
							   mat4::scale(vec3(sr, sr, 0.01f) * scaler);
			const mat4 tr_rz =
				mat4::translation(pos) * rot * mat4::scale(vec3(sr, sr, 0.01f) * scaler);

			const mat4 tr_s = mat4::translation(pos) * mat4::scale(vec3(sr, sr, sr) * scaler);
			add_debug_mesh(rfc, sphere, tr_s, vec4(.5f, 0.5f, .5f, .8f),
						   ReservedObjIds::kGizmoRotateXYZ);

			add_debug_mesh(rfc, torus, tr_rx, vec4(1.0f, 0.f, 0.f, 1.0f),
						   ReservedObjIds::kGizmoRotateX);
			add_debug_mesh(rfc, torus, tr_ry, vec4(0.f, 1.0f, 0.f, 1.0f),
						   ReservedObjIds::kGizmoRotateY);
			add_debug_mesh(rfc, torus, tr_rz, vec4(0.f, 0.f, 1.0f, 1.0f),
						   ReservedObjIds::kGizmoRotateZ);

		}
	}
};

const float Gizmo::kNoScaleDistance = 50.0f;
const float Gizmo::kAxisLength = 2.0f;
const float Gizmo::kRotSphereRadius = 3.0f;
const float Gizmo::kScaleCubesScale = 0.15f;
Gizmo g_gizmo;

// TODO: move all variables in a single Editor state
static EditorOpMode s_editor_mode = EditorOpMode::kMove;
static GameObject* g_sel_obj = nullptr;

static std::vector<UserEditorInterface> registered_editors;
static int g_active_user_editor = -1;
void initialize_editor()
{
}

void finalize_editor()
{
}

int editor_register_user_editor(UserEditorInterface ue_interface) {
    registered_editors.push_back(ue_interface);
    return (int)registered_editors.size()-1;
}
void editor_unregister_user_editor(int id) {
    assert(id>=0 && id < registered_editors.size());
    registered_editors.erase(registered_editors.begin() + id);

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
static quaternion drag_start_obj_rot;
static vec3 drag_start_obj_scale;
static vec3 drag_start_obj_world_scale;

static vec3 drag_cur_mouse_world_pos;
static float drag_obj_view_dist;
static int drag_type;

static vec3 drag_rotation_gizmo_helper_pos;

static EditorOpMode update_input_mode(const EditorOpMode ed_mode) {

	 if(gos_GetKeyStatus(KEY_Q) == KEY_PRESSED)
        return EditorOpMode::kMove;
	 if(gos_GetKeyStatus(KEY_W) == KEY_PRESSED)
        return EditorOpMode::kRotate;
	 if(gos_GetKeyStatus(KEY_E) == KEY_PRESSED)
        return EditorOpMode::kScale;

	 return ed_mode;
}

void editor_update(camera *cam, const float dt) {

	int XDelta, YDelta, WheelDelta;
	float XPos, YPos;
	DWORD buttonsPressed;
	gos_GetMouseInfo(&XPos, &YPos, &XDelta, &YDelta, &WheelDelta, &buttonsPressed);
	vec2 cur_mouse_proj_pos = 2 * vec2(XPos, 1 - YPos) - 1;
	const float screen_width = (float)Environment.drawableWidth;
	const float screen_height = (float)Environment.drawableHeight;

	if (gos_GetKeyStatus(KEY_ESCAPE) == KEY_RELEASED) {
        if(g_active_user_editor == -1) {
    		gos_TerminateApplication();
        } else {
            g_active_user_editor = -1;
            log_info("Switched to main editor\n");
        }
    }

	if (gos_GetKeyStatus(KEY_1) == KEY_RELEASED)
		g_gizmo.set_world_space(!g_gizmo.get_world_space());
		
	s_editor_mode = update_input_mode(s_editor_mode);
	g_gizmo.update_mode(s_editor_mode);

	const uint32_t sel_id = scene_get_object_id_under_cursor();
	GameObject *go_under_cursor = nullptr;
	if (sel_id >= scene::kFirstGameObjectId) {
		go_under_cursor = scene_get_object_by_id(sel_id);
	}

	if (gos_GetKeyStatus(KEY_LMOUSE) == KEY_PRESSED) {
		if (go_under_cursor) {
			g_sel_obj = go_under_cursor;
			const auto *tc = g_sel_obj->GetComponent<TransformComponent>();
			if (tc) {
				g_gizmo.set_position(tc->GetPosition());
				g_gizmo.set_rotation(tc->GetRotation());
			}
		}
		else if (sel_id >= ReservedObjIds::kGizmoFirst && sel_id < ReservedObjIds::kGizmoLast) {
			gosASSERT(g_sel_obj);
			drag_type = sel_id;
			const auto *tc = g_sel_obj->GetComponent<TransformComponent>();
			if (tc) {
				drag_start_obj_pos = tc->GetPosition();
				drag_start_obj_rot = tc->GetRotation();
				drag_start_obj_scale = tc->GetScale();
				drag_start_obj_world_scale = tc->GetWorldScale();
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

	if (drag_started && gos_GetKeyStatus(KEY_LMOUSE) == KEY_HELD && drag_start_mouse_proj_pos!=cur_mouse_proj_pos) {
		gosASSERT(g_sel_obj);
		drag_cur_mouse_world_pos = screen2world(cam, cur_mouse_proj_pos, drag_obj_view_dist);

		auto *tc = g_sel_obj->GetComponent<TransformComponent>();
		g_gizmo.set_position(tc->GetPosition());
		vec3 ray_origin = (cam->get_inv_view() * vec4(0, 0, 0, 1)).xyz();
		vec3 ray_dir = normalize(drag_cur_mouse_world_pos - ray_origin);
		vec3 ray_dir_start = normalize(drag_start_mouse_world_pos - ray_origin);
		switch (drag_type) {
			case ReservedObjIds::kGizmoMoveX:
			case ReservedObjIds::kGizmoMoveY:
			case ReservedObjIds::kGizmoMoveZ:
			{
				const int axis_idx = drag_type - ReservedObjIds::kGizmoMoveX;
				static const vec3 axes[3] = {vec3(1.0f, 0.0f, 0.0f), vec3(0.0f, 1.0f, 0.0f),
											 vec3(0.0f, 0.0f, 1.0f)};
				vec3 axis = axes[axis_idx];
				if(!g_gizmo.get_world_space())
					axis = (g_gizmo.get_rotation() * vec4(axis, 0.0f)).xyz();
				vec3 pr_start = project_on_vector(ray_dir, drag_start_mouse_world_pos, axis);
				vec3 pr_end = project_on_vector(ray_dir, drag_cur_mouse_world_pos, axis);
				vec3 upd_pos = drag_start_obj_pos + (pr_end - pr_start);
				tc->SetPosition(upd_pos);
				break;
			}
			case ReservedObjIds::kGizmoMoveXZ:
			case ReservedObjIds::kGizmoMoveYX:
			case ReservedObjIds::kGizmoMoveYZ:
			{
				const int plane_idx = drag_type - ReservedObjIds::kGizmoMoveXZ;
				vec3 cur_pos = tc->GetPosition(); // maybe start pos?
				const vec4 planes[3] = {vec4(0.0f, 1.0f, 0.0f, cur_pos.y),
										vec4(0.0f, 0.0f, 1.0f, cur_pos.z),
										vec4(1.0f, 0.0f, 0.0f, cur_pos.x)};
				vec4 plane = planes[plane_idx];
				if (!g_gizmo.get_world_space()) {
					plane = transform_plane_around_point(g_gizmo.get_rotation(), plane, cur_pos);
				}

				vec3 start_pos = ray_plane_intersect(ray_dir_start, ray_origin, plane);
				vec3 offset = start_pos - drag_start_obj_pos;
				vec3 upd_pos = ray_plane_intersect(ray_dir, ray_origin, plane);
				drag_rotation_gizmo_helper_pos = upd_pos;

				tc->SetPosition(upd_pos - offset);
				break;
			}
			case ReservedObjIds::kGizmoRotateX:
			case ReservedObjIds::kGizmoRotateY:
			case ReservedObjIds::kGizmoRotateZ:
			{
				vec3 cur_pos = tc->GetPosition();
				int plane_idx = drag_type - ReservedObjIds::kGizmoRotateX;
				const vec4 planes[3] = {vec4(1.0f, 0.0f, 0.0f, cur_pos.x),
										vec4(0.0f, 1.0f, 0.0f, cur_pos.y),
										vec4(0.0f, 0.0f, 1.0f, cur_pos.z)};

				mat3 rot_m = drag_start_obj_rot.to_mat3();
				const vec4 plane = g_gizmo.get_world_space()
									   ? planes[plane_idx]
									   : make_plane(rot_m.getCol(plane_idx), cur_pos);

				vec3 start_pos = ray_plane_intersect(ray_dir_start, ray_origin, plane);
				vec3 upd_pos = ray_plane_intersect(ray_dir, ray_origin, plane);
				float r = g_gizmo.get_rotation_sphere_radius(cam);
				drag_rotation_gizmo_helper_pos = normalize(upd_pos - cur_pos) * r + cur_pos;
				// calculate angle between 2 vectors
				vec3 v0 = normalize(start_pos - drag_start_obj_pos);
				vec3 v1 = normalize(upd_pos - drag_start_obj_pos);
				float dp = clamp(dot(v0, v1), -1.0f, 1.0f);
				float angle = acosf(dp);
				
				vec3 perp = cross(v0, v1);
				vec3 rot_axis = plane.xyz();
				float k = dot(perp, rot_axis) < 0.0f ? -1.0f : 1.0f;

				quaternion add_rot = quaternion(rot_axis, angle * k);
				quaternion upd_rot = drag_start_obj_rot * add_rot;
				tc->SetRotation(upd_rot);
				g_gizmo.set_rotation(upd_rot);

				break;
			}
			case ReservedObjIds::kGizmoRotateXYZ:
			{
				float r = g_gizmo.get_rotation_sphere_radius(cam);
				vec4 sphere = vec4(drag_start_obj_pos, r);

				vec3 start_pos = ray_sphere_intersect(ray_dir_start, ray_origin, sphere);
				vec3 upd_pos = ray_sphere_intersect(ray_dir, ray_origin, sphere);
				if (upd_pos.x == FLT_MAX) { // no intersection
					break;
				}

				vec3 v0 = normalize(start_pos - drag_start_obj_pos);
				vec3 v1 = normalize(upd_pos - drag_start_obj_pos);
				gosASSERT(v0 != v1);
				vec3 axis = normalize(cross(v0, v1));
				float angle = acosf(clamp(dot(v0, v1), -1.0f ,1.0f));

				quaternion add_rot = quaternion(axis, angle);
				quaternion upd_rot = drag_start_obj_rot * add_rot;
				tc->SetRotation(upd_rot);
				g_gizmo.set_rotation(upd_rot);

				drag_rotation_gizmo_helper_pos = upd_pos;
				break;
			}
			case ReservedObjIds::kGizmoScaleX:
			case ReservedObjIds::kGizmoScaleY:
			case ReservedObjIds::kGizmoScaleZ:
			case ReservedObjIds::kGizmoScaleXZ:
			case ReservedObjIds::kGizmoScaleYX:
			case ReservedObjIds::kGizmoScaleYZ:
			case ReservedObjIds::kGizmoScaleXYZ:
			{
				const int axis_idx = drag_type - ReservedObjIds::kGizmoScaleX;
				const vec3 axes[7] = { vec3(1,0,0), vec3(0,1,0), vec3(0,0,1), vec3(1,0,1), vec3(1,1,0), vec3(0,1,1), vec3(1,1,1) };
				const vec2 screen_delta =
					proj2screen(cur_mouse_proj_pos, screen_width, screen_height) -
					proj2screen(drag_start_mouse_proj_pos, screen_width, screen_height);
				// TODO: use distance to the object as an additional multiplier for better UX?
				const float scale = screen_delta.x * 0.025f; 
				const vec3 scale_axis = scale * axes[axis_idx] + vec3(1,1,1);

				// for scaling in world space we would need to support skew, that means we will not have pure scale, rotate transform anymore
				// could be handled by adding 4th matrix Sw so the wole stack will be: T * Sw * R * Sl,
				// but I think it is a bit of an overkill, so no support for World Space scale at the moment
				if (g_gizmo.get_world_space()) {
					tc->SetWorldScale(drag_start_obj_world_scale * scale_axis);
				}
				else {
					tc->SetScale(drag_start_obj_scale * scale_axis);
				}
				break;
			}
		}

	}
	if (gos_GetKeyStatus(KEY_LMOUSE) == KEY_RELEASED) {
		drag_started = false;
		log_debug("drag stop\n");
	}

    for(auto i=0; i<registered_editors.size(); ++i) {
        if(registered_editors[i].wants_activate()) {
            g_active_user_editor = i;
            log_info("Switched to editor: %s\n", registered_editors[i].name());
        }
    }

    if(g_active_user_editor != -1) {
        g_sel_obj = registered_editors[g_active_user_editor].update(cam, dt, g_sel_obj);
    }
}


void editor_render_update(struct RenderFrameContext *rfc)
{
    if(g_sel_obj) {
        auto* tc = g_sel_obj->GetComponent<TransformComponent>();
        if(tc) {
			g_gizmo.draw(rfc);
			// TODO: move this variable to gizmo and set when approprate
			add_debug_sphere_constant_size(rfc, drag_rotation_gizmo_helper_pos, 250.0f,
										   vec4(1, 1, 1, 1));
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
			add_debug_mesh_constant_size(rfc, sphere, c, l.transform_, 1000.0f);
		}
    }

    if(g_active_user_editor != -1) {
        registered_editors[g_active_user_editor].render_update(rfc);
    }
}

