#include "render_utils.h"
#include "renderer.h"
#include "res_man.h"

void add_debug_mesh(struct RenderFrameContext *rfc, const RenderMesh *mesh, const mat4 &mat,
						   const vec4 &color, uint32_t selection_id/* = 0*/) {
    RenderList *frame_render_list = rfc->rl_;
    frame_render_list->ReservePackets(1);
    RenderPacket *rp = frame_render_list->AddPacket();
    rp->mesh_ = *mesh;
	rp->m_ = mat;
	rp->id_ = selection_id;
	rp->debug_color = color;

    rp->is_debug_pass = 1;
	rp->is_selection_pass = selection_id ? 1 : 0;
	rp->is_gizmo_pass = selection_id ? 1 : 0;

    rp->is_opaque_pass = 0;
    rp->is_render_to_shadow = 0;
    rp->is_transparent_pass = 0;
}

void add_debug_sphere_constant_size(struct RenderFrameContext *rfc, const vec3& pos, float no_scale_distance, const vec4& color) {

	const float oo_no_scale_distance = 1.0f / no_scale_distance;
	const float cam_z = (rfc->view_ * vec4(pos, 1)).z;
	const mat4 tr = mat4::translation(pos) * mat4::scale(vec3(cam_z * oo_no_scale_distance));
	add_debug_mesh(rfc, res_man_load_mesh("sphere"), tr, color);
}

// no_scale_distance - distance at which desired object will be drawn with no scale applied
void add_debug_mesh_constant_size(struct RenderFrameContext *rfc, const RenderMesh *mesh,
										 const vec4 &color, const mat4 &tr_m,
                                         const float no_scale_distance/* = .1f*/,
										 uint32_t selection_id/* = 0*/) {

	const float oo_no_scale_distance = 1.0f / no_scale_distance;
	const float cam_z = (rfc->view_ * tr_m.getTranslationPoint()).z;
	const mat4 tr = tr_m * mat4::scale(vec3(cam_z * oo_no_scale_distance));
	add_debug_mesh(rfc, mesh, tr, color, selection_id);
}

// only useful for 2d screen aligned objects
// pixels - approx amount of pixels object should have on the screen
void add_debug_mesh_constant_size_px(struct RenderFrameContext *rfc, const RenderMesh *mesh,
										 const vec4 &color, const mat4 &tr_m,
                                         const uint32_t pixels,
										 uint32_t selection_id/* = 0*/) {

    // for real we must take AABB, project it on screen and calculate from this

	const float cam_z = (rfc->view_ * tr_m.getTranslationPoint()).z;
    const float oo_w = rfc->proj_.elem[0][0];
    const float oo_h = rfc->proj_.elem[1][1];
    float width_by_2 = 800.0f / 2.0f;
    float height_by_2 = 600.0f / 2.0f;
    float size_x = pixels * (cam_z * (1.0f/oo_w)) / width_by_2;
    float size_y = pixels * (cam_z * (1.0f/oo_h)) / height_by_2;

	const mat4 tr = tr_m * mat4::scale(vec3(size_x, size_y, 1.0f));
	add_debug_mesh(rfc, mesh, tr, color, selection_id);
}

