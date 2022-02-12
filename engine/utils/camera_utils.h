#pragma once

#include "utils/vec.h"
#include "utils/camera.h"
#include "utils/math_utils.h"

// converts screen position to world space at a given distance from camera
// x,y = [-1,1] screen space range
// z - distance from camera pos (in world space)
// returns: world position at a given distance from camera
inline vec3 get_cursor_pos(const mat4* inv_proj, const mat4* inv_view, const float x, const float y,
                    const float dist_from_cam) {
    vec4 view_pos = *inv_proj * vec4(x, y, 0.5f, 1.0f);
    vec3 view_pos_n = dist_from_cam * normalize(view_pos.xyz());
    vec3 wpos = (*inv_view * vec4(view_pos_n, 1.0f)).xyz();
    return wpos;
}

inline vec3 screen2world(const camera *cam, const vec2 screen_pos,
                    const float dist_from_cam) {
    return cam->unproject(screen_pos, dist_from_cam);
}

inline vec3 screen2world_vec(const camera *cam, const vec2& screen_pos) {
    return cam->unproject_vec(screen_pos);
}

inline vec3 screen2world_vec(bool b_is_perspective, const mat4& inv_view, const mat4& inv_proj, const vec2& screen_pos) {
    return camera::unproject_vec(screen_pos, b_is_perspective, inv_view, inv_proj);
}

inline vec2 proj2screen(vec2 proj, float w, float h) {
	return vec2((0.5f*proj.x + 0.5f)*w, (0.5f*(1.0f - proj.y) + 0.5f)*h);
}


// returns world position of screen position projected on plane (as if ray went from camera into screen and intersected plane in world space)
inline vec3 screen2world_projected_on_plane(vec2 pos, bool b_perspective, const mat4& inv_view, const mat4& inv_proj, vec4 plane) {
	if (b_perspective) {
		const vec3 dir = camera::unproject_vec(pos, b_perspective, inv_view, inv_proj);
		const vec3 ws_cam_pos = (inv_view * vec4(0, 0, 0, 1)).xyz();
		return ray_plane_intersect(dir, ws_cam_pos, plane);
	} else {
		vec3 wpos = camera::unproject(pos, 0.5f, b_perspective, inv_view, inv_proj);
        // view forward vec
        vec3 fwd_vec = inv_view.getCol2().xyz(); // same as view_.getForwardVec()
		return ray_plane_intersect(fwd_vec, wpos, plane);
	}
}

