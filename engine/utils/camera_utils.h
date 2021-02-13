#pragma once

#include "utils/vec.h"
#include "utils/camera.h"

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
    vec4 view_pos = cam->get_inv_projection() * vec4(screen_pos, 0.0f, 1.0f);
    vec3 view_pos_n = dist_from_cam * normalize(view_pos.xyz());
    vec3 wpos = (cam->get_inv_view() * vec4(view_pos_n, 1.0f)).xyz();
    return wpos;
}

inline vec3 screen2world_vec(const camera *cam, const vec2& screen_pos) {
    vec4 view_pos = cam->get_inv_projection() * vec4(screen_pos, 0.0f, 1.0f);
    vec3 n = normalize(view_pos.xyz());
    return (cam->get_inv_view() * vec4(n, 0.0f)).xyz();
}

inline vec2 proj2screen(vec2 proj, float w, float h) {
	return vec2((0.5f*proj.x + 0.5f)*w, (0.5f*(1.0f - proj.y) + 0.5f)*h);
}
