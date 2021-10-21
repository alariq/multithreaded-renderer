#ifndef SDF_H
#define SDF_H

#include "defines.h"
#include "vec.h"

FORCE_INLINE float distance_box(const vec3& box, const vec3 &p, const float invert, const float tolerance) {
	const vec3 d = vec3(abs(p) - box);
	const vec3 max_d = max(d, vec3(0.0f));
	return invert*(min(max(d.x, max(d.y, d.z)), 0.0f) + length(max_d)) - tolerance;
}

FORCE_INLINE float distance_sphere(const float radius, const vec3& p, const float invert, const float tolerance) {
	return invert*(length(p) - radius) - tolerance;
}


FORCE_INLINE float sd_circle( vec2 p, float r ) {
    return length(p) - r;
}

// b.x = width
// b.y = height
// r.x = roundness top-right  
// r.y = roundness boottom-right
// r.z = roundness top-left
// r.w = roundness bottom-left
FORCE_INLINE float sd_rounded_box(vec2 p, vec2 b, vec4 r) {
	r.x = (p.x > 0.0) ? r.x : r.z;
	r.y = (p.x > 0.0) ? r.y : r.w;
	r.x = (p.y > 0.0) ? r.x : r.y; 
    vec2 q = abs(p) - b + vec2(r.x, r.x);
	return min(max(q.x, q.y), 0.0f) + length(max(q, vec2(0.0f))) - r.x;
}

FORCE_INLINE vec3 sdg_circle(vec2 p, float r) {
	float d = length(p);
    return vec3( d-r, p.x/d, p.y/d);
}

FORCE_INLINE vec3 sdg_box2d(vec2 p, vec2 b) {
	vec2 w = abs(p) - b;
	vec2 s = vec2(p.x < 0.0f ? -1.0f : 1.0f, p.y < 0.0f ? -1.0f : 1);
	float g = max(w.x, w.y);
	vec2 q = max(w, vec2(0.0f));
	float l = length(q);
    vec2 grad = s * ((g > 0.0) ? q / l : ((w.x > w.y) ? vec2(1, 0) : vec2(0, 1)));
	return vec3((g > 0.0) ? l : g, grad.x, grad.y);
}

#endif // SDF_H
