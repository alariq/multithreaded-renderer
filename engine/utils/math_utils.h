#pragma once

#include "vec.h"
#include <cfloat>

// [s, e]
inline float random(float s, float e) {
    float v = ((float)rand()) / (float)RAND_MAX;
    return s + (e - s) * v;
}

// [s, e)
inline int random(int s, int e) {
    int v = rand() % e;
    return s + v;
}


inline vec3 random_vec(const vec3& s, const vec3& e) {
    return vec3(random(s.x, e.x), random(s.y, e.y), random(s.z, e.z));
}
inline vec4 random_vec(const vec4& s, const vec4& e) {
    return vec4(random(s.x, e.x), random(s.y, e.y), random(s.z, e.z), random(s.w, e.w));
}

inline vec3 random_vec(const float s, const float e) {
    return vec3(random(s, e), random(s, e), random(s, e));
}
inline vec4 random_vec4(const float s, const float e) {
    return vec4(random(s, e), random(s, e), random(s, e), random(s, e));
}

inline float lerp(float a, float b, float t) {
    return a + (b - a) * t;
}

inline vec3 lerp_vec(const vec3& a, const vec3& b, const float t) {
    return a + (b - a) * t;
}
inline vec4 lerp_vec(const vec4& a, const vec4& b, const float t) {
    return a + (b - a) * t;
}

inline vec3 lerp_vec(const vec3& a, const vec3& b, const vec3& t) {
    return a + (b - a) * t;
}
inline vec4 lerp_vec(const vec4& a, const vec4& b, const vec4& t) {
    return a + (b - a) * t;
}

inline vec3 ray_plane_intersect(const vec3& ray_dir, const vec3& ray_origin, const vec4& plane) {
	const vec3 n = plane.xyz(); // plane normal
	const vec3 p0 = n * plane.w; // point on plane
	// (r0 + rd*t - p0) ^ n = 0;
	// t = (p0 - r0)^n / rd^n;
	return ray_origin + ray_dir * dot((p0 - ray_origin), n) / dot(ray_dir, n);
}

inline vec3 ray_sphere_intersect(const vec3 ray_dir, const vec3 ray_origin, const vec4 sphere) {
	const float r = sphere.w;
	const vec3 p = ray_origin - sphere.xyz();
	const vec3 d = ray_dir;
	vec3 res;
	// (dx*t+x0)^2 +(dy*t+y0)^2 +(dz*t+z0)^2 = r^2
	float discriminant_sq = 4 * dot(d, p) * dot(d, p) - 4 * dot(d, d)*(dot(p, p) - r * r);
	if (discriminant_sq < 0.0f) {
		res = vec3(FLT_MAX, FLT_MAX, FLT_MAX);
	}
	else {
		float t = (-2 * dot(d, p) - sqrtf(discriminant_sq)) / (2 * dot(d,d));
		if(t < 0)
			t = (-2 * dot(d, p) + sqrtf(discriminant_sq)) / (2 * dot(d,d));
		if (t < 0)
			res = vec3(FLT_MAX, FLT_MAX, FLT_MAX);
		else
			res = ray_origin + d * t;
	}

	return res;
}

// takes plane, rotates it and puts at point "pt"
inline vec4 transform_plane_around_point(const mat4& tr, const vec4& plane, vec3 pt) {
	vec3 old_normal = plane.xyz();
	vec3 new_normal = (tr * vec4(old_normal, 0.0f)).xyz();
	float new_dist = dot(new_normal, pt);
	return vec4(new_normal, new_dist);
}

// Building an Orthonormal Basis, Revisited
// http://jcgt.org/published/0006/01/01/ 
inline void calculate_basis(const vec3 &n, vec3 &b1, vec3 &b2) {
	if (n.z < 0.0f) {
		const float a = 1.0f / (1.0f - n.z);
		const float b = n.x * n.y * a;
		b1 = vec3(1.0f - n.x * n.x * a, -b, n.x);
		b2 = vec3(b, n.y * n.y * a - 1.0f, -n.y);
	} else {
		const float a = 1.0f / (1.0f + n.z);
		const float b = -n.x * n.y * a;
		b1 = vec3(1.0f - n.x * n.x * a, b, -n.x);
		b2 = vec3(b, 1.0f - n.y * n.y * a, -n.y);
	}
}

// assume that axis passes through origin
inline vec3 project_on_vector(const vec3 ray_dir, const vec3 ray_origin, const vec3& axis) {
	// get some plane (which our axis lies at) to intersect with
	vec3 b1, b2;
	calculate_basis(axis, b1, b2);
	vec3 int_pt = ray_plane_intersect(ray_dir, ray_origin, vec4(b1, 0.0));
	float int_dot = dot(int_pt, axis);
	return int_dot * axis;
}

inline vec4 make_plane(const vec3& normal, const vec3& pt) {
	return vec4(normal, dot(normal, pt));
}

