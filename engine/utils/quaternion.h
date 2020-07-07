#pragma once

#include "vec.h"
#include <cmath>
#include <cassert>

 // roll (X), yaw (Z), pitch (Y), 
struct quaternion euler_to_quat(float roll, float pitch, float yaw);

struct quaternion mat4_to_quat(const mat4 &mat);
mat4 quat_to_mat4(const struct quaternion& q);
mat3 quat_to_mat3(const struct quaternion& q);

struct quaternion {
	float x, y, z, w;

	quaternion() = default;
	quaternion(const quaternion& q) = default;
	quaternion(float xx, float yy, float zz, float ww) : x(xx), y(yy), z(zz), w(ww) {}
	quaternion(const vec3 &axis, float angle) {
		float sa = sin(0.5f * angle);
		x = axis.x * sa;
		y = axis.y * sa;
		z = axis.z * sa;
		w = cos(0.5f * angle);
	}

	mat3 to_mat3() const {
		return quat_to_mat3(*this);
	}

	static quaternion identity() {
		return quaternion(0, 0, 0, 1);
	}

};

inline quaternion operator*(const quaternion& a, const quaternion& b) {
	quaternion q;
	q.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
	q.x = a.x * b.w + a.w * b.x + a.y * b.z - a.z * b.y;
	q.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x;
	q.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w;
	return q;
}

inline quaternion normalize(const quaternion q) {
	const float norm = sqrtf(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w);
	assert(norm > 0.00001f);
	const float oo_norm = 1.0f / norm;
	return quaternion(q.x * oo_norm, q.y * oo_norm, q.z*oo_norm, q.w*oo_norm);
}

inline quaternion conjugate(const quaternion q) {
	return quaternion(-q.x, -q.y, -q.z, q.w);
}

inline quaternion inverse(const quaternion q) {
	return normalize(conjugate(q));

}
