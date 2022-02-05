#include "quaternion.h"
#include "vec.h"

// http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
// http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToEuler/

 // roll (X), yaw (Z), pitch (Y), 
quaternion euler_to_quat(float roll, float pitch, float yaw)
{
    // Abbreviations for the various angular functions
    float cy = cosf(yaw * 0.5f);
    float sy = sinf(yaw * 0.5f);
    float cp = cosf(pitch * 0.5f);
    float sp = sinf(pitch * 0.5f);
    float cr = cosf(roll * 0.5f);
    float sr = sinf(roll * 0.5f);

    quaternion q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;

    return q;
}

// x - roll, y - pitch, z - roll
vec3 quat_to_euler(const quaternion& q) {

    // roll (x-axis rotation)
    float sinr_cosp = 2.0f * (q.w * q.x + q.y * q.z);
    float cosr_cosp = 1.0f - 2.0f * (q.x * q.x + q.y * q.y);
    float roll = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    float sinp = 2 * (q.w * q.y - q.z * q.x);
	float pitch;
    if (std::abs(sinp) >= 1)
        pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        pitch = std::asin(sinp);

    // yaw (z-axis rotation)
    float siny_cosp = 2.0f * (q.w * q.z + q.x * q.y);
    float cosy_cosp = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
    float yaw = std::atan2(siny_cosp, cosy_cosp);

    return vec3(roll, pitch, yaw);
}

mat3 quat_to_mat3(const quaternion& q) {
	mat3 m;

	float x2 = q.x + q.x;
	float y2 = q.y + q.y;
	float z2 = q.z + q.z;
	{
		float xx2 = q.x * x2;
		float yy2 = q.y * y2;
		float zz2 = q.z * z2;
		m.elem[0][0] = 1.0f - yy2 - zz2;
		m.elem[1][1] = 1.0f - xx2 - zz2;
		m.elem[2][2] = 1.0f - xx2 - yy2;
	}
	{
		float yz2 = q.y * z2;
		float wx2 = q.w * x2;
		m.elem[1][2] = yz2 - wx2;
		m.elem[2][1] = yz2 + wx2;
	}
	{
		float xy2 = q.x * y2;
		float wz2 = q.w * z2;
		m.elem[0][1] = xy2 - wz2;
		m.elem[1][0] = xy2 + wz2;
	}
	{
		float xz2 = q.x * z2;
		float wy2 = q.w * y2;
		m.elem[2][0] = xz2 - wy2;
		m.elem[0][2] = xz2 + wy2;
	}

	return m;
}

mat4 quat_to_mat4(const quaternion& q) {
	mat4 m;

	m.elem[3][0] = 0;
	m.elem[3][1] = 0;
	m.elem[3][2] = 0;
	m.elem[3][3] = 1;

	float x2 = q.x + q.x;
	float y2 = q.y + q.y;
	float z2 = q.z + q.z;
	{
		float xx2 = q.x * x2;
		float yy2 = q.y * y2;
		float zz2 = q.z * z2;
		m.elem[0][0] = 1.0f - yy2 - zz2;
		m.elem[1][1] = 1.0f - xx2 - zz2;
		m.elem[2][2] = 1.0f - xx2 - yy2;
	}
	{
		float yz2 = q.y * z2;
		float wx2 = q.w * x2;
		m.elem[1][2] = yz2 - wx2;
		m.elem[2][1] = yz2 + wx2;
	}
	{
		float xy2 = q.x * y2;
		float wz2 = q.w * z2;
		m.elem[0][1] = xy2 - wz2;
		m.elem[1][0] = xy2 + wz2;
	}
	{
		float xz2 = q.x * z2;
		float wy2 = q.w * y2;
		m.elem[2][0] = xz2 - wy2;
		m.elem[0][2] = xz2 + wy2;
	}

	m.elem[0][3] = 0;
	m.elem[1][3] = 0;
	m.elem[2][3] = 0;

	return m;
}

static float rsqrt(float v) { return 1.0f / sqrtf(v); }

quaternion mat4_to_quat(const mat4 &m) {
	quaternion q;

    const float m00 = m[0][0];
    const float m11 = m[1][1];
    const float m22 = m[2][2];
    
	if (m00 + m11 + m22 > 0.0f) {
		float t = m00 + m11 + m22 + 1.0f;
		float s = rsqrt(t) * 0.5f;
		q.w = s * t;
		q.z = (m[1][0] - m[0][1]) * s;
		q.y = (m[0][2] - m[2][0]) * s;
		q.x = (m[2][1] - m[1][2]) * s;
	} else if (m00 > m11 && m00 > m22) {
		float t = +m00 - m11 - m22 + 1.0f;
		float s = rsqrt(t) * 0.5f;
		q.x = s * t;
		q.y = (m[0][1] + m[1][0]) * s;
		q.z = (m[0][2] + m[2][0]) * s;
		q.w = (m[2][1] - m[1][2]) * s;
	} else if (m11 > m22) {
		float t = -m00 + m11 - m22 + 1.0f;
		float s = rsqrt(t) * 0.5f;
		q.y = s * t;
		q.x = (m[1][0] + m[0][1]) * s;
		q.w = (m[0][2] - m[2][0]) * s;
		q.z = (m[2][1] + m[1][2]) * s;
	} else {
		float t = -m00 - m11 + m22 + 1.0f;
		float s = rsqrt(t) * 0.5f;
		q.z = s * t;
		q.w = (m[1][0] - m[0][1]) * s;
		q.x = (m[0][2] + m[2][0]) * s;
		q.y = (m[1][2] + m[2][1]) * s;
	}
	return q;
}


quaternion mat3_to_quat(const mat3 &m) {
	quaternion q;

    const float m00 = m[0][0];
    const float m11 = m[1][1];
    const float m22 = m[2][2];
    
	if (m00 + m11 + m22 > 0.0f) {
		float t = m00 + m11 + m22 + 1.0f;
		float s = rsqrt(t) * 0.5f;
		q.w = s * t;
		q.z = (m[1][0] - m[0][1]) * s;
		q.y = (m[0][2] - m[2][0]) * s;
		q.x = (m[2][1] - m[1][2]) * s;
	} else if (m00 > m11 && m00 > m22) {
		float t = +m00 - m11 - m22 + 1.0f;
		float s = rsqrt(t) * 0.5f;
		q.x = s * t;
		q.y = (m[0][1] + m[1][0]) * s;
		q.z = (m[0][2] + m[2][0]) * s;
		q.w = (m[2][1] - m[1][2]) * s;
	} else if (m11 > m22) {
		float t = -m00 + m11 - m22 + 1.0f;
		float s = rsqrt(t) * 0.5f;
		q.y = s * t;
		q.x = (m[1][0] + m[0][1]) * s;
		q.w = (m[0][2] - m[2][0]) * s;
		q.z = (m[2][1] + m[1][2]) * s;
	} else {
		float t = -m00 - m11 + m22 + 1.0f;
		float s = rsqrt(t) * 0.5f;
		q.z = s * t;
		q.w = (m[1][0] - m[0][1]) * s;
		q.x = (m[0][2] + m[2][0]) * s;
		q.y = (m[1][2] + m[2][1]) * s;
	}

	return q;
}
