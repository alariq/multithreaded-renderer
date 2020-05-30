#pragma once

#include "vec.h"

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



