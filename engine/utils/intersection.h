#pragma once

#include "math_utils.h"
#include "vec.h"


struct AABB {
    vec3 min_;
    vec3 max_;

    AABB() = default;
    AABB(const vec3 &pmin, const vec3 &pmax) : min_(pmin), max_(pmax) {}
};

inline vec3 aabb_center(const AABB& aabb) {
    return 0.5f * (aabb.min_ + aabb.max_);
}

inline vec3 aabb_extents(const AABB& aabb) {
    return 0.5f * (aabb.max_ - aabb.min_);
}

template <typename T>
inline void swap_v(T& a, T& b) {
    T t = a;
    a = b;
    b = t;
}

// returns: x - tmin, y - tmax, z - is intersects
inline vec3 intersect_aabb_ray(const AABB& aabb, const vec3& orig, const vec3& dir) 
{ 
    float tmin = (aabb.min_.x - orig.x) / dir.x; 
    float tmax = (aabb.max_.x - orig.x) / dir.x; 
 
    if (tmin > tmax) swap_v(tmin, tmax); 
 
    float tymin = (aabb.min_.y - orig.y) / dir.y; 
    float tymax = (aabb.max_.y - orig.y) / dir.y; 
 
    if (tymin > tymax) swap_v(tymin, tymax); 
 
    if ((tmin > tymax) || (tymin > tmax)) 
        return vec3(0,0,0); 
 
    if (tymin > tmin) 
        tmin = tymin; 
 
    if (tymax < tmax) 
        tmax = tymax;
 
    float tzmin = (aabb.min_.z - orig.z) / dir.z; 
    float tzmax = (aabb.max_.z - orig.z) / dir.z; 
 
    if (tzmin > tzmax) swap_v(tzmin, tzmax); 
 
    if ((tmin > tzmax) || (tzmin > tmax)) 
        return vec3(0,0,0); 
 
    if (tzmin > tmin) 
        tmin = tzmin; 
 
    if (tzmax < tmax) 
        tmax = tzmax; 
 
    return vec3(tmin, tmax, 1.0); 
} 

