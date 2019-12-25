#pragma  once

#include "vec.h"

class Frustum {
public:
    void updateFromCamera(const vec3& p, const vec3& dir, const vec3& right, const vec3& up, float fov, float aspect, float near, float far);
    static void makeMeshFromFrustum(const Frustum* f, char* out_vertices, int count, int stride);
    enum { kLEFT, kRIGHT, kTOP, kBOTTOM, kNEAR, kFAR, kNUM_FRUSTUM_PLANES };

    enum { 
        kNEAR_TOP_LEFT, kNEAR_TOP_RIGHT, kNEAR_BOTTOM_LEFT, kNEAR_BOTTOM_RIGHT,
        kFAR_TOP_LEFT, kFAR_TOP_RIGHT, kFAR_BOTTOM_LEFT, kFAR_BOTTOM_RIGHT,
        kNUM_FRUSTUM_POINTS
    };

    const vec3* getPoints() const { return &points[0]; }
private:


    vec3 points[kNUM_FRUSTUM_POINTS];
    vec4 planes[kNUM_FRUSTUM_PLANES];
};

inline vec4 planeFromNormalAndPoint(const vec3& n, const vec3& p)
{
    float dist = dot(n, p);
    return vec4(n, dist);
}

