#include "vec.h"

class Frustum {
public:
    void updateFromCamera(const mat4& view, float fov, float aspect, float near, float far);
    static void makeMeshFromFrustum(const Frustum* f, char* out_vertices, int count, int stride);
    enum { LEFT, RIGHT, TOP, BOTTOM, NEAR, FAR, NUM_FRUSTUM_PLANES };
private:

    enum { 
        NEAR_TOP_LEFT, NEAR_TOP_RIGHT, NEAR_BOTTOM_LEFT, NEAR_BOTTOM_RIGHT,
        FAR_TOP_LEFT, FAR_TOP_RIGHT, FAR_BOTTOM_LEFT, FAR_BOTTOM_RIGHT,
        NUM_FRUSTUM_POINTS
    };

    vec3 points[NUM_FRUSTUM_POINTS];
    vec4 planes[NUM_FRUSTUM_PLANES];
};

inline vec4 planeFromNormalAndPoint(const vec3& n, const vec3& p)
{
    float dist = dot(n, p);
    return vec4(n, dist);
}

