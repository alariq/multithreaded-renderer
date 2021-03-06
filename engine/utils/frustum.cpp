#include "frustum.h"
#include <cassert>

void Frustum::updateFromCamera(const vec3& p, const vec3& dir, const vec3& right, const vec3& up, float fov, float aspect, float near, float far)
{
    float w = tanf(0.5f * fov);
    float h = w / aspect;

    vec3 ntr = p + near * dir + right * w * near + up * h * near;
    vec3 ntl = p + near * dir - right * w * near + up * h * near;
    vec3 nbr = p + near * dir + right * w * near - up * h * near;
    vec3 nbl = p + near * dir - right * w * near - up * h * near;

    vec3 ftr = p + far * dir + right * w * far + up * h * far;
    vec3 ftl = p + far * dir - right * w * far + up * h * far;
    vec3 fbr = p + far * dir + right * w * far - up * h * far;
    vec3 fbl = p + far * dir - right * w * far - up * h * far;

    points[kNEAR_TOP_RIGHT] = ntr;
    points[kNEAR_TOP_LEFT] = ntl;
    points[kNEAR_BOTTOM_RIGHT] = nbr;
    points[kNEAR_BOTTOM_LEFT] = nbl;

    points[kFAR_TOP_RIGHT] = ftr;
    points[kFAR_TOP_LEFT] = ftl;
    points[kFAR_BOTTOM_RIGHT] = fbr;
    points[kFAR_BOTTOM_LEFT] = fbl;

    // planes are constructed in a way, that their normals point into frustum

    planes[kNEAR] = planeFromNormalAndPoint(dir, p + near * dir);
    planes[kFAR] = planeFromNormalAndPoint(-dir, p + far * dir);

    {
        vec3 nr = p + near * dir + right * w; //near right point
        vec3 dr = normalize(nr - p); // dir from origin to nr
        vec3 norm = cross(up, dr);
        planes[kRIGHT] = planeFromNormalAndPoint(norm, p);
    }

    {
        vec3 nl = p + near * dir - right * w;
        vec3 dl = normalize(nl - p);
        vec3 norm = cross(dl, up);
        planes[kRIGHT] = planeFromNormalAndPoint(norm, p);
    }

    {
        vec3 nt = p + near * dir + up * w;
        vec3 dt = normalize(nt - p);
        vec3 norm = cross(right, dt);
        planes[kTOP] = planeFromNormalAndPoint(norm, p);
    }

    {
        vec3 nb = p + near * dir - up * w;
        vec3 db = normalize(nb - p);
        vec3 norm = cross(db, right);
        planes[kBOTTOM] = planeFromNormalAndPoint(norm, p);
    }
}

// counter clock-wise ordering
void Frustum::makeMeshFromFrustum(const Frustum* f, char* out_vertices, int count, int stride)
{
    assert(count >= kNUM_FRUSTUM_PLANES * 3 * 2);

    // right plane
    *((vec3*)out_vertices) = f->points[kNEAR_BOTTOM_RIGHT]; out_vertices += stride;
    *((vec3*)out_vertices) = f->points[kFAR_TOP_RIGHT];out_vertices += stride;
    *((vec3*)out_vertices) = f->points[kNEAR_TOP_RIGHT];out_vertices += stride;

    *((vec3*)out_vertices) = f->points[kNEAR_BOTTOM_RIGHT];out_vertices += stride;
    *((vec3*)out_vertices) = f->points[kFAR_BOTTOM_RIGHT];out_vertices += stride;
    *((vec3*)out_vertices) = f->points[kFAR_TOP_RIGHT];out_vertices += stride;

    // left plane
    *((vec3*)out_vertices) = f->points[kNEAR_BOTTOM_LEFT];out_vertices += stride;
    *((vec3*)out_vertices) = f->points[kNEAR_TOP_LEFT];out_vertices += stride;
    *((vec3*)out_vertices) = f->points[kFAR_TOP_LEFT];out_vertices += stride;
                           
    *((vec3*)out_vertices) = f->points[kNEAR_BOTTOM_LEFT];out_vertices += stride;
    *((vec3*)out_vertices) = f->points[kFAR_TOP_LEFT];out_vertices += stride;
    *((vec3*)out_vertices) = f->points[kFAR_BOTTOM_LEFT];out_vertices += stride;

    // top plane
    *((vec3*)out_vertices) = f->points[kFAR_TOP_LEFT];out_vertices += stride;
    *((vec3*)out_vertices) = f->points[kNEAR_TOP_LEFT];out_vertices += stride;
    *((vec3*)out_vertices) = f->points[kNEAR_TOP_RIGHT];out_vertices += stride;
                           
    *((vec3*)out_vertices) = f->points[kFAR_TOP_RIGHT];out_vertices += stride;
    *((vec3*)out_vertices) = f->points[kFAR_TOP_LEFT];out_vertices += stride;
    *((vec3*)out_vertices) = f->points[kNEAR_TOP_RIGHT];out_vertices += stride;

    // bottom plane
    *((vec3*)out_vertices) = f->points[kNEAR_BOTTOM_LEFT];out_vertices += stride;
    *((vec3*)out_vertices) = f->points[kFAR_BOTTOM_LEFT];out_vertices += stride;
    *((vec3*)out_vertices) = f->points[kNEAR_BOTTOM_RIGHT];out_vertices += stride;
                           
    *((vec3*)out_vertices) = f->points[kNEAR_BOTTOM_RIGHT];out_vertices += stride;
    *((vec3*)out_vertices) = f->points[kFAR_BOTTOM_LEFT];out_vertices += stride;
    *((vec3*)out_vertices) = f->points[kFAR_BOTTOM_RIGHT];out_vertices += stride;

    // near plane
    *((vec3*)out_vertices) = f->points[kNEAR_TOP_RIGHT];out_vertices += stride;
    *((vec3*)out_vertices) = f->points[kNEAR_TOP_LEFT];out_vertices += stride;
    *((vec3*)out_vertices) = f->points[kNEAR_BOTTOM_LEFT];out_vertices += stride;
                           
    *((vec3*)out_vertices) = f->points[kNEAR_BOTTOM_LEFT];out_vertices += stride;
    *((vec3*)out_vertices) = f->points[kNEAR_BOTTOM_RIGHT];out_vertices += stride;
    *((vec3*)out_vertices) = f->points[kNEAR_TOP_RIGHT];out_vertices += stride;

    // far plane
    *((vec3*)out_vertices) = f->points[kFAR_TOP_LEFT];out_vertices += stride;
    *((vec3*)out_vertices) = f->points[kFAR_TOP_RIGHT];out_vertices += stride;
    *((vec3*)out_vertices) = f->points[kFAR_BOTTOM_RIGHT];out_vertices += stride;
                           
    *((vec3*)out_vertices) = f->points[kFAR_BOTTOM_RIGHT];out_vertices += stride;
    *((vec3*)out_vertices) = f->points[kFAR_BOTTOM_LEFT];out_vertices += stride;
    *((vec3*)out_vertices) = f->points[kFAR_TOP_LEFT];out_vertices += stride;

}
