#include "frustum.h"
#include <cassert>

void Frustum::updateFromCamera(const mat4& view, float fov, float aspect, float near, float far)
{
    vec3 dir = view.getRow(2).xyz();
    vec3 right = view.getRow(0).xyz();
    vec3 up = view.getRow(1).xyz();
    vec3 p = -view.getCol3().xyz();

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

    points[NEAR_TOP_RIGHT] = ntr;
    points[NEAR_TOP_LEFT] = ntl;
    points[NEAR_BOTTOM_RIGHT] = nbr;
    points[NEAR_BOTTOM_LEFT] = nbl;

    points[FAR_TOP_RIGHT] = ftr;
    points[FAR_TOP_LEFT] = ftl;
    points[FAR_BOTTOM_RIGHT] = fbr;
    points[FAR_BOTTOM_LEFT] = fbl;

    // planes are constructed in a way, that their normals point into frustum

    planes[NEAR] = planeFromNormalAndPoint(dir, p + near * dir);
    planes[FAR] = planeFromNormalAndPoint(-dir, p + far * dir);

    {
        vec3 nr = p + near * dir + right * w; //near right point
        vec3 dr = normalize(nr - p); // dir from origin to nr
        vec3 norm = cross(up, dr);
        planes[RIGHT] = planeFromNormalAndPoint(norm, p);
    }

    {
        vec3 nl = p + near * dir - right * w;
        vec3 dl = normalize(nl - p);
        vec3 norm = cross(dl, up);
        planes[RIGHT] = planeFromNormalAndPoint(norm, p);
    }

    {
        vec3 nt = p + near * dir + up * w;
        vec3 dt = normalize(nt - p);
        vec3 norm = cross(right, dt);
        planes[TOP] = planeFromNormalAndPoint(norm, p);
    }

    {
        vec3 nb = p + near * dir - up * w;
        vec3 db = normalize(nb - p);
        vec3 norm = cross(db, right);
        planes[BOTTOM] = planeFromNormalAndPoint(norm, p);
    }
}

// counter clock-wise ordering
void Frustum::makeMeshFromFrustum(const Frustum* f, char* out_vertices, int count, int stride)
{
    assert(count >= NUM_FRUSTUM_PLANES * 3 * 2);

    // right plane
    *((vec3*)out_vertices) = f->points[NEAR_BOTTOM_RIGHT]; out_vertices += stride;
    *((vec3*)out_vertices) = f->points[FAR_TOP_RIGHT];out_vertices += stride;
    *((vec3*)out_vertices) = f->points[NEAR_TOP_RIGHT];out_vertices += stride;

    *((vec3*)out_vertices) = f->points[NEAR_BOTTOM_RIGHT];out_vertices += stride;
    *((vec3*)out_vertices) = f->points[FAR_BOTTOM_RIGHT];out_vertices += stride;
    *((vec3*)out_vertices) = f->points[FAR_TOP_RIGHT];out_vertices += stride;

    // left plane
    *((vec3*)out_vertices) = f->points[NEAR_BOTTOM_LEFT];out_vertices += stride;
    *((vec3*)out_vertices) = f->points[NEAR_TOP_LEFT];out_vertices += stride;
    *((vec3*)out_vertices) = f->points[FAR_TOP_LEFT];out_vertices += stride;
                           
    *((vec3*)out_vertices) = f->points[NEAR_BOTTOM_LEFT];out_vertices += stride;
    *((vec3*)out_vertices) = f->points[FAR_TOP_LEFT];out_vertices += stride;
    *((vec3*)out_vertices) = f->points[FAR_BOTTOM_LEFT];out_vertices += stride;

    // top plane
    *((vec3*)out_vertices) = f->points[FAR_TOP_LEFT];out_vertices += stride;
    *((vec3*)out_vertices) = f->points[NEAR_TOP_LEFT];out_vertices += stride;
    *((vec3*)out_vertices) = f->points[NEAR_TOP_RIGHT];out_vertices += stride;
                           
    *((vec3*)out_vertices) = f->points[FAR_TOP_RIGHT];out_vertices += stride;
    *((vec3*)out_vertices) = f->points[FAR_TOP_LEFT];out_vertices += stride;
    *((vec3*)out_vertices) = f->points[NEAR_TOP_RIGHT];out_vertices += stride;

    // bottom plane
    *((vec3*)out_vertices) = f->points[NEAR_BOTTOM_LEFT];out_vertices += stride;
    *((vec3*)out_vertices) = f->points[FAR_BOTTOM_LEFT];out_vertices += stride;
    *((vec3*)out_vertices) = f->points[NEAR_BOTTOM_RIGHT];out_vertices += stride;
                           
    *((vec3*)out_vertices) = f->points[NEAR_BOTTOM_RIGHT];out_vertices += stride;
    *((vec3*)out_vertices) = f->points[FAR_BOTTOM_LEFT];out_vertices += stride;
    *((vec3*)out_vertices) = f->points[FAR_BOTTOM_RIGHT];out_vertices += stride;

    // near plane
    *((vec3*)out_vertices) = f->points[NEAR_TOP_RIGHT];out_vertices += stride;
    *((vec3*)out_vertices) = f->points[NEAR_TOP_LEFT];out_vertices += stride;
    *((vec3*)out_vertices) = f->points[NEAR_BOTTOM_LEFT];out_vertices += stride;
                           
    *((vec3*)out_vertices) = f->points[NEAR_BOTTOM_LEFT];out_vertices += stride;
    *((vec3*)out_vertices) = f->points[NEAR_BOTTOM_RIGHT];out_vertices += stride;
    *((vec3*)out_vertices) = f->points[NEAR_TOP_RIGHT];out_vertices += stride;

    // far plane
    *((vec3*)out_vertices) = f->points[FAR_TOP_LEFT];out_vertices += stride;
    *((vec3*)out_vertices) = f->points[FAR_TOP_RIGHT];out_vertices += stride;
    *((vec3*)out_vertices) = f->points[FAR_BOTTOM_RIGHT];out_vertices += stride;
                           
    *((vec3*)out_vertices) = f->points[FAR_BOTTOM_RIGHT];out_vertices += stride;
    *((vec3*)out_vertices) = f->points[FAR_BOTTOM_LEFT];out_vertices += stride;
    *((vec3*)out_vertices) = f->points[FAR_TOP_LEFT];out_vertices += stride;

}
