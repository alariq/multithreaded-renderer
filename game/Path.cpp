#include "game/Path.h"
#include "renderer.h"
#include <assert.h>
#include "engine/utils/vec.h"

void Path::SetCurve(const Curve<vec3>* c) {
    curve_ = c;
}

int32_t Path::GetNumNodes() const {
    return curve_->getNumSegments();
}

vec3 Path::Get(float t) const {
    return curve_->getAt(t);
}

vec3 Path::GetDerivative(float t) const {
    return curve_->getDerivativeAt(t);
}

void CurveDebugDraw(const Curve<vec3>& curve, int num_pts_per_segment, bool b_draw_basis, const mat4* transform, RenderList* rl) {
    const int nseg = curve.getNumSegments();
    vec3 p_prev = curve.getAt(0);
    const float num_pts = num_pts_per_segment;

    int num_lines = nseg * (int)num_pts;
    num_lines = 2 * num_lines; // also drawing derivative
                               //
    vec3* dbg_pts = new vec3[2*num_lines];
    vec4* colours = new vec4[num_lines];
    int li = 0;

    for(int s = 0; s<nseg; s++) {

        for (size_t i = 0; i < num_pts; ++i) {
            const float t = (float)i / num_pts;
            const float alpha = 1;
            const float brightness = 1;//0.35f + 0.65f * t;
            vec4 colour(0.8f * brightness, 0.8f * brightness, .1f * brightness, alpha);
            vec3 p = curve.getAt((float)s + t);
            const vec3 dp = curve.getDerivativeAt((float)s + t);
            const float vmag = length(dp);
            const vec3 vdir = dp/vmag;
            const vec4 vc = saturate(vec4(0.5f*vdir + vec3(0.5f), 1) * vec4(0.05f*vmag, 0.05f*vmag, 0.05f*vmag, 1.0f));

            p = transform ? ((*transform) * vec4(p, 1)).xyz() : p;

            //rl->addDebugLine(p_prev, p, colour);
            //rl->addDebugLine(p, p + 0.1f*dp, vc);
            dbg_pts[2*li + 0] = p_prev;
            dbg_pts[2*li + 1] = p;
            colours[li] = colour;
            li++;
            dbg_pts[2*li + 0] = p;
            dbg_pts[2*li + 1] = p + 0.1f*dp;
            colours[li] = vc;
            li++;


            p_prev = p;


            if(b_draw_basis) {
                vec3 right, up, fwd;
                spline_get_basis_at(curve, (float)s + t, 0.5f/num_pts, right, up, fwd);
                rl->addDebugLine(p, p + 1*right, vec4(1, 0,0, 1));
                rl->addDebugLine(p, p + 1*up, vec4(0, 1,0, 1));
                rl->addDebugLine(p, p + 1*fwd, vec4(0, 0,1, 1));
            }
        }
    }
    assert(li == num_lines);
    rl->addDebugLines(dbg_pts, colours, vec4(1), num_lines);
    delete[] dbg_pts;
    delete[] colours;
}


