#pragma once

#include <cstdint>

#include "engine/utils/spline.h"
#include "engine/utils/vec.h"

class Path {
    const Curve<vec3>* curve_;

    public:
        void SetCurve(const Curve<vec3>* c);
        int32_t GetNumNodes() const;
        vec3 Get(float t) const;
        vec3 GetDerivative(float t) const;

        const Curve<vec3>* GetCurve() const { return curve_; }
};

void CurveDebugDraw(const Curve<vec3>& curve, int num_pts_per_segment, bool b_draw_basis, const struct mat4* transform, class RenderList* rl);
