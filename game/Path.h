#pragma once

#include <cstdint>

#include "engine/utils/spline.h"
#include "engine/utils/vec.h"

class Path {
    Curve<vec3> curve_;

    public:
        void SetCurve(const Curve<vec3>& c);
        int32_t GetNumNodes() const;
        vec3 Get(float t) const;
        vec3 GetDerivative(float t) const;
};
