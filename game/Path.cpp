#include "game/Path.h"
#include <assert.h>

void Path::SetCurve(const Curve<vec3>& c) {
    curve_ = c;
}

int32_t Path::GetNumNodes() const {
    return curve_.getNSegments();
}

vec3 Path::Get(float t) const {
    return curve_.getAt(t);
}

vec3 Path::GetDerivative(float t) const {
    return curve_.getDerivativeAt(t);
}
