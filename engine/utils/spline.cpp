#include "spline.h"
#include "myarray.h"
#include "vec.h"
#include <cfloat>
#include <cmath>
//#include <vector>

// https://www.geometrictools.com/GTE/Mathematics/ParametricCurve.h

typedef float r32;

template<int order, typename F>
static r32 Romberg(r32 a, r32 b, F integrand)
{
    r32 const half = (r32)0.5;
    r32 rom[order][2];
    r32 h = b - a;
    rom[0][0] = half * h * (integrand(a) + integrand(b));

    for (int32_t i0 = 2, p0 = 1; i0 <= order; ++i0, p0 *= 2, h *= half)
    {
        // Approximations via the trapezoid rule.
        r32 sum = (r32)0;
        int32_t i1;
        for (i1 = 1; i1 <= p0; ++i1)
        {
            sum += integrand(a + h * (i1 - half));
        }

        // Richardson extrapolation.
        rom[0][1] = half * (rom[0][0] + h * sum);
        for (int32_t i2 = 1, i2m1 = 0, p2 = 4; i2 < i0; ++i2, ++i2m1, p2 *= 4)
        {
            rom[i2][1] = (p2 * rom[i2m1][1] - rom[i2m1][0]) / (static_cast<size_t>(p2) - 1);
        }

        for (i1 = 0; i1 < i0; ++i1)
        {
            rom[i1][0] = rom[i1][1];
        }
    }

    r32 result = rom[static_cast<size_t>(order) - 1][0];
    return result;
}

template<typename T>
int lower_bound(const T* data, int first, int last, const T& value)
{
    int it, step;
    int count = last - first;
 
    while (count > 0)
    {
        it = first;
        step = count / 2;
        it += step;
 
        if (data[it] < value)
        {
            first = ++it;
            count -= step + 1;
        }
        else
            count = step;
    }
 
    return first;
}

template<typename T>
r32 Curve<T>::GetLength(r32 t0, r32 t1) const
{
    auto speed = [this](r32 t)
    {
        return length(getDerivativeAt(t));
    };

    if(pts_.size() < 4) {
        return 0.0f;
    }

    constexpr int order = 4;

    if(segment_lengths_.size() == 0)
    {
        times_.resize(pts_.size() - 2);
        segment_lengths_.resize(pts_.size() - 3);
        accumulated_lengths_.resize(pts_.size() - 3);
        // Lazy initialization of lengths of segments.
        int32_t const numSegments = segment_lengths_.size();
        r32 accumulated = (r32)0;
        for (int32_t i = 0, ip1 = 1; i < numSegments; ++i, ++ip1)
        {
            // time is equal to segment index for now
            segment_lengths_[i] = Romberg<order>((float)i, (float)ip1, speed);
            accumulated += segment_lengths_[i];
            accumulated_lengths_[i] = accumulated;
            times_[i] = i;
        }
        times_[times_.size() - 1] = times_.size() - 1;
    }

    float tmin = times_[0];
    float tmax = times_.last();

    t0 = std::max(t0, tmin);
    t1 = std::min(t1, tmax);
    int i0 = lower_bound(times_.data(), 0, times_.size(), t0);
    float v0 = times_[i0];
    int i1 = lower_bound(times_.data(), 0, times_.size(), t1);
    float v1 = times_[i1];

    r32 length;
    if (i0 < i1)
    {
        length = 0;
        if (t0 < v0)
        {
            length += Romberg<order>(t0, times_[i0], speed);
        }

        int32_t isup;
        if (t1 < v1)
        {
            length += Romberg<order>(times_[i1 - 1], t1, speed);
            isup = i1 - 1;
        }
        else
        {
            isup = i1;
        }
        for (int32_t i = i0; i < isup; ++i)
        {
            length += segment_lengths_[i];
        }
    }
    else
    {
        length = Romberg<order>(t0, t1, speed);
    }
    return length;
}

template float Curve<float>::GetLength(float t0, float t1) const;
template float Curve<vec3>::GetLength(float t0, float t1) const;

SplineClosestPointResult spline_get_closest_point(
    const Curve<vec3>& curve,
    const vec3& position,
    int coarse_samples,
    int refine_iters)
{
    SplineClosestPointResult result{};
    result.t = 0.0f;
    result.point = vec3(0.0f);
    result.distanceSqr = FLT_MAX;
    result.distance = FLT_MAX;

    if (curve.getNumSegments() <= 0) {
        return result;
    }

    const float tmin = curve.getTMin();
    const float tmax = curve.getTMax();
    const int samples = coarse_samples < 1 ? 1 : coarse_samples;
    const float span = (tmax - tmin) / static_cast<float>(samples);

    float best_t = tmin;
    vec3 best_p = curve.getAt(tmin);
    float best_d2 = lengthSqr(position - best_p);

    for (int i = 1; i <= samples; ++i) {
        const float t = tmin + static_cast<float>(i) * span;
        const vec3 p = curve.getAt(t);
        const float d2 = lengthSqr(position - p);
        if (d2 < best_d2) {
            best_d2 = d2;
            best_t = t;
            best_p = p;
        }
    }

    float search_span = span;
    for (int it = 0; it < refine_iters && search_span > 0.0f; ++it) {
        float tt[5] = {
            clamp(best_t - search_span, tmin, tmax),
            clamp(best_t - 0.5f * search_span, tmin, tmax),
            clamp(best_t, tmin, tmax),
            clamp(best_t + 0.5f * search_span, tmin, tmax),
            clamp(best_t + search_span, tmin, tmax)
        };

        for (int i = 0; i < 5; ++i) {
            const float t = tt[i];
            const vec3 p = curve.getAt(t);
            const float d2 = lengthSqr(position - p);
            if (d2 < best_d2) {
                best_d2 = d2;
                best_t = t;
                best_p = p;
            }
        }
        search_span *= 0.5f;
    }

    result.t = best_t;
    result.point = best_p;
    result.distanceSqr = best_d2;
    result.distance = sqrtf(best_d2);
    return result;
}
