#include <cassert>
#include <cstdlib>
#include <cmath>
#include <cfloat>
#include <stdio.h>
#include <inttypes.h> // PRIu64

#include "engine/utils/spline.h"
#include "engine/utils/math_utils.h"
#include "engine/utils/vec.h"
#include "engine/utils/timing.h"

void test_spline_second_derivative()
{
    srand(1234);

    const int kCases = 200;
    const float tol = 2e-2f;

    for (int i = 0; i < kCases; ++i) {
        float alpha = random(0.0f, 1.0f);
        vec3 p0, p1, p2, p3;

        bool ok = false;
        for (int tries = 0; tries < 100 && !ok; ++tries) {
            p0 = random_vec(-5.0f, 5.0f);
            p1 = random_vec(-5.0f, 5.0f);
            p2 = random_vec(-5.0f, 5.0f);
            p3 = random_vec(-5.0f, 5.0f);

            float t0 = 0.0f;
            float t1 = CatmullRom::get_knot_interval<vec3>(t0, alpha, p0, p1);
            float t2 = CatmullRom::get_knot_interval<vec3>(t1, alpha, p1, p2);
            float t3 = CatmullRom::get_knot_interval<vec3>(t2, alpha, p2, p3);

            ok = (t1 > t0 + 1e-4f) && (t2 > t1 + 1e-4f) && (t3 > t2 + 1e-4f);
        }

        if (!ok) {
            assert(0 && "failed to generate good random values for p0/p1/p2/p3");
            alpha = 0.5f;
            p0 = vec3(-1, 0, 0);
            p1 = vec3(0, 1, 0);
            p2 = vec3(1, 0, 0);
            p3 = vec3(2, -1, 0);
        }

        float t = random(0.05f, 0.95f);

        vec3 dd_analytic = CatmullRom::getSecondDerivativeV<vec3>(p0, p1, p2, p3, t, alpha);
        vec3 dd_numeric = CatmullRom::getSecondDerivativeV_Numeric<vec3>(p0, p1, p2, p3, t, alpha);

        float err = length(dd_analytic - dd_numeric);
        float denom = max(1.0f, length(dd_numeric));
        assert(err <= tol * denom);
    }
}

#define ASSERT_NEAR(a, b, tol)                                                     \
    do {                                                                          \
        const float _a = (a);                                                     \
        const float _b = (b);                                                     \
        const float _tol = (tol);                                                 \
        if (std::fabs(_a - _b) > _tol) {                                          \
            fprintf(stderr, "ASSERT_NEAR failed: a=%f b=%f tol=%f\n", _a, _b, _tol); \
            assert(false);                                                       \
        }                                                                         \
    } while (0)

static float approx_length_samples(const Curve<vec3>& c, float t0, float t1, float dt)
{
    if (t1 <= t0) {
        return 0.0f;
    }
    float acc = 0.0f;
    for (float t = t0; t < t1; t += dt) {
        float tnext = t + dt;
        if (tnext > t1) {
            tnext = t1;
        }
        vec3 p0 = c.getAt(t);
        vec3 p1 = c.getAt(tnext);
        acc += length(p1 - p0);
    }
    return acc;
}

void test_spline_get_length()
{
    // Not enough points -> zero length.
    {
        Curve<vec3> c;
        c.addPoint(vec3(0.0f, 0.0f, 0.0f));
        c.addPoint(vec3(1.0f, 0.0f, 0.0f));
        c.addPoint(vec3(2.0f, 0.0f, 0.0f));
        assert(c.GetLength(0.0f, 1.0f) == 0.0f);
    }

    Curve<vec3> c;
    for (int i = 0; i < 6; ++i) {
        // Collinear points: curve is a straight line in 3D.
        c.addPoint(vec3((float)i, 0.0f, 0.0f));
    }

    const float t0 = 0.25f;
    const float t1 = 2.75f;

    const vec3 v0 = c.getAt(t0);
    const vec3 v1 = c.getAt(t1);
    const float expected = length(v1 - v0);
    const float len = c.GetLength(t0, t1);

    const float tol = 1e-3f * (1.0f + std::fabs(expected));
    ASSERT_NEAR(len, expected, tol);

    // Clamping to the [0, numSegments] range.
    const float tmax = (float)c.getNumSegments();
    const float expected_clamped = length(c.getAt(tmax) - c.getAt(0.0f));
    const float len_clamped = c.GetLength(-10.0f, 10.0f);
    const float tol_clamped = 1e-3f * (1.0f + std::fabs(expected_clamped));
    ASSERT_NEAR(len_clamped, expected_clamped, tol_clamped);

    // Zero interval.
    assert(c.GetLength(1.5f, 1.5f) == 0.0f);

    // Non-linear curve: compare to dense sampling of getAt().
    {
        Curve<vec3> s;
        s.addPoint(vec3(0.0f, 0.0f, 0.0f));
        s.addPoint(vec3(1.0f, 2.0f, 0.5f));
        s.addPoint(vec3(2.0f, 0.0f, 1.5f));
        s.addPoint(vec3(3.0f, -1.0f, 0.0f));
        s.addPoint(vec3(4.0f, 1.0f, -1.0f));
        s.addPoint(vec3(5.0f, 0.0f, 0.5f));

        const float st0 = 0.15f;
        const float st1 = 2.65f;
        const float dt = 1e-3f;

        const float sampled = approx_length_samples(s, st0, st1, dt);
        const float exact = s.GetLength(st0, st1);

        const float denom = (sampled > 1.0f) ? sampled : 1.0f;
        const float tol2 = 5e-3f * denom;
        ASSERT_NEAR(exact, sampled, tol2);
    }

    int count = 10;
    while(count--)
    {
        Curve<vec3> s;
        s.addPoint(random_vec(-5, 5));
        s.addPoint(random_vec(-5, 5));
        s.addPoint(random_vec(-5, 5));
        s.addPoint(random_vec(-5, 5));
        s.addPoint(random_vec(-5, 5));
        s.addPoint(random_vec(-5, 5));
        s.addPoint(random_vec(-5, 5));
        s.addPoint(random_vec(-5, 5));

        float max_t = s.getNumSegments();
        const float st0 = random(0.0f, max_t/2);
        const float st1 = random(max_t/2, max_t);
        const float dt = 1e-4f;

        const float sampled = approx_length_samples(s, st0, st1, dt);
        const float exact = s.GetLength(st0, st1);

        const float denom = (sampled > 1.0f) ? sampled : 1.0f;
        const float tol2 = 5e-3f * denom;
        ASSERT_NEAR(exact, sampled, tol2);
    }
}

void test_spline_reparametrization() {

    int count = 10;
    while(count--)
    {
        Curve<vec3> s;
        s.addPoint(random_vec(-5, 5));
        s.addPoint(random_vec(-5, 5));
        s.addPoint(random_vec(-5, 5));
        s.addPoint(random_vec(-5, 5));
        s.addPoint(random_vec(-5, 5));
        s.addPoint(random_vec(-5, 5));
        s.addPoint(random_vec(-5, 5));
        s.addPoint(random_vec(-5, 5));

        ReparameterizeByArclength<vec3> repar(s);
        float len = random(0.0f, s.getTotalLength());
        uint64_t ts = timing::gettickcount();
        ReparameterizeByArclength<vec3>::Output o = repar.GetT(len, true);
        uint64_t ts2 = timing::gettickcount();
        float arclen = s.GetLength(s.getTMin(), o.t);
        uint64_t dt = timing::ticks2ns(timing::gettickcount() - ts2);
        uint64_t dt2 = timing::ticks2ns(ts2 - ts);

        printf("dt: %" PRIu64 "usec dt2:%" PRIu64 "usec\n", dt, dt2);

        const float denom = (len > 1.0f) ? len : 1.0f;
        const float tol2 = 5e-3f * denom;
        ASSERT_NEAR(len, arclen, tol2);
    }
}

void test_spline_closest_point() {
    Curve<vec3> c;
    c.addPoint(vec3(-1.0f, 0.0f, 0.0f));
    c.addPoint(vec3(0.0f, 0.0f, 0.0f));
    c.addPoint(vec3(1.0f, 0.0f, 0.0f));
    c.addPoint(vec3(2.0f, 0.0f, 0.0f));
    c.addPoint(vec3(3.0f, 0.0f, 0.0f));
    c.addPoint(vec3(4.0f, 0.0f, 0.0f));

    const vec3 pos(1.3f, 2.0f, 0.0f);
    const SplineClosestPointResult r = spline_get_closest_point(c, pos, 32, 8);

    ASSERT_NEAR(r.t, 1.3f, 1e-2f);
    ASSERT_NEAR(r.point.x, 1.3f, 1e-2f);
    ASSERT_NEAR(r.point.y, 0.0f, 1e-3f);
    ASSERT_NEAR(r.point.z, 0.0f, 1e-3f);
    ASSERT_NEAR(r.distance, 2.0f, 1e-2f);
    ASSERT_NEAR(r.distanceSqr, 4.0f, 5e-2f);
}

void test_spline_closest_point_endpoints() {
    Curve<vec3> c;
    c.addPoint(vec3(-1.0f, 0.0f, 0.0f));
    c.addPoint(vec3(0.0f, 0.0f, 0.0f));
    c.addPoint(vec3(1.0f, 0.0f, 0.0f));
    c.addPoint(vec3(2.0f, 0.0f, 0.0f));
    c.addPoint(vec3(3.0f, 0.0f, 0.0f));
    c.addPoint(vec3(4.0f, 0.0f, 0.0f));

    {
        const vec3 pos(-2.0f, 0.5f, 0.0f);
        const SplineClosestPointResult r = spline_get_closest_point(c, pos, 32, 8);
        ASSERT_NEAR(r.t, c.getTMin(), 1e-3f);
        ASSERT_NEAR(r.point.x, c.getAt(c.getTMin()).x, 1e-3f);
        ASSERT_NEAR(r.point.y, 0.0f, 1e-3f);
    }

    {
        const vec3 pos(5.0f, -0.25f, 0.0f);
        const SplineClosestPointResult r = spline_get_closest_point(c, pos, 32, 8);
        ASSERT_NEAR(r.t, c.getTMax(), 1e-3f);
        ASSERT_NEAR(r.point.x, c.getAt(c.getTMax()).x, 1e-3f);
        ASSERT_NEAR(r.point.y, 0.0f, 1e-3f);
    }
}

void test_spline_closest_point_randomized() {
    const int kCases = 100;
    for (int i = 0; i < kCases; ++i) {
        Curve<vec3> c;
        for (int p = 0; p < 8; ++p) {
            c.addPoint(random_vec(-5.0f, 5.0f));
        }

        vec3 pos = random_vec(-5.0f, 5.0f);
        SplineClosestPointResult r = spline_get_closest_point(c, pos, 32, 8);

        const int brute_samples = 2000;
        const float tmin = c.getTMin();
        const float tmax = c.getTMax();
        const float dt = (tmax - tmin) / (float)brute_samples;
        float best_d2 = FLT_MAX;
        for (int s = 0; s <= brute_samples; ++s) {
            float t = tmin + (float)s * dt;
            vec3 p = c.getAt(t);
            float d2 = lengthSqr(pos - p);
            if (d2 < best_d2) {
                best_d2 = d2;
            }
        }

        const float tol = 2.5e-2f * max(1.0f, best_d2);
        assert(r.distanceSqr <= best_d2 + tol);
    }
}

void test_spline() {
    test_spline_second_derivative();
    test_spline_get_length();
    test_spline_reparametrization();
    test_spline_closest_point();
    test_spline_closest_point_endpoints();
    test_spline_closest_point_randomized();
}
