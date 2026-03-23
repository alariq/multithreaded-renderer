#pragma once

#include "math.h"
#include "math_utils.h"
#include "vec.h"
#include "myarray.h"

// https://en.wikipedia.org/wiki/Centripetal_Catmull%E2%80%93Rom_spline
struct CatmullRom {

    template <typename T>
    static float get_knot_interval(float t, float alpha, const T& p0, const T& p1 )
    {
        T d  = p1 - p0;
        float a = dot(d, d);
        float b = powf(a, alpha * 0.5f);
        return b + t;
    }

    // alpha 0.5 for centripetal
    template <typename T>
    static T getV(const T& p0, const T& p1, const T& p2, const T& p3, float t /* between 0 and 1 */, float alpha=.5f /* between 0 and 1 */ )
    {
        float t0 = 0.0f;
        float t1 = get_knot_interval( t0, alpha, p0, p1 );
        float t2 = get_knot_interval( t1, alpha, p1, p2 );
        float t3 = get_knot_interval( t2, alpha, p2, p3 );
        t = lerp( t1, t2, t );
        T A1 = ( t1-t )/( t1-t0 )*p0 + ( t-t0 )/( t1-t0 )*p1;
        T A2 = ( t2-t )/( t2-t1 )*p1 + ( t-t1 )/( t2-t1 )*p2;
        T A3 = ( t3-t )/( t3-t2 )*p2 + ( t-t2 )/( t3-t2 )*p3;
        T B1 = ( t2-t )/( t2-t0 )*A1 + ( t-t0 )/( t2-t0 )*A2;
        T B2 = ( t3-t )/( t3-t1 )*A2 + ( t-t1 )/( t3-t1 )*A3;
        T C  = ( t2-t )/( t2-t1 )*B1 + ( t-t1 )/( t2-t1 )*B2;
        return C;
    }

    template <typename T>
    static T getdV(const T& p0, const T& p1, const T& p2, const T& p3, float t /* between 0 and 1 */, float alpha=.5f /* between 0 and 1 */ )
    {
        return getDerivativeV<T>(p0, p1, p2, p3, t, alpha);
    }

    template <typename T>
    static T getddV(const T& p0, const T& p1, const T& p2, const T& p3, float t /* between 0 and 1 */, float alpha=.5f /* between 0 and 1 */ )
    {
        return getSecondDerivativeV<T>(p0, p1, p2, p3, t, alpha);
    }

    // Derivative of getV with respect to the normalized segment parameter t in [0, 1].
    template <typename T>
    static T getDerivativeV(const T& p0, const T& p1, const T& p2, const T& p3, float t /* between 0 and 1 */, float alpha=.5f /* between 0 and 1 */ )
    {
        float t0 = 0.0f;
        float t1 = get_knot_interval(t0, alpha, p0, p1 );
        float t2 = get_knot_interval( t1, alpha, p1, p2 );
        float t3 = get_knot_interval( t2, alpha, p2, p3 );
        float x = lerp( t1, t2, t );

        T A1 = ( t1-x )/( t1-t0 )*p0 + ( x-t0 )/( t1-t0 )*p1;
        T A2 = ( t2-x )/( t2-t1 )*p1 + ( x-t1 )/( t2-t1 )*p2;
        T A3 = ( t3-x )/( t3-t2 )*p2 + ( x-t2 )/( t3-t2 )*p3;
        T B1 = ( t2-x )/( t2-t0 )*A1 + ( x-t0 )/( t2-t0 )*A2;
        T B2 = ( t3-x )/( t3-t1 )*A2 + ( x-t1 )/( t3-t1 )*A3;

        T dA1dx = (p1 - p0) / (t1 - t0);
        T dA2dx = (p2 - p1) / (t2 - t1);
        T dA3dx = (p3 - p2) / (t3 - t2);

        // chain rule: y = f(x)*g(x) => dy = f(x)*g'(x) + f'(x)*g(x)
        T dB1dx = ( t2-x )/( t2-t0 )*dA1dx + ( x-t0 )/( t2-t0 )*dA2dx + (A2 - A1)/(t2 - t0);
        T dB2dx = ( t3-x )/( t3-t1 )*dA2dx + ( x-t1 )/( t3-t1 )*dA3dx + (A3 - A2)/(t3 - t1);

        // chain rule again
        T dCdx = ( t2-x )/( t2-t1 )*dB1dx + ( x-t1 )/( t2-t1 )*dB2dx + (B2 - B1)/(t2 - t1);

        // due to variable substitution: x = lerp(t1, t2, t) = t1*t + t2*(1-t), so dx/dt = (t2 - t1). 
        // and because: dC/dt = (dC/dx) * (dx/dt) we multiply by dx/dt
        return (t2 - t1) * dCdx;
    }

    template <typename T>
    static T getSecondDerivativeV(const T& p0, const T& p1, const T& p2, const T& p3, float t /* between 0 and 1 */, float alpha=.5f /* between 0 and 1 */ )
    {
        float t0 = 0.0f;
        float t1 = get_knot_interval( t0, alpha, p0, p1 );
        float t2 = get_knot_interval( t1, alpha, p1, p2 );
        float t3 = get_knot_interval( t2, alpha, p2, p3 );
        float x = lerp( t1, t2, t );

        T A1 = ( t1-x )/( t1-t0 )*p0 + ( x-t0 )/( t1-t0 )*p1;
        T A2 = ( t2-x )/( t2-t1 )*p1 + ( x-t1 )/( t2-t1 )*p2;
        T A3 = ( t3-x )/( t3-t2 )*p2 + ( x-t2 )/( t3-t2 )*p3;
        //T B1 = ( t2-x )/( t2-t0 )*A1 + ( x-t0 )/( t2-t0 )*A2;
        //T B2 = ( t3-x )/( t3-t1 )*A2 + ( x-t1 )/( t3-t1 )*A3;

        T dA1dx = (p1 - p0) / (t1 - t0);
        T dA2dx = (p2 - p1) / (t2 - t1);
        T dA3dx = (p3 - p2) / (t3 - t2);

        T dB1dx = ( t2-x )/( t2-t0 )*dA1dx + ( x-t0 )/( t2-t0 )*dA2dx + (A2 - A1)/(t2 - t0);
        T dB2dx = ( t3-x )/( t3-t1 )*dA2dx + ( x-t1 )/( t3-t1 )*dA3dx + (A3 - A2)/(t3 - t1);

        T d2B1dx2 = (T(2) * (dA2dx - dA1dx)) / (t2 - t0);
        T d2B2dx2 = (T(2) * (dA3dx - dA2dx)) / (t3 - t1);

        T d2Cdx2 =
            ( t2-x )/( t2-t1 )*d2B1dx2 +
            ( x-t1 )/( t2-t1 )*d2B2dx2 +
            (T(2) * (dB2dx - dB1dx)) / (t2 - t1);

        const float dxdt = (t2 - t1);
        return (dxdt * dxdt) * d2Cdx2;
    }

    // Numerical second derivative (central difference) of getV with respect to normalized segment t in [0, 1].
    template <typename T>
    static T getSecondDerivativeV_Numeric(const T& p0, const T& p1, const T& p2, const T& p3, float t /* between 0 and 1 */, float alpha=.5f /* between 0 and 1 */, float eps=1e-3f )
    {
        float t0 = t - eps;
        float t1 = t + eps;
        if (t0 < 0.0f) t0 = 0.0f;
        if (t1 > 1.0f) t1 = 1.0f;
        if (t1 <= t0) return T(0);

        const T d0 = getDerivativeV<T>(p0, p1, p2, p3, t0, alpha);
        const T d1 = getDerivativeV<T>(p0, p1, p2, p3, t1, alpha);
        return (d1 - d0) / (t1 - t0);
    }
};


template <typename T>
class Curve {
    BufferT<T, int> pts_;
    mutable BufferT<float, int> times_;
    mutable BufferT<float, int> segment_lengths_;
    mutable BufferT<float, int> accumulated_lengths_;
    float alpha_ = 0.5f; // centripetal

    public:
        void addPoint(const T& p) {
            pts_.push(p);
        }
        void removeAt(int idx) {
            pts_.remove(idx);
        }

        void setAlpha(float alpha) { alpha_ = alpha; }

        const T& operator[](int i) const {
            assert(i > 0 && i < pts_.size());
            return pts_[i];
        }
        int count() { return pts_.size(); }

        // TODO: add optional alpha override?
        T getAt(float t) const {
            float fseg;
            float time = modf(t, &fseg);
            int seg = (int)fseg;

            if(seg < 0) {
                seg = 0;
                return CatmullRom::getV(pts_[seg], pts_[seg+1], pts_[seg+2], pts_[seg+3], 0, alpha_);
            } else if (seg + 3 >= pts_.size()) {
                seg = pts_.size() - 4;
                return CatmullRom::getV(pts_[seg], pts_[seg+1], pts_[seg+2], pts_[seg+3], 1, alpha_);
            } else {
                return CatmullRom::getV(pts_[seg], pts_[seg+1], pts_[seg+2], pts_[seg+3], time, alpha_);
            }
        }

        T getDerivativeAt(float t) const {
            float fsegment;
            float time = modf(t, &fsegment);
            int seg = (int)fsegment;

            if(seg < 0) {
                seg = 0;
                return CatmullRom::getdV(pts_[seg], pts_[seg+1], pts_[seg+2], pts_[seg+3], 0, alpha_);
            } else if (seg + 3 >= pts_.size()) {
                seg = pts_.size() - 4;
                return CatmullRom::getdV(pts_[seg], pts_[seg+1], pts_[seg+2], pts_[seg+3], 1, alpha_);
            } else {
                return CatmullRom::getdV(pts_[seg], pts_[seg+1], pts_[seg+2], pts_[seg+3], time, alpha_);
            }
        }

        T getSecondDerivativeAt(float t) const {
            float fseg;
            float time = modf(t, &fseg);
            int seg = (int)fseg;

            if(seg < 0 || seg + 3 >= pts_.size())
                return T(0);

            return CatmullRom::getSecondDerivativeV(pts_[seg], pts_[seg+1], pts_[seg+2], pts_[seg+3], time, alpha_);
        }
    
        int getNumSegments() const { return pts_.size() >= 4 ? pts_.size() - 3 : 0; }
        int getNumNodes() const { return pts_.size() >= 2 ? pts_.size() - 2 : 0; }

        float GetLength(float t0, float t1) const;

        //float getTMin() const { assert(times_.size()>0); return times_[0]; }
        //float getTMax() const { assert(times_.size()>0); return times_.last(); }
        // TODO: actually fill times_ array, either when adding points or automatically
        // maybe also change interface, so it is not possible to add / remove points one by one
        // there is no sense it thes operations anyway.. I guess
        float getTMin() const { return 0; }
        float getTMax() const { assert(getNumSegments()>0); return getNumSegments(); }
        float getTotalLength() const { return (accumulated_lengths_.size() && accumulated_lengths_.last() != 0) ? accumulated_lengths_.last() : GetLength(getTMin(), getTMax()); }

};

// https://www.geometrictools.com/GTE/Mathematics/ReparameterizeByArclength.h
template <typename T>
class ReparameterizeByArclength {
    const Curve<T>& curve_;
    float total_len_;
    float tmin_, tmax_;
public:
    ReparameterizeByArclength(const Curve<T>& curve)
        :curve_(curve)
        ,total_len_(curve.getTotalLength())
        ,tmin_(curve.getTMin())
        ,tmax_(curve.getTMax())
    {}

    // The output object stores the curves t-parameter corresponding to a
    // user-specified arclength s or a fraction r. The t-member stores the
    // t-parameter. The f-member is output.f = F(output.t, s). The member
    // output.numIterations is the number of iterations used to compute t
    // for the corresponding s or r.
    struct Output
    {
        Output(): t(0), f(0), numIterations(0) { }

        Output(float inT, float inF, size_t inIterations)
            : t(inT), f(inF), numIterations(inIterations)
        {}

        float t, f;
        size_t numIterations;
    };

    // Given an arclength s in [0,L] where the total arclength of the
    // curve is L = Arclength(tMin,tMax)), the function returns the
    // root t for F(t,s) = Arclength(tMin,t) - s. Set 'useBisection'
    // to 'true' to use bisection only. Set it to 'false' to use the
    // hybrid of Newton's method and bisection.
    Output GetT(float const& s, bool useBisection) const
    {
        // Clamp the input to the valid interval.
        if (s <= 0) {
            return Output(tmin_, 0, 0);
        }

        if (s >= total_len_) {
            return Output(tmax_, 0, 0);
        }

        // Compute a t-root of F(t, s) for the specified s-value. We know
        // that F(mTMin) < 0 and F(mTMax) > 0. Rather than use the initial
        // interval [mTMin,mTMax], choose a subinterval using an initial
        // guess for the t-root.
        float tMin = tmin_;
        float tMax = tmax_;
        float tMid = tmin_ + (tMax - tMin) * (s / total_len_);
        float fMid = F(tMid, s);
        if (fMid > 0)
        {
            tMax = tMid;
        }
        else
        {
            tMin = tMid;
        }

        //if (useBisection)
        //{
        return DoBisection(tMin, tMax, s);
        //}
        //else
        //{
        //    return DoNewtonsMethod(tMin, tMax, tMid, s);
        //}
    }

    private:
    // Choose maxIterations sufficiently large for convergence. The value
    // 4096 is sufficient. In practice, the number of iterations for type
    // 'float' is no larger than approximately 24 and for type 'double'
    // is no larger than approximately 53.
    static size_t constexpr maxIterations = 128;

    inline float F(float const& t, float const& s) const
    {
        return curve_.GetLength(tmin_, t) - s;
    }

    inline float DFDT(float const& t) const
    {
        return curve_.getDerivativeAt(t);
    }

    bool BisectionConverged(float const& tMin, float const& tMax, float const& s, float& tMid, float& fMid) const
    {
        if (tMid == tMin || tMid == tMax)
        {
            // The precision of type T is such that tMin and tMax are
            // consecutive floating-point numbers. Their average cannot
            // be a floating-point number strictly between them. This is
            // the best you can do using type T. Return the t-endpoint
            // whose f-value has smaller magnitude.
            float fMin = F(tMin, s);
            float fMax = F(tMax, s);
            if (fMin <= fMax)
            {
                tMid = tMin;
                fMid = fMin;
            }
            else
            {
                tMid = tMax;
                fMid = fMax;
            }
            return true;
        }
        return false;
    }

    Output DoBisection(float tMin, float tMax, float const& s) const
    {
        float const zero = static_cast<float>(0);
        float const half = static_cast<float>(0.5);

        float tMid{}, fMid{};
        size_t numIterations{};
        for (numIterations = 1; numIterations <= maxIterations; ++numIterations)
        {
            // Compute the t-midpoint and the corresponding f-value. Exit
            // early if the f-value is zero.
            tMid = half * (tMin + tMax);
            fMid = F(tMid, s);
            if (fMid == zero)
            {
                break;
            }

            // Convergence occurs when tMid is tMin or tMax.
            if (BisectionConverged(tMin, tMax, s, tMid, fMid))
            {
                break;
            }

            // Update the correct t-endpoint using the t-midpoint.
            if (fMid > zero)
            {
                tMax = tMid;
            }
            else
            {
                tMin = tMid;
            }
        }

        return Output(tMid, fMid, numIterations);
    }
};

struct SplineClosestPointResult {
    float t;
    vec3 point;
    float distance;
    float distanceSqr;
};

SplineClosestPointResult spline_get_closest_point(
    const Curve<vec3>& curve, const vec3& position,
    int coarse_samples = 25, int refine_iters = 6);

inline void spline_get_basis_at(const Curve<vec3>& curve, const float t, const float dt, vec3& right, vec3& up, vec3& fwd) {

    float s1, s2;
    modf(t, &s1);
    modf(t+dt, &s2);
    assert(s1 == s2);


    fwd = normalize(curve.getDerivativeAt(t));
#if 0
    //calculate_basis(fwd, right, up);
    vec3 fwd_next = normalize(curve.getDerivativeAt(t + dt));


    up = cross(fwd, fwd_next);
    up = length(up) < 0.0001f ? vec3(0, 1, 0) : normalize(up);
    right = cross(up, fwd);
#else
    up = normalize(curve.getSecondDerivativeAt(t));
    right = normalize(cross(up, fwd));
    up = normalize(cross(fwd, right));
#endif


}
