#pragma once

#include "utils/vec.h"
#include <vector>
#include <tuple>
#include <cassert>
#include <cfloat>

//template <typename Storage_t>
class SPHLattice {

    static constexpr float kEps = 1.0e+31F*FLT_MIN;

    vec3 domain_min_;
    vec3 domain_max_;
    vec3 cell_size_;
    ivec3 res_;

    typedef std::tuple<std::vector<float>, std::vector<float>, std::vector<vec2>> Storage_t;
    Storage_t storage_;

    //std::vector<std::vector<float>> storage_;

	public:

    template<int i>
    struct get_layer {
        typedef typename std::tuple_element<i, Storage_t>::type type_;
    };

    SPHLattice(const vec3& dmin, const vec3& dmax, const ivec3& res);

    template<int layer_index>
    inline typename get_layer<layer_index>::type_::value_type interpolate_value_xy(const vec3& pos) const;

    template<int layer_index>
    inline typename get_layer<layer_index>::type_::value_type interpolate_value(const vec3& pos) const;

    int pos2idx(vec3 local_pos) const;
    inline int idx(int x, int y, int z) const {
        return x + y * res_.x + z * res_.x * res_.y;
    }
    // returns local pos
    vec3 idx2pos(ivec3 idx);

	ivec3 wrapi(vec3 p) const;

	ivec3 res() const { return res_; }
    vec3 cell_size() const { return cell_size_; }
    vec3 domain_min() const { return domain_min_; }
    vec3 domain_max() const { return domain_max_; }

    template<int layer_index>
    const typename get_layer<layer_index>::type_::value_type* get() const { return std::get<layer_index>(storage_).data(); }

    template<int layer_index>
    typename get_layer<layer_index>::type_::value_type* get() { return std::get<layer_index>(storage_).data(); }

    vec3 getDomainDimension() const { return domain_max_ - domain_min_; }

};


template<int layer_index>
inline typename SPHLattice::get_layer<layer_index>::type_::value_type SPHLattice::interpolate_value(const vec3& pos) const
{
    vec3 p = (pos - domain_min_) / (domain_max_ - domain_min_) ;
    p.x = clamp(p.x, 0.0f, 1.0f);
    p.y = clamp(p.y, 0.0f, 1.0f);
    p.z = clamp(p.z, 0.0f, 1.0f);

    p *= vec3((float)res_.x, (float)res_.y, (float)res_.z);

    // OpenGL 4.6 spec 8.14. TEXTURE MINIFICATION

    ivec3 s = wrapi(floor(p - vec3(0.5f)));
    ivec3 e = wrapi(floor(p - vec3(0.5f)) + vec3(1.0f));
    vec3 k = frac(p-vec3(0.5f));

    const int i0 = s.x;
    const int j0 = s.y;
    const int k0 = s.z;

    const int i1 = e.x;
    const int j1 = e.y;
    const int k1 = e.z;
    
    const float a = k.x;
    const float b = k.y;
    const float c = k.z;

    auto value = [this](const int idx) {
        return std::get<layer_index>(storage_)[idx];
    };

    float v = (1 - a) * (1 - b) * (1 - c) * value(idx(i0, j0, k0)) +
			  a * (1 - b) * (1 - c) * value(idx(i1, j0, k0)) +
			  (1 - a) * b * (1 - c) * value(idx(i0, j1, k0)) +
			  a * b * (1 - c) * value(idx(i1, j1, k0)) +
			  (1 - a) * (1 - b) * c * value(idx(i0, j0, k1)) +
			  a * (1 - b) * c * value(idx(i1, j1, k0)) +
			  (1 - a) * b * c * value(idx(i0, j1, k1)) +
			  a * b * c * value(idx(i1, j1, k1));

    return v;
}

template<int layer_index>
inline typename SPHLattice::get_layer<layer_index>::type_::value_type SPHLattice::interpolate_value_xy(const vec3& pos) const
{
    vec3 p = (pos - domain_min_) / (domain_max_ - domain_min_) ;
    p.x = clamp(p.x, 0.0f, 1.0f);
    p.y = clamp(p.y, 0.0f, 1.0f);
    p.z = clamp(p.z, 0.0f, 1.0f);

    p *= vec3((float)res_.x, (float)res_.y, (float)res_.z);
    ivec3 s = wrapi(floor(p - vec3(0.5f)));
    ivec3 e = wrapi(floor(p - vec3(0.5f)) + vec3(1.0f));

	int i00 = (s.x) + ((s.y) + s.z * res_.y) * res_.x;
    int i01 = (s.x) + ((e.y) + s.z * res_.y) * res_.x;
    int i10 = (e.x) + ((s.y) + s.z * res_.y) * res_.x;
    int i11 = (e.x) + ((e.y) + s.z * res_.y) * res_.x;

    vec3 k = frac(p-vec3(0.5f));

    auto value = [this](const int idx) {
        return std::get<layer_index>(storage_)[idx];
    };

    float v0 = value(i00)*(1.0f - k.x) + value(i01)*k.x;
    float v1 = value(i10)*(1.0f - k.x) + value(i11)*k.x;
    float v = v0*(1.0f - k.y) + v1*k.y;
    return v;
}

