#include "sph_lattice.h"
#include "utils/vec.h"


template<typename T, int i, bool b = (i == std::tuple_size<T>::value)>
struct resizer {
    static void resize(T& storage, int n) {
        std::get<i>(storage).resize(n);
        resizer<T, i+1>::resize(storage, n);
    }
};

template<typename T, int i>
struct resizer<T, i, true> {
    static void resize(T& storage, int n) {}
};


SPHLattice::SPHLattice(const vec3& dmin, const vec3& dmax, const ivec3& res) {

    domain_min_ = dmin;
    domain_max_ = dmax;
    res_ = res;

    cell_size_ = (domain_max_ - domain_min_) / vec3((float)res_.x, (float)res_.y, (float)res_.z);

    const int bufsize = res_.x * res_.y * res_.z;
    resizer<Storage_t,0>::resize(storage_, bufsize);
}

int SPHLattice::pos2idx(vec3 p) const {

    p = (p - domain_min_) / (domain_max_ - domain_min_) ;
    // -kEps to avoid getting index which equals to resolution
    p.x = clamp(p.x, 0.0f, 1.0f - kEps);
    p.y = clamp(p.y, 0.0f, 1.0f - kEps);
    p.z = clamp(p.z, 0.0f, 1.0f - kEps);


    vec3 fres = vec3((float)res_.x, (float)res_.y, (float)res_.z);
    vec3 fi = fres * p;
    ivec3 i = ivec3((int)fi.x, (int)fi.y, (int)fi.z);
    assert(i.x >= 0 && i.x < res_.x);
    assert(i.y >= 0 && i.y < res_.y);
    assert(i.z >= 0 && i.z < res_.z);

	int idx = i.x + (i.y + i.z * res_.y) * res_.x;
    assert(idx >= 0 && idx < (int)std::get<0>(storage_).size());
    return idx;
}

// value is stored at the cell center
vec3 SPHLattice::idx2pos(ivec3 idx) {
    assert(idx.x >= 0 && idx.x < res_.x);
    assert(idx.y >= 0 && idx.y < res_.y);
    assert(idx.z >= 0 && idx.z < res_.z);

    vec3 p = vec3((float)idx.x, (float)idx.y, (float)idx.z);
    p += vec3(0.5f);
    p /= vec3((float)res_.x, (float)res_.y, max(1.0f, (float)res_.z - 1.0f));
    return p * (domain_max_ - domain_min_) + domain_min_;
}

ivec3 SPHLattice::wrapi(vec3 p) const {
    p.x = clamp(p.x, 0.0f, res_.x - 1.0f);
    p.y = clamp(p.y, 0.0f, res_.y - 1.0f);
    p.z = clamp(p.z, 0.0f, res_.z - 1.0f);
    return ivec3((int)p.x, (int)p.y, (int)p.z);
}

