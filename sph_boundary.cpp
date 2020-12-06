#include "sph_boundary.h"
#include "sph_kernels.h"
#include "engine/gameos.hpp"
#include "utils/vec.h"
#include "utils/gl_utils.h"
#include "utils/simple_quadrature.h"
#include <functional>
#include <cassert>
#include <cfloat>

static const float eps = 10.0f*FLT_MIN;

float sdBox(vec3 p, vec3 b)
{
  vec3 q = abs(p) - b;
  return length(max(q,vec3(0.0f))) + min(max(q.x,max(q.y,q.z)),0.0f);
}

float sdBox2D(vec2 p, vec2 b)
{
  vec2 q = abs(p) - b;
  return length(max(q,vec2(0.0f))) + min(max(q.x,q.y),0.0f);
}

int SPHBoundaryModel::pos2idx(vec3 p) {

    p = (p - domain_min_) / (domain_max_ - domain_min_) ;
    // -eps to avoid getting index which equals to resolution
    p.x = clamp(p.x, 0.0f, 1.0f - eps);
    p.y = clamp(p.y, 0.0f, 1.0f - eps);
    p.z = clamp(p.z, 0.0f, 1.0f - eps);

    ivec3 i = ivec3(p.x, p.y, p.z) * res_;
    assert(i.x >= 0 && i.x < res_.x);
    assert(i.y >= 0 && i.y < res_.y);
    assert(i.z >= 0 && i.z < res_.z);

	int idx = i.x + (i.y + i.z * res_.y) * res_.x;
    assert(idx >= 0 && idx < (int)distance_.size());
    return idx;
}

// value is stored at the cell center
vec3 SPHBoundaryModel::idx2pos(ivec3 idx) {
    assert(idx.x >= 0 && idx.x < res_.x);
    assert(idx.y >= 0 && idx.y < res_.y);
    assert(idx.z >= 0 && idx.z < res_.z);

    vec3 p = vec3((float)idx.x, (float)idx.y, (float)idx.z);
    p += vec3(0.5f);
    p /= vec3(res_.x, res_.y, max(1.0f, res_.z - 1.0f));
    return p * (domain_max_ - domain_min_) + domain_min_;
}

void SPHBoundaryModel::calculate_distance_field(bool b_invert, float particle_radius) {
    float sign = b_invert ? -1.0f : 1.0f;

    vec3 side_length = 0.5f * (boundary_max_ - boundary_min_);
    vec3 center = 0.5f * (boundary_max_ + boundary_min_);

	for (int z = 0; z < res_.z; ++z) {
		for (int y = 0; y < res_.y; ++y) {
			for (int x = 0; x < res_.x; ++x) {
                vec3 p = idx2pos(ivec3(x, y, z));
                int idx = x + (y + z * res_.y) * res_.x;
                // subtract 0.5*radius to avoid particle penetration
                float v = 0.0f;
                if(b_is2d_) {
                    v = sign * (sdBox2D((p - center).xy(), side_length.xy()) - 0.5f*particle_radius);
                } else {
                    v = sign * (sdBox(p - center, side_length) - 0.5f*particle_radius);
                }
                distance_[idx] = v;
                printf("%.2f ", v);
			}
            printf("\n");
		}
        printf("\n");
	}

        printf("\n");
        printf("\n");
        printf("\n");
        printf("\n");
	for (int z = 0; z < res_.z; ++z) {
		for (int y = 0; y < res_.y; ++y) {
			for (int x = 0; x < res_.x; ++x) {
                vec3 p = idx2pos(ivec3(x, y, z));
                int idx = x + (y + z * res_.y) * res_.x;

				float dx = sign * (sdBox2D((p - center).xy(), side_length.xy()) -
								   sdBox2D(p.xy() + vec2(0.01f, 0.0f) - center.xy(),
										   side_length.xy()));
				float dy = sign * (sdBox2D((p - center).xy(), side_length.xy()) -
								   sdBox2D(p.xy() + vec2(0.0f, 0.01f) - center.xy(),
										   side_length.xy()));
				vec2 normal = normalize(vec2(-dx, dy));
                //normal_[idx] = getNormal2D(p.xy() + cell_size_.xy()*0.5f);
                normal_[idx] = normal;
                printf("%.2f,%.2f ", normal_[idx].x, normal_[idx].y);
			}
            printf("\n");
		}
        printf("\n");
	}
}

ivec3 SPHBoundaryModel::wrapi(vec3 p) const {
    p.x = clamp(p.x, 0.0f, res_.x - 1.0f);
    p.y = clamp(p.y, 0.0f, res_.y - 1.0f);
    p.z = clamp(p.z, 0.0f, res_.z - 1.0f);
    return ivec3((int)p.x, (int)p.y, (int)p.z);
}

float SPHBoundaryModel::interpolate_value_xy(const vec3& pos, std::function<float(const int)> value) const
{
    vec3 p = (pos - domain_min_) / (domain_max_ - domain_min_) ;
    p.x = clamp(p.x, 0.0f, 1.0f);
    p.y = clamp(p.y, 0.0f, 1.0f);
    p.z = clamp(p.z, 0.0f, 1.0f);

    p *= vec3(res_.x, res_.y, res_.z);
    ivec3 s = wrapi(floor(p - 0.5f));
    ivec3 e = wrapi(floor(p - 0.5f) + 1.0f);

	int i00 = (s.x) + ((s.y) + s.z * res_.y) * res_.x;
    int i01 = (s.x) + ((e.y) + s.z * res_.y) * res_.x;
    int i10 = (e.x) + ((s.y) + s.z * res_.y) * res_.x;
    int i11 = (e.x) + ((e.y) + s.z * res_.y) * res_.x;

    vec3 k = frac(p-0.5f);

    float v0 = value(i00)*(1.0f - k.x) + value(i01)*k.x;
    float v1 = value(i10)*(1.0f - k.x) + value(i11)*k.x;
    float v = v0*(1.0f - k.y) + v1*k.y;
    return v;

	//return (1.0f - k.x) * (1.0f - k.y) * value(vec3(c0, 0.0f)) +
	//	   k.x * (1.0f - k.y) * value(vec2(c1.x, c0.y)) +
	//	   (1.0f - k.x) * k.y * value(vec2(c0.x, c1.y)) + k.x * k.y * value(c1);

}

float SPHBoundaryModel::interpolate_value_xy_old(const vec3& pos, std::function<float(const int)> value) const {
    vec3 p = (pos - domain_min_) / (domain_max_ - domain_min_) ;
    p.x = clamp(p.x, 0.0f, 1.0f);
    p.y = clamp(p.y, 0.0f, 1.0f);
    p.z = clamp(p.z, 0.0f, 1.0f);

    p *= vec3(res_.x - 1, res_.y - 1, res_.z - 1);
    vec3 k = vec3(p.x - floorf(p.x), p.y - floorf(p.y), p.z - floorf(p.z));
    ivec3 s = ivec3(p.x, p.y, p.z);
	ivec3 e = ivec3(min(s.x + 1, res_.x - 1), min(s.y + 1, res_.y - 1), s.z);

	int i00 = (s.x) + ((s.y) + s.z * res_.y) * res_.x;
    int i01 = (s.x) + ((e.y) + s.z * res_.y) * res_.x;
    int i10 = (e.x) + ((s.y) + s.z * res_.y) * res_.x;
    int i11 = (e.x) + ((e.y) + s.z * res_.y) * res_.x;

    float dist0 = value(i00)*(1.0f - k.x) + value(i01)*k.x;
    float dist1 = value(i10)*(1.0f - k.x) + value(i11)*k.x;
    float dist = dist0*(1.0f - k.y) + dist1*k.y;

    return dist;
}

void SPHBoundaryModel::generate_volume_map(float support_radius) {

    simple_quadrature::determineSamplePointsInCircle(support_radius, 30);
    float factor = b_is2d_ ? 1.75f : 1.0f;
    assert(b_is2d_);

	auto get_distance = [this](const int idx) -> float { return d(idx); };

	auto volume_func = [&](vec3 const &x) -> float {
		auto dist = interpolate_value_xy(x, get_distance);
		if (dist > (1.0 + 1.0 /*/ factor*/) * support_radius) {
			return 0.0;
		}

		auto integrand = [this, &x, support_radius, factor, &get_distance](vec3 const &xi) -> float {
			if (lengthSqr(xi) > support_radius * support_radius)
                return 0.0f;

			auto dist = interpolate_value_xy(x + xi, get_distance);

			if (dist <= 0.0f)
                return 1.0f - 0.1f * dist / support_radius;
			if (dist < 1.0f / factor * support_radius)
				return CubicKernel::W(factor * dist) / CubicKernel::W_zero();
			return 0.0;
		};

		float res = 0.0;
		//if (b_is2d)
			res = 0.8f * simple_quadrature::integrate(integrand);
		//else
		//	res = 0.8 * GaussQuadrature::integrate(integrand, int_domain, 30);

		return res;
	};
 
    printf("volume:\n");
	for (int z = 0; z < res_.z; ++z) {
		for (int y = 0; y < res_.y; ++y) {
			for (int x = 0; x < res_.x; ++x) {
				vec3 p = idx2pos(ivec3(x, y, z));
				int idx = x + (y + z * res_.y) * res_.x;
                volume_[idx] = volume_func(p);
                printf("%.2f ", volume_[idx]);
			}
            printf("\n");
		}
        printf("\n");
	}
}

bool SPHBoundaryModel::Initialize(vec3 cube, float particle_radius, float support_radius, ivec3 resolution, bool b_is2d) {

    // extend by support radius
    vec3 ext = vec3(4.0f*support_radius);
    b_is2d_ = b_is2d;
    if(b_is2d)
        ext.z = 0.0f;

    b_invert_distance_ = true;

    boundary_min_ = vec3(0);
    boundary_max_ = cube;

    domain_min_ = boundary_min_ - ext;
    domain_max_ = boundary_max_ + ext;

    position_ = vec3(0,0,0);
    rotation_ = identity3();
    res_ = resolution;
    cell_size_ = (domain_max_ - domain_min_) / vec3(res_.x, res_.y, res_.z);
    const int bufsize = resolution.x * resolution.y * resolution.z;
    distance_.resize(bufsize);
    volume_.resize(bufsize);
    normal_.resize(bufsize);

    calculate_distance_field(b_invert_distance_, particle_radius);

    generate_volume_map(support_radius);

	auto get_distance = [this](const int idx) -> float { return d(idx); };
    float dist0 = interpolate_value_xy(idx2pos(ivec3(0,0,0)), get_distance);(void)dist0;
    float dist1 = interpolate_value_xy(idx2pos(ivec3(0,1,0)), get_distance);(void)dist1;
    float dist2 = interpolate_value_xy(idx2pos(ivec3(1,0,0)), get_distance);(void)dist2;
    float dist3 = interpolate_value_xy(idx2pos(ivec3(1,1,0)), get_distance);(void)dist3;
    float dist_c = interpolate_value_xy(domain_min_ + cell_size_ , get_distance);(void)dist_c;

    float dist_center = interpolate_value_xy(idx2pos(ivec3(res_.x/2, res_.y/2, 0)), get_distance);
    gosASSERT(dist_center > 0.0f);

    vec2 norm_right = getNormal2D(idx2pos(ivec3(res_.x-1,res_.y/2,0)).xy());
    gosASSERT(norm_right.x < -0.5f);        

    vec2 norm_left = getNormal2D(idx2pos(ivec3(0,res_.y/2,0)).xy());
    gosASSERT(norm_left.x > 0.5f);        

    vec2 norm_bottom = getNormal2D(idx2pos(ivec3(res_.x/2, 0, 0)).xy());
    gosASSERT(norm_bottom.y > 0.5f);        

    vec2 norm_top = getNormal2D(idx2pos(ivec3(res_.x/2, res_.y-1, 0)).xy());
    gosASSERT(norm_top.y < -0.5f);        

    return true;
}

void SPHBoundaryModel::Destroy() {
    gos_DestroyBuffer(boundary_mesh_->ib_);
    gos_DestroyBuffer(boundary_mesh_->vb_);
    delete boundary_mesh_;
}

float SPHBoundaryModel::getDistance2D(const vec2& pos) const {
//	auto get_distance = [this](const int idx) -> float { return d(idx); };
  //  return interpolate_value_xy(vec3(pos.x, pos.y, 0.0f), get_distance);

    float sign = b_invert_distance_ ? -1.0f : 1.0f;
    vec3 side_length = 0.5f * (boundary_max_ - boundary_min_);
    vec3 center = 0.5f * (boundary_max_ + boundary_min_);
    vec3 p = vec3(pos.x, pos.y, 0.0f);

    float v = 0.0f;
    float part_r = 0.1f;
    if(b_is2d_) {
        v = sign * (sdBox2D((p - center).xy(), side_length.xy()) - 0.5f*part_r);
    } else {
        v = sign * (sdBox(p - center, side_length) - 0.5f*part_r);
    }

    return v;
}

float SPHBoundaryModel::getVolume2D(const vec2& pos) const {
	//auto get_volume = [this](const int idx) -> float { return v(idx); };
    //return interpolate_value_xy(vec3(pos.x, pos.y, 0.0f), get_volume);

    float part_r = 0.1f;
    float support_r = 4.0f * part_r;
    float factor = 1.75f;

	auto volume_func = [&](vec3 const &x) -> float {
		auto dist = getDistance2D(x.xy());
		if (dist > (1.0 + 1.0 /*/ factor*/) * support_r) {
			return 0.0;
		}

		auto integrand = [this, &x, support_r, factor](vec3 const &xi) -> float {
			if (lengthSqr(xi) > support_r* support_r)
                return 0.0f;

			auto dist = getDistance2D((x + xi).xy());

			if (dist <= 0.0f)
                return 1.0f - 0.1f * dist / support_r;
			if (dist < 1.0f / factor * support_r)
				return CubicKernel::W(factor * dist) / CubicKernel::W_zero();
			return 0.0;
		};

		float res = 0.0;
		//if (b_is2d)
			res = 0.8f * simple_quadrature::integrate(integrand);
		//else
		//	res = 0.8 * GaussQuadrature::integrate(integrand, int_domain, 30);

		return res;
	};

    return volume_func(vec3(pos.x, pos.y, 0.0f));
}

vec2 SPHBoundaryModel::getNormal2D(const vec2& pos) const {

#if 1
    vec3 side_length = 0.5f * (boundary_max_ - boundary_min_);
    vec3 center = 0.5f * (boundary_max_ + boundary_min_);
    vec2 dpx = vec2(0.01f, 0.0f);
    vec2 dpy = vec2(0.0f, 0.01f);

	float dx = (sdBox2D(pos + dpx - center.xy(), side_length.xy()) -
				sdBox2D(pos - center.xy(), side_length.xy()));

	float dy = (sdBox2D(pos + dpy - center.xy(), side_length.xy()) -
				sdBox2D(pos - center.xy(), side_length.xy()));
#else
    const float h = 0.01f*min(cell_size_.x, cell_size_.y);
	auto get_distance = [this](const int idx) -> float { return d(idx); };
	float dx = interpolate_value_xy(vec3(pos.x + h, pos.y, 0.0f), get_distance) -
			   interpolate_value_xy(vec3(pos.x - h, pos.y, 0.0f), get_distance);

	float dy = interpolate_value_xy(vec3(pos.x, pos.y + h, 0.0f), get_distance) -
			   interpolate_value_xy(vec3(pos.x, pos.y - h, 0.0f), get_distance);

#endif
    if(fabsf(dx) < eps && fabsf(dy) < eps)
        return vec2(0.0f);

    vec2 normal = vec2(-dx, -dy);
    return normalize(normal);
}

void SPHBoundaryModel::InitializeRenderResources() {

    DWORD wh = (res_.x << 16) | res_.y;
    volume_tex_ = gos_NewEmptyTexture(gos_Texture_R32F, "volume", wh);
	distance_tex_ = gos_NewEmptyTexture(gos_Texture_R32F, "distance", wh);
	normal_tex_ = gos_NewEmptyTexture(gos_Texture_RGBA8, "normal", wh);

	//
	constexpr const size_t NUM_VERT = 8;
	constexpr const size_t NUM_IND = 16;

	float z = 0.0f;
	vec3 vb[NUM_VERT] = {
		vec3(boundary_min_.x, boundary_min_.y, z),
		vec3(boundary_min_.x, boundary_max_.y, z),
		vec3(boundary_max_.x, boundary_max_.y, z),
		vec3(boundary_max_.x, boundary_min_.y, z),

		vec3(domain_min_.x, domain_min_.y, z),
		vec3(domain_min_.x, domain_max_.y, z),
		vec3(domain_max_.x, domain_max_.y, z),
		vec3(domain_max_.x, domain_min_.y, z),
	};
	uint16_t ib[NUM_IND] = {0, 1, 1, 2, 2, 3, 3, 0,
    4,5, 5, 6, 6, 7, 7, 4
    };

	boundary_mesh_ = new RenderMesh();
	boundary_mesh_->vdecl_ = get_pos_only_vdecl();
	boundary_mesh_->ib_ =
		gos_CreateBuffer(gosBUFFER_TYPE::INDEX, gosBUFFER_USAGE::STATIC_DRAW,
						 sizeof(uint16_t), NUM_IND, ib);
	boundary_mesh_->vb_ = gos_CreateBuffer(
		gosBUFFER_TYPE::VERTEX, gosBUFFER_USAGE::STATIC_DRAW, sizeof(vec3), NUM_VERT, vb);

	boundary_mesh_->prim_type_ = PRIMITIVE_LINELIST;
}

void SPHBoundaryModel::UpdateTexturesByData() {

	TEXTUREPTR texinfo;
	{
		gos_LockTexture(distance_tex_, 0, false, &texinfo);
		for (int y = 0; y < res_.y; ++y) {
			float *row = (float*)texinfo.pTexture + y * texinfo.Pitch;
			const float *src_row = distance_.data() + y * res_.x;
			for (int x = 0; x < res_.x; ++x) {
				row[x] = src_row[x];
			}
		}
		gos_UnLockTexture(distance_tex_);
	}

	{
		gos_LockTexture(volume_tex_, 0, false, &texinfo);
		for (int y = 0; y < res_.y; ++y) {
			float *row = (float*)texinfo.pTexture + y * texinfo.Pitch;
			const float *src_row = volume_.data() + y * res_.x;
			for (int x = 0; x < res_.x; ++x) {
				row[x] = src_row[x];
			}
		}
		gos_UnLockTexture(volume_tex_);
	}

	{
		gos_LockTexture(normal_tex_, 0, false, &texinfo);
		for (int y = 0; y < res_.y; ++y) {
			uint8_t* row = (uint8_t*)((uint32_t*)texinfo.pTexture + y * texinfo.Pitch);
			const vec2* src_row = normal_.data() + y * res_.x;
			for (int x = 0; x < res_.x; ++x) {
                vec2 n = 255.0f*(src_row[x] * 0.5 + 0.5f);
                 
				row[4*x + 0] = (uint8_t)clamp(n.x, 0.0f, 255.0f);
				row[4*x + 1] = (uint8_t)clamp(n.y, 0.0f, 255.0f);
				row[4*x + 2] = (uint8_t)0;
				row[4*x + 3] = (uint8_t)0;
			}
		}
		gos_UnLockTexture(normal_tex_);
	}
}


