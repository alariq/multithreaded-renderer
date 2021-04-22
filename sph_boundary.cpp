#include "sph_boundary.h"
#include "sph_kernels.h"
#include "engine/gameos.hpp"
#include "utils/vec.h"
#include "utils/gl_utils.h"
#include "utils/simple_quadrature.h"
#include "utils/matrix.h"
#include "utils/quaternion.h"
#include <cassert>
#include <cfloat>
#include <cstdint>

static const float eps = 1.0e+31F*FLT_MIN;

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

void SPHBoundaryModel::calculate_distance_field(bool b_invert, float particle_radius) {
    float sign = b_invert ? -1.0f : 1.0f;

    vec3 side_length = 0.5f * (boundary_max_ - boundary_min_);
    vec3 center = 0.5f * (boundary_max_ + boundary_min_);

    ivec3 res = lattice_->res();
    float* distances = lattice_->get<kDistanceIdx>();

	for (int z = 0; z < res.z; ++z) {
		for (int y = 0; y < res.y; ++y) {
			for (int x = 0; x < res.x; ++x) {
                vec3 p = lattice_->idx2pos(ivec3(x, y, z));
                int idx = x + (y + z * res.y) * res.x;
                // subtract 0.5*radius to avoid particle penetration
                float v = 0.0f;
                if(b_is2d_) {
                    v = sign * (sdBox2D((p - center).xy(), side_length.xy()) - 0.5f*particle_radius);
                } else {
                    v = sign * (sdBox(p - center, side_length) - 0.5f*particle_radius);
                }
                distances[idx] = v;
                //printf("%.2f ", v);
			}
            printf("\n");
		}
        printf("\n");
	}

        printf("\n");
        printf("\n");
        printf("\n");
        printf("\n");
    vec2* normals = lattice_->get<kNormalIdx>();
	for (int z = 0; z < res.z; ++z) {
		for (int y = 0; y < res.y; ++y) {
			for (int x = 0; x < res.x; ++x) {
                vec3 p = lattice_->idx2pos(ivec3(x, y, z));
                int idx = x + (y + z * res.y) * res.x;

				float dx = sign * (sdBox2D((p - center).xy(), side_length.xy()) -
								   sdBox2D(p.xy() + vec2(0.01f, 0.0f) - center.xy(),
										   side_length.xy()));
				float dy = sign * (sdBox2D((p - center).xy(), side_length.xy()) -
								   sdBox2D(p.xy() + vec2(0.0f, 0.01f) - center.xy(),
										   side_length.xy()));
				vec2 normal = normalize(vec2(-dx, dy));
                //normal_[idx] = getNormal2D(p.xy() + cell_size_.xy()*0.5f);
                normals[idx] = normal;
                //printf("%.2f,%.2f ", normal_[idx].x, normal_[idx].y);
			}
            printf("\n");
		}
        printf("\n");
	}
}

void SPHBoundaryModel::generate_volume_map(float support_radius) {

    simple_quadrature::determineSamplePointsInCircle(support_radius, 30);
    float factor = b_is2d_ ? 1.75f : 1.0f;
    assert(b_is2d_);

	auto volume_func = [&](vec3 const &x) -> float {
		auto dist = lattice_->interpolate_value_xy<kDistanceIdx>(x);
		if (dist > (1.0f + 1.0f /*/ factor*/) * support_radius) {
			return 0.0f;
		}

		auto integrand = [this, &x, support_radius, factor](vec3 const &xi) -> float {
			if (lengthSqr(xi) > support_radius * support_radius)
                return 0.0f;

			auto idist = lattice_->interpolate_value_xy<kDistanceIdx>(x + xi);

			if (idist <= 0.0f)
                return 1.0f - 0.1f * idist / support_radius;
			if (idist < 1.0f / factor * support_radius)
				return CubicKernel::W(factor * idist) / CubicKernel::W_zero();
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
    ivec3 res = lattice_->res();
    float* volumes = lattice_->get<kVolumeIdx>();
	for (int z = 0; z < res.z; ++z) {
		for (int y = 0; y < res.y; ++y) {
			for (int x = 0; x < res.x; ++x) {
				vec3 p = lattice_->idx2pos(ivec3(x, y, z));
				int idx = x + (y + z * res.y) * res.x;
                volumes[idx] = volume_func(p);
                //printf("%.2f ", volume_[idx]);
			}
            printf("\n");
		}
        printf("\n");
	}
}

bool SPHBoundaryModel::Initialize(vec3 cube, float particle_radius, float support_radius, ivec3 resolution, bool b_is2d, bool b_invert) {

    // extend by support radius
    vec3 ext = vec3(4.0f*support_radius);
    b_is2d_ = b_is2d;
    if(b_is2d)
        ext.z = 0.0f;

    b_invert_distance_ = b_invert;

    boundary_min_ = -0.5f*cube;//vec3(0);
    boundary_max_ = 0.5f*cube;

    setTransform(vec3(0,0,0), quaternion::identity(), vec3(1,1,1));

    vec3 domain_min = boundary_min_ - ext;
    vec3 domain_max = boundary_max_ + ext;
    lattice_ = new SPHLattice(domain_min, domain_max, resolution);

    calculate_distance_field(b_invert_distance_, particle_radius);

    generate_volume_map(support_radius);

    ivec3 res = lattice_->res();
	//auto get_distance = [this](const int idx) -> float { return d(idx); };
    float dist0 = lattice_->interpolate_value_xy<kDistanceIdx>(lattice_->idx2pos(ivec3(0,0,0)));(void)dist0;
    float dist1 = lattice_->interpolate_value_xy<kDistanceIdx>(lattice_->idx2pos(ivec3(0,1,0)));(void)dist1;
    float dist2 = lattice_->interpolate_value_xy<kDistanceIdx>(lattice_->idx2pos(ivec3(1,0,0)));(void)dist2;
    float dist3 = lattice_->interpolate_value_xy<kDistanceIdx>(lattice_->idx2pos(ivec3(1,1,0)));(void)dist3;
    float dist_c = lattice_->interpolate_value_xy<kDistanceIdx>(lattice_->domain_min() + lattice_->cell_size());(void)dist_c;

    float dist_center = lattice_->interpolate_value_xy<kDistanceIdx>(lattice_->idx2pos(ivec3(resolution.x/2, resolution.y/2, 0)));
    gosASSERT((!b_invert_distance_ && dist_center < 0.0f) || (b_invert_distance_ && dist_center > 0.0f));

    vec2 norm_right = getNormal2D(lattice_->idx2pos(ivec3(res.x-1,res.y/2,0)).xy());
    gosASSERT((!b_invert_distance_ && norm_right.x > 0.5f) || (b_invert_distance_ && norm_right.x < -0.5f));

    vec2 norm_left = getNormal2D(lattice_->idx2pos(ivec3(0,res.y/2,0)).xy());
    gosASSERT((!b_invert_distance_ && norm_left.x < -0.5f) || (b_invert_distance_ && norm_left.x > 0.5f));

    vec2 norm_bottom = getNormal2D(lattice_->idx2pos(ivec3(res.x/2, 0, 0)).xy());
    gosASSERT((!b_invert_distance_ && norm_bottom.y < -0.5f) || (b_invert_distance_ && norm_bottom.y > 0.5f));

    vec2 norm_top = getNormal2D(lattice_->idx2pos(ivec3(res.x/2, res.y-1, 0)).xy());
    gosASSERT((!b_invert_distance_ && norm_top.y > 0.5f) || (b_invert_distance_ && norm_top.y < -0.5f));

    return true;
}

void SPHBoundaryModel::Destroy() {
    gos_DestroyBuffer(boundary_mesh_->ib_);
    gos_DestroyBuffer(boundary_mesh_->vb_);
    delete boundary_mesh_;
}

void SPHBoundaryModel::setTransform(const vec3& pos, const quaternion& rot, const vec3& scale)
{
    pose_.pos = pos;
    pose_.rot = rot;
    pose_.scale = scale;
}

float SPHBoundaryModel::getDistance2D(const vec2& world_pos) const {
//	auto get_distance = [this](const int idx) -> float { return d(idx); };
  //  return interpolate_value_xy(vec3(pos.x, pos.y, 0.0f), get_distance);

	vec3 loc_pos = pose_w2l(pose_, vec3(world_pos,0));
	const vec2 pos = loc_pos.xy();

    float sign = b_invert_distance_ ? -1.0f : 1.0f;
    vec3 side_length = 0.5f * (boundary_max_ - boundary_min_) * pose_.scale;
    vec3 center = 0.5f * (boundary_max_ + boundary_min_) * pose_.scale;
    vec3 p = vec3(pos.x, pos.y, 0.0f);

    float v = 0.0f;
    float part_r = 0.1f;
    if(b_is2d_) {
        float box_d = sdBox2D((p - center).xy(), side_length.xy());
        v = sign * box_d ;//- 0.5f*part_r;
    } else {
        v = sign * (sdBox(p - center, side_length) - 0.5f*part_r);
    }

    return v;
}

float SPHBoundaryModel::getVolume2D(const vec2& world_pos) const {
	//auto get_volume = [this](const int idx) -> float { return v(idx); };
    //return interpolate_value_xy(vec3(pos.x, pos.y, 0.0f), get_volume);

	const vec2 pos = world_pos;
    float part_r = 0.1f;
    float support_r = 4.0f * part_r;
    float factor = 1.75f;

	auto volume_func = [&](vec3 const &x) -> float {
		auto dist = getDistance2D(x.xy());
		if (dist > (1.0f + 1.0f /*/ factor*/) * support_r) {
			return 0.0;
		}

		auto integrand = [this, &x, support_r, factor](vec3 const &xi) -> float {
			if (lengthSqr(xi) > support_r* support_r)
                return 0.0f;

			auto idist = getDistance2D((x + xi).xy());

			if (idist <= 0.0f)
                return 1.0f - 0.1f * idist / support_r;
			if (idist < 1.0f / factor * support_r)
				return CubicKernel::W(factor * idist) / CubicKernel::W_zero();
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

vec2 SPHBoundaryModel::getNormal2D(const vec2& world_pos) const {
	vec3 loc_pos = pose_w2l(pose_, vec3(world_pos, 0));
	const vec2 pos = loc_pos.xy();

#if 1
    vec3 side_length = 0.5f * (boundary_max_ - boundary_min_) * pose_.scale;
    vec3 center = 0.5f * (boundary_max_ + boundary_min_) * pose_.scale;
    vec2 dpx = vec2(0.01f, 0.0f);
    vec2 dpy = vec2(0.0f, 0.01f);

    float x2 = sdBox2D(pos + dpx - center.xy(), side_length.xy());
    float x1 = sdBox2D(pos - center.xy(), side_length.xy());
    float dx = (x2 - x1);

    float y2 = sdBox2D(pos + dpy - center.xy(), side_length.xy());
    float y1 = sdBox2D(pos - center.xy(), side_length.xy());
    float dy = y2 - y1;

#else
    const float h = 0.01f*min(cell_size_.x, cell_size_.y);
	auto get_distance = [this](const int idx) -> float { return d(idx); };
	float dx = interpolate_value_xy(vec3(pos.x + h, pos.y, 0.0f), get_distance) -
			   interpolate_value_xy(vec3(pos.x - h, pos.y, 0.0f), get_distance);

	float dy = interpolate_value_xy(vec3(pos.x, pos.y + h, 0.0f), get_distance) -
			   interpolate_value_xy(vec3(pos.x, pos.y - h, 0.0f), get_distance);

#endif
    
    // it may be that dx=0 and dy=0, for cube it may be when we are close to the diagonal line
    // so in this case return normalize(vec2(1,1));
    vec2 normal;
    if(fabsf(dx) < eps && fabsf(dy) < eps)
        normal = normalize(vec2(1.0f*sign(dx), 1.0f*sign(dy)));
    else
        normal = normalize(vec2(-dx, -dy));
    // warning: will not work with any rotation not in XY plane, as we only us XY
    normal = quat_rotate(pose_.rot, vec3(normal,0)).xy();

    return b_invert_distance_ == false ? -normal : normal; 
}

void SPHBoundaryModel::InitializeRenderResources() {

    ivec3 res = lattice_->res();
    vec3 domain_min = lattice_->domain_min();
    vec3 domain_max = lattice_->domain_max();

    volume_tex_ = gos_NewEmptyTexture(gos_Texture_R32F, "volume", res.x, res.y);
	distance_tex_ = gos_NewEmptyTexture(gos_Texture_R32F, "distance", res.x, res.y);
	normal_tex_ = gos_NewEmptyTexture(gos_Texture_RGBA8, "normal", res.x, res.y);

	//
	constexpr const size_t NUM_VERT = 8;
	constexpr const size_t NUM_IND = 16;

	float z = 0.0f;
	vec3 vb[NUM_VERT] = {
		vec3(boundary_min_.x, boundary_min_.y, z),
		vec3(boundary_min_.x, boundary_max_.y, z),
		vec3(boundary_max_.x, boundary_max_.y, z),
		vec3(boundary_max_.x, boundary_min_.y, z),

		vec3(domain_min.x, domain_min.y, z),
		vec3(domain_min.x, domain_max.y, z),
		vec3(domain_max.x, domain_max.y, z),
		vec3(domain_max.x, domain_min.y, z),
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
    boundary_mesh_->vb_first_ = 0;
    boundary_mesh_->vb_count_ = UINT32_MAX;
}

void SPHBoundaryModel::UpdateTexturesByData() {

	TEXTUREPTR texinfo;
    ivec3 res = lattice_->res();
	{
		gos_LockTexture(distance_tex_, 0, false, &texinfo);
		for (int y = 0; y < res.y; ++y) {
			float *row = (float*)texinfo.pTexture + y * texinfo.Pitch;
			const float *src_row = lattice_->get<SPHBoundaryModel::kDistanceIdx>() + y * res.x;
			for (int x = 0; x < res.x; ++x) {
				row[x] = src_row[x];
			}
		}
		gos_UnLockTexture(distance_tex_);
	}

	{
		gos_LockTexture(volume_tex_, 0, false, &texinfo);
		for (int y = 0; y < res.y; ++y) {
			float *row = (float*)texinfo.pTexture + y * texinfo.Pitch;
			const float *src_row = lattice_->get<SPHBoundaryModel::kVolumeIdx>() + y * res.x;
			for (int x = 0; x < res.x; ++x) {
				row[x] = src_row[x];
			}
		}
		gos_UnLockTexture(volume_tex_);
	}

	{
		gos_LockTexture(normal_tex_, 0, false, &texinfo);
		for (int y = 0; y < res.y; ++y) {
			uint8_t* row = (uint8_t*)((uint32_t*)texinfo.pTexture + y * texinfo.Pitch);
			const vec2* src_row = lattice_->get<SPHBoundaryModel::kNormalIdx>() + y * res.x;
			for (int x = 0; x < res.x; ++x) {
                vec2 n = 255.0f*(src_row[x] * 0.5 + 0.5f);
                 
				row[4*x + 0] = (uint8_t)clamp(n.x, 0.0f, 255.0f);
				row[4*x + 1] = (uint8_t)clamp(n.y, 0.0f, 255.0f);
				row[4*x + 2] = (uint8_t)0;
				row[4*x + 3] = (uint8_t)255;
			}
		}
		gos_UnLockTexture(normal_tex_);
	}

    // draw debug quad with texture
    vec2 size = lattice_->getDomainDimension().xy();
    vec3 center = getCenter();
    center.z = 0.4f;

    mat4 tr = pose_s_to_mat4(pose_) * mat4::translation(center);
    gos_AddQuad(size, vec4(1,1,1, 1.3f), volume_tex_, &tr, true);
}


