#pragma once
#include "sph_lattice.h"
#include "engine/gameos.hpp" 
#include "renderer.h"
#include "utils/vec.h"
#include "utils/quaternion.h"
#include "utils/math_utils.h"

//#include <functional>
#include <vector>

class SPHBoundaryModel {

    vec3 boundary_min_;
    vec3 boundary_max_;

    SPHLattice* lattice_;
    bool b_is2d_;
    bool b_invert_distance_;
    bool b_dynamic_;

    pose_s pose_;

    // used in case of dynamic boundary
	struct RigidBody* rb_;
    struct ICollisionObject* rb_collision_;
    
    uint32_t distance_tex_;
    uint32_t volume_tex_;
    uint32_t normal_tex_;

    struct RenderMesh* boundary_mesh_;

    void calculate_distance_field(bool b_invert, float particle_radius);
    void generate_volume_map(float support_radius);

public:

    enum { 
        kDistanceIdx = 0,
        kVolumeIdx = 1,
        kNormalIdx = 2
    };

	bool Initialize(vec3 cube, float particle_radius, float support_radius,
					ivec3 resolution, bool b_is2d, bool b_invert, bool b_dynamic);
	void Destroy();

    vec3 getDimension() const { return boundary_max_ - boundary_min_; }
    vec3 getCenter() const { return 0.5f * (boundary_max_ + boundary_min_); }

    float getDistance2D(const vec2& pos) const;
    float getVolume2D(const vec2& pos) const;

    vec2 getNormal2D(const vec2& pos) const;

    const RenderMesh* getBoundaryMesh() const { return boundary_mesh_; }

    void InitializeRenderResources();
    void UpdateTexturesByData();
    uint32_t getVolumeTexture() const { return volume_tex_; }
    uint32_t getDistanceTexture() const { return distance_tex_; }
    uint32_t getNormalTexture() const { return normal_tex_; }

    void setTransform(const vec3& pos, const quaternion& rot, const vec3& scale);
    mat4 getTransformMatrix() const;

    bool isDynamic() const { return b_dynamic_; }
    void addForce(const vec3& at, const vec3& f);
};
