#pragma once
#include "sph_lattice.h"
#include "engine/gameos.hpp" 
#include "renderer.h"
#include "utils/vec.h"

//#include <functional>
#include <vector>

class SPHBoundaryModel {

    vec3 boundary_min_;
    vec3 boundary_max_;

    SPHLattice* lattice_;
    bool b_is2d_;
    bool b_invert_distance_;

    // world transform
    vec3 position_;
    mat3 rotation_;
    
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


    bool Initialize(vec3 cube, float particle_radius, float support_radius, ivec3 resolution, bool b_is2d, bool b_invert);
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

};