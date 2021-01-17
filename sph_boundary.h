#pragma once

#include "engine/gameos.hpp" 
#include "renderer.h"
#include "utils/vec.h"

#include <functional>
#include <vector>

class SPHBoundaryModel {

    vec3 boundary_min_;
    vec3 boundary_max_;

    vec3 domain_min_;
    vec3 domain_max_;
    vec3 cell_size_;
    ivec3 res_;
    bool b_is2d_;
    bool b_invert_distance_;

    // world transform
    vec3 position_;
    mat3 rotation_;
    
    // min distance from a point to the boundary surface
    std::vector<float> distance_;
    // integrated volume at the point
    std::vector<float> volume_;

    std::vector<vec2> normal_;

    uint32_t volume_tex_;
    uint32_t distance_tex_;
    uint32_t normal_tex_;

    struct RenderMesh* boundary_mesh_;

    int pos2idx(vec3 local_pos);
    int idx(int x, int y, int z) const {
        return x + y * res_.x + z * res_.x * res_.y;
    }
    // returns local pos
    vec3 idx2pos(ivec3 idx);

    void calculate_distance_field(bool b_invert, float particle_radius);
    void generate_volume_map(float support_radius);

    float interpolate_value(const vec3& pos, std::function<float(const int)> value) const;
    float interpolate_value_xy(const vec3& pos, std::function<float(const int)> value) const;
    float interpolate_value_xy_old(const vec3& pos, std::function<float(const int)> value) const;
    ivec3 wrapi(vec3 p) const;

    float v(const int index) const {
        return volume_[index];
    }

    float d(const int index) const {
        return distance_[index];
    }

    vec3 getDomainDimension() const { return domain_max_ - domain_min_; }
public:

    bool Initialize(vec3 cube, float particle_radius, float support_radius, ivec3 resolution, bool b_is2d);
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
