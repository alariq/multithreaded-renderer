#pragma once

#include "engine/utils/vec.h"
#include <vector>
#include <stdint.h>

struct PBDParticleFlags {
	enum : uint32_t {
		kRigidBody = 0x01,
		kSleep = 0x16,
	};
};

struct PBDParticle {
    vec2 x, v;
    float inv_mass;
    uint32_t flags;
    int32_t phase;
};

struct PBDRigidBodyParticleData {
    vec2 x0;
    float sdf_value;
    vec2 sdf_grad;
    int32_t index;
    uint32_t b_is_boundary: 1; // could be in flags;
};

struct PBDRigidBody{
    vec2 x;
    float angle;
    // index and number of particles in PBDRigidBodyParticleData
    int start_part_idx;
    int num_part;
    uint32_t flags;
};


void pbd_unified_timestep(struct PBDUnifiedSimulation* sim, float dt);

struct PBDUnifiedSimulation* pbd_unified_sim_create(vec2& sim_dim);
void pbd_unified_sim_destroy(struct PBDUnifiedSimulation* sim);
int pbd_unified_sim_add_particle(struct PBDUnifiedSimulation* sim, vec2 pos, float density);
int pbd_unified_sim_add_box_rigid_body(struct PBDUnifiedSimulation* sim, int size_x, int size_y, vec2 pos, float rot, float density);
float pbd_unified_sim_get_particle_radius(struct PBDUnifiedSimulation* sim);
void pbd_unified_sim_reset(struct PBDUnifiedSimulation*);
int pbd_unified_sim_get_particle_count(struct PBDUnifiedSimulation*);
const PBDParticle* pbd_unified_sim_get_particles(struct PBDUnifiedSimulation*);
