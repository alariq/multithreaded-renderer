#pragma once

#include "engine/utils/vec.h"
#include <vector>
#include <stdint.h>

// TODO: remove constrants which are not flags ( e.g. fluid, solid, move them to phase?)
struct PBDParticleFlags {
	enum : uint32_t {
		kRigidBody = 0x01,
		kFluid = 0x02,
		kSolid = 0x04,
		kSleep = 0x10,
	};
};

struct PBDParticle {
    vec2 x, v;
    float inv_mass;
    uint32_t flags;
    int32_t phase;
    union {
        // only valid if flags say it is rigid body particle
        int32_t rb_data_idx;
        // only valid if flags say it is fluid partice
        int32_t fluid_data_idx;
    };
	// TODO: move common params to particle type data structure and store this structure's
	// handl here instead
	float mu_s, mu_k;
};

struct PBDRigidBodyParticleData {
    vec2 x0;
	float sdf_value;
    vec2 sdf_grad;
    // remove? we have rb_data_idx 
    int32_t index;

    uint32_t b_is_boundary: 1; // could be in flags;
};

struct PBDRigidBody{
    // move to other struct as those are changing?
    vec2 x;
    float angle;
    float angle0;
    float mu_s; // static friction
    float mu_k; // kinetic friction

    // index and number of particles in PBDRigidBodyParticleData
    int start_part_idx;
    int num_part;
    uint32_t flags;
};

struct PBDFluidModel {
    float density0_;
    float mass_;
    float viscosity_;
};

struct PBDFluidParicleRuntimeData {
    float density_;
    float lambda_;
};


void pbd_unified_timestep(struct PBDUnifiedSimulation* sim, float dt);

struct PBDUnifiedSimulation* pbd_unified_sim_create(vec2& sim_dim);
void pbd_unified_sim_destroy(struct PBDUnifiedSimulation* sim);
void pbd_unified_sim_reset(struct PBDUnifiedSimulation*);

int pbd_unified_sim_add_particle(struct PBDUnifiedSimulation* sim, vec2 pos, float density);
int pbd_unified_sim_add_particle(struct PBDUnifiedSimulation* sim, vec2 pos, vec2 init_vel, float density);
int pbd_unified_sim_add_box_rigid_body(struct PBDUnifiedSimulation* sim, int size_x, int size_y, vec2 pos, float rot, float density);

int pbd_unified_sim_add_fluid_model(struct PBDUnifiedSimulation* sim, float viscosity, float density);
int pbd_unified_sim_add_fluid_particle(struct PBDUnifiedSimulation* sim, vec2 pos, int fluid_model_idx);

void pbd_unified_sim_rb_add_velocity(struct PBDUnifiedSimulation* sim, int rb_idx, const vec2& vel);
// TODO: make it throuht particle type data structure, do not really need per partcle friction
void pbd_unified_sim_particle_set_friction(int idx, float mu_s, float mu_k);

float pbd_unified_sim_get_particle_radius(const struct PBDUnifiedSimulation* sim);
int pbd_unified_sim_get_particle_count(const struct PBDUnifiedSimulation*);
const PBDParticle* pbd_unified_sim_get_particles(const struct PBDUnifiedSimulation*);

// TODO: this was a workaround for debug output
const PBDRigidBodyParticleData* pbd_unified_sim_get_rb_particle_data(const struct PBDUnifiedSimulation*);
const PBDRigidBody* pbd_unified_sim_get_rigid_bodies(const struct PBDUnifiedSimulation*);
vec2 pbd_unified_sim_get_world_bounds(const struct PBDUnifiedSimulation*);
