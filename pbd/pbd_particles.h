#pragma once

#include "engine/utils/vec.h"
#include "engine/utils/adt.h"
#include <vector>
#include <stdint.h>

enum: uint32_t { kPBDInvalidIndex = 0xFFFFFFFFu };

struct PBDSettings {
    vec2 world_bounds;
    bool supports_diag_links;
    //...
};

struct DistanceConstraint {
    int idx0, idx1;
    float dist;
};

// not needed if we have static particles (infinite mass)
struct DistanceConstraint2 {
    int idx0;
    vec2 pos;
    float dist;
};

// TODO: remove constrants which are not flags ( e.g. fluid, solid, move them to phase?)
struct PBDParticleFlags {
	enum : uint32_t {
		kRigidBody = 0x01,
		kFluid = 0x02,
		kSolid = 0x04,
		kSleep = 0x10,
		kSoftBody = 0x20,
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
        // only valid if flags say it is soft body particle
        int32_t sb_data_idx;
        // only valid if flags say it is fluid partice
        int32_t fluid_data_idx;
    };
	// TODO: move common params to particle type data structure and store this structure's
	// handl here instead
	float mu_s, mu_k;
    // restitution coefficient
    float e;
};

struct PBDRigidBodyParticleData {
    vec2 x0;
	float sdf_value;
    vec2 sdf_grad;
    // remove? we have rb_data_idx 
    int32_t index;

    uint32_t b_is_boundary: 1; // could be in flags;
};

struct PBDRigidBody {
    // move to other struct as those are changing?
    vec2 x; // apparently not really used
    float angle;
    float angle0;
    float mu_s; // static friction
    float mu_k; // kinetic friction
    float e; // restitution
    float Iinv; // inverse inertia tensor

    // index and number of particles in PBDRigidBodyParticleData
    int start_pdata_idx;
    int num_part;
    uint32_t flags;
};

struct PBDFluidModel {
    float density0_;
    float mass_;
    float viscosity_;
    uint32_t debug_color_;
};

struct PBDFluidParicleRuntimeData {
    float density_;
    float lambda_;
};

struct PBDRegion {
    // indices into particles_per_region array
	uint32_t start_part_idx;
	uint32_t num_part;
	float mass; // mass of the region (depends on the amount of prticles there), basically = num_part
	vec2 cm0;	// non deformed center of mass

    // runtime data
	vec2 cm;	 // deformed
	float angle; // rotation
	mat2 A;		 // A = R*U
	mat2 R;		 // rotation decomposed
};

struct PBDSoftBodyParticleData {
    PBDRigidBodyParticleData base;
    int x, y; // position in lattice

    // used for breakage calculation
    vec2 goal_x;
    mat2 goal_R;
    
    // in sb_regions_per_particle_
    uint32_t start_region_idx;
    uint32_t num_regions;
};

struct PBDSoftBody {
    PBDRigidBody base;

    uint32_t w; // region size: 2*w + 1 x 2w + 1 (actually not)
    float alpha; // stiffness
    uint32_t start_region_idx;
    uint32_t num_regions;
    uint32_t parent_breakable_idx; // if part of breakable, otherwise kPBDInvalidIndex
    uint32_t next_soft_body_idx; // index of next soft body idx if part of breakable soft body
    int32_t island; // island number (if soft body was created for a specific island)
};

struct PBDBodyLatticeData {
    int x, y; // topology index, used in region calculations
    uint32_t nflags; // neighbour flags
    uint32_t pdata_idx;
    uint32_t particle_idx; // not sure if needed
    int island;
};

struct PBDBreakableSoftBody {
    uint32_t w; // region size: 2*w + 1 x 2w + 1 (actually not)
    float alpha; // stiffness
    float fracture_len; // relative to rest len
    float fracture_cos_angle; // cos(angle)
    uint32_t lattice_sx;
    uint32_t lattice_sy;
    uint32_t lattice_idx;
    // index to first PBDSoftBody
    uint32_t first_soft_body;
};

void pbd_unified_timestep(struct PBDUnifiedSimulation* sim, float dt);

struct PBDUnifiedSimulation* pbd_unified_sim_create(vec2& sim_dim);
void pbd_unified_sim_destroy(struct PBDUnifiedSimulation* sim);
void pbd_unified_sim_reset(struct PBDUnifiedSimulation*);

void pbd_unified_sim_set_collision_world(struct PBDUnifiedSimulation* sim, struct CollisionWorld* cworld);
struct CollisionWorld* pbd_unified_sim_get_collision_world(const struct PBDUnifiedSimulation* sim);

int pbd_unified_sim_add_particle(struct PBDUnifiedSimulation* sim, vec2 pos, float density);
int pbd_unified_sim_add_particle(struct PBDUnifiedSimulation* sim, vec2 pos, vec2 init_vel, float density);
int pbd_unified_sim_add_box_rigid_body(struct PBDUnifiedSimulation* sim, int size_x, int size_y, vec2 pos, float rot, float density);
int pbd_unified_sim_add_box_soft_body(struct PBDUnifiedSimulation* sim, int size_x,
									  int size_y, vec2 pos, float rot, float density,
									  uint32_t w, float alpha);
void pbd_util_build_connectivity_lattice(uint32_t sx, uint32_t sy,
										 const uint8_t* blueprint,
										 PBDBodyLatticeData* lattice);
int pbd_unified_sim_add_soft_body(struct PBDUnifiedSimulation* sim, uint32_t size_x,
								  uint32_t size_y, PBDBodyLatticeData* lattice,
								  int island, vec2 pos, float density, uint32_t w,
								  float alpha);
int pbd_unified_sim_add_breakable_soft_body(struct PBDUnifiedSimulation* sim,
											uint32_t size_x, uint32_t size_y,
											const uint8_t* blueprint, vec2 pos,
											float density, uint32_t w, float compliance,
											float fr_len, float fr_angle);
void pbd_unified_sim_remove_breakable_soft_body(struct PBDUnifiedSimulation* sim, int id);

int pbd_unified_sim_add_distance_constraint(struct PBDUnifiedSimulation* sim, int p0_idx, int p1_idx, float dist);
int pbd_unified_sim_add_distance2_constraint(struct PBDUnifiedSimulation* sim, int p0_idx, vec2 pos, float dist);
void pbd_unified_sim_update_distance2_constraint(struct PBDUnifiedSimulation* sim, int c_idx, vec2* pos, float dist);
bool pbd_unified_sim_remove_distance_constraint(struct PBDUnifiedSimulation* sim, int c_idx);
bool pbd_unified_sim_remove_distance2_constraint(struct PBDUnifiedSimulation* sim, int c_idx);

int pbd_unified_sim_add_fluid_model(struct PBDUnifiedSimulation* sim, float viscosity, float density);
int pbd_unified_sim_add_fluid_particle(struct PBDUnifiedSimulation* sim, vec2 pos, int fluid_model_idx);
uint32_t pbd_unified_sim_get_fluid_model_debug_color(struct PBDUnifiedSimulation* sim, int fm_idx);

void pbd_unified_sim_particle_add_velocity(struct PBDUnifiedSimulation* sim, int idx, const vec2& vel);
void pbd_unified_sim_rb_add_velocity(struct PBDUnifiedSimulation* sim, int rb_idx, const vec2& vel);
void pbd_unified_sim_rb_set_density(struct PBDUnifiedSimulation* sim, int rb_idx, float density);
// TODO: make it through particle type data structure, do not really need per partcle friction
void pbd_unified_sim_particle_set_params(struct PBDUnifiedSimulation* sim, int idx, float mu_s, float mu_k, float e);

float pbd_unified_sim_get_particle_radius(const struct PBDUnifiedSimulation* sim);
int pbd_unified_sim_get_particle_count(const struct PBDUnifiedSimulation*);
const PBDParticle* pbd_unified_sim_get_particles(const struct PBDUnifiedSimulation*);

// TODO: this was a workaround for debug output
const PBDRigidBodyParticleData* pbd_unified_sim_get_rb_particle_data(const struct PBDUnifiedSimulation*);
const PBDSoftBodyParticleData* pbd_unified_sim_get_sb_particle_data(const struct PBDUnifiedSimulation* sim);
const PBDFluidModel* pbd_unified_sim_get_fluid_particle_model(const struct PBDUnifiedSimulation*, int idx);
uint32_t pbd_unified_sim_get_particle_flags(const struct PBDUnifiedSimulation* sim, int idx);
const PBDRigidBody* pbd_unified_sim_get_rigid_bodies(const struct PBDUnifiedSimulation*);
const PBDSoftBody& pbd_unified_sim_get_soft_body(int i, const struct PBDUnifiedSimulation* sim);
const uint32_t* pbd_unified_sim_get_soft_body_idxs(const struct PBDUnifiedSimulation* sim);
const int pbd_unified_sim_get_soft_body_count(const struct PBDUnifiedSimulation* sim);
const PBDRegion* pbd_unified_sim_get_sb_regions(const struct PBDUnifiedSimulation* sim);
const uint32_t* pbd_unified_sim_get_sb_regions_pidxs(const struct PBDUnifiedSimulation* sim);

const int* pbd_unified_sim_get_distance_constraint_idxs(const struct PBDUnifiedSimulation* sim, int* count);
const DistanceConstraint* pbd_unified_sim_get_distance_constraint(const struct PBDUnifiedSimulation* sim, int idx);
const int* pbd_unified_sim_get_distance2_constraint_idxs(const struct PBDUnifiedSimulation* sim, int* count);
const DistanceConstraint2* pbd_unified_sim_get_distance2_constraint(const struct PBDUnifiedSimulation* sim, int idx);

vec2 pbd_unified_sim_get_world_bounds(const struct PBDUnifiedSimulation*);
const PBDSettings* pbd_unified_sim_settings(const struct PBDUnifiedSimulation* sim);


struct DbgContactInfo {
    int i0, i1;
    vec2 n;
    vec2 dp0, dp1;
    vec2 p0, p1;
};

struct DbgFrictionInfo {
    int i;
    vec2 n;
    vec2 dx;
    float mu_s, mu_k;
    vec2 s_path;
    vec2 d_path;
};

const DbgContactInfo* pbd_unified_sim_get_dbg_contacts(const struct PBDUnifiedSimulation*, int* count);
const DbgFrictionInfo* pbd_unified_sim_get_dbg_friction(const struct PBDUnifiedSimulation*, int* count);

