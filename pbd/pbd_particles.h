#pragma once

#include "engine/utils/vec.h"
#include <vector>

struct PBDParticle {
    vec2 x, v;
    float inv_mass;
};


void pbd_unified_timestep(struct PBDUnifiedSimulation* sim, float dt);

struct PBDUnifiedSimulation* pbd_unified_sim_create(vec2& sim_dim);
void pbd_unified_sim_destroy(struct PBDUnifiedSimulation* sim);
void pbd_unified_sim_add_particle(struct PBDUnifiedSimulation* sim, vec2 pos, float density);
float pbd_unified_sim_get_particle_radius(struct PBDUnifiedSimulation* sim);
void pbd_unified_sim_reset(struct PBDUnifiedSimulation*);
int pbd_unified_sim_get_particle_count(struct PBDUnifiedSimulation*);
const PBDParticle* pbd_unified_sim_get_particles(struct PBDUnifiedSimulation*);
