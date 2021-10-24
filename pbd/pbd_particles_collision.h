#pragma once

#include "engine/utils/vec.h"

#include <vector> //  :-(

struct SDFBoxCollision {
    vec2 pos;
    mat2 rot;
    vec2 size;
    // flags etc..
};

struct CollisionContact {
    vec2 cp; // collision point
    vec2 n;  // normal
    float dist; // penetration depth
    int idx; // particle idx
    int coll_idx; // collision object index
};

struct SDFBoxCollision;
struct CollisionWorld;

int collision_add_box(CollisionWorld* world, const SDFBoxCollision& box);

CollisionWorld* collision_create_world();
void collision_destroy_world(CollisionWorld* cw);

std::vector<CollisionContact> collision_world_check_collision(CollisionWorld* cworld,
															  const vec2* part_positions,
															  int num_particles,
															  float radius);

const SDFBoxCollision* collision_world_get_box(const CollisionWorld* cworld, int* count);
