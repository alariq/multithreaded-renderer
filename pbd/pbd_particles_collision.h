#pragma once

#include "engine/utils/vec.h"

#include <vector> //  :-(
#include <stdint.h>

struct SDFBoxCollision {
    vec2 pos;
    mat2 rot;
    vec2 scale; // incorporate into size?
    vec2 size;
    // flags etc..
};

struct SDFCollisionObject {
    enum Type: uint8_t {
        kBox,
        kNumTypes
    };
    int index;
    Type type_;
    // flags, etc.
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
void collision_remove_box(CollisionWorld* world, int id);
void collision_set_box_transform(CollisionWorld* world, int id, const vec2& pos, float angle, const vec2& scale);

CollisionWorld* collision_create_world();
void collision_destroy_world(CollisionWorld* cw);

std::vector<CollisionContact> collision_world_check_collision(CollisionWorld* cworld,
															  const vec2* part_positions,
															  int num_particles,
															  float radius);

const SDFCollisionObject* collision_world_get_objects(const CollisionWorld* cworld, int* count);
const SDFBoxCollision& collision_world_get_box(const CollisionWorld* cworld, int id);
const int* collision_world_get_box_ids(const CollisionWorld* cworld, int* count);
