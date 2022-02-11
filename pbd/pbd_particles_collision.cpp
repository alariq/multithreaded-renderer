#include "pbd_particles_collision.h"
#include "engine/utils/vec.h"
#include "engine/utils/sdf.h"
#include "engine/utils/adt.h"

#include <vector>
#include <algorithm>
#include <stdint.h>
#include <cassert>

// this could be a common interface so that different world type could exist
struct CollisionWorld {
    // do we need to have separate arrays per obj type?
    FreeList<SDFBoxCollision> boxes_;
    std::vector<int> boxes_list_;
    // do we need this list of all objects at all?
    std::vector<SDFCollisionObject> objs_;

    std::vector<CollisionContact> CheckCollisions(const vec2* part_positions, int num_particles, float radius) const;
};

std::vector<CollisionContact> collision_world_check_collision(CollisionWorld* cworld,
															  const vec2* part_positions,
															  int num_particles,
															  float radius) {
	return cworld->CheckCollisions(part_positions, num_particles, radius);
}

CollisionWorld* collision_create_world() {
    return new CollisionWorld();
}

void collision_destroy_world(CollisionWorld* cw) {
    delete cw;
}

const SDFCollisionObject* collision_world_get_objects(const CollisionWorld* cworld, int* count) {
    assert(cworld && count);
    *count = (int)cworld->objs_.size();
    return cworld->objs_.data();
}

const int* collision_world_get_box_ids(const CollisionWorld* cworld, int* count) {
    assert(cworld && count);
    *count = (int)cworld->boxes_list_.size();
    return cworld->boxes_list_.data();
}

const SDFBoxCollision& collision_world_get_box(const CollisionWorld* cworld, int id) {
    assert(cworld && id>=0);
    return cworld->boxes_.at(id);
}

int collision_add_box(CollisionWorld* world, const SDFBoxCollision& box) { //, int flags)
    int idx = world->boxes_.insert(box);
    world->boxes_list_.push_back(idx);
    world->objs_.push_back(SDFCollisionObject{idx, SDFCollisionObject::kBox});
    return idx;
}

void collision_remove_box(CollisionWorld* world, int id) {
    world->boxes_.release(id);
    // remove from box list
    auto bit = std::remove(world->boxes_list_.begin(), world->boxes_list_.end(), id);
    assert(std::distance(bit, world->boxes_list_.end()) == 1);
	world->boxes_list_.erase(bit);

    // remove from list of all objects
	auto it = std::remove_if(world->objs_.begin(), world->objs_.end(),
				   [id](SDFCollisionObject& o) { return o.index == id; });
    // only unique ids
    assert(std::distance(it, world->objs_.end()) == 1);
	world->objs_.erase(it);
}

void collision_set_box_transform(CollisionWorld* world, int id, const vec2& pos, float angle, const vec2& scale) {
    SDFBoxCollision& box = world->boxes_.at(id);
    box.pos = pos;
    box.rot = rotate2(angle);
    box.scale = scale;
}

typedef bool(*collision_handler_t)(const vec2& x, const vec2& box, const float tolerance, vec2* cp, vec2* n,
					   float* dist, const float max_dis);

bool collision_box_particle(const vec2& ppos, const vec2& bpos, const vec2& box,
							const float tolerance, vec2* cp, vec2* n, float* dist,
							const float max_dist /* = 0.0f*/) {

	bool collided = false;
    vec3 dist_grad = sdg_box2d(bpos, box);
    dist_grad.x -= tolerance;
    if(dist_grad.x < max_dist) {
        // collision point
        *n = -dist_grad.yz();
        *dist = -(dist_grad.x);
		*cp = (ppos - (*dist) * (*n));
        collided = true;
    }
    return collided;
}

// hope for RVO
std::vector<CollisionContact> CollisionWorld::CheckCollisions(const vec2* part_positions,
															  int num_particles,
															  float radius) const {

	std::vector<CollisionContact> ccs;
	const int num_boxes = (int)boxes_list_.size();

	for (int i = 0; i < num_particles; i++) {
		for (int j = 0; j < num_boxes; j++) {
            const int id = boxes_list_[j];
			vec2 bpos = transpose(boxes_[id].rot) * (boxes_[id].pos - part_positions[i]); 
			CollisionContact cc;
			if (collision_box_particle(part_positions[i], bpos, boxes_[id].size*boxes_[id].scale, radius,
									   &cc.cp, &cc.n, &cc.dist, 0.0f)) {
                cc.n = boxes_[id].rot * cc.n;
                cc.idx = i;
                cc.coll_idx = j;
				ccs.push_back(cc);
			}
		}
	}

    return ccs;
}

