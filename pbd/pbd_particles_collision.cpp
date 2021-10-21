#include "pbd_particles_collision.h"
#include "engine/utils/vec.h"
#include "engine/utils/sdf.h"

#include <vector>
#include <stdint.h>
#include <cassert>

struct SDFCollisionObject {
    enum Type: uint8_t {
        kBox,
        kNumTypes
    };
    int index;
    Type type_;
    // flags, etc.
};

// this could be a common interface so that different world type could exist
struct CollisionWorld {
    std::vector<SDFBoxCollision> boxes_;
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

const SDFBoxCollision* collision_world_get_box(const CollisionWorld* cworld, int* count) {
    assert(cworld && count);
    *count = (int)cworld->boxes_.size();
    return cworld->boxes_.data();
}


int collision_add_box(CollisionWorld* world, const SDFBoxCollision& box) { //, int flags)
    world->boxes_.push_back(box);
    int idx = (int)(world->boxes_.size() - 1);
    world->objs_.push_back(SDFCollisionObject{idx, SDFCollisionObject::kBox});
    return (int)(world->objs_.size() - 1);
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
	const int num_boxes = (int)boxes_.size();

	for (int i = 0; i < num_particles; i++) {
		for (int j = 0; j < num_boxes; j++) {
			vec2 bpos = boxes_[j].pos - part_positions[i]; // TODO: transform to world, or inverse transform particle
			CollisionContact cc;
			if (collision_box_particle(part_positions[i], bpos, boxes_[j].size, radius,
									   &cc.cp, &cc.n, &cc.dist, 0.0f)) {
                cc.idx = i;
                cc.coll_idx = j;
				ccs.push_back(cc);
			}
		}
	}

    return ccs;
}

