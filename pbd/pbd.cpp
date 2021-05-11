#include "pbd.h"
#include "pbd_collision.h"

#include "engine/utils/vec.h"
#include <algorithm>
#include <cassert>

void PBDSimulation::removeRigidBody(RigidBody* rb_to_del) {
    // TODO: get rid of those arrays and move to pool-allocaror like strategy
    // cannot resize,because somebody may point to RB by index
    // so we just nullify the array

	auto b = std::begin(rb_);
	auto e = std::end(rb_);
    auto rb_iter = std::find_if(b,e,[rb_to_del](RigidBody* o) { return o == rb_to_del; });
    assert(rb_iter!=e);
    *rb_iter = nullptr;
}

bool PBDSimulation::addRigidBodyContactConstraint(const int body_index1,
												  const int body_index2, const vec3 &cp1,
												  const vec3 &cp2, const vec3 &normal,
												  const float dist,
												  const float restitution_c,
												  const float friction_c) {
	rb_contacts_.emplace_back(RigidBodyContactConstraint());
	RigidBodyContactConstraint &cc = rb_contacts_.back();
	// wtf? need to avoid failing...
	if (!cc.initConstraint(getRigidBodies(), body_index1, body_index2, cp1, cp2, normal,
						   dist, restitution_c, contact_stiffness_rb_, friction_c)) {
		rb_contacts_.pop_back();
		return false;
	}

	return true;
}

void PBDSolver::setCollisionDetection(ICollisionDetection *cd, PBDSimulation *sim) {
	cd->setRigidBodyContactCallback(&PBDSolver::contactCallbackFunction,
									ContactType::kRigidBody, sim);
	cd_ = cd;
}

void PBDSolver::contactCallbackFunction(const ContactType contact_type,
										const int body_index1, const int body_index2,
										const vec3 &cp1, const vec3 &cp2,
										const vec3 &normal, const float dist,
										const float restitution_c, const float friction_c,
										void *user_data) {
	PBDSimulation *sim = (PBDSimulation *)user_data;
	if (contact_type == ContactType::kRigidBody) {
		sim->addRigidBodyContactConstraint(body_index1, body_index2, cp1, cp2, normal,
										   dist, restitution_c, friction_c);
	}
}

void PBDSolver::velocityConstraintProjection(PBDSimulation *sim, int iter) {
	// do stuff
	RigidBody *const *rbs = sim->getRigidBodies();
	for (auto &c : sim->getRigidBodyContactConstraints()) {
		c.solveVelocityConstraint(rbs, iter);
	}
}

void PBDSolver::updateTransforms(PBDSimulation* sim) {

	RigidBody **bodies = sim->getRigidBodies();
	const int count = sim->getNumRigidBodies();
    for(int bi=0; bi< count; ++bi) {
		RigidBody *rb = bodies[bi];
		std::vector<vec3>& vd = rb->vertices_;
		int idx = 0;
		for(auto& v : vd) {
            (void)v;
			rb->vertices_[idx] = rb->pose.transform(rb->local_vertices_[idx]);
			idx++;
		}
	}
}

void PBDSolver::simulate(PBDSimulation *sim, float dt) {

	RigidBody *const *bodies = sim->getRigidBodies();
	const int count = sim->getNumRigidBodies();

	dt = dt / num_iter;

	for (int iter = 0; iter < num_iter; ++iter) {

		for (int i = 0; i < count; ++i) {
            bodies[i]->old_pose = bodies[i]->prev_pose;
            bodies[i]->prev_pose = bodies[i]->pose;
			bodies[i]->integrate(dt, 1*gravity);
            bodies[i]->onUpdateRotation();
		}

#if IMPLEMENTED
		// constraints
		for (int i = 0; i < count; ++i) {
			joints[i]->solvePos(dt);
		}
#endif

		for (int i = 0; i < count; ++i) {
			bodies[i]->update(dt);
		}

#if IMPLEMENTED
		// damping
		for (int i = 0; i < count; ++i) {
			joints[i]->solveVel(dt);
		}
#endif
	}

    updateTransforms(sim);

	cd_->collisionDetection(sim);

	int iter = 0;
    while (iter++ < num_iter_vel) {
		velocityConstraintProjection(sim, iter);
	}
}

void pbd_init_scene() {
	PBDSolver *solver = new PBDSolver(); // aka timestep
	DistanceFieldCollisionDetection *cd = new DistanceFieldCollisionDetection();
	PBDSimulation *sim = new PBDSimulation();

    const vec3 box_size = vec3(1.0f);
    const vec3 floor_size = vec3(20.0f, 1.0f, 20.0f);

	RigidBody *rb_cube = new RigidBody();
	rb_cube->setBox(box_size, 1000.0f);
	rb_cube->pose.p = vec3(0, 4, 0);
	rb_cube->pose.q = quaternion::identity();

	RigidBody *rb_floor = new RigidBody();
	rb_floor->setBox(floor_size, 1000.0f);
	rb_cube->pose.p = vec3(0, 0, 0);
	rb_cube->pose.q = quaternion::identity();
	rb_floor->inv_mass = 0.0f; // kinematic

	int rb_cube_index = sim->addRigidBody(rb_cube);
	int rb_floor_index = sim->addRigidBody(rb_floor);

	// maybe return object handle upon addition to CollisionDetection ?
	DistanceFieldCollisionObjectBox *co_box = new DistanceFieldCollisionObjectBox();
	co_box->box_ = box_size;
	co_box->enable_ = true;
	co_box->invert_ = false;
	co_box->aabb_ = AABB(-0.5f*box_size, 0.5f*box_size);
	co_box->body_type_ = ICollisionObject::RigidBodyCollisionObjectType;
	co_box->body_index_ = rb_cube_index;

	DistanceFieldCollisionObjectBox *co_floor = new DistanceFieldCollisionObjectBox();
	co_box->box_ = floor_size;
	co_box->enable_ = true;
	co_box->invert_ = false;
	co_box->aabb_ = AABB(-0.5f*floor_size, 0.5f*floor_size);
	co_box->body_type_ = ICollisionObject::RigidBodyCollisionObjectType;
	co_box->body_index_ = rb_floor_index;

	cd->setTolerance(0.05f);
	cd->addCollisionObject(co_box);
	cd->addCollisionObject(co_floor);

	solver->setCollisionDetection(cd, sim);
	solver->simulate(sim, 1.0f / 60.0f);
}

////////////////////////////////////////////////////////////////////////////////
static PBDSolver *solver = new PBDSolver(); // aka timestep
static PBDSimulation* g_pbd_simulation = nullptr;
static ICollisionDetection* g_collision_detection = nullptr;

PBDSimulation* pbd_get_simulation() {
    return g_pbd_simulation;
}

ICollisionDetection* pbd_get_collision_detection() {
    return g_collision_detection;
}

void pbd_create_simulation() {
    g_pbd_simulation = new PBDSimulation();
}

void pbd_create_collision_detection() {
    g_collision_detection = new DistanceFieldCollisionDetection();
	solver->setCollisionDetection(g_collision_detection, g_pbd_simulation);
}

void pbd_destroy_simulation() {
    delete g_pbd_simulation;
}

void pbd_destroy_collision_detection() {
    delete g_collision_detection;
}

void pbd_simulate(float dt) {
	solver->simulate(g_pbd_simulation, 1.0f / 60.0f);
}

////////////////////////////////////////////////////////////////////////////////
RigidBody* pbd_create_box_rigid_body(const vec3& box_size, const float density,
									 const vec3& pos, const quaternion& rot,
									 const float restitution, const float friction) {

	RigidBody *rb = new RigidBody();
	rb->setBox(box_size, density);
    rb->resetPose({rot, pos});
    rb->restitution_c = restitution;
    rb->friction_c = friction;
    return rb;
}

ICollisionObject* pbd_create_box_collision(int rb_index, const vec3& box_size) {
	// maybe return object handle upon addition to CollisionDetection ?
	DistanceFieldCollisionObjectBox* co_box = new DistanceFieldCollisionObjectBox();
	co_box->box_ = 0.5f * box_size;
	co_box->enable_ = true;
	co_box->invert_ = false;
	co_box->aabb_ = AABB(-0.5f * box_size, 0.5f * box_size);
	co_box->body_type_ = ICollisionObject::RigidBodyCollisionObjectType;
	co_box->body_index_ = rb_index;
	return co_box;
}

