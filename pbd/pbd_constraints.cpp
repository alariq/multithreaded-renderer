#include "pbd_constraints.h"
#include "pbd.h"

#include "pbd_rb_dynamics.h"
#include "engine/utils/vec.h"

static int getNextConstraintId() {
    static int id = 1;
    return id++;
}

int RigidBodyContactConstraint::TYPE_ID = getNextConstraintId();

bool RigidBodyContactConstraint::initConstraint(
	RigidBody **rbodies, const int body_index1, const int body_index2, const vec3 &cp1,
	const vec3 &cp2, const vec3 &normal, const float dist, const float restitution_c,
	const float stiffness, const float friction_c) {

	stiffness_ = stiffness;
	friction_c_ = friction_c;
	sum_impulses_ = 0.0f;

	bodies_[0] = body_index1;
	bodies_[1] = body_index2;

	RigidBody *rb1 = rbodies[body_index1];
	RigidBody *rb2 = rbodies[body_index2];

	return PBD::initRigidBodyContactConstraint(
		rb1->inv_mass,
		rb1->pose.p,
		rb1->vel,
		rb1->inv_inertia_w,
		rb1->pose.q,
		rb1->omega,
		rb2->inv_mass,
		rb2->pose.p,
		rb2->vel,
		rb2->inv_inertia_w,
		rb2->pose.q,
		rb2->omega,
		cp1, cp2, normal, restitution_c, 
		constraint_info_);
}

bool RigidBodyContactConstraint::solveVelocityConstraint(struct RigidBody * const* rbodies, const unsigned int iter) {

	RigidBody *rb1 = rbodies[bodies_[0]];
	RigidBody *rb2 = rbodies[bodies_[1]];

    vec3 corr_vel1, corr_vel2;
    vec3 corr_omega1, corr_omega2;

	bool rv = PBD::velocitySolveRigidBodyContactConstraint(
		rb1->inv_mass,
		rb1->pose.p,
		rb1->vel,
		rb1->inv_inertia_w,
		rb1->omega,
		rb2->inv_mass,
		rb2->pose.p,
		rb2->vel,
		rb2->inv_inertia_w,
		rb2->omega,
        stiffness_,
        friction_c_,
        sum_impulses_,
		constraint_info_,
        corr_vel1,
        corr_omega1,
        corr_vel2,
        corr_omega2);

    if(rv) {
        if(rb1->inv_mass!=0.0f) {
            rb1->vel += corr_vel1;
            rb1->omega += corr_omega1;
        }

        if(rb2->inv_mass!=0.0f) {
            rb2->vel += corr_vel2;
            rb2->omega += corr_omega2;
        }
    }

    return rv;
}

