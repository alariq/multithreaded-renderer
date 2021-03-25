#pragma once

#include "pbd/pbd_rb_dynamics.h" // RigidBodyConstraintInfo 

#include "engine/utils/quaternion.h"
#include "engine/utils/vec.h"

struct RigidBody;

class RigidBodyContactConstraint 
{
        static int TYPE_ID;
    public:
        /** indices of the linked bodies */
        int bodies_[2];
        float stiffness_; 
        float friction_c_;
        float sum_impulses_;
        RigidBodyConstraintInfo constraint_info_;

        RigidBodyContactConstraint() = default;
        ~RigidBodyContactConstraint() = default;
        virtual int &getTypeId() const { return TYPE_ID; }

		bool initConstraint(struct RigidBody **rbodies, const int bod_index1,
							const int bod_index2, const vec3 &cp1, const vec3 &cp2,
							const vec3 &normal, const float dist,
							const float restitution_c, const float stiffness,
							const float friction_c);

		bool solveVelocityConstraint(struct RigidBody* const* rbodies, const unsigned int iter);
};
