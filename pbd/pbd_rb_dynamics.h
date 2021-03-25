#pragma once
#include "engine/utils/quaternion.h"
#include "engine/utils/vec.h"

struct RigidBodyConstraintInfo {
	// contact point in body 0 (global)
    vec3 body0_x;
	// contact point in body 1 (global)
    vec3 body1_x;
	// contact normal in body 1 (global)
    vec3 body1_normal;
	// contact tangent (global)
    vec3 tangent;
	// 1.0 / normal^T * K * normal
    float nKn_inv;
	// maximal impulse in tangent direction
    float p_max;
	// goal velocity in normal direction after collision
    float goal_u_rel_normal;
};

struct PBD {

		/** Initialize contact between two rigid bodies and return 
		* info which is required by the solver step.
		*
		* @param invMass0 inverse mass of rigid body
		* @param x0 center of mass of first body
		* @param v0 velocity of body 0
		* @param inertiaInverseW0 inverse inertia tensor in world coordinates of first body
		* @param q0 rotation of first body
		* @param omega0 angular velocity of first body
		* @param invMass1 inverse mass of second body
		* @param x1 center of mass of second body
		* @param v1 velocity of body 1
		* @param inertiaInverseW1 inverse inertia tensor in world coordinates of second body
		* @param q1 rotation of second body
		* @param omega1 angular velocity of second body
		* @param cp0 contact point of body 0
		* @param cp1 contact point of body 1
		* @param normal contact normal in body 1
		* @param restitutionCoeff coefficient of restitution
		* @param constraintInfo Stores the local and global position of the contact points and
		* the contact normal. \n
		* The joint info contains the following columns:\n
		* 0:	connector in body 0 (global)\n
		* 1:	connector in body 1 (global)\n
		* 2:	contact normal in body 1 (global)\n
		* 3:	contact tangent (global)\n
		* 0,4:   1.0 / normal^T * K * normal\n
		* 1,4:  maximal impulse in tangent direction\n
		* 2,4:  goal velocity in normal direction after collision
		*/
		static bool initRigidBodyContactConstraint(
			const float invMass0,							// inverse mass is zero if body is static
			const vec3 &x0,						// center of mass of body 0
			const vec3 &v0,						// velocity of body 0
			const mat3 &inertiaInverseW0,		// inverse inertia tensor (world space) of body 0
			const quaternion &q0,					// rotation of body 0	
			const vec3 &omega0,					// angular velocity of body 0
			const float invMass1,							// inverse mass is zero if body is static
			const vec3 &x1,						// center of mass of body 1
			const vec3 &v1,						// velocity of body 1
			const mat3 &inertiaInverseW1,		// inverse inertia tensor (world space) of body 1
			const quaternion &q1,					// rotation of body 1
			const vec3 &omega1,					// angular velocity of body 1
			const vec3 &cp0,						// contact point of body 0
			const vec3 &cp1,						// contact point of body 1
			const vec3 &normal,					// contact normal in body 1
			const float restitutionCoeff,					// coefficient of restitution
			RigidBodyConstraintInfo &constraintInfo);


		/** Perform a solver step for a contact constraint between two rigid bodies.
		* A contact constraint handles collisions and resting contacts between the bodies.
		* The contact info must be generated in each time step.
		*
		* @param invMass0 inverse mass of rigid body
		* @param x0 center of mass of first body
		* @param v0 velocity of body 0
		* @param inertiaInverseW0 inverse inertia tensor in world coordinates of first body
		* @param omega0 angular velocity of first body
		* @param invMass1 inverse mass of second body
		* @param x1 center of mass of second body
		* @param v1 velocity of body 1
		* @param inertiaInverseW1 inverse inertia tensor in world coordinates of second body
		* @param omega1 angular velocity of second body
		* @param stiffness stiffness parameter of penalty impulse
		* @param frictionCoeff friction coefficient
		* @param sum_impulses sum of all correction impulses in normal direction
		* @param constraintInfo information which is required by the solver. This
		* information must be generated in the beginning by calling init_RigidBodyContactConstraint().
		* @param corr_v0 velocity correction of first body
		* @param corr_omega0 angular velocity correction of first body
		* @param corr_v1 velocity correction of second body
		* @param corr_omega1 angular velocity correction of second body
		*/
		static bool velocitySolveRigidBodyContactConstraint(
			const float invMass0,							// inverse mass is zero if body is static
			const vec3 &x0, 						// center of mass of body 0
			const vec3 &v0,						// velocity of body 0
			const mat3 &inertiaInverseW0,		// inverse inertia tensor (world space) of body 0
			const vec3 &omega0,					// angular velocity of body 0
			const float invMass1,							// inverse mass is zero if body is static
			const vec3 &x1, 						// center of mass of body 1
			const vec3 &v1,						// velocity of body 1
			const mat3 &inertiaInverseW1,		// inverse inertia tensor (world space) of body 1
			const vec3 &omega1,					// angular velocity of body 1
			const float stiffness,							// stiffness parameter of penalty impulse
			const float frictionCoeff,						// friction coefficient
			float &sum_impulses,							// sum of all impulses
			RigidBodyConstraintInfo &constraintInfo,		// precomputed contact info
			vec3 &corr_v0, vec3 &corr_omega0,
			vec3 &corr_v1, vec3 &corr_omega1);
};

