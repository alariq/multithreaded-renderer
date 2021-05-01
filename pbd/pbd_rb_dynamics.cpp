#include "pbd_rb_dynamics.h"
#include "engine/utils/vec.h"
#include <cstring> // memset

static void computeMatrixK(const vec3 &connector, const float invMass, const vec3 &x,
						   const mat3 &inertiaInverseW, mat3 &K) {
	if (invMass != 0.0)
	{
		const vec3 v = connector - x;
		const float a = v[0];
		const float b = v[1];
		const float c = v[2];

		// J is symmetric
		const float j11 = inertiaInverseW[0][0];
		const float j12 = inertiaInverseW[0][1];
		const float j13 = inertiaInverseW[0][2];
		const float j22 = inertiaInverseW[1][1];
		const float j23 = inertiaInverseW[1][2];
		const float j33 = inertiaInverseW[2][2];
		K[0][0] = c*c*j22 - b*c*(j23 + j23) + b * b * j33 + invMass;
		K[0][1] = -(c*c*j12) + a*c*j23 + b * c * j13 - a * b * j33;
		K[0][2] = b*c*j12 - a*c*j22 - b * b * j13 + a * b * j23;
		K[1][0] = K[0][1];
		K[1][1] = c*c*j11 - a*c*(j13 + j13) + a*a*j33 + invMass;
		K[1][2] = -(b * c * j11) + a * c * j12 + a*b*j13 - a*a*j23;
		K[2][0] = K[0][2];
		K[2][1] = K[1][2];
		K[2][2] = b * b * j11 - a * b * (j12 + j12) + a * a * j22 + invMass;
	} else {
		memset(K[0], 0, sizeof(K));
	}
}

// this is a part of impulse magnitude calculation ( see e.g.
// https://www.scss.tcd.ie/~manzkem/CS7057/cs7057-1516-09-CollisionResponse-mm.pdf slide
// 28). Cross products are transformed to Hat operators so we have -ra^ * I-1 * ra^ 
// or GDC2006_Catto_Erin_PhysicsTutorial.pdf (search for K Matrix)
static void computeMatrixK(const vec3 &connector0, const vec3 &connector1,
						   const float invMass, const vec3 &x,
						   const mat3 &inertiaInverseW, mat3 &K) {
    if(invMass!=0.0f) {

		const vec3 v0 = connector0 - x;
		const float a = v0[0];
		const float b = v0[1];
		const float c = v0[2];

		const vec3 v1 = connector1 - x;
		const float d = v1[0];
		const float e = v1[1];
		const float f = v1[2];

		// J is symmetric
		const float j11 = inertiaInverseW[0][0];
		const float j12 = inertiaInverseW[0][1];
		const float j13 = inertiaInverseW[0][2];
		const float j22 = inertiaInverseW[1][1];
		const float j23 = inertiaInverseW[1][2];
		const float j33 = inertiaInverseW[2][2];

		K[0][0] = c * f * j22 - c * e * j23 - b * f * j23 + b * e * j33 + invMass;
		K[0][1] = -(c * f * j12) + c * d * j23 + b * f * j13 - b * d * j33;
		K[0][2] = c * e * j12 - c * d * j22 - b * e * j13 + b * d * j23;
		K[1][0] = -(c * f * j12) + c * e * j13 + a * f * j23 - a * e * j33;
		K[1][1] = c * f * j11 - c * d * j13 - a * f * j13 + a * d * j33 + invMass;
		K[1][2] = -(c * e * j11) + c * d * j12 + a * e * j13 - a * d * j23;
		K[2][0] = b * f * j12 - b * e * j13 - a * f * j22 + a * e * j23;
		K[2][1] = -(b * f * j11) + b * d * j13 + a * f * j12 - a * d * j23;
		K[2][2] = b * e * j11 - b * d * j12 - a * e * j12 + a * d * j22 + invMass;
	} else {
		memset(K[0], 0, sizeof(K));
    }
}

bool PBD::initRigidBodyContactConstraint(
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
	RigidBodyConstraintInfo &constraintInfo)
{
	// compute goal velocity in normal direction after collision
	const vec3 r0 = cp0 - x0;
	const vec3 r1 = cp1 - x1;

	const vec3 u0 = v0 + cross(omega0, r0);
	const vec3 u1 = v1 + cross(omega1, r1);
	const vec3 u_rel = u0 - u1;
	const float u_rel_n = dot(normal, u_rel);

	constraintInfo.body0_x = cp0;
	constraintInfo.body1_x = cp1;
	constraintInfo.body1_normal = normal;

	// tangent direction
	vec3 t = u_rel - u_rel_n*normal;
	float tl2 = lengthSqr(t);
	if (tl2 > 1.0e-6f)
		t *= 1.0f / sqrtf(tl2);
	constraintInfo.tangent = t;

	// determine K matrix
	mat3 K1, K2;
	computeMatrixK(cp0, invMass0, x0, inertiaInverseW0, K1);
	computeMatrixK(cp1, invMass1, x1, inertiaInverseW1, K2);
	mat3 K = K1 + K2;

	constraintInfo.nKn_inv = 1.0f / dot(normal, K * normal);

	// maximal impulse in tangent direction
	constraintInfo.p_max = 1.0f / dot(t, K * t) * dot(u_rel, t);

	// goal velocity in normal direction after collision
	constraintInfo.goal_u_rel_normal = 0.0;
	if (u_rel_n < 0.0)
		constraintInfo.goal_u_rel_normal = -restitutionCoeff * u_rel_n;

    // ok so why it returns bool?
	return true;
}

//--------------------------------------------------------------------------------------------
bool PBD::velocitySolveRigidBodyContactConstraint(
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
	vec3 &corr_v1, vec3 &corr_omega1)
{
	if ((invMass0 == 0.0) && (invMass1 == 0.0))
		return false;

	const vec3 &connector0 = constraintInfo.body0_x;
	const vec3 &connector1 = constraintInfo.body1_x;
	const vec3 &normal = constraintInfo.body1_normal;
	const vec3 &tangent = constraintInfo.tangent;

	// 1.0 / normal^T * K * normal
	const float nKn_inv = constraintInfo.nKn_inv;
	// maximal impulse in tangent direction
	const float pMax = constraintInfo.p_max;
	// goal velocity in normal direction after collision
	const float goal_u_rel_n = constraintInfo.goal_u_rel_normal;

	const vec3 r0 = connector0 - x0;
	const vec3 r1 = connector1 - x1;

	const vec3 u0 = v0 + cross(omega0, r0);
	const vec3 u1 = v1 + cross(omega1, r1);

	const vec3 u_rel = u0 - u1;
	const float u_rel_n = dot(u_rel, normal);
	const float delta_u_reln = goal_u_rel_n - u_rel_n;

	// j (as in
	// https://www.scss.tcd.ie/~manzkem/CS7057/cs7057-1516-09-CollisionResponse-mm.pdf
	// slide 28)
	float correctionMagnitude = nKn_inv * delta_u_reln;

	if (correctionMagnitude < -sum_impulses)
		correctionMagnitude = -sum_impulses;

	// penetration depth 
	const float d = dot(normal, connector0 - connector1);
	// add penalty impulse to counteract penetration
	if (d < 0.0)
		correctionMagnitude -= stiffness * nKn_inv * d;

	vec3 p(correctionMagnitude * normal);
	sum_impulses += correctionMagnitude;

	// dynamic friction
	const float pn = dot(p, normal);
	if (frictionCoeff * pn > pMax)
		p -= pMax * tangent;
	else if (frictionCoeff * pn < -pMax)
		p += pMax * tangent;
	else
		p -= frictionCoeff * pn * tangent;

    // same, slide 19, p = J = j*n

	if (invMass0 != 0.0)
	{
		corr_v0 = invMass0*p;
		corr_omega0 = inertiaInverseW0 * cross(r0, p);
	}

	if (invMass1 != 0.0)
	{
		corr_v1 = -invMass1*p;
		corr_omega1 = inertiaInverseW1 * cross(r1, -p);
	}

	return true;
}

