#pragma once

#include "pbd_constraints.h"

#include "engine/utils/quaternion.h"
#include "engine/utils/vec.h"
#include <vector>
#include <cstdio>

struct Pose {
    quaternion q;
    vec3 p;

    vec3 rotate(const vec3& v) const {
        return quat_rotate(q, v);
    }

    vec3 invRotate(const vec3& v) const {
        return quat_inv_rotate(q, v);
    }

    vec3 transform(const vec3& v) const {
        return quat_rotate(q, v) + p;
    }

    vec3 invTransform(const vec3& v) const {
        return quat_inv_rotate(q, v - p);
    }

    Pose transformPose(const Pose& pose) const {
        vec3 t = p + quat_rotate(q, pose.p);
		return {q * pose.q, t};
	}

};


struct RigidBody {

    const constexpr static float maxRotationPerSubstep = 0.5f;

    Pose orig_pose; // does not belong here
    Pose pose;
    Pose prev_pose;
    Pose old_pose;

    float inv_mass;
    vec3 inv_inertia;
    mat3 inv_inertia_w;
    vec3 vel;
    vec3 omega; // angular velocity

    float restitution_c;
    float friction_c;

    std::vector<vec3> local_vertices_;
    std::vector<vec3> vertices_;

	void setBox(const vec3 &size, float density) {
		float mass = size.x * size.y * size.z * density;
		inv_mass = 1.0 / mass;
		mass /= 12.0;
		inv_inertia = vec3(1.0 / (size.y * size.y + size.z * size.z) / mass,
						   1.0 / (size.z * size.z + size.x * size.x) / mass,
						   1.0 / (size.x * size.x + size.y * size.y) / mass);
    
        int i=0;
        const vec3 scale = 0.5f * size;
        vertices_.resize(8);
        vertices_[i++] = vec3(1,1,1)*scale;
        vertices_[i++] = vec3(-1,1,1)*scale;
        vertices_[i++] = vec3(1,-1,1)*scale;
        vertices_[i++] = vec3(1,1,-1)*scale;
        vertices_[i++] = vec3(1,-1,-1)*scale;
        vertices_[i++] = vec3(-1,1,-1)*scale;
        vertices_[i++] = vec3(-1,-1,1)*scale;
        vertices_[i++] = vec3(-1,-1,-1)*scale;
        local_vertices_ = vertices_;
	}

    void resetPose(const Pose& p) {
        pose = p;
        old_pose = p;
        prev_pose = p;
        orig_pose = p;
		vel = vec3(0);
		omega = vec3(0);
        onUpdateRotation();
    }

    void onUpdateRotation() {
        const mat3 rot_m = quat_to_mat3(pose.q);

        // this should be in unit tests
        const quaternion q2 = mat3_to_quat(rot_m);
        // second condition necessary because -q and q represent the same rotations
        assert(lengthSqr(pose.q - q2) < 0.001f || lengthSqr(pose.q + q2) < 0.001f);

        inv_inertia_w = rot_m * diag(inv_inertia) * transpose(rot_m);
    }


    quaternion applyRotation(const vec3& ang_vel, float dt) const {
        // safety clamping. This happens very rarely if the solver
        // wants to turn the body by more than 30 degrees in the
        // orders of milliseconds

        const float phi = length(ang_vel);
        if (phi * dt > maxRotationPerSubstep) 
            dt = maxRotationPerSubstep / phi;
        
        // A Survey on Position Based Dynamics, 2017 Section 3.1, 3.2
        // https://github.com/matthias-research/pages/blob/master/challenges/PBD.js
        
        quaternion dq = quaternion(ang_vel.x, ang_vel.y, ang_vel.z, 0.0f);
        quaternion rot  = pose.q +
            0.5f * dt * dq * pose.q; // (eq. 4)
#if 0
        float len = dq.length();
        quaternion W_t = dq;
        quaternion q_t = quaternion(vec3(1), W_t.length()*dt);
        q_t.x *= W_t.x / len;
        q_t.y *= W_t.y / len;
        q_t.z *= W_t.z / len;
        quaternion d_q = (0.5f * W_t * q_t);
        quaternion rot = pose.q + d_q;
#endif
        return normalize(rot);

    }

	// imtegrate by symplectic Euler 3.2
	// TODO: move gravity to application of all forces
	void integrate(float dt, const vec3 &gravity) {
		if (inv_mass != 0.0f) {
			vel += dt * gravity; // predicted location
			pose.p = pose.p + vel * dt;

			// omega += dt * inv_inertia * torque;

			pose.q = applyRotation(omega, dt);
		}
	}

	// after constrain & collision resolve
	void update(float dt) {
		if (inv_mass != 0.0f) {
			// we are PBD
			vel = (pose.p - prev_pose.p) / dt;
			// eq. 4 solved for omega
			quaternion dq = pose.q * conjugate(prev_pose.q);
			omega = (2.0f / dt) * dq.imag();
			if (dq.w < 0.0f) omega = -omega;

			// omega *= (1.0 - 1.0 * dt);
			// vel *= (1.0 - 1.0 * dt);

			// here we can update mesh by our pose
		}
	}

	// get relative velocity to thjis object from point "pos"?
    vec3 getVelocityAt(const vec3& pos) {					
        vec3 r = pos - pose.p;
        vec3 v = cross(r, omega);
        // why minus and not + ?
        return v - vel;
    }

    float getInverseMass(const vec3& normal) {
        vec3 n = pose.invRotate(normal);

        // Inverse Mass Matrix https://cseweb.ucsd.edu/classes/sp16/cse169-a/slides/CSE169_17.pdf
        float w = 
            n.x * n.x * inv_inertia.x +
            n.y * n.y * inv_inertia.y +
            n.z * n.z * inv_inertia.z;

        return w;
    }

    float getInverseMass(const vec3& normal, const vec3& pos) {
        vec3 n = pos - pose.p;
        n = cross(n, normal);
        n = pose.invRotate(n);

        // Inverse Mass Matrix https://cseweb.ucsd.edu/classes/sp16/cse169-a/slides/CSE169_17.pdf
        // M^-1 = M - r * I^-1 * r
        // M^-1 = M - w;
        float w = 
            n.x * n.x * inv_inertia.x +
            n.y * n.y * inv_inertia.y +
            n.z * n.z * inv_inertia.z;
            
        // not sure why we do not have minus here
        return inv_mass + w;
    }

};

enum class ContactType: int {
    kRigidBody = 0,
    kParticleRigidBody,
    kCount
};

class ICollisionDetection {
	

    public:
	  typedef void (*RBContactCallback_fptr)(const ContactType contact_type,
										   const int body_index1,
										   const int body_index2,
										   const vec3 &cp1, const vec3 &cp2,
										   const vec3 &normal, const float dist,
										   const float restitution_c,
										   const float friction_c, void *userData);

      virtual void setTolerance(float tolerance) = 0;
	  virtual void collisionDetection(class PBDSimulation *sim) = 0;
	  virtual void addCollisionObject(struct ICollisionObject *o) = 0;
	  virtual void removeCollisionObject(struct ICollisionObject *o) = 0;
	  virtual void setRigidBodyContactCallback(RBContactCallback_fptr f, ContactType contact_type, void* user_data) = 0;
      virtual ~ICollisionDetection() {}
};

class PBDSimulation {

    float contact_stiffness_rb_ = 0.9f;

    std::vector<RigidBody*> rb_;
    std::vector<RigidBodyContactConstraint> rb_contacts_;

    public:
        RigidBody** getRigidBodies() { return rb_.data(); }
        RigidBody* const* getRigidBodies() const { return rb_.data(); }
		int getNumRigidBodies() const { return (int)rb_.size(); }
        int addRigidBody(RigidBody* rb) { rb_.push_back(rb); return (int)rb_.size()-1; }
        void removeRigidBody(RigidBody* rb); 

		bool addRigidBodyContactConstraint(const int body_index1, const int body_index2,
										   const vec3 &cp1, const vec3 &cp2,
										   const vec3 &normal, const float dist,
										   const float restitution_c,
										   const float friction_c);
		std::vector<RigidBodyContactConstraint> &getRigidBodyContactConstraints() {
			return rb_contacts_;
		}
        void clearRigidBodyContacts() {
            rb_contacts_.clear();
        }

};

class PBDSolver {

    static const constexpr vec3 gravity = vec3(0.0f, -9.81f, 0.0f);
    static const constexpr int num_iter = 5;
    static const constexpr int num_iter_vel = 5;

    ICollisionDetection* cd_;

	static void contactCallbackFunction(const ContactType contact_type,
										const int body_index1, const int body_index2,
										const vec3 &cp1, const vec3 &cp2,
										const vec3 &normal, const float dist,
										const float restitution_c, const float friction_c,
										void *userData);

    void velocityConstraintProjection(PBDSimulation* sim, int iter);
    void updateTransforms(PBDSimulation* sim);
  public:
	void setCollisionDetection(ICollisionDetection *cd, PBDSimulation *sim);
	void simulate(PBDSimulation *sim, float dt);
};

PBDSimulation* pbd_get_simulation();
ICollisionDetection* pbd_get_collision_detection();

void pbd_create_simulation();
void pbd_create_collision_detection();

void pbd_destroy_simulation();
void pbd_destroy_collision_detection();

void pbd_simulate(float dt);

