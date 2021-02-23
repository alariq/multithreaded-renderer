#pragma once

#include "pbd.h"

#include "engine/utils/intersection.h" // aabb
#include "engine/utils/quaternion.h"
#include "engine/utils/vec.h"

#include <vector>
#include <utility>

// TODO: just use increasing int index ?
enum CollisionObjectType: int {
    kDistanceFieldBox = 0,
    kDistanceFieldSphere
};

// TODO: move to common place
inline float distance_box(const vec3& box, const vec3 &p, const float invert, const float tolerance)
{
	const vec3 d = vec3(abs(p) - box);
	const vec3 max_d = max(d, vec3(0.0f));
	return invert*(min(max(d.x, max(d.y, d.z)), 0.0f) + length(max_d)) - tolerance;
}

inline float distance_sphere(const float radius, const vec3& p, const float invert, const float tolerance)
{
	return invert*(length(p) - radius) - tolerance;
}


struct ICollisionObject {

	static const unsigned int RigidBodyCollisionObjectType = 0;
	static const unsigned int TriangleModelCollisionObjectType = 1;
	static const unsigned int TetModelCollisionObjectType = 2;

	AABB aabb_;
	int body_index_;
	int body_type_;

	virtual int getTypeId() = 0;
    virtual ~ICollisionObject() {}
};

struct DistanceFieldCollisionObject : public ICollisionObject {
    float invert_;
    bool enable_;

    virtual bool collisionTest(const vec3& x, const float tolerance, vec3* cp, vec3* n, float* dist, const float max_dist = 0.0f);
    virtual vec3 approximateNormal(const vec3 &x, const float tolerance);

    virtual float distance(const vec3 &x, const float tolerance) = 0;
};

struct DistanceFieldCollisionObjectBox : public DistanceFieldCollisionObject {
    static const CollisionObjectType type_ = kDistanceFieldBox;
    vec3 box_;

	virtual int getTypeId() override { return type_; }
    virtual float distance(const vec3 &x, const float tolerance) override {
        return distance_box(box_, x, invert_?-1.0f : 1.0f, tolerance);
    }
};

struct DistanceFieldCollisionObjectSphere : public DistanceFieldCollisionObject {
    static const CollisionObjectType type_ = kDistanceFieldSphere;
    float radius_;

	virtual int getTypeId() override { return type_; }
    virtual float distance(const vec3 &x, const float tolerance) override {
        return distance_sphere(radius_, x, invert_?-1.0f : 1.0f, tolerance);
    }
};


class DistanceFieldCollisionDetection:  public ICollisionDetection {

	struct ContactData {
		ContactType type_;
		int index1_;
		int index2_;
		vec3 cp1_;
		vec3 cp2_;
		vec3 normal_;
		float dist_;
		float restitution_;
		float friction_;

		// Test
		int elementIndex1_;
		int elementIndex2_;
		vec3 bary1_;
		vec3 bary2_;
	};

    float tolerance_ = 0.04f;

	// ok, keep it like this for now, later split by type and code type in index
    std::vector<ICollisionObject*> co_;

	RBContactCallback_fptr rb_contact_cb_ = nullptr;
    void* rb_contact_cb_user_data = nullptr;

		void
		collisionRigidBodies(
			RigidBody *rb1, DistanceFieldCollisionObject *co1, RigidBody *rb2,
			DistanceFieldCollisionObject *co2, float restitution, float friction,
			std::vector<DistanceFieldCollisionDetection::ContactData> *cd);

        void updateAABB(class PBDSimulation* model, ICollisionObject *co);
  public:
    virtual void setTolerance(float tolerance) override { tolerance_ = tolerance; }
    virtual void collisionDetection(PBDSimulation* sim) override;
    virtual void addCollisionObject(struct ICollisionObject* o) override { co_.push_back(o); }
    virtual void removeCollisionObject(struct ICollisionObject* o) override;

	virtual void setRigidBodyContactCallback(RBContactCallback_fptr f, ContactType contact_type, void* user_data) override;

};

