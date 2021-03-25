#include "pbd_collision.h"
#include "pbd.h"

#include <algorithm>



vec3 DistanceFieldCollisionObject::approximateNormal(const vec3&p, const float tolerance)
{
	// approximate gradient
	float eps = 1.e-6f;
    vec3 n;

    float e_p = distance(p + vec3(eps, 0, 0), tolerance);
    float e_m = distance(p - vec3(eps, 0, 0), tolerance);
    n.x = (e_p - e_m) * (1.0f / (2.0f * eps));

    e_p = distance(p + vec3(0, eps, 0), tolerance);
    e_m = distance(p - vec3(0, eps, 0), tolerance);
    n.y = (e_p - e_m) * (1.0f / (2.0f * eps));

    e_p = distance(p + vec3(0, 0, eps), tolerance);
    e_m = distance(p - vec3(0, 0, eps), tolerance);
    n.z = (e_p - e_m) * (1.0f / (2.0f * eps));


	const float norm2 = lengthSqr(n);
	if (norm2 < eps) {
		n = vec3(0.0f);
    } else {
		n = n / sqrtf(norm2);
    }

    return n;
}

bool DistanceFieldCollisionObject::collisionTest(const vec3 &x, const float tolerance,
												 vec3* cp, vec3* n, float* dist,
												 const float max_dist /* = 0.0f*/) {

	*dist = distance(x, tolerance);
	if (*dist < max_dist)
	{
		// approximate gradient
		*n = approximateNormal(x, tolerance);

		*cp = (x - (*dist) * (*n));
		return true;
	}
	return false;
}

////////////////////////////////////////////////////////////////////////////////

void DistanceFieldCollisionDetection::updateAABB(class PBDSimulation* model, ICollisionObject *co)
{
	const RigidBody* const* rigidBodies = model->getRigidBodies();
	if (co->body_type_ == ICollisionObject::RigidBodyCollisionObjectType)
	{
		const unsigned int rbIndex = co->body_index_;
		const RigidBody *rb = rigidBodies[rbIndex];
		const std::vector<vec3>& vd = rb->vertices_;

		co->aabb_.min_ = vd[0];
		co->aabb_.max_ = vd[0];
		for (auto p: vd) {
			aabb_update(co->aabb_, p);
		}
	}
	else 
	{
        assert(0 && "unsupported collision object type");
	}

	aabb_grow(co->aabb_, tolerance_);
}


void DistanceFieldCollisionDetection::removeCollisionObject(struct ICollisionObject* o_to_del) {
    // TODO: get rid of those arrays and move to pool-allocaror like strategy
    // nobody references us by index, so it is safe to delete like this
	auto b = std::begin(co_);
	auto e = std::end(co_);
	co_.erase(
		std::remove_if(b, e, [o_to_del](ICollisionObject* o) { return o == o_to_del; }), e);
}

void DistanceFieldCollisionDetection::setRigidBodyContactCallback(
	ICollisionDetection::RBContactCallback_fptr f, ContactType contact_type, void *user_data) {

    (void)contact_type; // for now same for all
    rb_contact_cb_ = f;
    rb_contact_cb_user_data = user_data;
}

void DistanceFieldCollisionDetection::collisionRigidBodies(
	RigidBody *rb1, DistanceFieldCollisionObject *co1, RigidBody *rb2,
	DistanceFieldCollisionObject *co2, float restitution, float friction,
	std::vector<DistanceFieldCollisionDetection::ContactData>* cd_array) {

	if (rb1->inv_mass == 0.0f && rb2->inv_mass == 0.0f)
		return;

    for(const auto& p: rb1->vertices_) {
        // transform to local space of rb2
        const vec3& pw = p;//rb1->pose.transform(p);
        const vec3& pl = rb2->pose.invTransform(pw);

        vec3 cp, n;
        float dist;

        if(co2->collisionTest(pl, tolerance_, &cp, &n, &dist)) {

            const vec3& cp_w = rb2->pose.transform(cp);
            const vec3& nw = rb2->pose.rotate(n);

            // generate contact
            ContactData cd = {
                .type_ = ContactType::kRigidBody,
                .index1_ = co1->body_index_,
                .index2_ = co2->body_index_,
		        .cp1_ = pw,
                .cp2_ = cp_w,
		        .normal_ = nw,
            	.dist_ = dist,
                .restitution_ = restitution,
                .friction_ = friction
            };

            cd_array->push_back(cd);
        }
    }
}

void DistanceFieldCollisionDetection::collisionDetection(PBDSimulation *sim) {

    sim->clearRigidBodyContacts();

	// very basic scheme, just add all posible pairs
	std::vector<std::pair<unsigned int, unsigned int>> co_pairs;
	for (int i = 0; i < (int)co_.size(); i++) {
		// ICollisionObject *co1 = co_[i];
		for (int k = 0; k < (int)co_.size(); k++) {
			// ICollisionObject *co2 = co_[k];
			if (i != k) {
				// ToDo: self collisions for deformables
				co_pairs.push_back({i, k});
			}
		}
	}

	for (ICollisionObject* co: co_) {
        updateAABB(sim, co);
    }

    std::vector<ContactData> contacts;

	RigidBody *const *rbs = sim->getRigidBodies();
	for (const auto &couple : co_pairs) {
        ICollisionObject* co1 = co_[couple.first];
        ICollisionObject* co2 = co_[couple.second];

        // for now
		assert(co1->body_type_ == co2->body_type_ &&
			   co1->body_type_ == ICollisionObject::RigidBodyCollisionObjectType);

        auto df_co1 = (DistanceFieldCollisionObject*)co1;
        auto df_co2 = (DistanceFieldCollisionObject*)co2;


        if(!aabb_intersect(co1->aabb_, co2->aabb_))
            continue;

		RigidBody *rb1 = rbs[co1->body_index_];
		RigidBody *rb2 = rbs[co2->body_index_];

		const float restitution_c = rb1->restitution_c * rb2->restitution_c;
		const float friction_c = rb1->friction_c + rb2->friction_c;
    
        collisionRigidBodies(rb1, df_co1, rb2, df_co2, restitution_c, friction_c, &contacts);
	}


    for(const auto& c: contacts) {
        if(ContactType::kRigidBody == c.type_) {
            // Just pass ContactData?
			rb_contact_cb_(ContactType::kRigidBody, c.index1_, c.index2_, c.cp1_, c.cp2_,
						   c.normal_, c.dist_, c.restitution_, c.friction_,
						   rb_contact_cb_user_data);
		} else {
            assert(0 && "not supported yet");
        }
    }

}
