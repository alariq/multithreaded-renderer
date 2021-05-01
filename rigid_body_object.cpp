#include "rigid_body_object.h"
#include "obj_model.h"
#include "pbd/pbd.h"
#include "pbd/pbd_collision.h"



RigidBodyComponent* RigidBodyComponent::Create(PBDSimulation *sim, ICollisionDetection* cd, const vec3 &dim, bool b_invert) {

    RigidBodyComponent* c = new RigidBodyComponent();
    
    const vec3 box_size = dim;
    float density = 1000.0f;
    const float restitution = 0.15f;
    const float friction = 0.0f;

	RigidBody* rb = pbd_create_box_rigid_body(
		box_size, density, vec3(0), quaternion::identity(), restitution, friction);
	int rb_index = sim->addRigidBody(rb);

    ICollisionObject* collision = pbd_create_box_collision(rb_index, box_size);
    cd->addCollisionObject(collision);

    c->rigid_body_ = rb;
    c->collision_= collision;
    return c;
}

void RigidBodyComponent::Destroy(RigidBodyComponent* comp, PBDSimulation *sim, ICollisionDetection* cd)
{
    cd->removeCollisionObject(comp->collision_);
    sim->removeRigidBody(comp->rigid_body_);
    delete comp->collision_;
    delete comp->rigid_body_;
    delete comp;
}

void RigidBodyComponent::setKinematic(bool b_kinematic) {
    if(b_kinematic) {
        rigid_body_->inv_mass = 0.0f;
    } else {
        assert(0);
       // rigid_body_->inv_mass = rigid_body_->mass;
    }
}

void RigidBodyComponent::on_transformed() {
	if (!b_self_update_) {
        Pose p;
        p.p = GetPosition();
        p.q = GetRotation();
		rigid_body_->resetPose(p);
	}
}

void RigidBodyComponent::on_transformed(TransformComponent* c) {
    assert(c->GetType() == ComponentType::kRigidBody);
    RigidBodyComponent* comp = (RigidBodyComponent*)c;
    comp->on_transformed();
}

void RigidBodyComponent::UpdateComponent(float dt) {
    Pose pose = rigid_body_->pose;
    b_self_update_ = true;
    SetPosition(pose.p);
    SetRotation(pose.q);
    b_self_update_ = false;
}

RigidBodyObject* RigidBodyObject::Create(const vec3 &dim)
{
    PBDSimulation* sim = pbd_get_simulation();
    ICollisionDetection* cd = pbd_get_collision_detection(); 

    RigidBodyObject* rbo = new RigidBodyObject();
    rbo->Tuple_.mesh_ = MeshComponent::Create("cube");
    rbo->Tuple_.mesh_->SetScale(0.5f*dim);
    bool b_invert = false;
    rbo->Tuple_.rb_ = RigidBodyComponent::Create(sim, cd, dim, b_invert);

    rbo->Tuple_.tr_ = rbo->AddComponent<TransformComponent>();
    MeshComponent* mesh_comp = rbo->AddComponent(rbo->Tuple_.mesh_);
    /*RigidBodyComponent* rb_comp = */rbo->AddComponent(rbo->Tuple_.rb_);
    mesh_comp->SetParent(rbo->Tuple_.tr_);
    //rb_comp->SetParent(rbo->Tuple_.tr_);
    rbo->Tuple_.tr_->on_transformed_fptr_ = on_transformed;
    rbo->Tuple_.rb_->on_transformed_fptr_ = RigidBodyComponent::on_transformed;

    return rbo;
}

void RigidBodyObject::SetTransform(const vec3& pos, const quaternion& rot) {
    Tuple_.tr_->SetPosition(pos);
    Tuple_.tr_->SetRotation(rot);
}

RigidBodyObject::~RigidBodyObject()
{
    delete RemoveComponent(Tuple_.mesh_);
    
    auto c = RemoveComponent(Tuple_.rb_);
	// not very good to have those functions to return pointers to singletons as we might
	// want to  have e.g. several simulations, better to destroy all components in a scene and scene will provide sim & cd
    // or even better have several physics scenes (islands)
	RigidBodyComponent::Destroy(c, pbd_get_simulation(), pbd_get_collision_detection());
}

void RigidBodyObject::setKinematic(bool b_kinematic) {
    Tuple_.rb_->setKinematic(b_kinematic);
}

void RigidBodyObject::on_transformed(TransformComponent* c) {
    GameObject* go = getGameObject(c->getGameObjectHandle());
    RigidBodyObject* rbo = (RigidBodyObject*)go;
    rbo->Tuple_.rb_->SetPosition(c->GetPosition());
    rbo->Tuple_.rb_->SetRotation(c->GetRotation());
}

void RigidBodyObject::Update(float dt) {

/*
    vec3 pos = Tuple_.tr_->GetPosition();
    quaternion rot = Tuple_.tr_->GetRotation();

    Tuple_.bm_->getBoundary()->setTransform(pos, quat_to_mat3(rot));
*/

    Pose pose = Tuple_.rb_->getPose();
    Tuple_.tr_->on_transformed_fptr_ = TransformComponent::on_transformed_default;
    Tuple_.tr_->SetPosition(pose.p);
    Tuple_.tr_->SetRotation(pose.q);
    Tuple_.tr_->on_transformed_fptr_ = on_transformed;

}

