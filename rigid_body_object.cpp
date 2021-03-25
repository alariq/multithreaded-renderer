#include "rigid_body_object.h"
#include "obj_model.h"
#include "pbd/pbd.h"
#include "pbd/pbd_collision.h"



RigidBodyComponent* RigidBodyComponent::Create(PBDSimulation *sim, ICollisionDetection* cd, const vec3 &dim, bool b_invert) {

    RigidBodyComponent* c = new RigidBodyComponent();
    
    const vec3 box_size = dim;

	RigidBody *rb_cube = new RigidBody();
	rb_cube->setBox(box_size, 1000.0f);
	rb_cube->pose.p = vec3(0, 4, 0);
	rb_cube->pose.q = quaternion::identity();
	rb_cube->prev_pose.p = vec3(0, 4, 0);
	rb_cube->prev_pose.q = quaternion::identity();
	rb_cube->old_pose.p = vec3(0, 4, 0);
	rb_cube->old_pose.q = quaternion::identity();
    rb_cube->restitution_c = 0.15f;
    rb_cube->friction_c = 0.0f;
    rb_cube->omega = vec3(0,0,0);


	int rb_cube_index = sim->addRigidBody(rb_cube);

	// maybe return object handle upon addition to CollisionDetection ?
	DistanceFieldCollisionObjectBox *co_box = new DistanceFieldCollisionObjectBox();
	co_box->box_ = 0.5f*box_size;
	co_box->enable_ = true;
	co_box->invert_ = false;
	co_box->aabb_ = AABB(-0.5f*box_size, 0.5f*box_size);
	co_box->body_type_ = ICollisionObject::RigidBodyCollisionObjectType;
	co_box->body_index_ = rb_cube_index;
    cd->addCollisionObject(co_box);

    c->rigid_body_ = rb_cube;
    c->collision_= co_box;


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

void RigidBodyComponent::on_transformed() {
    rigid_body_->pose.p = GetPosition();
    rigid_body_->pose.q = GetRotation();

    rigid_body_->orig_pose.p = GetPosition();
    rigid_body_->orig_pose.q = GetRotation();
    rigid_body_->prev_pose.p = GetPosition();
    rigid_body_->prev_pose.q = GetRotation();
    rigid_body_->old_pose.p = GetPosition();
    rigid_body_->old_pose.q = GetRotation();
}

void RigidBodyComponent::setKinematic(bool b_kinematic) {
    if(b_kinematic) {
        rigid_body_->inv_mass = 0.0f;
    } else {
        assert(0);
       // rigid_body_->inv_mass = rigid_body_->mass;
    }
}


void RigidBodyComponent::UpdateComponent(float dt) {
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
    /*RigidBodyComponent* rb_comp =*/ rbo->AddComponent(rbo->Tuple_.rb_);
    mesh_comp->SetParent(rbo->Tuple_.tr_);
    //rb_comp->SetParent(rbo->Tuple_.tr_);
    return rbo;
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

void RigidBodyObject::Update(float dt) {

/*
    vec3 pos = Tuple_.tr_->GetPosition();
    quaternion rot = Tuple_.tr_->GetRotation();

    Tuple_.bm_->getBoundary()->setTransform(pos, quat_to_mat3(rot));
*/

    Pose pose = Tuple_.rb_->getPose();
    Tuple_.tr_->SetPosition(pose.p);
    Tuple_.tr_->SetRotation(pose.q);
}

