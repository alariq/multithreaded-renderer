#include "obj_model.h"
#include "pbd/pbd.h"


class RigidBodyComponent : public TransformComponent {
	struct RigidBody* rigid_body_ = nullptr;
    struct ICollisionObject* collision_ = nullptr; 
    bool b_self_update_ = false;
    virtual ~RigidBodyComponent() {};
    void on_transformed();
public:
	static const ComponentType type_ = ComponentType::kRigidBody;
	virtual ComponentType GetType() const override { return type_; }
    RigidBodyComponent() {};

    const Pose& getPose() const { return rigid_body_->pose; }
    static void on_transformed(TransformComponent* );
    void setKinematic(bool b_kinematic);


    virtual void UpdateComponent(float dt) override;

    static RigidBodyComponent* Create(class PBDSimulation *sim, ICollisionDetection* cd, const vec3 &dim, bool b_invert);
    static void Destroy(RigidBodyComponent* comp, PBDSimulation *sim, ICollisionDetection* cd);
};

class RigidBodyObject: public GameObject {

    struct RigidBodyTuple {
        TransformComponent* tr_;
        RigidBodyComponent* rb_;
        MeshComponent* mesh_;
    };
    RigidBodyTuple Tuple_;
    static void on_transformed(TransformComponent*);
public:
    static RigidBodyObject* Create(const vec3 &dim);
    RigidBodyObject() = default;
    ~RigidBodyObject();

    // IEditorObject
    virtual int GetIconID() const override { return 2; }
    //

	virtual void InitRenderResources() override {}
    virtual void DeinitRenderResources() override {};

    void addToSimulation(PBDSimulation* sim);
    void removeFromSimulation(PBDSimulation* sim);
    void setKinematic(bool);

    virtual const char* GetName() const override { return "rigid body"; };
    virtual void Update(float dt) override;
    virtual RenderMesh* GetMesh() const override { return nullptr; }

    void SetTransform(const vec3& pos, const quaternion& rot); 

};

