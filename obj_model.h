#pragma once

#include "utils/vec.h"
#include "utils/quaternion.h"
#include "utils/frustum.h"
#include "scene.h"

#include <cstdint>
#include <algorithm>
#include <functional>
#include <vector>
#include <atomic>

struct RenderMesh;
struct camera;
class ParticleSystem;

class IRenderable {
public:
    virtual void InitRenderResources() = 0;
    virtual void AddRenderPackets(struct RenderFrameContext* ) const {};
    virtual ~IRenderable() {}
};

enum class ComponentType {
    kUnknown,
    kTransform,
};

class Component {
    public:
    virtual ComponentType GetType() const = 0;
};

class TransformComponent: public Component {
    vec3 scale_;
    vec3 wscale_;
    quaternion rot_;
    vec3 pos_;
public:
	static const ComponentType type_ = ComponentType::kTransform;
    TransformComponent(): scale_(1), wscale_(1), rot_(quaternion::identity()), pos_(0) {}

    virtual ComponentType GetType() const { return type_; }

    mat4 GetTransform() const {
        return translate(pos_) *scale4(wscale_.x, wscale_.y, wscale_.z)* quat_to_mat4(rot_) *
               scale4(scale_.x, scale_.y, scale_.z);
    }

    vec3 GetPosition() const { return pos_; }
    quaternion GetRotation() const { return rot_; }
    vec3 GetScale() const { return scale_; }
    vec3 GetWorldScale() const { return wscale_; }

    void SetPosition(const vec3& pos) { pos_ = pos; }
    void SetRotation(const vec3& rot)  { rot_ = euler_to_quat(rot.x, rot.y, rot.z); }
    void SetRotation(const quaternion& q)  { rot_ = q; }
    void SetScale(const vec3& scale) { scale_ = scale; }
    void SetWorldScale(const vec3& wscale) { wscale_ = wscale; }
};

typedef uint32_t GameObjectId;
class GameObject: public IRenderable {
    std::vector<Component*> components_;
	GameObjectId id_;
public:
	GameObjectId GetId() const { return id_; }

    virtual const char* GetName() const = 0;
    virtual void Update(float dt) = 0;
    virtual RenderMesh* GetMesh() const = 0;

    Component* GetComponent(ComponentType type) const {
        auto cmp = std::find_if(
            components_.begin(), components_.end(),
            [type](Component *cmp) { return cmp->GetType() == type; });
        return cmp!=components_.end() ? *cmp : nullptr;
    }

    template<typename T>
    T* GetComponent() const {
        auto cmp = std::find_if(
            components_.begin(), components_.end(),
            [](Component *cmp) { return cmp->GetType() == T::type_; });
        return cmp!=components_.end() ? (T*)*cmp : nullptr;
    }

    template<typename T>
    T* AddComponent() {
        T* comp = new T();
        components_.push_back(comp);
        return comp;
    }

	GameObject() {
		static std::atomic<GameObjectId> counter{scene::kFirstGameObjectId};
		id_ = ++counter;
	}
    virtual ~GameObject() {}
};

class ParticleSystemObject: public GameObject {
    ParticleSystem* ps_;
public:
    static ParticleSystemObject* Create();

    void InitRenderResources();

    virtual void Update(float /*dt*/) { }

    virtual const char* GetName() const { return "particle system"; };
    virtual RenderMesh* GetMesh() const { return nullptr; }
    virtual ~ParticleSystemObject();
};

class FrustumObject: public GameObject {
    Frustum frustum_;
    RenderMesh* mesh_;

    public:

        static FrustumObject* Create(const camera* pcam);

        FrustumObject(/*const mat4 &view, float fov, float aspect, float nearv,
                      float farv*/)
            : mesh_(nullptr) {}

        ~FrustumObject() {
            DeinitRenderResources(); // dangerous, should schedule to render thread
        }

        virtual const char* GetName() const { return "frustum"; };
        void Update(float /*dt*/) {}
        void UpdateFrustum(const camera* pcam);

        void InitRenderResources();
        void DeinitRenderResources();

        RenderMesh* GetMesh() const { return mesh_; }
};

class MeshObject: public GameObject {
public:
  typedef std::function<void(float dt, MeshObject *)> Updater_t;

private:
    std::string name_;
    std::string mesh_name_;
    RenderMesh* mesh_;

    vec3 scale_;
    vec3 rot_;
    vec3 pos_;

    Updater_t updater_;

    MeshObject():mesh_(nullptr), scale_(0), rot_(0), pos_(0), updater_(nullptr) {}


public:
   static MeshObject* Create(const char* res);
   void InitRenderResources();

   // TODO: store and return ref counted object
   // and test deferred deletion
   RenderMesh* GetMesh() const { return mesh_; }

   const char* GetName() const { return name_.c_str(); } 

   void SetUpdater(Updater_t updater) { updater_ = updater; }
   void Update(float dt) {
       if (updater_)
           updater_(dt, this);
   }
};

