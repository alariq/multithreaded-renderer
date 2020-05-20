#pragma once

#include "utils/vec.h"
#include "utils/frustum.h"

#include <algorithm>
#include <functional>
#include <vector>

struct RenderMesh;
struct camera;
class ParticleSystem;

class Renderable {
public:
    virtual void InitRenderResources() = 0;
    virtual ~Renderable() {}
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
    public:
    static const ComponentType type_ = ComponentType::kTransform;
    private:

    vec3 scale_;
    vec3 rot_;
    vec3 pos_;

    public:

    TransformComponent(): scale_(1), rot_(0), pos_(0) {}

    virtual ComponentType GetType() const { return type_; }

    mat4 GetTransform() const {
        return translate(pos_) * rotateZXY4(rot_.x, rot_.y, rot_.z) *
               scale4(scale_.x, scale_.y, scale_.z);
    }

    vec3 GetPosition() const { return pos_; }
    vec3 GetRotation() const { return rot_; }
    vec3 GetScale() const { return scale_; }

    void SetPosition(const vec3& pos) { pos_ = pos; }
    void SetRotation(const vec3& rot)  { rot_ = rot; }
    void SetScale(const vec3& scale) { scale_ = scale; }
};

class GameObject: public Renderable {
    std::vector<Component*> components_;
public:
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

