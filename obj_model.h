#pragma once

#include "utils/vec.h"
#include "utils/frustum.h"

#include <functional>

class RenderMesh;
class camera;
class ParticleSystem;

class Renderable {
public:
    virtual void InitRenderResources() = 0;
    virtual ~Renderable() {}
};

class GameObject: public Renderable {
public:
    virtual const char* GetName() const = 0;
    virtual void Update(float dt) = 0;
    virtual RenderMesh* GetMesh() const = 0;
    virtual mat4 GetTransform() const = 0;
    virtual ~GameObject() {}
};

class ParticleSystemObject: public GameObject {
    ParticleSystem* ps_;
    mat4 tr_;
public:
    static ParticleSystemObject* Create();

    void InitRenderResources();

    virtual void Update(float /*dt*/) { }

    virtual const char* GetName() const { return "particle system"; };
    virtual RenderMesh* GetMesh() const { return nullptr; }
    virtual mat4 GetTransform() const { return tr_; }
    virtual mat4 SetTransform(const mat4& tr) { return tr_ = tr; }
    virtual ~ParticleSystemObject();
};

class FrustumObject: public GameObject {
    Frustum frustum_;
    RenderMesh* mesh_;

    public:

        static FrustumObject* Create(const camera* pcam);

        FrustumObject(const mat4 &view, float fov, float aspect, float nearv,
                      float farv)
            : mesh_(nullptr) {}

        ~FrustumObject() {
            DeinitRenderResources(); // dangerous, should schedule to render thread
        }

        virtual const char* GetName() const { return "frustum"; };
        void Update(float dt) {}
        void UpdateFrustum(const camera* pcam);

        void InitRenderResources();
        void DeinitRenderResources();

        RenderMesh* GetMesh() const { return mesh_; }
        mat4 GetTransform() const { return mat4::identity(); }
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

    MeshObject():mesh_(nullptr) {}

    Updater_t updater_;

public:
   static MeshObject* Create(const char* res);
   void InitRenderResources();

   // TODO: store and return ref counted object
   // and test deferred deletion
   RenderMesh* GetMesh() const { return mesh_; }

   const vec3& GetPosition() const { return pos_; }
   vec3& GetPosition() { return pos_; }
   const char* GetName() const { return name_.c_str(); } 

   void SetPosition(const vec3& pos) { pos_ = pos; }
   void SetRotation(const vec3& rot) { rot_ = rot; }
   void SetScale(const vec3& scale) { scale_ = scale; }
   void SetUpdater(Updater_t updater) { updater_ = updater; }

   void Update(float dt) {
       if (updater_)
           updater_(dt, this);
   }

   mat4 GetTransform() const {
       return translate(pos_) * rotateZXY4(rot_.x, rot_.y, rot_.z) *
              scale4(scale_.x, scale_.y, scale_.z);
   }
};

