#pragma once

#include "utils/vec.h"
#include "utils/quaternion.h"
#include "utils/frustum.h"
#include "scene.h"

#include <cstdint>
#include <algorithm>
#include <functional>
#include <vector>
#include <string>
#include <atomic>

struct RenderMesh;
struct camera;
class ParticleSystem;

class IRenderable {
    std::atomic_bool bRenderInitialized = false;
public:
    bool IsRenderInitialized() const { return bRenderInitialized; }
	void DoInitRenderResources() {
		// check in case we've already scheduled it, but initialization did not happed
		// prev. frame due to e.g. latency issues (2 frames latency)
		if (!bRenderInitialized) {
			InitRenderResources();
			bRenderInitialized = true;
		}
	}

	void DoDeinitRenderResources() {
		if (bRenderInitialized) {
			InitRenderResources();
			bRenderInitialized = false;
		}
	}

	virtual void InitRenderResources() = 0;
    virtual void DeinitRenderResources() = 0;
    virtual void AddRenderPackets(struct RenderFrameContext* ) const {};
    virtual ~IRenderable() {}
};

class IEditorObject {
public:
    virtual int GetIconID() const { return -1; }
    virtual int IsSelectable() const { return true; }
};

// TODO: use array with compile time hashes as Component type
enum class ComponentType: int {
    kTransform = 0,
    kSPHBoundary,
    kCount
};

class Component {
    public:
    virtual ComponentType GetType() const = 0;
    virtual void UpdateComponent(float dt) {};
    virtual ~Component() {};
};

class TransformComponent: public Component {
    vec3 scale_;
    vec3 wscale_;
    quaternion rot_;
    vec3 pos_;

    mutable mat4 transform_;
    mutable bool b_need_recalculate = true;
    TransformComponent* parent_ = nullptr;
    std::vector<TransformComponent*> children_;

protected:
    virtual void on_transformed() {};
    void update_transform() const {
        transform_ = translate(pos_) * scale4(wscale_.x, wscale_.y, wscale_.z) *
            quat_to_mat4(rot_) * scale4(scale_.x, scale_.y, scale_.z);
        if(parent_) {
            transform_ = parent_->GetTransform() * transform_;
        }

        b_need_recalculate = false;

		for (TransformComponent *child : children_) {
			child->update_transform();
		}

    }
public:
	static const ComponentType type_ = ComponentType::kTransform;
    TransformComponent(): scale_(1), wscale_(1), rot_(quaternion::identity()), pos_(0) {}

    virtual ComponentType GetType() const { return type_; }

    mat4 GetTransform() const {
        if(b_need_recalculate) {
            update_transform();
		}
        return transform_;
    }

    bool NeedRecalculate() const { return  b_need_recalculate; }

    vec3 GetPosition() const { return pos_; }
    quaternion GetRotation() const { return rot_; }
    vec3 GetScale() const { return scale_; }
    vec3 GetWorldScale() const { return wscale_; }

    void SetPosition(const vec3& pos) { pos_ = pos; b_need_recalculate = true; on_transformed(); }
    void SetRotation(const vec3& rot)  { rot_ = euler_to_quat(rot.x, rot.y, rot.z); b_need_recalculate = true; on_transformed();}
    void SetRotation(const quaternion& q)  { rot_ = q; b_need_recalculate = true; on_transformed();}
    void SetScale(const vec3& scale) { scale_ = scale; b_need_recalculate = true; on_transformed();}
    void SetWorldScale(const vec3& wscale) { wscale_ = wscale; b_need_recalculate = true; on_transformed();}

    void SetParent(TransformComponent* parent);
    void AddChild(TransformComponent* child);
    void RemoveChild(TransformComponent* child);

    virtual void UpdateComponent(float dt);

};

typedef uint32_t GameObjectId;
class GameObject: public IRenderable, public IEditorObject {
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

    const std::vector<Component*>& GetComponents() const { return components_; }

    template<typename T>
    T* AddComponent() {
        T* comp = new T();
        components_.push_back(comp);
        return comp;
    }

    template<typename T>
    T* AddComponent(T* comp) {
        static_assert( std::is_base_of<Component, T>::value == true ); 
        components_.push_back(comp);
        return comp;
    }

    // returns component if removed, otherwise nullptr
    template<typename T>
    T* RemoveComponent(T* comp) {
        static_assert( std::is_base_of<Component, T>::value == true ); 
        auto b = std::begin(components_);
        auto e = std::end(components_);
        auto it = std::remove(b, e, comp);
        assert(it!=e);
        if(it!=e) {
            components_.erase(it, e);
            return comp;
        }
        return nullptr;
    }

	GameObject() {
		static std::atomic<GameObjectId> counter{scene::kFirstGameObjectId};
		id_ = ++counter;
	}
    virtual ~GameObject() {
        for(Component* c: components_) {
            delete c;
        }
    }
};

class ParticleSystemObject: public GameObject {
    ParticleSystem* ps_;
public:
    static ParticleSystemObject* Create();

    // done by the particle system
    void InitRenderResources() {};
    void DeinitRenderResources() {};

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
   void DeinitRenderResources() { mesh_ = nullptr; }

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
