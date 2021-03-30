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
    virtual int GetIconID() const { return 1; }
    virtual int IsSelectable() const { return true; }
};

// TODO: use array with compile time hashes as Component type
enum class ComponentType: int {
    kTransform = 0,
    kSPHBoundary,
    kRigidBody,
    kMesh,
    kCount
};
// stub for the future
struct GameObjectHandle {
	GameObject *go_handle_ = nullptr;
};

inline GameObject *getGameObject(GameObjectHandle go_handle) { return go_handle.go_handle_; }

class Component {
	GameObjectHandle go_handle_;
  protected:

  public:
    GameObjectHandle getGameObjectHandle() const { return go_handle_; }
	virtual ComponentType GetType() const = 0;
	virtual void UpdateComponent(float dt){};
	void Initialize(GameObjectHandle go_handle) { go_handle_ = go_handle; }
	virtual ~Component(){};
};

class TransformComponent : public Component {
  public:
	typedef void (*on_transformed_fptr_t)(TransformComponent*);

  private:

	vec3 scale_;
	vec3 wscale_;
	quaternion rot_;
	vec3 pos_;

	vec3 world_scale_;
	vec3 world_wscale_;
	quaternion wrot_;
	vec3 wpos_;

	mutable mat4 transform_;
	mutable mat4 wtransform_;
	mutable bool b_need_recalculate = false;
	TransformComponent *parent_ = nullptr;
	std::vector<TransformComponent *> children_;

  protected:
	void update_transform() {

		transform_ = translate(pos_) * scale4(wscale_.x, wscale_.y, wscale_.z) *
					 quat_to_mat4(rot_) * scale4(scale_.x, scale_.y, scale_.z);

		if (parent_) {
			wpos_ = parent_->Transform(pos_);
			wrot_ = parent_->GetRotation() * rot_;
			// this is accounted for in Transform?
			world_scale_ = parent_->GetScale() * scale_;
			world_wscale_ = parent_->GetWorldSpaceScale() * wscale_;
			wtransform_ = parent_->GetTransform() * transform_;
		} else {
			wtransform_ = transform_;
		}

		b_need_recalculate = false;
		on_transformed_fptr_(this);
		for (TransformComponent *child : children_) {
			child->update_transform();
		}
	}

  public:
	static void on_transformed_default(TransformComponent* ) {}
	on_transformed_fptr_t on_transformed_fptr_ = on_transformed_default;
	static const ComponentType type_ = ComponentType::kTransform;
	TransformComponent()
		: scale_(1), wscale_(1), rot_(quaternion::identity()), pos_(0), world_scale_(1),
		  world_wscale_(1), wrot_(quaternion::identity()), wpos_(0),
		  transform_(identity4()), wtransform_(identity4()) {}

	virtual ComponentType GetType() const override { return type_; }

	mat4 GetTransform() const {
		assert(!b_need_recalculate);
		return wtransform_;
	}

	bool NeedRecalculate() const { return b_need_recalculate; }

	vec3 Transform(const vec3 &p) const {
		return wscale_ * quat_rotate(wrot_, world_scale_ * p) + wpos_;
	}
	vec3 ToLocal(const vec3 &p) const {
		return (vec3(1.0f) / world_scale_) *
			   quat_inv_rotate(wrot_, (p - wpos_) / world_wscale_);
	}
	quaternion ToLocal(const quaternion &q) const { return inverse(wrot_) * q; }
	vec3 Rotate(const vec3 &n) const { return quat_rotate(rot_, n); }

	vec3 GetPosition() const {
		assert(!b_need_recalculate);
		return wpos_;
	}
	quaternion GetRotation() const {
		assert(!b_need_recalculate);
		return wrot_;
	}
	vec3 GetScale() const {
		assert(!b_need_recalculate);
		return scale_;
	}
	vec3 GetWorldSpaceScale() const {
		assert(!b_need_recalculate);
		return wscale_;
	}

	vec3 GetLocalPosition() const { return pos_; }

	void SetPosition(const vec3 &pos);
	void SetRotation(const quaternion &q);
	void SetScale(const vec3 &scale) {
		scale_ = scale;
		b_need_recalculate = true;
		update_transform();
	}
	void SetWorldSpaceScale(const vec3 &wscale) {
		wscale_ = wscale;
		b_need_recalculate = true;
		update_transform();
	}

	void SetParent(TransformComponent *parent);
	void AddChild(TransformComponent *child);
	void RemoveChild(TransformComponent *child);

	virtual void UpdateComponent(float dt) override;
};

class MeshComponent : public TransformComponent, public IRenderable {
	std::string mesh_name_;
	RenderMesh *mesh_;

	mutable std::string pending_mesh_name_;
	mutable std::atomic<void *> pending_mesh_;

	static const ComponentType type_ = ComponentType::kMesh;
	MeshComponent() : mesh_(nullptr), pending_mesh_(nullptr) {}

  public:
	virtual ComponentType GetType() const override { return type_; }
	static MeshComponent *Create(const char *res);
	virtual void InitRenderResources() override;
	virtual void DeinitRenderResources() override { mesh_ = nullptr; }
	virtual void AddRenderPackets(struct RenderFrameContext *) const override;
	virtual void UpdateComponent(float dt) override;
	void SetMesh(const char *mesh);
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
        return AddComponent(comp);
    }

    template<typename T>
    T* AddComponent(T* comp) {
        static_assert( std::is_base_of<Component, T>::value == true ); 
        auto b = std::begin(components_);
        auto e = std::end(components_);
		assert(e == std::find(b, e, comp));
		comp->Initialize(GameObjectHandle{this});
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
    virtual void InitRenderResources() override {};
    virtual void DeinitRenderResources() override {};

    virtual void Update(float /*dt*/) override { }

    virtual const char* GetName() const override { return "particle system"; };
    virtual RenderMesh* GetMesh() const override { return nullptr; }
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

        virtual const char* GetName() const override { return "frustum"; };
        virtual void Update(float /*dt*/) override {}
        void UpdateFrustum(const camera* pcam);

        virtual void InitRenderResources() override;
        virtual void DeinitRenderResources() override;

        virtual RenderMesh* GetMesh() const override { return mesh_; }
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
   virtual void InitRenderResources() override;
   virtual void DeinitRenderResources() override { mesh_ = nullptr; }

   // TODO: store and return ref counted object
   // and test deferred deletion
   virtual RenderMesh* GetMesh() const override { return mesh_; }

   virtual const char* GetName() const override { return name_.c_str(); } 

   void SetUpdater(Updater_t updater) { updater_ = updater; }
   virtual void Update(float dt) override {
       if (updater_)
           updater_(dt, this);
   }
};
