#pragma once

#include "utils/vec.h"
#include "utils/quaternion.h"
#include "utils/frustum.h"
#include "utils/imgui_property_list.h"
#include "utils/logging.h"
#include "scene.h"
#include "editor.h"

#include <cstdint>
#include <algorithm>
#include <functional>
#include <vector>
#include <atomic>
#include <string>
#include <assert.h>

struct RenderMesh;
struct camera;
class ParticleSystem;

class IRenderable {
    public:
    enum InitState_t:int { kUninitialized, kPendingInit, kInitialized, kPendingDeinit};

    private:
    bool SetPendingDestroy() { 
        int exp = (int)kInitialized;
		bool rv = initState.compare_exchange_strong(exp, (int)kPendingDeinit);
        assert((exp == kInitialized || exp == kPendingDeinit) && "Object is not in expected state");
        return rv;
    }
    bool SetPendingInit() { 
        int exp = (int)kUninitialized;
		bool rv = initState.compare_exchange_strong(exp, (int)kPendingInit);
        assert(exp<=kPendingInit && "Object is not in expected state");
        return rv;
    }
protected:
    std::atomic_int initState = kUninitialized;
public:

    bool IsRenderUninitialized() const { return initState.load() == (int)kUninitialized; }
    bool IsRenderInitialized() const { return initState.load() == (int)kInitialized; }
    //bool IsRenderDeinit() const { return initState.load() == (int)kUninitialized; }


    // these are called on Main thread
    void StartDeinit(struct RenderFrameContext *rfc) {
        if(SetPendingDestroy()) {
		    ScheduleRenderCommand(rfc, [this]() { this->DoDeinitRenderResources(); });
        }
    }
    void StartInit(struct RenderFrameContext *rfc) {
        if(SetPendingInit()) {
		    ScheduleRenderCommand(rfc, [this]() { this->DoInitRenderResources(); });
        }
    }

    // these are called on Render thread
	void DoInitRenderResources() {
		if (initState.load() == (int)kPendingInit) {
			InitRenderResources();
            assert(initState.load() == (int)kPendingInit);
            initState.store((int)kInitialized);
            return;
		}
        assert(!"Object is not in expected state");
	}
	void DoDeinitRenderResources() {
        if(initState.load() == (int)kPendingDeinit) {
            DeinitRenderResources();
        } else {
            log_error("Object is not in expected state %d\n", initState.load());
            gosASSERT(!"Object is not in expected state");
		}
	}

	virtual void InitRenderResources() = 0;
    virtual void DeinitRenderResources() = 0;
    virtual void AddRenderPackets(struct RenderFrameContext* ) const {};
    virtual ~IRenderable() {}
};

class IEditorObject: public imgui_props::IPolymorphicPropertyObject {
public:
    virtual int GetIconID() const { return 1; }
    virtual int IsSelectable() const { return true; }
    virtual const struct ITransformInterface* GetTransformInterface() const = 0;
    virtual ITransformInterface* GetTransformInterface() = 0;
};

// TODO: use array with compile time hashes as Component type
enum class ComponentType: int {
    kTransform = 0,
    kSPHBoundary,
    kSPHSceneComponent,
    kPBDStaticCollision,
    kPBDVisComponent,
    kFrustumComponent,
    kRigidBody,
    kGameText,
    kEnemyText,
    kMesh,
    kBillboard,
    kCurve,
    kCount
};
// stub for the future
struct GameObjectHandle {
	GameObject *go_handle_ = nullptr;
};

inline GameObject *getGameObject(GameObjectHandle go_handle) { return go_handle.go_handle_; }

template<typename T> inline constexpr ComponentType get_component_type();

class Component: public imgui_props::IPolymorphicPropertyObject {
    friend GameObject;
	GameObjectHandle go_handle_;
    public:
    enum State:int {kUninitialized, kPendingInit, kInitialized, kPendingDeinit };
  protected:
    int state_ = kUninitialized;
  public:

    GameObjectHandle getGameObjectHandle() const { return go_handle_; }
    virtual IRenderable* getRenderableInterface() { return nullptr; }
    virtual int getState() const { return state_; }
	virtual ComponentType GetType() const = 0;
	virtual void UpdateComponent(float dt){};

    virtual IRenderProxy* CreateRenderProxy() {return nullptr; }
    virtual IRenderProxy* GetRenderProxy() {return nullptr; }
    virtual void DestroyRenderProxy(struct RenderFrameContext* ) {};
	virtual void RenderUpdateComponent(struct RenderFrameContext *) {};

	virtual void Initialize() = 0;
	virtual void Deinitialize() = 0;

	virtual ~Component(){};
};


template<> inline constexpr ComponentType 
get_component_type<class TransformComponent>() { return ComponentType::kTransform; }

class TransformComponent : public Component, public ITransformInterface {
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
			world_scale_ = scale_;
			world_wscale_ = wscale_;
		}

		b_need_recalculate = false;
		on_transformed_fptr_(this);
		for (TransformComponent *child : children_) {
			child->update_transform();
		}
	}

  public:
    PROPERTY_SUPPORT(TransformComponent);
    PROPERTY_POLYMORPHIC_DRAW_IMPL(TransformComponent);

	static void on_transformed_default(TransformComponent* ) {}
	on_transformed_fptr_t on_transformed_fptr_ = on_transformed_default;
	static const ComponentType type_ = ComponentType::kTransform;
	TransformComponent()
		: scale_(1), wscale_(1), rot_(quaternion::identity()), pos_(0), world_scale_(1),
		  world_wscale_(1), wrot_(quaternion::identity()), wpos_(0),
		  transform_(identity4()), wtransform_(identity4()) {}

	virtual ComponentType GetType() const override { return type_; }
	virtual void Initialize() override { state_ = kInitialized; }
	virtual void Deinitialize() override { state_ = kUninitialized; }

	const mat4& GetTransform() const {
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
		return world_scale_;
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

    bool IsValid() { return true; }
    void SetPosition(vec3 p, void* ) override { SetPosition(p); }
    vec3 GetPosition(void* ) const override { return GetPosition(); } 
    void SetRotation(quaternion q, void* ) override { SetRotation(q); }
    quaternion GetRotation(void* ) const override { return GetRotation(); }
    void SetScale(vec3 s, void* ) override { SetScale(s); }
    vec3 GetScale(void* ) const override { return GetScale(); }
    vec3 GetWorldSpaceScale(void* ) const override { return GetWorldSpaceScale(); }
    void SetWorldSpaceScale(const vec3 ws, void* ) override { SetWorldSpaceScale(ws); }

    virtual bool HasMove() const override { return true; }
    virtual bool HasRotate() const override { return true; }
    virtual bool HasScale() const override { return true; }
};

PROPERTY_LIST_DECLARE_DERIVED(TransformComponent, Component)

class StaticMeshRenderProxy: public IRenderProxy {
    std::string name_;
    StaticMesh* mesh_ = nullptr;
    mat4 transform_ = mat4::identity();
public:
    u8 b_is_forward_pass:1;
    u8 b_is_opaque_pass:1;

    void SetMesh(StaticMesh* mesh) { mesh_ = mesh; }
    void SetName(const std::string& name) { name_ = name; }
    void SetTexture(TextureHandle h) { mesh_->tex_handle_ = h; }
    void SetTransform(const mat4& tr) { transform_ = tr; }

    virtual void Initialize(struct RenderFrameContext* ) override {}
    virtual void Deinitialize(struct RenderFrameContext* ) override {}

    virtual void AddRenderPackets(struct RenderFrameContext* rfc) override;
};


template<> inline constexpr ComponentType 
get_component_type<class MeshComponent>() { return ComponentType::kMesh; }
class MeshComponent : public TransformComponent {
protected:
	std::string mesh_name_;
	std::string tex_name_;
	StaticMesh *mesh_;
    TextureHandle texture_;
    class StaticMeshRenderProxy* proxy_;
    bool b_update_transform_;
    bool b_update_mesh_;
    bool b_update_texture_;
public:
	MeshComponent() : mesh_(nullptr), b_update_transform_(false), b_update_mesh_(false), b_update_texture_(false){}

    PROPERTY_SUPPORT(MeshComponent)
    PROPERTY_POLYMORPHIC_DRAW_IMPL(MeshComponent)

	virtual ComponentType GetType() const override { return get_component_type<MeshComponent>(); }
	static MeshComponent *Create(const char *res, GameObject* go);

    virtual IRenderProxy* CreateRenderProxy() override;
    virtual void DestroyRenderProxy(struct RenderFrameContext* rfc) override;
	virtual IRenderProxy* GetRenderProxy() override;
	virtual void RenderUpdateComponent(struct RenderFrameContext *) override;

    static void OnTransformed(TransformComponent* tc);

	void SetMesh(const char *mesh);
	void SetTexture(const char *name);
    const AABB& GetAABB() const { return mesh_->aabb_; }
    // this will be updated by Init/Deinit Render Resoueces
	virtual void Initialize() override;
	virtual void Deinitialize() override;
};

PROPERTY_LIST_DECLARE_DERIVED(MeshComponent, TransformComponent)

template<> inline constexpr ComponentType 
get_component_type<class FrustumComponent>() { return ComponentType::kFrustumComponent; }
class FrustumComponent: public TransformComponent {
    class FrustumRenderProxy* proxy_;
    bool b_update_proxy_;

    mat4 view_, inv_view_;
    float fov_, near_, far_, aspect_;
    bool b_override_;

  public:
    PROPERTY_SUPPORT(FrustumComponent)
    PROPERTY_POLYMORPHIC_DRAW_IMPL(FrustumComponent)

    FrustumComponent():
        proxy_(nullptr), b_update_proxy_(false),
        view_(mat4::identity()), 
        inv_view_(mat4::identity()),
        fov_(0), near_(0), far_(0), aspect_(0),
        b_override_(false) {}

	virtual ComponentType GetType() const override { return get_component_type<FrustumComponent>(); }
    // TODO: do we need a state var at all?
	virtual void Initialize() override { state_ = Component::kInitialized; }
	virtual void Deinitialize() override { state_ = Component::kUninitialized; }

    virtual IRenderProxy* CreateRenderProxy() override;
    virtual void DestroyRenderProxy(struct RenderFrameContext* rfc) override;
	virtual IRenderProxy* GetRenderProxy() override;
	virtual void RenderUpdateComponent(struct RenderFrameContext *) override;

    
    void OverrideView(const mat4* view, const mat4* inv_view = 0, float fov = 0, float n = 0, float f = 0, float aspect = 0) {
        if(view && inv_view) {
            view_ = *view;
            inv_view_ = *inv_view;
            fov_ = fov;
            near_ = n;
            far_ = f;
            aspect_ = aspect;
            b_override_ = true;
        } else {
            b_override_ = false;
        }
        b_update_proxy_ = true;
    }

};
PROPERTY_LIST_DECLARE_DERIVED(FrustumComponent, Component)

typedef uint32_t GameObjectId;
class GameObject: public IEditorObject {
    std::vector<Component*> components_;
	GameObjectId id_;
public:
    enum State:int {/*kUninitialized, */ kInitialized, kPendingDestroy, kDestroyed };
private:
    State state_;
public:
    PROPERTY_SUPPORT(GameObject)

	GameObjectId GetId() const { return id_; }

    virtual const char* GetName() const = 0;
    virtual void Update(float dt) {};
    //virtual RenderMesh* GetMesh() const = 0;
    virtual void Destroy() { state_ = kPendingDestroy; }
    State GetState() const { return state_; }
    void SetState(GameObject::State s) { state_ = s; }

    Component* GetComponent(ComponentType type) const {
        auto cmp = std::find_if(
            components_.begin(), components_.end(),
            [type](Component *comp) { return comp->GetType() == type; });
        return cmp!=components_.end() ? *cmp : nullptr;
    }

    template<typename T>
    T* GetComponent() const {
        auto cmp = std::find_if(
            components_.begin(), components_.end(),
            [](Component *comp) { return comp->GetType() == get_component_type<T>(); });
        return cmp!=components_.end() ? (T*)*cmp : nullptr;
    }

    const std::vector<Component*>& GetComponents() const { return components_; }

    bool AttachComponent(Component* comp) {
        auto b = std::begin(components_);
        auto e = std::end(components_);
        const auto it = std::find(b, e, comp);
        gosASSERT(it==e);
        if(it==e) {
            comp->go_handle_ = GameObjectHandle{this};
            components_.push_back(comp);
        }
        return it==e;
    }

    bool DetachComponent(Component* comp) {
        auto b = std::begin(components_);
        auto e = std::end(components_);
        auto it = std::remove(b, e, comp);
        gosASSERT(it!=e);
        if(it!=e) {
            components_.erase(it, e);
            comp->go_handle_ = GameObjectHandle{nullptr};
            return true;
        }
        return false;
    }

    void RemoveAllComponents() {
        components_.clear();
    }

	GameObject() {
		static std::atomic<GameObjectId> counter{scene::kFirstGameObjectId};
		id_ = ++counter;
        state_ = kInitialized;
	}
    virtual ~GameObject() {
        //smells
        while(components_.size())
            scene_delete_component(components_[0]);
    }

    virtual void AddRenderPackets(struct RenderFrameContext* ) const {};
    virtual const TransformComponent* GetTransformInterface() const override { return GetComponent<TransformComponent>(); };
    virtual TransformComponent* GetTransformInterface() override { return GetComponent<TransformComponent>(); };
};

PROPERTY_LIST_DECLARE_DERIVED(GameObject, void)

class ParticleSystemObject: public GameObject {
    ParticleSystem* ps_;
public:
    static ParticleSystemObject* Create();

    virtual void Update(float /*dt*/) override { }

    virtual const char* GetName() const override { return "particle system"; };
    virtual ~ParticleSystemObject();
};

class FrustumObject: public GameObject {
    Frustum frustum_;
    FrustumComponent* frustum_comp_;

    public:
        static FrustumObject* Create();
        virtual const char* GetName() const override { return "frustum"; };
        virtual void Update(float /*dt*/) override {}
        void UpdateFrustum(const camera* pcam);
};


class MeshObject: public GameObject {
public:
  typedef std::function<void(float dt, MeshObject *)> Updater_t;

protected:
    std::string name_;
    MeshComponent* mesh_comp_;

    //vec3 scale_;
    //vec3 rot_;
    //vec3 pos_;

    Updater_t updater_;

    MeshObject()://mesh_(nullptr), 
        mesh_comp_(0), /*scale_(0), rot_(0), pos_(0),*/ updater_(nullptr) {}

public:
    PROPERTY_SUPPORT(MeshObject)
    PROPERTY_POLYMORPHIC_DRAW_IMPL(MeshObject)

   static MeshObject* Create(const char* res);
   virtual const char* GetName() const override { return name_.c_str(); } 

   void SetUpdater(Updater_t updater) { updater_ = updater; }
   virtual void Update(float dt) override {
       if (updater_)
           updater_(dt, this);
   }
};

PROPERTY_LIST_DECLARE_DERIVED(MeshObject, GameObject)


class ICameraController : public imgui_props::IPolymorphicPropertyObject {
    public:
        //PROPERTY_SUPPORT(ICameraController);
        //PROPERTY_POLYMORPHIC_DRAW_IMPL(ICameraController)
    virtual struct camera& Update(float dt)  = 0;
    virtual const struct camera& GetCamera() const  = 0;
};

