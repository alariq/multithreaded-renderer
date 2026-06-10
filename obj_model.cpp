#include "res_man.h"
#include "obj_model.h"
#include "particle_system.h"
#include "utils/obj_loader.h"

PROPERTY_LIST_BEGIN_DERIVED(GameObject, void)
    PROPERTY_ARRAY(components_, "components");
    PROPERTY_UINT(id_, "id", 0, UINT32_MAX, 0, PropertyFlags::kPropertyFlagReadOnly);
PROPERTY_LIST_END()

void TransformComponent::SetParent(TransformComponent* parent) {

    if(parent_) {
        assert(parent_ != parent);
        parent_->RemoveChild(this);
    }

    parent_ = parent;

    if(parent_)
        parent_->AddChild(this);
    else {
        // notify scene that this component is now parent, so should be added to update list
        // maybe just add all of them and only update if root component?
        // also update our transform from parent
    }

    b_need_recalculate = true;
    update_transform();
}

void TransformComponent::AddChild(TransformComponent *child) {
    assert(child);
	auto b = std::begin(children_);
	auto e = std::end(children_);
	assert(std::find(b, e, child) == children_.end());
    children_.push_back(child);
}

void TransformComponent::RemoveChild(TransformComponent *child) {
    assert(child);
	auto b = std::begin(children_);
	auto e = std::end(children_);
	children_.erase(
		std::remove_if(b, e, [child](TransformComponent *o) { return o == child; }), e);
}

void TransformComponent::SetPosition(const vec3& pos) {
    wpos_ = pos;
    if(parent_) {
        pos_ =  parent_->ToLocal(wpos_);
    } else {
        pos_ = wpos_;
    }
    update_transform();
}

void TransformComponent::SetRotation(const quaternion& q) {
    wrot_ = q;
    if(parent_) {
        rot_ = parent_->ToLocal(wrot_);
    } else {
        rot_ = wrot_;
    }
    update_transform();
}

void TransformComponent::UpdateComponent(float dt) {
    // all hierarchy gets transformed, so no need to explicitly transform children
	if (NeedRecalculate() && nullptr == parent_) {
		update_transform();
	}
}

PROPERTY_LIST_BEGIN_DERIVED(TransformComponent, Component)
    PROPERTY_SECTION("TransformComponent");
    PROPERTY_VEC3_ACC("Pos", 0, 0.1f,
            [](const TransformComponent& c)->vec3{ return c.GetPosition();},
            [](TransformComponent& c, const vec3& pos){c.SetPosition(pos);});
    PROPERTY_VEC3_ACC("Scale", 0, 0.1f,
            [](const TransformComponent& c)->vec3{ return c.GetScale();},
            [](TransformComponent& c, const vec3& s){c.SetScale(s);});
    PROPERTY_ARRAY(children_, "Children");
PROPERTY_LIST_END()

void StaticMeshRenderProxy::AddRenderPackets(struct RenderFrameContext* rfc) {
    if(mesh_) {
        class RenderList *rl = rfc->rl_;
        RenderPacket *rp = rl->AddPacket();
        memset(rp, 0, sizeof(RenderPacket));
        //rp->mesh_ = *mesh_;

        rp->mesh_.vb_ = mesh_->rd.vb_;
        rp->mesh_.ib_ = mesh_->rd.ib_;
        rp->mesh_.inst_vb_ = mesh_->rd.inst_vb_;
        rp->mesh_.vdecl_ = mesh_->rd.vdecl_;
        rp->mesh_.mat_ = mesh_->rd.mat_;
        rp->mesh_.num_instances = mesh_->num_instances;
        rp->mesh_.tex_id_ = texture_res_get_gpu_id(mesh_->tex_handle_);
        rp->mesh_.prim_type_ = mesh_->prim_type_;
        rp->mesh_.aabb_ = mesh_->aabb_;
        rp->mesh_.vb_first_ = mesh_->vb_first_;
        rp->mesh_.vb_count_ = mesh_->vb_count_;
        rp->mesh_.two_sided_ = mesh_->two_sided_;

        rp->m_ = transform_;
        //rp->is_debug_pass = 1;
        rp->is_opaque_pass = b_is_forward_pass ? false : true;
        rp->is_forward_pass = b_is_forward_pass;
        rp->debug_color = vec4(0, 1, 0, 1);
    }
}

MeshComponent* MeshComponent::Create(const char *res, GameObject* go) {
    auto comp = scene_create_component<MeshComponent>(go);
    comp->SetMesh(res);
    return comp;
}

void MeshComponent::Initialize() {
    // problem: state is also set in TransformComponent
    TransformComponent::Initialize();

    if(!mesh_name_.empty())
        mesh_ = res_man_load_mesh2(mesh_name_);

    on_transformed_fptr_ = OnTransformed;

    // do we need state at all? we implicitly know if we are initialized if we called Initialize()
    // for multi-frame init, may need a different query way, like. virtual bool IsInitialized()
	state_ = Component::kInitialized;
}

void MeshComponent::Deinitialize() {
    res_man_release_mesh2(mesh_);
    mesh_ = nullptr;
	state_ = Component::kUninitialized;

    TransformComponent::Deinitialize();
}

IRenderProxy* MeshComponent::CreateRenderProxy() {
    proxy_ = new StaticMeshRenderProxy();
    return proxy_;
}

void MeshComponent::DestroyRenderProxy(struct RenderFrameContext* rfc) {
    ScheduleRenderCommand(rfc, [proxy = proxy_]() {
            delete proxy;
    });
}

IRenderProxy* MeshComponent::GetRenderProxy() {
    return proxy_;
}

void MeshComponent::SetMesh(const char *mesh) {
    mesh_name_ = mesh;
    if(mesh_)
        res_man_release_mesh2(mesh_);
    mesh_ = res_man_load_mesh2(mesh_name_);
    b_update_mesh_ = true;
}

void MeshComponent::SetTexture(const char *name) {
    tex_name_ = name;
//    if(texture_)
//        res_man_release_testure(texture_);
    texture_ = res_man_load_texture2(tex_name_);
    b_update_texture_ = true;
}

void MeshComponent::OnTransformed(TransformComponent* tc) {
    MeshComponent* mc = (MeshComponent*)tc;
    mc->b_update_transform_ = true;
}

void MeshComponent::RenderUpdateComponent(struct RenderFrameContext* rfc) {

    if(b_update_transform_ || b_update_mesh_ || b_update_texture_) {
        ScheduleRenderCommand(rfc, 
                [proxy = proxy_, mesh = mesh_, name = mesh_name_, tex_handle = texture_, tr = GetTransform(), 
                b_tr = b_update_transform_, b_mesh = b_update_mesh_, b_tex = b_update_texture_]() {

                if(b_tr) {
                    proxy->SetTransform(tr);
                }

                if(b_mesh) {
                    proxy->SetName(name);
                    proxy->SetMesh(mesh);
                }
                if(b_tex) {
                    proxy->SetTexture(tex_handle);
                }
                });
    }
    b_update_transform_ = b_update_mesh_ = b_update_texture_ = false;
}

PROPERTY_LIST_BEGIN_DERIVED(MeshComponent, TransformComponent)
    PROPERTY_SECTION("MeshComponent");
    PROPERTY_READONLY_TEXT("Name", [](const MeshComponent& c){ return c.mesh_name_;});
    PROPERTY_READONLY_TEXT("TextureName", [](const MeshComponent& c){ return c.tex_name_;});
    PROPERTY_TEXT("Texture", 
            [](const MeshComponent& c){ return c.tex_name_;},
            [](MeshComponent& c, const std::string& s){ return c.SetTexture(s.c_str());});
PROPERTY_LIST_END()


ParticleSystemObject* ParticleSystemObject::Create()
{
    ParticleSystem* ps = new ParticleSystem;

    ParticleSystemObject* pso = new ParticleSystemObject;
    pso->ps_ = ps;
    pso->ps_->AddEmitter(CreateStandardEmitter());
    ParticleSystemManager::Instance().Add(pso->ps_);

    return pso;
}

ParticleSystemObject::~ParticleSystemObject() {
    ParticleSystemManager::Instance().Remove(ps_);
    delete ps_;
}

FrustumObject *FrustumObject::Create() {
    FrustumObject *o = new FrustumObject();
    scene_create_component<FrustumComponent>(o);
    return o;
}

class FrustumRenderProxy: public IRenderProxy {
    HGOSBUFFER				vb_;
    HGOSBUFFER				ib_;
public:
    bool b_override_;
    mat4 view_, inv_view_;
    float near_,far_,fov_,aspect_;

    FrustumRenderProxy():b_override_(false), near_(0), far_(0), fov_(0), aspect_(0){}
    virtual void Initialize(struct RenderFrameContext* ) override {

        const int ib_size = Frustum::kNUM_FRUSTUM_PLANES * 6;
        const int vb_size = ib_size;
        uint16_t ib[ib_size];
        SVD vb[vb_size];
        memset(&vb, 0, sizeof(vb));
        for (uint16_t i = 0; i < ib_size; ++i)
            ib[i] = i;

        ib_ = gos_CreateBuffer(gosBUFFER_TYPE::INDEX, gosBUFFER_USAGE::STATIC_DRAW,
                             sizeof(uint16_t), ib_size, ib);

        Frustum f;
        Frustum::makeMeshFromFrustum(&f, (char *)&vb[0], vb_size,
                                     (int)sizeof(SVD));

        for (int i = 0; i < vb_size; ++i) {
            vb[i].uv = vec2(vb[i].pos.x, vb[i].pos.z);
            vb[i].normal = normalize(vb[i].pos);
        }

        vb_ = gos_CreateBuffer(gosBUFFER_TYPE::VERTEX, gosBUFFER_USAGE::DYNAMIC_DRAW,
                             sizeof(SVD), vb_size, vb);
        //mesh_->prim_type_ = PRIMITIVE_TRIANGLELIST;
        //mesh_->vb_first_ = 0;
        //mesh_->vb_count_ = -1;
        //mesh_->two_sided_ = 0;
        //mesh_->tex_id_ = res_man_load_texture("default");
    }
    virtual void Deinitialize(struct RenderFrameContext* ) override {
        gos_DestroyBuffer(ib_);
        gos_DestroyBuffer(vb_);
    }

    virtual void AddRenderPackets(struct RenderFrameContext* rfc) override {

        const mat4& view = b_override_ ? view_ : rfc->view_;
        const mat4& inv_view = b_override_ ? inv_view_ : rfc->inv_view_;
        const float zn = near_ ? near_ : rfc->z_near_;
        const float zf = far_ ? far_ : rfc->z_far_;
        const float fov = fov_ ? fov_ : rfc->fov_;
        const float aspect = aspect_ ? aspect_ : rfc->aspect_;

        vec3 fwd = view.getRow(2).xyz();
        vec3 right = view.getRow(0).xyz();
        vec3 up = view.getRow(1).xyz();
        vec4 pos = inv_view * vec4(0, 0, 0, 1);

        float safe_zone_deg = 0.5f;
        Frustum f;
        f.updateFromCamera(pos.xyz(), fwd, right, up, 
                (fov - safe_zone_deg) * 3.1415f/180.0f, aspect, zn, zf);

        ScheduleRenderCommand(rfc, [f, mesh_vb = vb_]() {
            const int vb_size = Frustum::kNUM_FRUSTUM_PLANES * 6;
            SVD vb[vb_size];
            Frustum::makeMeshFromFrustum(&f, (char*)&vb[0], vb_size, (int)sizeof(SVD));

            for (int i = 0; i < vb_size; ++i) {
                vb[i].uv = 0.025f*vec2(vb[i].pos.x, vb[i].pos.z);
                vb[i].normal = normalize(vb[i].pos);
            }

            gos_UpdateBuffer(mesh_vb, vb, 0, vb_size * sizeof(SVD));
        });

        class RenderList *rl = rfc->rl_;
        if(false) {
            RenderPacket *rp = rl->AddPacket();
            memset(rp, 0, sizeof(RenderPacket));
            rp->mesh_.vb_ = vb_;
            rp->mesh_.ib_ = ib_;
            rp->mesh_.vdecl_ = get_svd_vdecl();
            rp->m_ = mat4::identity();
            //rp->is_debug_pass = 1;
            rp->is_opaque_pass = 1;
            //rp->is_transparent_pass = 1;
            rp->debug_color = vec4(0, 1, 0, 1);
        }

        bool b_draw_wireframe_ = true;
        if(b_draw_wireframe_) {
            vec3 p[12*2]; // 12 lines x 2 points
            f.calculateLineList(p, COUNTOF(p));
            rl->addDebugLines(p, nullptr, vec4(1), 12);

            vec3 p3 = pos.xyz();
            rl->addDebugPoints(&p3, 1, vec4(1, 0, 0, 1), 10, true);
        }
    }
};

IRenderProxy* FrustumComponent::CreateRenderProxy() {
    proxy_ = new FrustumRenderProxy();
    return proxy_;
}

IRenderProxy* FrustumComponent::GetRenderProxy() {
    return proxy_;
}

void FrustumComponent::DestroyRenderProxy(struct RenderFrameContext* rfc) {
    ScheduleRenderCommand(rfc, [proxy = proxy_]() {
            delete proxy;
    });
}

void FrustumComponent::RenderUpdateComponent(struct RenderFrameContext* rfc) {
    if(b_update_proxy_) {
        ScheduleRenderCommand(rfc, [this]() {
                proxy_->view_ = view_;
                proxy_->inv_view_ = inv_view_;
                proxy_->fov_ = fov_;
                proxy_->near_ = near_;
                proxy_->far_ = far_;
                proxy_->aspect_ = aspect_;
                proxy_->b_override_ = b_override_;
        });
        b_update_proxy_ = false;
    }
}



PROPERTY_LIST_BEGIN_DERIVED(FrustumComponent, TransformComponent)
    PROPERTY_FLOAT(fov_, "fov", 0, 30.0f, 120.0f, 1);
    PROPERTY_FLOAT(near_, "near", 0, 30.0f, 120.0f, 1);
    PROPERTY_FLOAT(far_, "far", 0, 30.0f, 120.0f, 1);
    PROPERTY_FLOAT(aspect_, "aspect", 0, 30.0f, 120.0f, 1);
    PROPERTY_BOOL(b_override_, "overriden");
PROPERTY_LIST_END()

MeshObject *MeshObject::Create(const char *res) {
    static size_t obj_num = 0;
    MeshObject *obj = new MeshObject();
    obj->name_ = res;
    obj->name_ += std::to_string(obj_num++);

    auto tr = scene_create_component<TransformComponent>(obj);

    obj->mesh_comp_ = MeshComponent::Create(res, obj);
    obj->mesh_comp_->SetParent(tr);

    return obj;
}

PROPERTY_LIST_BEGIN_DERIVED(MeshObject, GameObject)
    PROPERTY_READONLY_TEXT("Name", [](const MeshObject& o){ return o.GetName();});
PROPERTY_LIST_END()

//PROPERTY_LIST_BEGIN(ICameraController)
//PROPERTY_LIST_END()
