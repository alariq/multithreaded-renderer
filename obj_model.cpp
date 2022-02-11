#include "res_man.h"
#include "obj_model.h"
#include "particle_system.h"
#include "utils/obj_loader.h"
#include <cstdint>


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

MeshComponent* MeshComponent::Create(const char *res) {
    static size_t comp_num = 0;
    MeshComponent* comp = new MeshComponent();
    comp->mesh_name_ = res;
    comp->mesh_name_ += std::to_string(comp_num++);
    return comp;
}

void MeshComponent::InitRenderResources() {
    // assert(IsOnRenderThread());
    mesh_ = res_man_load_mesh(mesh_name_);
	// !NB: strictly speaking cannot modify state on render thread, we can have an Update
	// method called for component even if it is not initialized, so that on game thread
	// this update method will set state_ = initState.load();
	state_ = Component::kInitialized;
}
void MeshComponent::DeinitRenderResources() {
    // assert(IsOnRenderThread());
    mesh_ = nullptr;
    state_ = Component::kUninitialized;
}

void MeshComponent::SetMesh(const char* mesh_name) {
    pending_mesh_name_ = mesh_name;
}

void MeshComponent::UpdateComponent(float dt) {

    if(pending_mesh_) {
        mesh_ = static_cast<decltype(mesh_)>(pending_mesh_.load());
    }
}

void MeshComponent::AddRenderPackets(struct RenderFrameContext* rfc) const {

    if(!pending_mesh_name_.empty()) {
        auto mesh2load = pending_mesh_name_;
		ScheduleRenderCommand(rfc, [this, mesh2load]() {
			RenderMesh* mesh = res_man_load_mesh(mesh2load);
            this->pending_mesh_.store((void*)mesh);
		});
        pending_mesh_name_.clear();
	}

    if(mesh_) {
        class RenderList *rl = rfc->rl_;
        RenderPacket *rp = rl->AddPacket();
        memset(rp, 0, sizeof(RenderPacket));
        rp->mesh_ = *mesh_;
        rp->m_ = GetTransform();
        //rp->is_debug_pass = 1;
        rp->is_opaque_pass = 1;
        rp->debug_color = vec4(0, 1, 0, 1);
    }
}

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

FrustumObject *FrustumObject::Create(const camera *pcam) {
    FrustumObject *o = new FrustumObject();
    //o->UpdateFrustum(pcam);
    return o;
}
#if 0
void FrustumObject::UpdateFrustum(const camera *pcam) {
    mat4 view;
    pcam->get_view(&view);
    vec3 dir = view.getRow(2).xyz();
    vec3 right = view.getRow(0).xyz();
    vec3 up = view.getRow(1).xyz();
    vec4 pos = pcam->get_inv_view() * vec4(0, 0, 0, 1);

    frustum_.updateFromCamera(pos.xyz(), dir, right, up, pcam->get_fov(),
                              pcam->get_aspect(), pcam->get_near(),
                              pcam->get_far());

    frustum_comp_->frustum_ = frustum_;

    // dangerous: we are on a game thread
    if (mesh_) {
        const int vb_size = Frustum::kNUM_FRUSTUM_PLANES * 6;
        SVD vb[vb_size];
        Frustum::makeMeshFromFrustum(&frustum_, (char *)&vb[0], vb_size,
                                     (int)sizeof(SVD));

        for (int i = 0; i < vb_size; ++i) {
            vb[i].uv = vec2(vb[i].pos.x, vb[i].pos.z);
            vb[i].normal = normalize(vb[i].pos);
        }
        gos_UpdateBuffer(mesh_->vb_, vb, 0, vb_size * sizeof(SVD));
    }
    //
}
#endif

void FrustumComponent::InitRenderResources() {

    mesh_ = new RenderMesh();
    mesh_->vdecl_ = get_svd_vdecl();

    const int ib_size = Frustum::kNUM_FRUSTUM_PLANES * 6;
    const int vb_size = ib_size;
    uint16_t ib[ib_size];
    SVD vb[vb_size];
    memset(&vb, 0, sizeof(vb));
    for (uint16_t i = 0; i < ib_size; ++i)
        ib[i] = i;

    mesh_->ib_ =
        gos_CreateBuffer(gosBUFFER_TYPE::INDEX, gosBUFFER_USAGE::STATIC_DRAW,
                         sizeof(uint16_t), ib_size, ib);

    Frustum f;
    Frustum::makeMeshFromFrustum(&f, (char *)&vb[0], vb_size,
                                 (int)sizeof(SVD));

    for (int i = 0; i < vb_size; ++i) {
        vb[i].uv = vec2(vb[i].pos.x, vb[i].pos.z);
        vb[i].normal = normalize(vb[i].pos);
    }

    mesh_->vb_ =
        gos_CreateBuffer(gosBUFFER_TYPE::VERTEX, gosBUFFER_USAGE::DYNAMIC_DRAW,
                         sizeof(SVD), vb_size, vb);
    mesh_->prim_type_ = PRIMITIVE_TRIANGLELIST;
    mesh_->vb_first_ = 0;
    mesh_->vb_count_ = UINT32_MAX;
    mesh_->two_sided_ = 0;
    mesh_->tex_id_ = res_man_load_texture("default");

    state_ = Component::kInitialized;
}

void FrustumComponent::AddRenderPackets(struct RenderFrameContext* rfc) const {

    const mat4& view = rfc->view_;
    vec3 dir = view.getRow(2).xyz();
    vec3 right = view.getRow(0).xyz();
    vec3 up = view.getRow(1).xyz();
    vec4 pos = rfc->inv_view_ * vec4(0, 0, 0, 1);

    Frustum f;
    f.updateFromCamera(pos.xyz(), dir, right, up, rfc->fov_,
                              rfc->aspect_, rfc->z_near_,
                              rfc->z_far_);

    HGOSBUFFER mesh_vb = mesh_->vb_;
	ScheduleRenderCommand(rfc, [f, mesh_vb]() {
		const int vb_size = Frustum::kNUM_FRUSTUM_PLANES * 6;
		SVD vb[vb_size];
		Frustum::makeMeshFromFrustum(&f, (char*)&vb[0], vb_size, (int)sizeof(SVD));

		for (int i = 0; i < vb_size; ++i) {
			vb[i].uv = vec2(vb[i].pos.x, vb[i].pos.z);
			vb[i].normal = normalize(vb[i].pos);
		}

		gos_UpdateBuffer(mesh_vb, vb, 0, vb_size * sizeof(SVD));
	});
}

void FrustumComponent::DeinitRenderResources() {
    gos_DestroyBuffer(mesh_->vb_);
    gos_DestroyBuffer(mesh_->ib_);
    mesh_->tex_id_ = 0;
    delete mesh_;
    mesh_ = nullptr;

    state_ = Component::kUninitialized;
}

MeshObject *MeshObject::Create(const char *res) {
    static size_t obj_num = 0;
    MeshObject *obj = new MeshObject();
    obj->name_ = res;
    obj->name_ += std::to_string(obj_num++);

    auto tr = obj->AddComponent<TransformComponent>();

    obj->mesh_comp_ = MeshComponent::Create(res);
    obj->AddComponent(obj->mesh_comp_);
    obj->mesh_comp_->SetParent(tr);

    return obj;
}

