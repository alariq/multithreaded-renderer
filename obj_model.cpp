#include "res_man.h"
#include "obj_model.h"
#include "particle_system.h"
#include "utils/obj_loader.h"

ParticleSystemObject* ParticleSystemObject::Create()
{
    ParticleSystem* ps = new ParticleSystem;

    ParticleSystemObject* pso = new ParticleSystemObject;
    pso->ps_ = ps;

    return pso;
}

void ParticleSystemObject::InitRenderResources() {

    ps_->AddEmitter(CreateStandardEmitter());
    ParticleSystemManager::Instance().Add(ps_);
}

ParticleSystemObject::~ParticleSystemObject() {
    ParticleSystemManager::Instance().Remove(ps_);
    delete ps_;
}

FrustumObject *FrustumObject::Create(const camera *pcam) {
    FrustumObject *o =
        new FrustumObject(/*pcam->view_, pcam->get_fov(), pcam->get_aspect(),
                          pcam->get_near(), pcam->get_far()*/);
    o->UpdateFrustum(pcam);
    return o;
}

void FrustumObject::UpdateFrustum(const camera *pcam) {
    mat4 view;
    pcam->get_view(&view);
    vec3 dir = view.getRow(2).xyz();
    vec3 right = view.getRow(0).xyz();
    vec3 up = view.getRow(1).xyz();
    vec4 pos = pcam->inv_view_ * vec4(0, 0, 0, 1);

    frustum_.updateFromCamera(pos.xyz(), dir, right, up, pcam->get_fov(),
                              pcam->get_aspect(), pcam->get_near(),
                              pcam->get_far());

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

void FrustumObject::InitRenderResources() {

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

    Frustum::makeMeshFromFrustum(&frustum_, (char *)&vb[0], vb_size,
                                 (int)sizeof(SVD));

    for (int i = 0; i < vb_size; ++i) {
        vb[i].uv = vec2(vb[i].pos.x, vb[i].pos.z);
        vb[i].normal = normalize(vb[i].pos);
    }

    mesh_->vb_ =
        gos_CreateBuffer(gosBUFFER_TYPE::VERTEX, gosBUFFER_USAGE::DYNAMIC_DRAW,
                         sizeof(SVD), vb_size, vb);
    mesh_->tex_id_ = res_man_load_texture("default");
}

void FrustumObject::DeinitRenderResources()
{
    delete mesh_;
}

MeshObject *MeshObject::Create(const char *res) {
    static size_t obj_num = 0;
    MeshObject *obj = new MeshObject();
    obj->name_ = res;
    obj->name_ += std::to_string(obj_num++);
    obj->mesh_name_ = res;

    obj->AddComponent<TransformComponent>();

    return obj;
}

void MeshObject::InitRenderResources() {
    // assert(IsOnRenderThread());
    mesh_ = res_man_load_mesh(mesh_name_);
}

