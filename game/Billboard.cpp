#include "Billboard.h"
#include "res_man.h"
#include "utils/camera.h"

C_Billboard* C_Billboard::Create(const char* name) {
    C_Billboard* comp = new C_Billboard();
    return comp;
}

void C_Billboard::InitRenderResources() {
    // assert(IsOnRenderThread());
    mesh_ = res_man_load_mesh("xy_quad");
    //mesh_ = res_man_load_mesh("gizmo");
    texture_id_ = res_man_load_texture("texture_density8x8");
	// !NB: strictly speaking cannot modify state on render thread, we can have an Update
	// method called for component even if it is not initialized, so that on game thread
	// this update method will set state_ = initState.load();
	state_ = Component::kInitialized;
}
void C_Billboard::DeinitRenderResources() {
    // assert(IsOnRenderThread());
    res_man_release_mesh(mesh_);
    //res_man_release_texture("texture_density_8x8");
    mesh_ = nullptr;
    state_ = Component::kUninitialized;
}

void C_Billboard::SetTexure(const char* texture_name) {
    pending_texture_name_ = texture_name;
    texture_name_ = texture_name;
}

void C_Billboard::UpdateComponent(float dt) {

    if(pending_mesh_) {
        mesh_ = static_cast<decltype(mesh_)>(pending_mesh_.load());
    }
    if(pending_texture_id_) {
        texture_id_ = pending_texture_id_.load(); // what the hell is this logick all about? TODO: rewrite
    }

    const SceneViewInfo& si = scene_get_view_info();

	vec3 cam_pos = si.inv_view_mat_.getTranslation();
    vec3 my_pos = GetPosition();

    vec3 fwd = cam_pos - my_pos;
    float fwd_len = length(fwd);
    fwd = fwd_len < 1e-9 ? vec3(0,0,1) : fwd / fwd_len;
    vec3 up = vec3(0,1,0);
    vec3 right = cross(up, fwd);
    float right_len = length(right);
    if( right_len < 1e-9) {
        calculate_basis(fwd, up, right);
    } else {
        right = normalize(right);
        up = cross(fwd, right);
    }

    mat3 m = mat3::fromBasis(right, up, fwd);
    const quaternion qnew = mat3_to_quat(m); 
    SetRotation(qnew);

    //mat4 tr = camera::make_lookat(my_pos, cam_pos, vec3(0,1,0));
    //const quaternion qnew = inverse(mat4_to_quat(tr));
    //SetRotation(qnew);
}

void C_Billboard::AddRenderPackets(struct RenderFrameContext* rfc) const {

    if(!pending_mesh_name_.empty()) {
        auto mesh2load = pending_mesh_name_;
        pending_mesh_name_.clear();
		ScheduleRenderCommand(rfc, [this, mesh2load]() {
			RenderMesh* mesh = res_man_load_mesh(mesh2load);
            this->pending_mesh_.store((void*)mesh);
		});
	}

    if(!pending_texture_name_.empty()) {
        auto texture_name = pending_texture_name_;
        pending_texture_name_.clear();
		ScheduleRenderCommand(rfc, [this, texture_name]() {
			this->pending_texture_id_.store(res_man_load_texture(texture_name));
		});
    }

    if(mesh_) {
        class RenderList *rl = rfc->rl_;
        RenderPacket *rp = rl->AddPacket();
        memset(rp, 0, sizeof(RenderPacket));

        mesh_->tex_id_ = texture_id_;

        rp->mesh_ = *mesh_;
        //rp->mesh_.two_sided_ = 1;
        rp->m_ = GetTransform();
        //rp->is_debug_pass = 1;
        rp->is_forward_pass = 1;
        rp->debug_color = vec4(0, 1, 0, 1);
    }
}

PROPERTY_LIST_BEGIN_DERIVED(C_Billboard, TransformComponent)
    PROPERTY_SECTION("C_Billboard");
    PROPERTY_READONLY_TEXT("TextureName", [](const C_Billboard& c){ return c.texture_name_;});
    PROPERTY_TEXT("Texture", 
            [](const C_Billboard& c){ return c.texture_name_;},
            [](C_Billboard& c, const std::string& s){ return c.SetTexure(s.c_str());});
PROPERTY_LIST_END()


O_Billboard* O_Billboard::Create(const char *name) {
    static size_t obj_num = 0;
    O_Billboard *obj = new O_Billboard();
    obj->name_ = name;
    obj->name_ += std::to_string(obj_num++);
    auto c_bb = obj->AddComponent<C_Billboard>();
    // small usability hack: try to set texture same as object name
    // TODO: this all needs res manager rework
    c_bb->SetTexure(name); 
    return obj;
}


PROPERTY_LIST_BEGIN_DERIVED(O_Billboard, GameObject)
    PROPERTY_READONLY_TEXT("Name", [](const O_Billboard& o){ return o.GetName();});
PROPERTY_LIST_END()
