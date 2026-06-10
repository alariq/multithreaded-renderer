#include "Billboard.h"
#include "res_man.h"
#include "utils/camera.h"

void C_Billboard::UpdateComponent(float dt) {

    // TODO: this should be done in AddRenderPackets, as camera now updates
    // last, and anyway this is only rendering related

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

IRenderProxy* C_Billboard::CreateRenderProxy() {
    MeshComponent::CreateRenderProxy();
    proxy_->b_is_opaque_pass = true;
    proxy_->b_is_forward_pass = true;
    return proxy_;
}

PROPERTY_LIST_BEGIN_DERIVED(C_Billboard, MeshComponent)
    PROPERTY_SECTION("C_Billboard");
    PROPERTY_BOOL(b_horizontal_only_, "horizontal only");
PROPERTY_LIST_END()


O_Billboard* O_Billboard::Create(const char *name) {
    static size_t obj_num = 0;
    O_Billboard *obj = new O_Billboard();
    obj->name_ = name;
    obj->name_ += std::to_string(obj_num++);
    auto c_bb = scene_create_component<C_Billboard>(obj);
    c_bb->SetMesh("xy_quad");
    // small usability hack: try to set texture same as object name
    // TODO: this all needs res manager rework
    c_bb->SetTexture(name); 
    return obj;
}


PROPERTY_LIST_BEGIN_DERIVED(O_Billboard, GameObject)
    PROPERTY_READONLY_TEXT("Name", [](const O_Billboard& o){ return o.GetName();});
PROPERTY_LIST_END()
