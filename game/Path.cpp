#include "obj_model.h"
#include "game/Path.h"
#include "renderer.h"
#include "res_man.h"
#include "render_utils.h"
#include <assert.h>
#include "engine/utils/vec.h"
#include "editor.h"

void C_Curve::SetCurve(Curve<vec3>* c) {
    curve_ = c;
}

int32_t C_Curve::GetNumNodes() const {
    return curve_->getNumSegments();
}

vec3 C_Curve::Get(float t) const {
    return curve_->getAt(t);
}

vec3 C_Curve::GetDerivative(float t) const {
    return curve_->getDerivativeAt(t);
}

void C_Curve::Initialize() {
    TransformComponent::Initialize();

}

void C_Curve::UpdateComponent(float dt) {
    //for(int i=0;i<curve_->getNumNodes();++i) {
    //}
}

void C_Curve::AddRenderPackets(struct RenderFrameContext* rfc) const {
    CurveDebugDraw(*curve_, 10, false, &GetTransform(), rfc->rl_);

    // :(
    ITransformInterface* ti = const_cast<ITransformInterface*>((const ITransformInterface*)this);
    RenderMesh* mesh = res_man_load_mesh("sphere");
    for(int i=0;i<curve_->getNumNodes();++i) {
        vec3 node_pos = curve_->getNodeValue(i);
        vec3 pos = this->Transform(node_pos);
        const int id = editor_add_gizmo(ti, (void*)(ptrdiff_t)i);
        add_debug_mesh_constant_size_px(rfc, mesh, 1, vec4(0.25f, 0.0125f, 0.0125f, 1), mat4::translation(pos), 10, id);
    }
}

void C_Curve::SetPosition(vec3 p, void* userdata) {
    int idx = (int)((ptrdiff_t)userdata & 0xffffffff);
    assert(!isnan(p.x));
    curve_->setNodeValue(idx, p);

}
vec3 C_Curve::GetPosition(void* userdata) const {
    int idx = (int)((ptrdiff_t)userdata & 0xffffffff);
    return curve_->getNodeValue(idx);
}

PROPERTY_LIST_BEGIN_DERIVED(C_Curve, TransformComponent)
    PROPERTY_SECTION("C_Curve");
PROPERTY_LIST_END()


O_Path* O_Path::Create(const char *name) {
    static size_t obj_num = 0;
    O_Path *obj = new O_Path();
    obj->name_ = name;
    obj->name_ += std::to_string(obj_num++);

    auto tr = obj->AddComponent<TransformComponent>();
    auto cc = obj->AddComponent<C_Curve>();
    cc->SetParent(tr);

    return obj;
}

PROPERTY_LIST_BEGIN_DERIVED(O_Path, GameObject)
    PROPERTY_READONLY_TEXT("Name", [](const O_Path& o){ return o.GetName();});
PROPERTY_LIST_END()


void CurveDebugDraw(const Curve<vec3>& curve, int num_pts_per_segment, bool b_draw_basis, const mat4* transform, RenderList* rl) {
    const int nseg = curve.getNumSegments();
    vec3 p_prev = curve.getAt(0);
    const float num_pts = num_pts_per_segment;

    int num_lines = nseg * (int)num_pts;
    num_lines = 2 * num_lines; // also drawing derivative
                               //
    vec3* dbg_pts = new vec3[2*num_lines];
    vec4* colours = new vec4[num_lines];
    int li = 0;

    for(int s = 0; s<nseg; s++) {

        for (size_t i = 0; i < num_pts; ++i) {
            const float t = (float)i / num_pts;
            const float alpha = 1;
            const float brightness = 1;//0.35f + 0.65f * t;
            vec4 colour(0.8f * brightness, 0.8f * brightness, .1f * brightness, alpha);
            vec3 p = curve.getAt((float)s + t);
            const vec3 dp = curve.getDerivativeAt((float)s + t);
            const float vmag = length(dp);
            const vec3 vdir = dp/vmag;
            const vec4 vc = saturate(vec4(0.5f*vdir + vec3(0.5f), 1) * vec4(0.05f*vmag, 0.05f*vmag, 0.05f*vmag, 1.0f));

            p = transform ? ((*transform) * vec4(p, 1)).xyz() : p;

            //rl->addDebugLine(p_prev, p, colour);
            //rl->addDebugLine(p, p + 0.1f*dp, vc);
            dbg_pts[2*li + 0] = p_prev;
            dbg_pts[2*li + 1] = p;
            colours[li] = colour;
            li++;
            dbg_pts[2*li + 0] = p;
            dbg_pts[2*li + 1] = p + 0.1f*dp;
            colours[li] = vc;
            li++;


            p_prev = p;


            if(b_draw_basis) {
                vec3 right, up, fwd;
                spline_get_basis_at(curve, (float)s + t, 0.5f/num_pts, right, up, fwd);
                rl->addDebugLine(p, p + 1*right, vec4(1, 0,0, 1));
                rl->addDebugLine(p, p + 1*up, vec4(0, 1,0, 1));
                rl->addDebugLine(p, p + 1*fwd, vec4(0, 0,1, 1));
            }
        }
    }
    assert(li == num_lines);
    rl->addDebugLines(dbg_pts, colours, vec4(1), num_lines);
    delete[] dbg_pts;
    delete[] colours;
}

