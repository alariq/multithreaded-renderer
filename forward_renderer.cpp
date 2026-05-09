#include "forward_renderer.h"
#include "gameos.hpp"
#include "profiler/profiler.h"
#include <algorithm>

class ForwardShapeRenderer {

	mat4 vp_;
    mat4 view_;
    mat4 shadow_view_;

    DWORD missing_tex_;
    HGOSTEXTURESAMPLER sampler_;

public:

	void setup(const mat4& view, const mat4& proj, const mat4& shadow_view, HGOSTEXTURESAMPLER sampler) {
        vp_ = proj * view;
        view_ = view;
        shadow_view_ = shadow_view;
        sampler_ = sampler;
	}

    void render(const RenderPacket &rp) {
        const RenderMesh& ro = rp.mesh_;

        HGOSRENDERMATERIAL mat = gos_getRenderMaterial("forward");
        mat4 wvp = vp_ * rp.m_;
        mat4 wv = view_ * rp.m_;

        vec4 lightdir = -shadow_view_.getRow(2);
        vec4 lightdir_view_space = view_ * vec4(lightdir.x, lightdir.y, lightdir.z, 0.0f);

        const float ld_vs[4] = {lightdir_view_space.x, lightdir_view_space.y, lightdir_view_space.z, 0.0f };

		gos_SetRenderMaterialParameterMat4(mat, "wv_", (const float*)wv);
        gos_SetRenderMaterialParameterFloat4(mat, "lightdir_view_space_", ld_vs);
		gos_SetRenderMaterialParameterMat4(mat, "wvp_", (const float*)wvp);

        DWORD texid = ro.tex_id_ ? ro.tex_id_ : 0;//missing_tex_; 
        gos_SetRenderState(gos_State_Texture, texid);
        gos_SetSamplerState(0, sampler_);

		gos_ApplyRenderMaterial(mat);

        if (ro.ib_) {
            gos_RenderIndexedArray(ro.ib_, ro.vb_, ro.vdecl_, ro.prim_type_);
        } else if(ro.inst_vb_) {
    		gos_RenderArrayInstanced(ro.vb_, ro.inst_vb_, ro.num_instances, ro.vdecl_, ro.prim_type_);
        } else {
    		gos_RenderArray(ro.vb_, ro.vdecl_, ro.prim_type_, ro.vb_first_, ro.vb_count_);
        }
    }
};

bool ForwardRenderer::Init() {

    smp_linear_wrap_mips_ = gos_CreateTextureSampler(
        gos_TextureWrap, gos_TextureWrap, gos_TextureWrap, gos_FilterBiLinear,
        gos_FilterBiLinear, gos_FilterBiLinear, true);

    return gos_AddRenderMaterial("forward");
}

void ForwardRenderer::Render(const struct RenderFrameContext* rfc)
{
    SCOPED_GPU_ZONE(RenderForwardObjects);
    SCOPED_ZONE_N(RenderForwardObjects, 0);

    const RenderPacketList_t& rpl = rfc->rl_->GetRenderPackets();
    RenderPacketList_t::const_iterator it = rpl.begin();
    RenderPacketList_t::const_iterator end = rpl.end();

    ForwardShapeRenderer fr;
    const mat4& view_m = rfc->view_;
    fr.setup(view_m, rfc->proj_, rfc->shadow_view_, smp_linear_wrap_mips_);

    std::vector<RenderPacket> tmp;
    for(;it!=end;++it)
    {
        const RenderPacket& rp = (*it);
        if(rp.is_forward_pass)
        {
            gosASSERT(!rp.is_gizmo_pass);
            gosASSERT(!rp.is_particle_pass);
            tmp.push_back(rp);
        }

    }

    struct RPSorter {
        mat4 view_;
        bool operator()(const RenderPacket& a, const RenderPacket& b) {
            return (view_ * a.m_.getCol3()).z < (view_ * b.m_.getCol3()).z;
        }
    };

    const bool b_double_sided = true;
	const gos_CullMode cull_mode = b_double_sided ? gos_Cull_None: gos_Cull_CCW;

	RPSorter sorter;
    sorter.view_ = view_m;
    std::sort(tmp.begin(), tmp.end(), sorter);

    //TODO: forward pass could be also just solid meshes (when I actually will have real meterials)
    // so grab all this from material data

    gos_SetRenderState(gos_State_ZCompare, 1);
    gos_SetRenderState(gos_State_ZWrite, 0);
    gos_SetRenderState(gos_State_Culling, cull_mode);
    gos_SetRS(gosRSStencil{.enable = false});

    // premultiplied transparency
    gos_SetRenderState(gos_State_AlphaMode, gos_Alpha_AlphaOne);

    for(auto& rp: tmp)
        fr.render(rp);

    gos_SetRenderState(gos_State_AlphaMode, gos_Alpha_OneZero);
    gos_SetRenderState(gos_State_ZWrite, 1);
}

