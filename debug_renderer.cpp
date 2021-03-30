#include "debug_renderer.h"
#include "gameos.hpp"
#include <algorithm>

class DebugRenderer {

	mat4 vp_;

public:

	void setup(const mat4& view, const mat4& proj) {
        vp_ = proj * view;
	}

    void render(const RenderPacket &rp) {
        const RenderMesh& ro = rp.mesh_;

        HGOSRENDERMATERIAL mat = gos_getRenderMaterial("debug");
        mat4 wvp = vp_ * rp.m_;
		gos_SetRenderMaterialParameterMat4(mat, "wvp_", (const float*)wvp);
		gos_SetRenderMaterialParameterFloat4(mat, "colour_", (const float*)rp.debug_color);

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

void RenderDebugObjects(const RenderPacketList_t& rpl, const mat4& view, const mat4& proj)
{
    RenderPacketList_t::const_iterator it = rpl.begin();
    RenderPacketList_t::const_iterator end = rpl.end();

    DebugRenderer dr;
    dr.setup(view, proj);

    std::vector<RenderPacket> tmp;
    for(;it!=end;++it)
    {
        const RenderPacket& rp = (*it);
        if(rp.is_debug_pass)
            tmp.push_back(rp);
    }

    struct RPSorter {
        mat4 view_;
        bool operator()(const RenderPacket& a, const RenderPacket& b) {
            if(a.is_gizmo_pass && !b.is_gizmo_pass) {
                return true;
            } else if(!a.is_gizmo_pass && b.is_gizmo_pass) {
                return false;
            }
            return (view_ * a.m_.getCol3()).z < (view_ * b.m_.getCol3()).z;
        }
    };

    const bool depth_test = false;
    const bool b_double_sided = true;
	const gos_CullMode cull_mode = b_double_sided ? gos_Cull_None: gos_Cull_CCW;

	RPSorter sorter;
    sorter.view_ = view;
    std::sort(tmp.begin(), tmp.end(), sorter);
//#define TEST_POINT_LIGTHS
#if defined(TEST_POINT_LIGTHS)
    gos_SetRenderState(gos_State_ZCompare, 3);
    gos_SetRenderState(gos_State_Culling, gos_Cull_CW);
#else
    gos_SetRenderState(gos_State_ZCompare, depth_test);
    gos_SetRenderState(gos_State_Culling, cull_mode);
#endif
    //gos_SetRenderState(gos_State_AlphaMode, gos_Alpha_OneZero);
    gos_SetRenderState(gos_State_AlphaMode, gos_Alpha_AlphaOne);
    gos_SetRenderState(gos_State_ZWrite, 1);

    for(auto& rp: tmp)
        dr.render(rp);

    gos_SetRenderState(gos_State_AlphaMode, gos_Alpha_OneZero);
    gos_SetRenderState(gos_State_ZWrite, 1);
}

