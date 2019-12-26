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
            gos_RenderIndexedArray(ro.ib_, ro.vb_, ro.vdecl_);
        } else if(ro.inst_vb_) {
    		gos_RenderArrayInstanced(ro.vb_, ro.inst_vb_, ro.num_instances, ro.vdecl_);
        } else {
    		gos_RenderArray(ro.vb_, ro.vdecl_);
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
            return (view_ * a.m_.getCol3()).z < (view_ * b.m_.getCol3()).z;
        }
    };

    RPSorter sorter;
    sorter.view_ = view;
    std::sort(tmp.begin(), tmp.end(), sorter);
//#define TEST_POINT_LIGTHS
#if defined(TEST_POINT_LIGTHS)
    gos_SetRenderState(gos_State_ZCompare, 3);
    gos_SetRenderState(gos_State_Culling, gos_Cull_CW);
#else
    gos_SetRenderState(gos_State_ZCompare, 1);
    gos_SetRenderState(gos_State_Culling, gos_Cull_CCW);
#endif
    //gos_SetRenderState(gos_State_AlphaMode, gos_Alpha_OneZero);
    gos_SetRenderState(gos_State_AlphaMode, gos_Alpha_AlphaOne);
    gos_SetRenderState(gos_State_ZWrite, 1);

    for(auto& rp: tmp)
        dr.render(rp);

    gos_SetRenderState(gos_State_AlphaMode, gos_Alpha_OneZero);
    gos_SetRenderState(gos_State_ZWrite, 1);
}
