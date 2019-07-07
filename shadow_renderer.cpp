#include "renderer.h"
#include "engine/utils/gl_utils.h"
#include "engine/utils/gl_fbo.h"

bool ShadowRenderPass::Init(uint32_t size)
{
    width_ = size;
    height_ = size;

	glGenFramebuffers(1, &fbo_);

	gos_depth_texture_ = gos_NewRenderTarget(gos_Texture_Depth, "shadow_map", width_);
    depth_texture_id_ = gos_TextureGetNativeId(gos_depth_texture_);

    glBindTexture(GL_TEXTURE_2D, depth_texture_id_);
    setSamplerParams(TT_2D, TAM_CLAMP_TO_EDGE, TFM_NEAREST);
    glBindTexture(GL_TEXTURE_2D, 0);

    glBindFramebuffer(GL_FRAMEBUFFER, fbo_);
	glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, depth_texture_id_, 0);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    return true;
}

void ShadowRenderPass::Render(const struct camera* shadow_camera, const RenderPacketList_t& rpl)
{
    RenderPacketList_t::const_iterator it = rpl.begin();
    RenderPacketList_t::const_iterator end = rpl.end();

    mat4 view_m;
    mat4 proj_m;
    shadow_camera->get_view(&view_m);
    shadow_camera->get_projection(&proj_m);

    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, fbo_);

    glDrawBuffer(GL_NONE);
    glReadBuffer(GL_NONE);

    checkFramebufferStatus();

    glClear(GL_DEPTH_BUFFER_BIT);

    gos_SetRenderViewport(0, 0, width_, height_);

    glViewport(0, 0, (GLsizei)width_, (GLsizei)height_);

    for(;it!=end;++it)
    {
        const RenderPacket& rp = (*it);

        RenderMesh& ro = *rp.mesh_;

        HGOSRENDERMATERIAL mat = gos_getRenderMaterial("directional_shadow");

        mat4 wvp = proj_m * view_m * rp.m_;

		gos_SetRenderMaterialParameterMat4(mat, "wvp_", (const float*)wvp);

		gos_ApplyRenderMaterial(mat);

		gos_RenderIndexedArray(ro.ib_, ro.vb_, ro.vdecl_);
    }

    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    glDrawBuffer(GL_BACK);

}

ShadowRenderPass::~ShadowRenderPass()
{
    glDeleteFramebuffers(1, &fbo_);
	gos_DestroyTexture(gos_depth_texture_);
}
