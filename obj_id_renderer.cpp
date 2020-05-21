#include "obj_id_renderer.h"
#include "renderer.h"
#include "gameos.hpp"

#include "engine/utils/gl_fbo.h"

bool ObjIdRenderer::Init(uint32_t width, uint32_t height)
{
    glGenFramebuffers(1, &obj_id_fbo_);

    width_ = width;
    height_ = height;
    uint32_t wh = (height_<<16) | width_;

    gos_obj_id_rt =
        gos_NewRenderTarget(gos_Texture_R32UI, "obj_id_rt", wh);
    g_obj_id_rt = gos_TextureGetNativeId(gos_obj_id_rt);

    gos_AddRenderMaterial("obj_id");

    return gos_obj_id_rt && obj_id_fbo_;
}

void ObjIdRenderer::Deinit()
{
    gosASSERT(gos_obj_id_rt && obj_id_fbo_);
    gos_DestroyTexture(gos_obj_id_rt);
    glDeleteFramebuffers(1, &obj_id_fbo_);
}

void ObjIdRenderer::Render(struct RenderFrameContext *rfc, GLuint scene_depth)
{
    glBindFramebuffer(GL_FRAMEBUFFER, obj_id_fbo_);
    {
        GLenum drawBuffers[] = {GL_COLOR_ATTACHMENT0};
        glDrawBuffers(sizeof(drawBuffers)/sizeof(drawBuffers[0]), drawBuffers);
        glReadBuffer(GL_NONE);
        glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, g_obj_id_rt, 0);
        // if we need ztest
        glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, scene_depth, 0);
        bool status = checkFramebufferStatus();
        assert(status);
    }

    gos_SetRenderViewport(0, 0, width_, height_);
    glViewport(0, 0, (GLsizei)width_, (GLsizei)height_);

    glClear(GL_COLOR_BUFFER_BIT);
    
    gos_SetRenderState(gos_State_AlphaMode, gos_Alpha_OneZero);
    gos_SetRenderState(gos_State_StencilEnable, 0);
    gos_SetRenderState(gos_State_Culling, gos_Cull_CCW);
    gos_SetRenderState(gos_State_ZCompare, false);
    gos_SetRenderState(gos_State_ZWrite, false);

    HGOSRENDERMATERIAL mat = gos_getRenderMaterial("obj_id");

    const mat4 vp = rfc->proj_ * rfc->view_;

    const RenderPacketList_t& rpl = rfc->rl_->GetRenderPackets();
    RenderPacketList_t::const_iterator it = rpl.begin();
    RenderPacketList_t::const_iterator end = rpl.end();
    int index = 1;
    for(;it!=end;++it)
    {
        const RenderPacket& rp = (*it);
        const RenderMesh& ro = rp.mesh_;

        if(!rp.is_debug_pass)
            continue;

        mat4 wvp = vp * rp.m_;

		gos_SetRenderMaterialParameterMat4(mat, "wvp_", (const float*)wvp);
        float obj_id[4] = { (float)index++, 0.0f, 0.0f, 0.0f }; 
        gos_SetRenderMaterialParameterFloat4(mat, "obj_id_", obj_id);

		gos_ApplyRenderMaterial(mat);

        if (ro.ib_) {
            gos_RenderIndexedArray(ro.ib_, ro.vb_, ro.vdecl_);
        } else if (ro.inst_vb_) {
            gos_RenderArrayInstanced(ro.vb_, ro.inst_vb_, ro.num_instances,
                                     ro.vdecl_);
        } else {
            gos_RenderArray(ro.vb_, ro.vdecl_);
        }
    }
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

uint32_t ObjIdRenderer::Readback(int x, int y)
{
    glBindFramebuffer(GL_FRAMEBUFFER, obj_id_fbo_);
    glReadBuffer(GL_COLOR_ATTACHMENT0);
    glDrawBuffer(GL_NONE);
    glFramebufferTexture2D(GL_READ_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, g_obj_id_rt, 0);
    bool status = checkFramebufferStatus();
    assert(status);

    uint32_t obj_id;
    glReadPixels(x, y, 1, 1, GL_RED_INTEGER, GL_UNSIGNED_INT, (GLvoid*)&obj_id);

    printf("x: %d y: %d -> obj_id: %d\n", x, y, obj_id);

    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    return obj_id;
    
}
