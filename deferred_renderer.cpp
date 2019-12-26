#include "deferred_renderer.h"
#include "gameos.hpp"
#include "renderer.h"
#include "utils/gl_fbo.h"
#include <functional>

bool DeferredRenderer::Init(int width, int height)
{
    GLuint fbos[3]; 
    glGenFramebuffers(3, fbos);
    deferred_fbo_ = fbos[0];
    lighting_fbo_ = fbos[1];
    forward_fbo_ = fbos[2];

    height_ = height;
    width_ = width;
    uint32_t wh = (height_<<16) | width_;

    gos_g_buffer_depth =
        gos_NewRenderTarget(gos_Texture_Depth, "g_bufer_depth", wh);
    g_buffer_depth = gos_TextureGetNativeId(gos_g_buffer_depth);

    gos_g_buffer_albedo =
        gos_NewRenderTarget(gos_Texture_RGBA8, "g_buffer_albedo", wh);
    g_buffer_albedo = gos_TextureGetNativeId(gos_g_buffer_albedo);

    gos_g_buffer_normal =
        gos_NewRenderTarget(gos_Texture_RGBA8, "g_buffer_normal", wh);
    g_buffer_normal = gos_TextureGetNativeId(gos_g_buffer_normal);

    gos_backbuffer =
        gos_NewRenderTarget(gos_Texture_RGBA8, "backbuffer", wh);
    backbuffer = gos_TextureGetNativeId(gos_backbuffer);

    GLuint t[] = { g_buffer_depth, g_buffer_albedo, g_buffer_normal, backbuffer};
    for(GLuint tex: t) {
        glBindTexture(GL_TEXTURE_2D, tex);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glBindTexture(GL_TEXTURE_2D, 0);
    }

    checker_tex_ = gos_NewTextureFromFile(gos_Texture_Detect, "data/textures/texture_density.tga");

    gos_AddRenderMaterial("deferred");
    gos_AddRenderMaterial("lightpass");
    gos_AddRenderMaterial("pointlightpass");

    bool status = false;
    // setup deferred fbo
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, deferred_fbo_);
    {
        GLenum drawBuffers[] = { GL_COLOR_ATTACHMENT0, GL_COLOR_ATTACHMENT1};
        glDrawBuffers(sizeof(drawBuffers)/sizeof(drawBuffers[0]), drawBuffers);
        glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, g_buffer_albedo, 0);
        glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, GL_COLOR_ATTACHMENT1, GL_TEXTURE_2D, g_buffer_normal, 0);
        glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, g_buffer_depth, 0);
        status = checkFramebufferStatus();
        assert(status);
    }
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);

    // setup forward fbo
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, lighting_fbo_);
    {
        GLuint drawBuffers[] = { GL_COLOR_ATTACHMENT0 };
        glDrawBuffers(sizeof(drawBuffers)/sizeof(drawBuffers[0]), drawBuffers);
        glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, backbuffer, 0);
        status = checkFramebufferStatus();
        assert(status);
    }

    // setup forward fbo
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, forward_fbo_);
    {
        GLuint drawBuffers[] = { GL_COLOR_ATTACHMENT0 };
        glDrawBuffers(sizeof(drawBuffers)/sizeof(drawBuffers[0]), drawBuffers);
        glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, backbuffer, 0);
        glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, g_buffer_depth, 0);
        status = checkFramebufferStatus();
        assert(status);
    }
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);


    return true;
}

class DeferredShapeRenderer {
	mat4 view_proj_;
	mat4 view_;
    DWORD missing_tex_;
public:
	void setup(const mat4& view, const mat4& proj, DWORD missing_tex) {
		view_proj_ =  proj * view;
        view_ = view;
        missing_tex_ = missing_tex;
	}

    void render(const RenderPacket &rp) {
        const RenderMesh& ro = rp.mesh_;

        HGOSRENDERMATERIAL mat = gos_getRenderMaterial("deferred");

        DWORD texid = ro.tex_id_ ? ro.tex_id_ : missing_tex_; 

        gos_SetRenderState(gos_State_Texture, texid);
        gos_SetRenderState(gos_State_Filter, gos_FilterBiLinear);

        mat4 wvp = view_proj_ * rp.m_;
        mat4 wv = view_ * rp.m_;
		gos_SetRenderMaterialParameterMat4(mat, "wvp_", (const float*)wvp);
		gos_SetRenderMaterialParameterMat4(mat, "wv_", (const float*)wv);

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

void DeferredRenderer::RenderGeometry(struct RenderFrameContext* rfc)
{
    const RenderPacketList_t& rpl = rfc->rl_->GetRenderPackets();
    RenderPacketList_t::const_iterator it = rpl.begin();
    RenderPacketList_t::const_iterator end = rpl.end();

    DeferredShapeRenderer r;
    r.setup(rfc->view_, rfc->proj_, checker_tex_);

    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, deferred_fbo_);

    gos_SetRenderViewport(0, 0, width_, height_);
    glViewport(0, 0, (GLsizei)width_, (GLsizei)height_);

    glClear(GL_DEPTH_BUFFER_BIT|GL_COLOR_BUFFER_BIT);

    gos_SetRenderState(gos_State_ZCompare, 1);
    gos_SetRenderState(gos_State_ZWrite, true);
    gos_SetRenderState(gos_State_AlphaMode, gos_Alpha_OneZero);
    gos_SetRenderState(gos_State_Culling, gos_Cull_CCW);
    for(;it!=end;++it)
    {
        const RenderPacket& rp = (*it);
        if(rp.is_opaque_pass)
            r.render(rp);
    }
}

void DeferredRenderer::RenderDirectionalLighting(const struct RenderFrameContext* rfc)
{
    glBindFramebuffer(GL_FRAMEBUFFER, lighting_fbo_);

    gos_SetRenderViewport(0, 0, width_, height_);
    glViewport(0, 0, (GLsizei)width_, (GLsizei)height_);

    glClear(GL_COLOR_BUFFER_BIT);
    
    gos_SetRenderState(gos_State_Texture, gos_g_buffer_albedo);
    gos_SetRenderState(gos_State_Texture2, gos_g_buffer_normal);
    gos_SetRenderState(gos_State_Texture3, gos_g_buffer_depth);
    gos_SetRenderState(gos_State_Culling, gos_Cull_CW);
    gos_SetRenderState(gos_State_ZCompare, false);
    gos_SetRenderState(gos_State_ZWrite, false);

    vec4 lightdir = -rfc->shadow_view_.getRow(2);
    vec4 lightdir_view_space = rfc->view_ * vec4(lightdir.x, lightdir.y, lightdir.z, 0.0f);

    HGOSRENDERMATERIAL mat = gos_getRenderMaterial("lightpass");
    const float ld_vs[4] = {lightdir_view_space.x, lightdir_view_space.y, lightdir_view_space.z, 0.0f };
    gos_SetRenderMaterialParameterFloat4(mat, "lightdir_view_space_", ld_vs);
    gos_SetRenderMaterialParameterMat4(mat, "proj_", rfc->proj_);
    gos_SetRenderMaterialParameterMat4(mat, "inv_proj_", rfc->inv_proj_);

    gos_ApplyRenderMaterial(mat);

    extern RenderMesh* g_fs_quad;
    gos_RenderIndexedArray(g_fs_quad->ib_, g_fs_quad->vb_, g_fs_quad->vdecl_);
}

void DeferredRenderer::RenderPointLighting(struct RenderFrameContext* rfc)
{
    ///glBindFramebuffer(GL_FRAMEBUFFER, lighting_fbo_);
    glBindFramebuffer(GL_FRAMEBUFFER, forward_fbo_);

    gos_SetRenderViewport(0, 0, width_, height_);
    glViewport(0, 0, (GLsizei)width_, (GLsizei)height_);

    gos_SetRenderState(gos_State_Texture, gos_g_buffer_albedo);
    gos_SetRenderState(gos_State_Texture2, gos_g_buffer_normal);
    gos_SetRenderState(gos_State_Texture3, gos_g_buffer_depth);
    gos_SetRenderState(gos_State_Culling, gos_Cull_CW);
    gos_SetRenderState(gos_State_ZCompare, 3);
    //make sure we do not write in depth because we are using it as depth rt
    gos_SetRenderState(gos_State_ZWrite, false);
    // additive blending
    gos_SetRenderState(gos_State_AlphaMode, gos_Alpha_AlphaOne);

    const auto& point_lights = rfc->point_lights_;
    const mat4 view_proj = rfc->proj_ * rfc->view_;

    HGOSRENDERMATERIAL mat = gos_getRenderMaterial("pointlightpass");
    for (const PointLight &l : point_lights) {

        const mat4 wvp = view_proj * l.transform_;
        const mat4 wv = rfc->view_ * l.transform_;
        // what is better?
        const vec4 viewpos = wv * vec4(0, 0, 0, 1);
        //const vec4 viewpos = rfc->view_ * vec4(l.pos.x, l.pos.y, l.pos.z, 1);

        const float viewpos_radius[4] = { viewpos.x, viewpos.y, viewpos.z, l.radius_ };

        gos_SetRenderMaterialParameterFloat4(mat, "color_", l.color_);
        gos_SetRenderMaterialParameterFloat4(mat, "viewpos_radius_", viewpos_radius);
        gos_SetRenderMaterialParameterMat4(mat, "wv_", wv);
        gos_SetRenderMaterialParameterMat4(mat, "wvp_", wvp);
        gos_SetRenderMaterialParameterMat4(mat, "proj_", rfc->proj_);

        gos_ApplyRenderMaterial(mat);

        extern RenderMesh* g_sphere;
        gos_RenderIndexedArray(g_sphere->ib_, g_sphere->vb_, g_sphere->vdecl_);
    }
}

void DeferredRenderer::RenderForward(std::function<void(void)> f)
{
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, forward_fbo_);

    gos_SetRenderState(gos_State_Texture, 0);
    gos_SetRenderState(gos_State_Texture2, 0);
    gos_SetRenderState(gos_State_Texture3, 0);

    gos_SetRenderViewport(0, 0, width_, height_);
    glViewport(0, 0, (GLsizei)width_, (GLsizei)height_);

    gos_SetRenderState(gos_State_ZCompare, 1);
    gos_SetRenderState(gos_State_ZWrite, false);
    gos_SetRenderState(gos_State_AlphaMode, gos_Alpha_OneZero);
    gos_SetRenderState(gos_State_Culling, gos_Cull_CCW);

    f();
}

void DeferredRenderer::Present(int w, int h)
{
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glDrawBuffer(GL_BACK);
    glClear(GL_DEPTH_BUFFER_BIT|GL_COLOR_BUFFER_BIT);

	gos_SetRenderViewport(0, 0, w, h);
    glViewport(0, 0, (GLsizei)w, (GLsizei)h);

    
    gos_SetRenderState(gos_State_Texture, gos_backbuffer);
    gos_SetRenderState(gos_State_Culling, gos_Cull_CW);
    gos_SetRenderState(gos_State_ZCompare, false);
    gos_SetRenderState(gos_State_ZWrite, 1);

    HGOSRENDERMATERIAL mat = gos_getRenderMaterial("textured_quad");

    gos_ApplyRenderMaterial(mat);

    extern RenderMesh* g_fs_quad;
    gos_RenderIndexedArray(g_fs_quad->ib_, g_fs_quad->vb_, g_fs_quad->vdecl_);
}



