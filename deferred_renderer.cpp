#include "deferred_renderer.h"
#include "gameos.hpp"
#include "res_man.h"
#include "renderer.h"
#include "utils/gl_fbo.h"
#include <functional>

bool DeferredRenderer::Init(uint32_t width, uint32_t height)
{
    GLuint fbos[5]; 
    glGenFramebuffers(5, fbos);
    deferred_fbo_ = fbos[0];
    lighting_fbo_ = fbos[1];
    forward_fbo_ = fbos[2];
    stencil_fbo_ = fbos[3];
    downsampled_fbo_ = fbos[4];

    // as we downsample make sure we are at least multiple of 2
    assert(0 == (height&0x1) && 0 == (width&0x1));

    width_ = width;
    height_ = height;
    uint32_t wh = (height_<<16) | width_;

    bool use_stencil = false;
    gos_TextureFormat depth_format = use_stencil ? gos_Texture_Depth_Stencil : gos_Texture_Depth;

    gos_g_buffer_depth =
        gos_NewRenderTarget(depth_format, "g_bufer_depth", wh);
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

    // downsampled buffers (used for particle rendering)
    // TODO: move them to particle related code
    ds_width_ = width / 2;
    ds_height_ = height / 2;
    uint32_t ds_wh = (ds_height_<<16) | ds_width_;
    gos_downsampled_color =
        gos_NewRenderTarget(gos_Texture_RGBA8, "downsampled_color", ds_wh);
    downsampled_color = gos_TextureGetNativeId(gos_downsampled_color);
    gos_downsampled_depth =
        gos_NewRenderTarget(depth_format, "downsampled_depth", ds_wh);
    downsampled_depth = gos_TextureGetNativeId(gos_downsampled_depth);

    GLuint t[] = {g_buffer_depth, g_buffer_albedo,   g_buffer_normal,
                  backbuffer,     downsampled_color, downsampled_depth};
    for(GLuint tex: t) {
        glBindTexture(GL_TEXTURE_2D, tex);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glBindTexture(GL_TEXTURE_2D, 0);
    }

    smp_nearest_clamp_nomips_ = gos_CreateTextureSampler(
        gos_TextureClamp, gos_TextureClamp, gos_TextureClamp, gos_FilterNone,
        gos_FilterNone, gos_FilterNone, false);

    smp_linear_wrap_nomips_ = gos_CreateTextureSampler(
        gos_TextureWrap, gos_TextureWrap, gos_TextureWrap, gos_FilterBiLinear,
        gos_FilterBiLinear, gos_FilterNone, false);

    smp_linear_clamp_nomips_ = gos_CreateTextureSampler(
        gos_TextureClamp, gos_TextureClamp, gos_TextureClamp, gos_FilterBiLinear,
        gos_FilterBiLinear, gos_FilterNone, false);

    checker_tex_ = gos_NewTextureFromFile(gos_Texture_Detect,
                                          "data/textures/texture_density.tga");

    gos_AddRenderMaterial("deferred");
    gos_AddRenderMaterial("lightpass");
    gos_AddRenderMaterial("pointlightpass");
    gos_AddRenderMaterial("null");
    gos_AddRenderMaterial("upsample_bilateral");
    gos_AddRenderMaterial("copy_depth");

    bool status = false;
    // setup deferred fbo
    glBindFramebuffer(GL_FRAMEBUFFER, deferred_fbo_);
    {
        GLenum drawBuffers[] = {GL_COLOR_ATTACHMENT0, GL_COLOR_ATTACHMENT1};
        glDrawBuffers(sizeof(drawBuffers)/sizeof(drawBuffers[0]), drawBuffers);
        glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, g_buffer_albedo, 0);
        glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, GL_COLOR_ATTACHMENT1, GL_TEXTURE_2D, g_buffer_normal, 0);
        glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, g_buffer_depth, 0);
        if(use_stencil)
            glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, GL_STENCIL_ATTACHMENT, GL_TEXTURE_2D, g_buffer_depth, 0);
        status = checkFramebufferStatus();
        assert(status);
    }
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    // setup forward fbo
    glBindFramebuffer(GL_FRAMEBUFFER, lighting_fbo_);
    {
        GLuint drawBuffers[] = { GL_COLOR_ATTACHMENT0 };
        glDrawBuffers(sizeof(drawBuffers)/sizeof(drawBuffers[0]), drawBuffers);
        glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, backbuffer, 0);
        status = checkFramebufferStatus();
        assert(status);
    }

    // setup forward fbo
    glBindFramebuffer(GL_FRAMEBUFFER, forward_fbo_);
    {
        GLuint drawBuffers[] = { GL_COLOR_ATTACHMENT0 };
        glDrawBuffers(sizeof(drawBuffers)/sizeof(drawBuffers[0]), drawBuffers);
        glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, backbuffer, 0);
        glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, g_buffer_depth, 0);
        if(use_stencil)
            glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, GL_STENCIL_ATTACHMENT, GL_TEXTURE_2D, g_buffer_depth, 0);
        status = checkFramebufferStatus();
        assert(status);
    }

    // setup stencil fbo
    glBindFramebuffer(GL_FRAMEBUFFER, stencil_fbo_);
    {
        GLuint drawBuffers[] = { GL_NONE };
        glDrawBuffers(sizeof(drawBuffers)/sizeof(drawBuffers[0]), drawBuffers);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, g_buffer_depth, 0);
        if(use_stencil)
            glFramebufferTexture2D(GL_FRAMEBUFFER, GL_STENCIL_ATTACHMENT, GL_TEXTURE_2D, g_buffer_depth, 0);
        status = checkFramebufferStatus();
        assert(status);
    }
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    glBindFramebuffer(GL_FRAMEBUFFER, downsampled_fbo_);
    {
        GLuint drawBuffers[] = { GL_COLOR_ATTACHMENT0 };
        glDrawBuffers(sizeof(drawBuffers)/sizeof(drawBuffers[0]), drawBuffers);
        glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, GL_COLOR_ATTACHMENT0 , GL_TEXTURE_2D, downsampled_color, 0);
        glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, downsampled_depth, 0);
        if(use_stencil)
            glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, GL_STENCIL_ATTACHMENT, GL_TEXTURE_2D, downsampled_depth, 0);
        status = checkFramebufferStatus();
        assert(status);
    }
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    CHECK_GL_ERROR;

    return true;
}

class DeferredShapeRenderer {
	mat4 view_proj_;
	mat4 view_;
    DWORD missing_tex_;
    HGOSTEXTURESAMPLER sampler_;
public:
	void setup(const mat4& view, const mat4& proj, DWORD missing_tex, HGOSTEXTURESAMPLER sampler) {
		view_proj_ =  proj * view;
        view_ = view;
        missing_tex_ = missing_tex;
        sampler_ = sampler;
	}

    void render(const RenderPacket &rp) {
        const RenderMesh& ro = rp.mesh_;

        HGOSRENDERMATERIAL mat = gos_getRenderMaterial("deferred");

        DWORD texid = ro.tex_id_ ? ro.tex_id_ : missing_tex_; 

        gos_SetRenderState(gos_State_Texture, texid);
        gos_SetSamplerState(0, sampler_);

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

        gos_SetSamplerState(0, 0);
    }
};

void DeferredRenderer::RenderGeometry(struct RenderFrameContext* rfc)
{
    const RenderPacketList_t& rpl = rfc->rl_->GetRenderPackets();
    RenderPacketList_t::const_iterator it = rpl.begin();
    RenderPacketList_t::const_iterator end = rpl.end();

    DeferredShapeRenderer r;
    r.setup(rfc->view_, rfc->proj_, checker_tex_, smp_linear_wrap_nomips_);

    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, deferred_fbo_);

    gos_SetRenderViewport(0, 0, width_, height_);
    glViewport(0, 0, (GLsizei)width_, (GLsizei)height_);

    glClear(GL_DEPTH_BUFFER_BIT|GL_COLOR_BUFFER_BIT);

    gos_SetRenderState(gos_State_StencilEnable, 0);
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
    gos_SetSamplerState(0, smp_nearest_clamp_nomips_);
    gos_SetRenderState(gos_State_Texture2, gos_g_buffer_normal);
    gos_SetSamplerState(1, smp_nearest_clamp_nomips_);
    gos_SetRenderState(gos_State_Texture3, gos_g_buffer_depth);
    gos_SetSamplerState(2, smp_nearest_clamp_nomips_);

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
    const float debug_params[] = { 0,0,0,0 };
    gos_SetRenderMaterialParameterFloat4(mat, "debug_params_", debug_params);

    gos_ApplyRenderMaterial(mat);

    RenderMesh* fs_quad = res_man_load_mesh("fs_quad");
    gos_RenderIndexedArray(fs_quad->ib_, fs_quad->vb_, fs_quad->vdecl_);
}

void DeferredRenderer::RenderPointLighting(struct RenderFrameContext* rfc)
{
    // forward_fbo_ has depth so shold be a bit faster due to depth culling
    //glBindFramebuffer(GL_FRAMEBUFFER, lighting_fbo_);
    glBindFramebuffer(GL_FRAMEBUFFER, forward_fbo_);

    gos_SetRenderViewport(0, 0, width_, height_);
    glViewport(0, 0, (GLsizei)width_, (GLsizei)height_);

    gos_SetRenderState(gos_State_Culling, gos_Cull_CW);
    gos_SetRenderState(gos_State_ZCompare, 3);
    //make sure we do not write in depth because we are using it as depth rt
    gos_SetRenderState(gos_State_ZWrite, false);
    // additive blending
    gos_SetRenderState(gos_State_AlphaMode, gos_Alpha_AlphaOne);

    gos_SetRenderState(gos_State_Texture, gos_g_buffer_albedo);
    gos_SetSamplerState(0, smp_nearest_clamp_nomips_);
    gos_SetRenderState(gos_State_Texture2, gos_g_buffer_normal);
    gos_SetSamplerState(1, smp_nearest_clamp_nomips_);
    gos_SetRenderState(gos_State_Texture3, gos_g_buffer_depth);
    gos_SetSamplerState(2, smp_nearest_clamp_nomips_);

    HGOSRENDERMATERIAL mat = gos_getRenderMaterial("pointlightpass");
    draw_point_lights(rfc, mat);
}

void DeferredRenderer::draw_point_lights(const struct RenderFrameContext* rfc, HGOSRENDERMATERIAL mat)
{
    const auto &point_lights = rfc->point_lights_;
    const mat4 view_proj = rfc->proj_ * rfc->view_;

    const bool is_null_mat = mat == gos_getRenderMaterial("null");
    RenderMesh *sphere = res_man_load_mesh("sphere");

    for (const PointLight &l : point_lights) {

        const mat4 wvp = view_proj * l.transform_;

        if (!is_null_mat) {
            const mat4 wv = rfc->view_ * l.transform_;
            // what is better?
            const vec4 viewpos = wv * vec4(0, 0, 0, 1);
            // const vec4 viewpos = rfc->view_ * vec4(l.pos.x, l.pos.y, l.pos.z, // 1);

            const float viewpos_radius[4] = {viewpos.x, viewpos.y, viewpos.z, l.radius_};
            gos_SetRenderMaterialParameterFloat4(mat, "color_", l.color_);
            gos_SetRenderMaterialParameterFloat4(mat, "viewpos_radius_", viewpos_radius);
            gos_SetRenderMaterialParameterMat4(mat, "wv_", wv);
            gos_SetRenderMaterialParameterMat4(mat, "proj_", rfc->proj_);
        }
        gos_SetRenderMaterialParameterMat4(mat, "wvp_", wvp);

        gos_ApplyRenderMaterial(mat);

        gos_RenderIndexedArray(sphere->ib_, sphere->vb_, sphere->vdecl_);
    }
}

// fil stencil buffer where point lights are
void DeferredRenderer::stencil_pass(const struct RenderFrameContext* rfc) {

    glBindFramebuffer(GL_FRAMEBUFFER, stencil_fbo_);

    gos_SetRenderViewport(0, 0, width_, height_);
    glViewport(0, 0, (GLsizei)width_, (GLsizei)height_);

    // draw back and front faces
    gos_SetRenderState(gos_State_Culling, gos_Cull_None);
    gos_SetRenderState(gos_State_ZCompare, 1);
    gos_SetRenderState(gos_State_ZWrite, false);
    // no blending
    gos_SetRenderState(gos_State_AlphaMode, gos_Alpha_OneZero);

    gos_SetRenderState(gos_State_StencilEnable, 1);
    // values for stencil test are not interesting
    gos_SetRenderState(gos_State_StencilRef_Front, 0);
    gos_SetRenderState(gos_State_StencilRef_Back, 0);
    gos_SetRenderState(gos_State_StencilMask_Front, 0);
    gos_SetRenderState(gos_State_StencilMask_Back, 0);

    gos_SetRenderState(gos_State_StencilFunc_Front, gos_Cmp_Always);
    gos_SetRenderState(gos_State_StencilFunc_Back, gos_Cmp_Always);

    // for front faces
    gos_SetRenderState(gos_State_StencilZFail_Front, gos_Stencil_Decr); // Z fail
    gos_SetRenderState(gos_State_StencilPass_Front, gos_Stencil_Keep); // Z pass
    // for back faces
    gos_SetRenderState(gos_State_StencilZFail_Back, gos_Stencil_Incr); // Z fail
    gos_SetRenderState(gos_State_StencilPass_Back, gos_Stencil_Keep); // Z pass
    // we always pass stencil test so nothing interesting here
    gos_SetRenderState(gos_State_StencilFail_Front, gos_Stencil_Keep);
    gos_SetRenderState(gos_State_StencilFail_Back, gos_Stencil_Keep);

    glClear(GL_STENCIL_BUFFER_BIT);
    HGOSRENDERMATERIAL mat = gos_getRenderMaterial("null");
    draw_point_lights(rfc, mat);
}

void DeferredRenderer::RenderPointLighting2(struct RenderFrameContext* rfc)
{
    stencil_pass(rfc);
    {
        glBindFramebuffer(GL_FRAMEBUFFER, forward_fbo_);

        gos_SetRenderState(gos_State_ZCompare, 0);
        gos_SetRenderState(gos_State_ZWrite, 0);

        gos_SetRenderState(gos_State_StencilEnable, 1);

        gos_SetRenderState(gos_State_StencilRef_Front, 0);
        gos_SetRenderState(gos_State_StencilRef_Back, 0);

        gos_SetRenderState(gos_State_StencilMask_Front, 0xff);
        gos_SetRenderState(gos_State_StencilMask_Back, 0xff);

        gos_SetRenderState(gos_State_StencilFunc_Front, gos_Cmp_NotEqual);
        gos_SetRenderState(gos_State_StencilFunc_Back, gos_Cmp_NotEqual);

        gos_SetRenderState(gos_State_StencilPass_Front, gos_Stencil_Keep);
        gos_SetRenderState(gos_State_StencilPass_Back, gos_Stencil_Keep);
        gos_SetRenderState(gos_State_StencilZFail_Front, gos_Stencil_Keep);
        gos_SetRenderState(gos_State_StencilZFail_Back, gos_Stencil_Keep);
        gos_SetRenderState(gos_State_StencilFail_Front, gos_Stencil_Keep);
        gos_SetRenderState(gos_State_StencilFail_Back, gos_Stencil_Keep);

#if defined(DEBUG_POINT_LIGTH_STENCIL)
        // draw fullscreen quad only where stencil is marked
        gos_SetRenderState(gos_State_Culling, gos_Cull_CW);
        HGOSRENDERMATERIAL mat = gos_getRenderMaterial("coloured_quad");
        float colour[4] = {1.0f,1.0f,1.0f,1.0f};
        gos_SetRenderMaterialParameterFloat4(mat, "colour", colour);

        gos_ApplyRenderMaterial(mat);
    
        extern RenderMesh* g_fs_quad;
        gos_RenderIndexedArray(g_fs_quad->ib_, g_fs_quad->vb_, g_fs_quad->vdecl_);
#else
        // cw/ccw? does it matter for convex in this case?
        gos_SetRenderState(gos_State_Culling, gos_Cull_CCW);

        // additive blending
        gos_SetRenderState(gos_State_AlphaMode, gos_Alpha_AlphaOne);

        gos_SetRenderState(gos_State_Texture, gos_g_buffer_albedo);
        gos_SetRenderState(gos_State_Texture2, gos_g_buffer_normal);
        gos_SetRenderState(gos_State_Texture3, gos_g_buffer_depth);

        HGOSRENDERMATERIAL mat = gos_getRenderMaterial("pointlightpass");
        draw_point_lights(rfc, mat);
#endif
    }

    gos_SetRenderState(gos_State_StencilEnable, 0);
}

void DeferredRenderer::RenderForward(std::function<void(void)> f)
{
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, forward_fbo_);

    gos_SetRenderState(gos_State_Texture, 0);
    gos_SetRenderState(gos_State_Texture2, 0);
    gos_SetRenderState(gos_State_Texture3, 0);

    gos_SetSamplerState(0, 0);
    gos_SetSamplerState(1, 0);
    gos_SetSamplerState(2, 0);

    gos_SetRenderViewport(0, 0, width_, height_);
    glViewport(0, 0, (GLsizei)width_, (GLsizei)height_);

    gos_SetRenderState(gos_State_ZCompare, 1);
    gos_SetRenderState(gos_State_ZWrite, false);
    gos_SetRenderState(gos_State_AlphaMode, gos_Alpha_OneZero);
    gos_SetRenderState(gos_State_Culling, gos_Cull_CCW);

    f();
}

void DeferredRenderer::RenderDownsampledForward(std::function<void(void)> f, const mat4& proj)
{

    // NEEDED?????
    // clear downsampled fbo before rendering into it
//    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, downsampled_fbo_);
//    gos_SetRenderViewport(0, 0, width_, height_);
//    glViewport(0, 0, (GLsizei)width_, (GLsizei)height_);
//    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
    //

    // downsample depth
    downlsampleFboDepth(stencil_fbo_, downsampled_fbo_, width_, height_,
                        ds_width_, ds_height_);
    if(0)
    {
        glBindFramebuffer(GL_DRAW_FRAMEBUFFER, downsampled_fbo_);
        glDrawBuffer(GL_NONE);

        gos_SetRenderState(gos_State_StencilEnable, 0);
        gos_SetRenderState(gos_State_ZCompare, 0);
        gos_SetRenderState(gos_State_ZWrite, 1);
        gos_SetRenderState(gos_State_AlphaMode, gos_Alpha_OneZero);
        gos_SetRenderState(gos_State_Culling, gos_Cull_CW);

        gos_SetRenderState(gos_State_Texture, gos_g_buffer_depth);
        gos_SetSamplerState(0, smp_nearest_clamp_nomips_);
        HGOSRENDERMATERIAL mat = gos_getRenderMaterial("copy_depth");
        gos_ApplyRenderMaterial(mat);

        extern RenderMesh *g_fs_quad;
        gos_RenderIndexedArray(g_fs_quad->ib_, g_fs_quad->vb_,
                               g_fs_quad->vdecl_);
        glDrawBuffer(GL_COLOR_ATTACHMENT0);
    }

    CHECK_GL_ERROR;

    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, downsampled_fbo_);

    gos_SetRenderState(gos_State_Texture, 0);
    gos_SetRenderState(gos_State_Texture2, 0);
    gos_SetRenderState(gos_State_Texture3, 0);

    gos_SetSamplerState(0, 0);
    gos_SetSamplerState(0, 0);
    gos_SetSamplerState(0, 0);

    gos_SetRenderViewport(0, 0, ds_width_, ds_height_);
    glViewport(0, 0, (GLsizei)ds_width_, (GLsizei)ds_height_);
    //glClearColor(0,0,0,0);
    glClear(GL_COLOR_BUFFER_BIT); // leave depth!
    //glClearColor(0,0,0,0);

    gos_SetRenderState(gos_State_StencilEnable, 0);
    gos_SetRenderState(gos_State_ZCompare, 1);
    gos_SetRenderState(gos_State_ZWrite, false);
    gos_SetRenderState(gos_State_AlphaMode, gos_Alpha_OneZero);
    gos_SetRenderState(gos_State_Culling, gos_Cull_CCW);

    f();

    // upsample & blend result with main color buffer by drawing quad
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, forward_fbo_);

    gos_SetRenderViewport(0, 0, width_, height_);
    glViewport(0, 0, (GLsizei)width_, (GLsizei)height_);
    
    gos_SetRenderState(gos_State_StencilEnable, 0);
    gos_SetRenderState(gos_State_ZCompare, 1);
    gos_SetRenderState(gos_State_ZWrite, false);
    //TODO: use separate blending to not overwrite destination alpha if it is used
    // dst.rgb = src.rgb + (1.0 - src.a) * dst.rgb
    // dst.a = 1.0 * src.rgb + 0.0 * dst.rgb
    //glBlendFuncSeparate(GL_ONE, GL_ONE_MINUS_SRC_ALPHA, GL_ZERO, GL_ONE);
    gos_SetRenderState(gos_State_AlphaMode, gos_Alpha_OneInvAlpha);
    gos_SetRenderState(gos_State_Culling, gos_Cull_CW);

    gos_SetRenderState(gos_State_TextureAddress, gos_TextureClamp);
    gos_SetRenderState(gos_State_Filter, gos_FilterNone);

    gos_SetRenderState(gos_State_Texture, gos_downsampled_color);
    gos_SetSamplerState(0, smp_nearest_clamp_nomips_);

    gos_SetRenderState(gos_State_Texture2, gos_downsampled_depth);
    gos_SetSamplerState(1, smp_nearest_clamp_nomips_);

    gos_SetRenderState(gos_State_Texture3, gos_g_buffer_depth);
    gos_SetSamplerState(2, smp_nearest_clamp_nomips_);

    HGOSRENDERMATERIAL mat = gos_getRenderMaterial("upsample_bilateral");
    
    const float low_res_texture_size[4] = { (float)ds_width_, (float)ds_height_, 0.0f, 0.0f };
    const float low_res_texel_size[4] = { 1.0f / (float)ds_width_, 1.0f / (float)ds_height_, 0.0f, 0.0f };
    gos_SetRenderMaterialParameterFloat4(mat, "low_res_texture_size_", low_res_texture_size);
    gos_SetRenderMaterialParameterFloat4(mat, "low_res_texel_size_", low_res_texel_size);
    gos_SetRenderMaterialParameterMat4(mat, "proj_", proj);

    gos_ApplyRenderMaterial(mat);

    RenderMesh* fs_quad = res_man_load_mesh("fs_quad");
    gos_RenderIndexedArray(fs_quad->ib_, fs_quad->vb_, fs_quad->vdecl_);
}

void DeferredRenderer::Present(int w, int h)
{
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glDrawBuffer(GL_BACK);
    glClear(GL_DEPTH_BUFFER_BIT|GL_COLOR_BUFFER_BIT);

	gos_SetRenderViewport(0, 0, w, h);
    glViewport(0, 0, (GLsizei)w, (GLsizei)h);

    
    gos_SetRenderState(gos_State_Texture, gos_backbuffer);
    gos_SetSamplerState(0, smp_nearest_clamp_nomips_);
    gos_SetRenderState(gos_State_Culling, gos_Cull_CW);
    gos_SetRenderState(gos_State_ZCompare, false);
    gos_SetRenderState(gos_State_ZWrite, 1);

    HGOSRENDERMATERIAL mat = gos_getRenderMaterial("textured_quad");

    gos_ApplyRenderMaterial(mat);

    RenderMesh* fs_quad = res_man_load_mesh("fs_quad");
    gos_RenderIndexedArray(fs_quad->ib_, fs_quad->vb_, fs_quad->vdecl_);
}




