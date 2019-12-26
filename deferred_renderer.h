#pragma once

#include "gameos.hpp"
#include "engine/utils/gl_utils.h"
#include <functional>

class DeferredRenderer {

    GLuint deferred_fbo_;
    GLuint lighting_fbo_;
    GLuint forward_fbo_;
    GLuint stencil_fbo_;
    DWORD gos_g_buffer_albedo;
    GLuint g_buffer_albedo;
    DWORD gos_g_buffer_normal;
    GLuint g_buffer_normal;
    DWORD gos_g_buffer_depth;
    GLuint g_buffer_depth;

    DWORD gos_backbuffer;
    GLuint backbuffer;

    DWORD checker_tex_;

    GLuint width_;
    GLuint height_;

    void setup();

    void stencil_pass(const struct RenderFrameContext* rfc);
    void draw_point_lights(const struct RenderFrameContext* rfc, HGOSRENDERMATERIAL mat);

    public:
    bool Init(int width, int height);
    void RenderGeometry(struct RenderFrameContext* rfc);
    void RenderDirectionalLighting(const struct RenderFrameContext* rfc);
    void RenderPointLighting(struct RenderFrameContext* rfc);
    void RenderPointLighting2(struct RenderFrameContext* rfc);
    void RenderForward(std::function<void(void)> f);
    void Present(int width, int height);

};
