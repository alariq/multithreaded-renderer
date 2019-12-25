#pragma once

#include "gameos.hpp"
#include "engine/utils/gl_utils.h"
#include <functional>

class DeferredRenderer {

    GLuint deferred_fbo_;
    GLuint forward_fbo_;
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
    public:
    bool Init(int width, int height);
    void RenderGeometry(struct RenderFrameContext* rfc);
    void RenderLighting(vec3 light_dir);
    void RenderForward(std::function<void(void)> f);
    void Present(int width, int height);

};
