#pragma once

#include "engine/gameos.hpp"
#include "engine/utils/gl_utils.h"
#include <cstdint>

class ObjIdRenderer {

    GLuint obj_id_fbo_ = 0;

    DWORD gos_obj_id_rt = 0;
    GLuint g_obj_id_rt = 0;

    GLuint width_ = 0;
    GLuint height_= 0;

    public:
    bool Init(uint32_t width, uint32_t height);
    void Deinit();
    void Render(struct RenderFrameContext* rfc, GLuint scene_depth);
	uint32_t Readback(uint32_t x, uint32_t y);
};
