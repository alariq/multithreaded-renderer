#pragma once

#include "renderer.h"

class ForwardRenderer {

    HGOSTEXTURESAMPLER smp_linear_wrap_mips_;

    public:

    bool Init();
    void Render(const struct RenderFrameContext* rfc);
};

