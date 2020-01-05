#pragma  once

#include "renderer.h"
#include "gameos.hpp"
#include "engine/utils/gl_utils.h"
#include "engine/utils/frustum.h"

void fill_csm_frustums(CSMInfo* csm_info, const struct camera* cam, const struct camera* shadow_cam);

class ShadowRenderPass {

    GLuint fbo_;
    DWORD gos_depth_texture_[MAX_SHADOW_CASCADES];
    GLuint depth_texture_id_[MAX_SHADOW_CASCADES];
    GLuint width_;
    GLuint height_;
    uint32_t num_cascades_;
    HGOSTEXTURESAMPLER smp_shadow_;
    bool use_pcf_;

public:
    bool Init(const uint32_t size, const int num_cascades);

    mat4 Render(const struct CSMInfo *csm_info, const RenderPacketList_t &rpl);

    uint32_t GetShadowMap(int cascade_index) { return gos_depth_texture_[cascade_index]; }
    HGOSTEXTURESAMPLER GetSadowSampler() { return smp_shadow_; }
    uint32_t GetNumCascades() { return num_cascades_; }

    ~ShadowRenderPass();
    
};


