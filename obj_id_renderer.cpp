#include "obj_id_renderer.h"
#include "renderer.h"
#include "gameos.hpp"

#include "engine/utils/gl_fbo.h"
#include "engine/profiler/profiler.h"
#include "utils/logging.h"
#include "scene.h"

#include <algorithm>

bool ObjIdRenderer::Init(uint32_t width, uint32_t height)
{
    width_ = width;
    height_ = height;
    uint32_t wh = (height_<<16) | width_;
    gos_AddRenderMaterial("obj_id");

	GLuint fbo[num_buffers_] = {0};
    glGenFramebuffers(num_buffers_, fbo);

    bool success = true;
	for (int i = 0; i < num_buffers_; ++i) {

		// using floating point texture because Nsight can't visualize integer
        // textures
        bufs_[i].gos_obj_id_rt =
            gos_NewRenderTarget(gos_Texture_R32F, "obj_id_rt", wh);
        bufs_[i].obj_id_rt = gos_TextureGetNativeId(bufs_[i].gos_obj_id_rt);
        bufs_[i].obj_id_fbo_ = fbo[i];

        success = success && bufs_[i].gos_obj_id_rt && bufs_[i].obj_id_fbo_;
	}

    return success;
}

void ObjIdRenderer::Deinit()
{
	for (int i = 0; i < num_buffers_; ++i) {
        gosASSERT(bufs_[i].gos_obj_id_rt && bufs_[i].obj_id_fbo_);
        gos_DestroyTexture(bufs_[i].gos_obj_id_rt);
        glDeleteFramebuffers(1, &bufs_[i].obj_id_fbo_);
    }
}

void draw_rp(HGOSRENDERMATERIAL mat, const mat4 &vp, const RenderPacket& rp) {
	const RenderMesh &ro = rp.mesh_;
	mat4 wvp = vp * rp.m_;
	gos_SetRenderMaterialParameterMat4(mat, "wvp_", (const float *)wvp);
	float obj_id[4] = {(float)rp.id_, 0.0f, 0.0f, 0.0f};
	gos_SetRenderMaterialParameterFloat4(mat, "obj_id_", obj_id);

	gos_ApplyRenderMaterial(mat);

	// TODO: fix dirty hack
	if (rp.id_ < scene::kFirstGameObjectId)
		gos_SetRenderState(gos_State_ZCompare, 0);
	else
		gos_SetRenderState(gos_State_ZCompare, 1); // less equal, equal should be enough

	if (ro.ib_) {
		gos_RenderIndexedArray(ro.ib_, ro.vb_, ro.vdecl_, ro.prim_type_);
	} else if (ro.inst_vb_) {
		gos_RenderArrayInstanced(ro.vb_, ro.inst_vb_, ro.num_instances, ro.vdecl_, ro.prim_type_);
	} else {
		gos_RenderArray(ro.vb_, ro.vdecl_, ro.prim_type_);
	}
}

void ObjIdRenderer::Render(struct RenderFrameContext *rfc, GLuint scene_depth)
{
    uint32_t write_idx = cur_write_buffer_;
    GLuint fbo = bufs_[write_idx].obj_id_fbo_;
    glBindFramebuffer(GL_FRAMEBUFFER, fbo);
    {
        GLenum drawBuffers[] = {GL_COLOR_ATTACHMENT0};
        glDrawBuffers(sizeof(drawBuffers)/sizeof(drawBuffers[0]), drawBuffers);
        glReadBuffer(GL_NONE);
        glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, bufs_[write_idx].obj_id_rt, 0);
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
    gos_SetRenderState(gos_State_Culling, gos_Cull_None);
    gos_SetRenderState(gos_State_ZCompare, 1); // less equal, equal should be enough
    gos_SetRenderState(gos_State_ZWrite, false);

    HGOSRENDERMATERIAL mat = gos_getRenderMaterial("obj_id");

    const mat4 vp = rfc->proj_ * rfc->view_;

    const RenderPacketList_t& rpl = rfc->rl_->GetRenderPackets();
    RenderPacketList_t::const_iterator it = rpl.begin();
    RenderPacketList_t::const_iterator end = rpl.end();

    for(;it!=end;++it)
    {
        const RenderPacket& rp = (*it);
        if(rp.is_selection_pass) {
            gosASSERT(!rp.is_gizmo_pass);
		    draw_rp(mat, vp, rp);
        }
	}

	// now draw all gizmo stuff back to front
    std::vector<const RenderPacket*> tmp;
	for (it = rpl.begin(); it != end; ++it) {
		if ((*it).is_gizmo_pass) {
			tmp.push_back(&(*it));
		}
	}
	std::sort(tmp.begin(), tmp.end(),
			  [&view = rfc->view_](const RenderPacket *a, const RenderPacket *b) {
				  return (view * a->m_.getCol3()).z > (view * b->m_.getCol3()).z;
			  });

    gos_SetRenderState(gos_State_Culling, gos_Cull_CCW);
	for (auto rp : tmp) {
		draw_rp(mat, vp, *rp);
	}

    bufs_[cur_write_buffer_].has_data_ = true;
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

}

uint32_t ObjIdRenderer::Readback(uint32_t x, uint32_t y)
{
    SCOPED_GPU_ZONE(Readback);
    SCOPED_ZONE_N(Readback, 0);

	if (x >= width_ || y >= height_)
		return 0;

    int read_idx = uint32_t(cur_write_buffer_ + num_buffers_ - 1) % num_buffers_;
    if(!bufs_[read_idx].has_data_)
        return true;

    GLuint fbo = bufs_[read_idx].obj_id_fbo_;
    glBindFramebuffer(GL_FRAMEBUFFER, fbo);
    glReadBuffer(GL_COLOR_ATTACHMENT0);
    glDrawBuffer(GL_NONE);
    glFramebufferTexture2D(GL_READ_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, bufs_[read_idx].obj_id_rt, 0);
    bool status = checkFramebufferStatus();
    assert(status);

    float obj_id = 0;
	glReadPixels(x, y, 1, 1, GL_RED, GL_FLOAT, (GLvoid*)&obj_id);
	//log_info("x: %d y: %d -> obj_id: %d\n", x, y, (int)obj_id);

    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    cur_write_buffer_ = (cur_write_buffer_ + 1) % num_buffers_;
    return (uint32_t)obj_id;
    
}
