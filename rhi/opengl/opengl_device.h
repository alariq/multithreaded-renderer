#pragma once 

#include "rhi.h"
#include "GL/glew.h"
#include "utils/vec.h"
#include <cassert>
#include <vector>
#include <unordered_map>
#include <memory>

namespace rhi_opengl {

struct SwapChainData {
	//VkSurfaceCapabilitiesKHR capabilities_;
	//std::vector<VkSurfaceFormatKHR> formats_;
	//std::vector<VkPresentModeKHR> present_modes_;
};

struct SwapChain {
	GLuint format_ = 0;
	ivec2 extent_{0, 0};
	std::vector<GLuint> images_;
};

} // namespace rhi_opengl

using SwapChainData = rhi_opengl::SwapChainData;
using SwapChain = rhi_opengl::SwapChain;

struct OpenGLDevice {
	OpenGLDevice() = default;
	OpenGLDevice(OpenGLDevice&&) = delete;
	OpenGLDevice(const OpenGLDevice&) = delete;
};

class RHIImageGL : public IRHIImage {
	GLuint handle_ = 0;
public:
  RHIImageGL(GLuint id, uint32_t width, uint32_t height) : handle_(id) {
	  width_ = width;
	  height_ = height;
  }
	GLuint Handle() { return handle_; }
};

class RHIImageViewGL : public IRHIImageView {
	GLuint handle_ = 0;
public:
	explicit RHIImageViewGL(GLuint id):handle_(id) {}
	GLuint Handle() { return handle_; }
};


class RHIFrameBufferGL : public IRHIFrameBuffer {
	virtual ~RHIFrameBufferGL() = default;
public:
	RHIFrameBufferGL() {}
	void Destroy(IRHIDevice*) { delete this; };
	void* Handle() const { return nullptr; }
};

class RHIRenderPassGL : public IRHIRenderPass {
	virtual ~RHIRenderPassGL() = default;
public:
	RHIRenderPassGL() {}
	void Destroy(IRHIDevice*) { delete this; };
	void* Handle() const { return nullptr; }
};

////////////////////////////////////////////////////////////////////////////////
class RHIShaderGL: public IRHIShader{
	GLuint shader_;
	RHIShaderStageFlags stage_;
	~RHIShaderGL() = default;
public:
	void Destroy(IRHIDevice* device);
	static RHIShaderGL* Create(IRHIDevice* device, const uint32_t* pdata, uint32_t size, RHIShaderStageFlags stage);
};

////////////////////////////////////////////////////////////////////////////////
class RHIPipelineLayoutGL : public IRHIPipelineLayout {
public:
	void Destroy(IRHIDevice* device);
	static RHIPipelineLayoutGL* Create(IRHIDevice* device, IRHIDescriptorSetLayout* desc_set_layout, RHIShaderStageFlags stage);
};

////////////////////////////////////////////////////////////////////////////////
class RHIBufferGL : public IRHIBuffer {
	GLuint handle_ = 0;

    uint32_t buf_size_;
    uint32_t usage_flags;
    uint32_t mem_flags;

    bool is_mapped_;
    uint32_t mapped_offset_;
    uint32_t mapped_size_;
    uint32_t mapped_flags_;

public:
	explicit RHIBufferGL(GLuint id):handle_(id) {}
	static RHIBufferGL* Create(IRHIDevice* device, uint32_t size, uint32_t usage, RHISharingMode sharing);
	void Destroy(IRHIDevice *device);

    void* Map(IRHIDevice* device, uint32_t offset, uint32_t size, uint32_t map_flags);
    void Unmap(IRHIDevice* device);

	GLuint Handle() { return handle_; }
};

////////////////////////////////////////////////////////////////////////////////
class RHIGraphicsPipelineGL : public IRHIPipelineLayout {
public:
	void Destroy(IRHIDevice* device);
	static RHIGraphicsPipelineGL *Create(
		IRHIDevice *device, RHIShaderStage *shader_stage, uint32_t shader_stage_count,
		RHIVertexInputState *vertex_input_state, RHIInputAssemblyState* input_assembly_state,
		RHIViewportState* viewport_state, RHIRasterizationState *raster_state,
		RHIMultisampleState* multisample_state, RHIColorBlendState* color_blend_state);
};

class RHICmdBufGL : public IRHICmdBuf {
	bool is_recording_ = false;

	GLuint cur_fb_ = 0;

public:
	RHICmdBufGL() = default;
	
	// TODO: this is ugly, change when interface will settle down a bit
	virtual void Barrier_ClearToPresent(IRHIImage* image) override;
	virtual void Barrier_PresentToClear(IRHIImage* image) override;
	virtual void Barrier_PresentToDraw(IRHIImage* image) override;
	virtual void Barrier_DrawToPresent(IRHIImage* image) override;

	virtual bool Begin() override;
	virtual bool BeginRenderPass(IRHIRenderPass *i_rp, IRHIFrameBuffer *i_fb, const ivec4 *render_area,
					   const RHIClearValue *clear_values, uint32_t count) override;

	virtual void Draw(uint32_t vertex_count, uint32_t instance_count, uint32_t first_vertex,
					  uint32_t first_instance) override;

    virtual void BindVertexBuffers(IRHIBuffer** i_vb, uint32_t first_binding, uint32_t count) override;

	virtual bool End() override;
	virtual void EndRenderPass(const IRHIRenderPass *i_rp, IRHIFrameBuffer *i_fb) override;
	virtual void Clear(IRHIImage* image_in, const vec4& color, uint32_t img_aspect_bits) override;
	virtual void BindPipeline(RHIPipelineBindPoint bind_point, IRHIGraphicsPipeline* pipeline) override;

	virtual ~RHICmdBufGL() {};

};

class RHIDeviceGL: public IRHIDevice {

	// 
	int32_t prev_frame_ = -1;
	int32_t cur_frame_ = -1;
	uint32_t cur_swap_chain_img_idx_ = 0xffffffff;
	bool between_begin_frame = false;
	std::unique_ptr<RHIImageGL> fb_;
	std::unique_ptr<RHIImageViewGL> fb_view_;

public:
	RHIDeviceGL();

	// interface implementation
	virtual ~RHIDeviceGL() override;

	virtual RHIFormat GetSwapChainFormat() override { return fb_->Format(); }
	virtual uint32_t GetSwapChainSize() override { return 1; }
	virtual IRHIImageView* GetSwapChainImageView(uint32_t ) override { return fb_view_.get(); };
	virtual IRHIImage* GetSwapChainImage(uint32_t ) override { return fb_.get(); }
	virtual IRHIImage* GetCurrentSwapChainImage() override { assert(between_begin_frame); return fb_.get(); }

    virtual uint32_t GetNumBufferedFrames() override { return 1; }
    virtual uint32_t GetCurrentFrame() { return cur_frame_; }

	virtual IRHICmdBuf* CreateCommandBuffer(RHIQueueType queue_type) override;
	virtual IRHIRenderPass* CreateRenderPass(const RHIRenderPassDesc* desc) override;
	virtual IRHIFrameBuffer* CreateFrameBuffer(RHIFrameBufferDesc* desc, const IRHIRenderPass* rp_in) override;
	virtual IRHIImageView* CreateImageView(const RHIImageViewDesc* desc) override;
	virtual IRHIBuffer* CreateBuffer(uint32_t size, uint32_t usage, uint32_t memprop, RHISharingMode sharing) override;

    virtual IRHIGraphicsPipeline *CreateGraphicsPipeline(
            const RHIShaderStage *shader_stage, uint32_t shader_stage_count,
            const RHIVertexInputState *vertex_input_state,
            const RHIInputAssemblyState *input_assembly_state, const RHIViewportState *viewport_state,
            const RHIRasterizationState *raster_state, const RHIMultisampleState *multisample_state,
            const RHIColorBlendState *color_blend_state, const IRHIPipelineLayout *i_pipleline_layout,
            const IRHIRenderPass *i_render_pass) override;

    virtual IRHIPipelineLayout* CreatePipelineLayout(IRHIDescriptorSetLayout* desc_set_layout) override;
    virtual IRHIShader* CreateShader(RHIShaderStageFlags stage, const uint32_t *pdata, uint32_t size) override;

	virtual bool Submit(IRHICmdBuf* cb_in, RHIQueueType queue_type) override;
	virtual bool BeginFrame() override;
	virtual bool Present() override;
	virtual bool EndFrame() override;
};
