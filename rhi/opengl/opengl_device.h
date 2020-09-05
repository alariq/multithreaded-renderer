#pragma once 

#include "rhi.h"
#include "gl/glew.h"
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
	explicit RHIImageGL(GLuint id):handle_(id) {}
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

	virtual bool Begin() override;
	virtual bool End() override;
	virtual void Clear(IRHIImage* image_in, const vec4& color, uint32_t img_aspect_bits) override;
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
	virtual const IRHIImageView* GetSwapChainImageView(uint32_t ) override { return fb_view_.get(); };
	virtual const IRHIImage* GetSwapChainImage(uint32_t ) override { return fb_.get(); }
	virtual IRHIImage* GetCurrentSwapChainImage() override { assert(between_begin_frame); return fb_.get(); }

	virtual IRHICmdBuf* CreateCommandBuffer(RHIQueueType queue_type) override;
	virtual IRHIRenderPass* CreateRenderPass(const RHIRenderPassDesc* desc) override;
	virtual IRHIFrameBuffer* CreateFrameBuffer(const RHIFrameBufferDesc* desc, const IRHIRenderPass* rp_in) override;
	virtual IRHIImageView* CreateImageView(const RHIImageViewDesc* desc) override;

	virtual bool Submit(IRHICmdBuf* cb_in, RHIQueueType queue_type) override;
	virtual bool BeginFrame() override;
	virtual bool Present() override;
	virtual bool EndFrame() override;
};
