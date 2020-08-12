#pragma once 

#include "rhi.h"
#include "gl/glew.h"
#include "utils/vec.h"
#include <cassert>
#include <vector>
#include <unordered_map>

namespace rhi_opengl {

struct Image {
	GLuint handle_ = 0;
};

struct SwapChainData {
	//VkSurfaceCapabilitiesKHR capabilities_;
	//std::vector<VkSurfaceFormatKHR> formats_;
	//std::vector<VkPresentModeKHR> present_modes_;
};

struct SwapChain {
	GLuint format_ = 0;
	ivec2 extent_{0, 0};
	std::vector<Image> images_;
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
	rhi_opengl::Image image_;
public:
	const rhi_opengl::Image& GetImage() const { return image_; }
	rhi_opengl::Image& GetImage() { return image_; }
	void SetImage(const rhi_opengl::Image& image) { image_ = image; }
	GLuint Handle() { return image_.handle_; }
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
	RHIImageGL fb_;

public:
	explicit RHIDeviceGL() {}

	// interface implementation
	virtual ~RHIDeviceGL() override;
	virtual IRHIImage* GetFrameBuffer() override { assert(between_begin_frame); return &fb_; }
	virtual IRHICmdBuf* CreateCommandBuffer(RHIQueueType queue_type) override;
	virtual bool Submit(IRHICmdBuf* cb_in, RHIQueueType queue_type) override;
	virtual bool BeginFrame() override;
	virtual bool Present() override;
	virtual bool EndFrame() override;
};
