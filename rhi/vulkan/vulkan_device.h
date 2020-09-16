#pragma once 

#include "rhi.h"
#include "SDL2/SDL_vulkan.h"
#include "vulkan/vulkan.h"
#include <cassert>
#include <vector>
#include <unordered_map>

class RHIImageVk;
class RHIImageViewVk;

namespace rhi_vulkan {

struct QueueFamilies {
	uint32_t graphics_ = 0xffffffff;
	uint32_t compute_ = 0xffffffff;
	uint32_t transfer_ = 0xffffffff;
	uint32_t family_bits_ = 0;

	uint32_t present_ = 0xffffffff;

	bool has_graphics() { return 0 != (family_bits_ & VK_QUEUE_GRAPHICS_BIT); }
	bool has_present() { return 0xffffffff != present_; }

};

struct Image {
	VkImage handle_ = VK_NULL_HANDLE;
	VkImageView view_ = VK_NULL_HANDLE;
	VkImageLayout layout_ = VK_IMAGE_LAYOUT_UNDEFINED ;
	VkAccessFlags access_flags_ = VK_ACCESS_FLAG_BITS_MAX_ENUM;
	VkFormat format_ = VK_FORMAT_UNDEFINED;
};

struct SwapChainData {
	VkSurfaceCapabilitiesKHR capabilities_;
	std::vector<VkSurfaceFormatKHR> formats_;
	std::vector<VkPresentModeKHR> present_modes_;
};

struct SwapChain {
	VkSwapchainKHR swap_chain_ = VK_NULL_HANDLE;
	VkFormat format_ = VK_FORMAT_UNDEFINED;
	VkExtent2D extent_{0, 0};
	std::vector<class ::RHIImageVk*> images_;
	std::vector<class ::RHIImageViewVk*> views_;
};

} // namespace rhi_vulkan

using SwapChainData = rhi_vulkan::SwapChainData;
using SwapChain = rhi_vulkan::SwapChain;
using QueueFamilies = rhi_vulkan::QueueFamilies;

struct VulkanDevice {
	VkInstance instance_;
	VkDebugUtilsMessengerEXT debug_messenger_;
	VkPhysicalDevice phys_device_ = VK_NULL_HANDLE;
	SwapChainData swap_chain_data_;
	SwapChain swap_chain_;
	QueueFamilies queue_families_;
	VkDevice device_ = VK_NULL_HANDLE;
	VkQueue graphics_queue_ = VK_NULL_HANDLE;
	//VkQueue compute_queue_ = VK_NULL_HANDLE;
	VkQueue present_queue_ = VK_NULL_HANDLE;
	VkSurfaceKHR surface_;
	VkSemaphore img_avail_sem_;
	VkSemaphore rendering_finished_sem_;

	VkAllocationCallbacks *pallocator_ = nullptr;

	bool is_initialized_ = false;

	VulkanDevice() = default;
	VulkanDevice(VulkanDevice&&) = delete;
	VulkanDevice(const VulkanDevice&) = delete;
};

////////////////////////////////////////////////////////////////////////////////
class RHIImageVk : public IRHIImage {
	//rhi_vulkan::Image image_;
	// should those be native Vk* members only?
	RHIFormat format_;
	RHIAccessFlags access_flags_;
	RHIImageLayout layout_;

	VkFormat vk_format_;
	VkImage handle_ = VK_NULL_HANDLE;
	~RHIImageVk() = default;
public:
	VkAccessFlags vk_access_flags_;
	VkImageLayout vk_layout_;

	void Destroy(IRHIDevice* device);
	RHIImageVk(VkImage image, uint32_t width, uint32_t height) : handle_(image) {
		width_ = width;
		height_ = height;
	}
	//const rhi_vulkan::Image& GetImage() const { return image_; }
	//rhi_vulkan::Image& GetImage() { return image_; }
	//void SetImage(const rhi_vulkan::Image& image);
	VkImage Handle() const { return handle_; }
	VkFormat Format() const { return vk_format_; }

};

////////////////////////////////////////////////////////////////////////////////
class RHIImageViewVk : public IRHIImageView {
	VkImageView handle_;
	~RHIImageViewVk() = default;

  public:
	RHIImageViewVk(VkImageView iv, RHIImageVk *image) : handle_(iv) { image_ = image; }
	void Destroy(IRHIDevice *);
	VkImageView Handle() const { return handle_; }
};

////////////////////////////////////////////////////////////////////////////////
class RHIShaderVk: public IRHIShader{
	VkShaderModule shader_module_;
	std::vector<uint32_t> code_;//do we need this?
	RHIShaderStageFlags stage_;
	VkShaderStageFlags vk_stage_;
	~RHIShaderVk() = default;
public:
	void Destroy(IRHIDevice* device);
	static RHIShaderVk* Create(IRHIDevice* device, const uint32_t* pdata, uint32_t size, RHIShaderStageFlags stage);
	//const unsigned char* code() const { return code_.get(); }
	VkShaderModule Handle() const { return shader_module_; }
};

////////////////////////////////////////////////////////////////////////////////
class RHIPipelineLayoutVk : public IRHIPipelineLayout {
	VkPipelineLayout handle_;
public:
	void Destroy(IRHIDevice* device);
	static RHIPipelineLayoutVk* Create(IRHIDevice* device, IRHIDescriptorSetLayout* desc_set_layout);
	VkPipelineLayout Handle() const { return handle_; }
};

////////////////////////////////////////////////////////////////////////////////
class RHIGraphicsPipelineVk : public IRHIGraphicsPipeline {
	VkPipeline handle_;
public:
	void Destroy(IRHIDevice *device);
	static RHIGraphicsPipelineVk *
	Create(IRHIDevice *device, const RHIShaderStage *shader_stage, uint32_t shader_stage_count,
		   const RHIVertexInputState *vertex_input_state,
		   const RHIInputAssemblyState *input_assembly_state,
		   const RHIViewportState *viewport_state, const RHIRasterizationState *raster_state,
		   const RHIMultisampleState *multisample_state,
		   const RHIColorBlendState *color_blend_state, const IRHIPipelineLayout *pipleline_layout,
		   const IRHIRenderPass *render_pass);

	VkPipeline Handle() const { return handle_; }
};

////////////////////////////////////////////////////////////////////////////////
class RHICmdBufVk : public IRHICmdBuf {
	VkCommandBuffer cb_;
	bool is_recording_ = false;
public:
	RHICmdBufVk(VkCommandBuffer cb/*, uint32_t qfi, VkCommandPool cmd_pool*/) :
		cb_(cb) {}
	VkCommandBuffer Handle() const { return cb_; }

	virtual void Barrier_ClearToPresent(IRHIImage* image) override;
	virtual void Barrier_PresentToClear(IRHIImage* image) override;
	virtual void Barrier_PresentToDraw(IRHIImage* image) override;
	virtual void Barrier_DrawToPresent(IRHIImage* image) override;

	virtual bool Begin() override;
	virtual bool BeginRenderPass(IRHIRenderPass *i_rp, IRHIFrameBuffer *i_fb, const ivec4 *render_area,
					   const RHIClearValue *clear_values, uint32_t count) override;
	virtual void Draw(uint32_t vertex_count, uint32_t instance_count, uint32_t first_vertex,
					  uint32_t first_instance) override;
	virtual bool End() override;
	virtual void EndRenderPass(const IRHIRenderPass *i_rp, IRHIFrameBuffer *i_fb) override;
	virtual void Clear(IRHIImage* image_in, const vec4& color, uint32_t img_aspect_bits) override;
	virtual void BindPipeline(RHIPipelineBindPoint bind_point, IRHIGraphicsPipeline* pipeline) override;

};

////////////////////////////////////////////////////////////////////////////////

class RHIFrameBufferVk : public IRHIFrameBuffer {
	VkFramebuffer handle_;
    std::vector<RHIImageViewVk*> attachments_;
	virtual ~RHIFrameBufferVk() = default;
public:
  RHIFrameBufferVk(VkFramebuffer fb, std::vector<RHIImageViewVk *> attachments)
	  : handle_(fb), attachments_(attachments) {}
  void Destroy(IRHIDevice *);
  VkFramebuffer Handle() const { return handle_; }
  const std::vector<RHIImageViewVk*> GetAttachments() const {
      return attachments_;
  }
};

class RHIRenderPassVk : public IRHIRenderPass {
	VkRenderPass handle_;
	std::vector<RHIImageLayout> att_final_layouts_;
	virtual ~RHIRenderPassVk() = default;

  public:
	RHIRenderPassVk(VkRenderPass rp, std::vector<RHIImageLayout> att_final_layouts)
		: handle_(rp), att_final_layouts_(att_final_layouts) {}
	void Destroy(IRHIDevice *device);
	VkRenderPass Handle() const { return handle_; }

	RHIImageLayout GetFinalLayout(uint32_t i) const {
		assert(att_final_layouts_.size() > i);
		return att_final_layouts_[i];
	}
};

class RHIDeviceVk : public IRHIDevice {
	VulkanDevice& dev_;

	// queue family index -> pool
	std::unordered_map<uint32_t, VkCommandPool> cmd_pools_;
	// queue family index -> cb
	std::unordered_map<uint32_t, VkCommandBuffer> cmd_buffers_;

	// 
	int32_t prev_frame_ = -1;
	int32_t cur_frame_ = -1;
	uint32_t cur_swap_chain_img_idx_ = 0xffffffff;
	bool between_begin_frame = false;

public:
	explicit RHIDeviceVk(VulkanDevice& device) : dev_(device) {}

	// interface implementation
	virtual ~RHIDeviceVk() override;
	virtual IRHICmdBuf* CreateCommandBuffer(RHIQueueType queue_type) override;
	virtual IRHIRenderPass* CreateRenderPass(const RHIRenderPassDesc* desc) override;
	virtual IRHIFrameBuffer* CreateFrameBuffer(RHIFrameBufferDesc* desc, const IRHIRenderPass* rp_in) override;
	virtual IRHIImageView* CreateImageView(const RHIImageViewDesc* desc) override;

    virtual IRHIGraphicsPipeline *CreateGraphicsPipeline(
            const RHIShaderStage *shader_stage, uint32_t shader_stage_count,
            const RHIVertexInputState *vertex_input_state,
            const RHIInputAssemblyState *input_assembly_state, const RHIViewportState *viewport_state,
            const RHIRasterizationState *raster_state, const RHIMultisampleState *multisample_state,
            const RHIColorBlendState *color_blend_state, const IRHIPipelineLayout *i_pipleline_layout,
            const IRHIRenderPass *i_render_pass) override;

    virtual IRHIPipelineLayout* CreatePipelineLayout(IRHIDescriptorSetLayout* desc_set_layout) override;
    virtual IRHIShader* CreateShader(RHIShaderStageFlags stage, const uint32_t *pdata, uint32_t size) override;

	virtual RHIFormat GetSwapChainFormat() override;
	virtual uint32_t GetSwapChainSize() override { return (uint32_t)dev_.swap_chain_.images_.size(); }
	virtual class IRHIImageView* GetSwapChainImageView(uint32_t index) override;
	virtual class IRHIImage* GetSwapChainImage(uint32_t index) override;
	virtual IRHIImage* GetCurrentSwapChainImage() override;

	virtual bool Submit(IRHICmdBuf* cb_in, RHIQueueType queue_type) override;
	virtual bool BeginFrame() override;
	virtual bool Present() override;
	virtual bool EndFrame() override;

	VkDevice Handle() const { return dev_.device_; }
	VkAllocationCallbacks* Allocator() const { return dev_.pallocator_; }
};
