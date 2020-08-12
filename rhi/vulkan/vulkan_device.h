#pragma once 

#include "rhi.h"
#include "SDL2/SDL_vulkan.h"
#include "vulkan/vulkan.h"
#include <cassert>
#include <vector>
#include <unordered_map>

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
	std::vector<Image> images_;
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

class RHIImageVk : public IRHIImage {
	rhi_vulkan::Image image_;
public:
	const rhi_vulkan::Image& GetImage() const { return image_; }
	rhi_vulkan::Image& GetImage() { return image_; }
	void SetImage(const rhi_vulkan::Image& image) { image_ = image; }
	VkImage Handle() { return image_.handle_; }
};

class RHICmdBufVk : public IRHICmdBuf {
	VkCommandBuffer cb_;
	bool is_recording_ = false;
public:
	RHICmdBufVk(VkCommandBuffer cb/*, uint32_t qfi, VkCommandPool cmd_pool*/) :
		cb_(cb) {}
	VkCommandBuffer Handle() const { return cb_; }

	virtual void Barrier_ClearToPresent(IRHIImage* image) override;
	virtual void Barrier_PresentToClear(IRHIImage* image) override;

	virtual bool Begin() override;
	virtual bool End() override;
	virtual void Clear(IRHIImage* image_in, const vec4& color, uint32_t img_aspect_bits) override;
	virtual ~RHICmdBufVk() {};

};

class RHIDeviceVk : public IRHIDevice {
	const VulkanDevice& dev_;

	// queue family index -> pool
	std::unordered_map<uint32_t, VkCommandPool> cmd_pools_;
	// queue family index -> cb
	std::unordered_map<uint32_t, VkCommandBuffer> cmd_buffers_;

	// 
	int32_t prev_frame_ = -1;
	int32_t cur_frame_ = -1;
	uint32_t cur_swap_chain_img_idx_ = 0xffffffff;
	bool between_begin_frame = false;
	RHIImageVk fb_;

public:
	explicit RHIDeviceVk(const VulkanDevice& device) : dev_(device) {}

	// interface implementation
	virtual ~RHIDeviceVk() override;
	virtual IRHIImage* GetFrameBuffer() override { assert(between_begin_frame); return &fb_; }
	virtual IRHICmdBuf* CreateCommandBuffer(RHIQueueType queue_type) override;
	virtual bool Submit(IRHICmdBuf* cb_in, RHIQueueType queue_type) override;
	virtual bool BeginFrame() override;
	virtual bool Present() override;
	virtual bool EndFrame() override;
};
