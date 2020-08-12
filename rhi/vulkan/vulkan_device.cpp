#include "rhi.h"
#include "vulkan_device.h"
#include "utils/logging.h"
#include "utils/macros.h"
#include <unordered_map>
#include <cassert>

////////////////////////////////////////////////////////////////////////////////
template <typename T> struct ResImplType;
template<> struct ResImplType<IRHICmdBuf> { typedef RHICmdBufVk Type; };
template<> struct ResImplType<IRHIImage> { typedef RHIImageVk Type; };
template<> struct ResImplType<IRHIDevice> { typedef RHIDeviceVk Type; };

template <typename R> 
typename ResImplType<R>::Type* ResourceCast(R* obj) {
	return static_cast<typename ResImplType<R>::Type*>(obj);
}
////////////////////////////////////////////////////////////////////////////////

VkImageAspectFlags translate_image_aspect(uint32_t bits) {
	VkImageAspectFlags rv = (bits & (uint32_t)RHIImageAspect::kColor) ? VK_IMAGE_ASPECT_COLOR_BIT: 0;
	rv |= (bits & (uint32_t)RHIImageAspect::kDepth) ? VK_IMAGE_ASPECT_DEPTH_BIT : 0;
	rv |= (bits & (uint32_t)RHIImageAspect::kStencil) ? VK_IMAGE_ASPECT_STENCIL_BIT: 0;
	return rv;
}

////////////// Image //////////////////////////////////////////////////


////////////// Command buffer //////////////////////////////////////////////////

void Barrier(RHICmdBufVk *cb, rhi_vulkan::Image &image,
			 VkPipelineStageFlags src_pipeline_stage_bits,
			 VkPipelineStageFlags dst_pipeline_stage_bits, VkAccessFlags new_access_flags,
			 VkImageLayout new_layout) {

	// TODO: incorporate this int oparameters, or use image view somehow and gfrab it from that
	VkImageSubresourceRange image_subresource_range = {
		VK_IMAGE_ASPECT_COLOR_BIT, // VkImageAspectFlags                     aspectMask
		0,						   // uint32_t                               baseMipLevel
		1,						   // uint32_t                               levelCount
		0,						   // uint32_t                               baseArrayLayer
		1						   // uint32_t                               layerCount
	};

	//VkImageLayout new_layout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;
	//VkAccessFlagBits new_access_flag_bits = VK_ACCESS_MEMORY_READ_BIT;
	VkImageMemoryBarrier barrier_copy2present = {
		VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER, // VkStructureType                        sType
		nullptr,								// const void                            *pNext
		image.access_flags_,					// VkAccessFlags                          srcAccessMask
		new_access_flags,						// VkAccessFlags                          dstAccessMask
		image.layout_,							// VkImageLayout                          oldLayout
		new_layout,								// VkImageLayout                          newLayout
		VK_QUEUE_FAMILY_IGNORED,				// uint32_t                               srcQueueFamilyIndex
		VK_QUEUE_FAMILY_IGNORED,				// uint32_t                               dstQueueFamilyIndex
		image.handle_,							// VkImage                                image
		image_subresource_range					// VkImageSubresourceRange                subresourceRange
	};

	vkCmdPipelineBarrier(cb->Handle(), src_pipeline_stage_bits, dst_pipeline_stage_bits, 0, 0,
						 nullptr, 0, nullptr, 1, &barrier_copy2present);

	// warning: updating states like this is not always correct if we have differect CBs and submit
	// them in different order
	image.access_flags_ = new_access_flags;
	image.layout_ = new_layout;

}



bool RHICmdBufVk::Begin() {

	VkCommandBufferBeginInfo cmd_buffer_begin_info = {
		VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO, // VkStructureType                        sType
		nullptr,									 // const void                            *pNext
		VK_COMMAND_BUFFER_USAGE_SIMULTANEOUS_USE_BIT, // VkCommandBufferUsageFlags flags
		nullptr // const VkCommandBufferInheritanceInfo  *pInheritanceInfo
	};

	vkBeginCommandBuffer(cb_, &cmd_buffer_begin_info );

	is_recording_ = true;
	return true;
}

bool RHICmdBufVk::End() {
	if (vkEndCommandBuffer(cb_) != VK_SUCCESS) {
		log_error("vkEndCommandBuffer: Could not record command buffers!\n");
		return false;
	}
	is_recording_ = false;
	return true;
}

void RHICmdBufVk::Clear(IRHIImage* image_in, const vec4& color, uint32_t img_aspect_bits) {
	assert(is_recording_);
	VkImageAspectFlags clear_bits = translate_image_aspect(img_aspect_bits);

	RHIImageVk* image = ResourceCast(image_in);
	VkClearColorValue clear_color = {{color.x, color.y, color.z, color.w}};
	// TODO: use image view for this?
	VkImageSubresourceRange image_subresource_range = {
		clear_bits,					// VkImageAspectFlags                     aspectMask
		0,						   // uint32_t                               baseMipLevel
		1,						   // uint32_t                               levelCount
		0,						   // uint32_t                               baseArrayLayer
		1						   // uint32_t                               layerCount
	};

	assert(image->GetImage().layout_ == VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL);

	vkCmdClearColorImage(cb_, image->Handle(), VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, &clear_color,
						 1, &image_subresource_range);
}

void RHICmdBufVk::Barrier_ClearToPresent(IRHIImage *image_in) {
	RHIImageVk* image = ResourceCast(image_in);
	Barrier(this, image->GetImage(), VK_PIPELINE_STAGE_TRANSFER_BIT,
			VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT, VK_ACCESS_MEMORY_READ_BIT,
			VK_IMAGE_LAYOUT_PRESENT_SRC_KHR);
}
void RHICmdBufVk::Barrier_PresentToClear(IRHIImage *image_in) {
	RHIImageVk* image = ResourceCast(image_in);
	Barrier(this, image->GetImage(), VK_PIPELINE_STAGE_TRANSFER_BIT,
			VK_PIPELINE_STAGE_TRANSFER_BIT, VK_ACCESS_MEMORY_READ_BIT,
			VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL);
}

////////////////RHI Device /////////////////////////////////////////////////////

RHIDeviceVk::~RHIDeviceVk() {
}


// should this be mobved to command buffer class / .cpp file and just pass Device as a parameter ?
IRHICmdBuf* RHIDeviceVk::CreateCommandBuffer(RHIQueueType queue_type) {

	assert(queue_type == RHIQueueType::kGraphics || queue_type == RHIQueueType::kPresentation);
	uint32_t qfi = (RHIQueueType::kGraphics == queue_type) ? dev_.queue_families_.graphics_
														   : dev_.queue_families_.present_;
	VkCommandPoolCreateFlagBits flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
	VkCommandPoolCreateInfo cmd_pool_create_info = {
		VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO, // VkStructureType              sType
		nullptr,									// const void*                  pNext
		flags,											// VkCommandPoolCreateFlags     flags
		qfi											// uint32_t                     queueFamilyIndex
	};

	VkCommandPool cmd_pool;
	if (!cmd_pools_.count(qfi)) {
		if (vkCreateCommandPool(dev_.device_, &cmd_pool_create_info, nullptr, &cmd_pool) !=
			VK_SUCCESS) {
			log_error("Could not create a command pool!\n");
			return nullptr;
		}
		cmd_pools_[qfi] = cmd_pool;
	}
	else {
		cmd_pool = cmd_pools_[qfi];
	}

	//uint32_t image_count = 0;
	//if ((vkGetSwapchainImagesKHR(dev_.device_, dev_.swap_chain_.swap_chain_, &image_count,
	//							 nullptr) != VK_SUCCESS) ||
	//	(image_count == 0)) {
	//	std::cout << "Could not get the number of swap chain images!" << std::endl;
	//	return false;
	//}

	//Vulkan.PresentQueueCmdBuffers.resize(image_count);

	VkCommandBufferAllocateInfo cmd_buffer_allocate_info = {
		VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO, // VkStructureType              sType
		nullptr,										// const void*                  pNext
		cmd_pool,										// VkCommandPool                commandPool
		VK_COMMAND_BUFFER_LEVEL_PRIMARY,				// VkCommandBufferLevel         level
		1 /*image_count	*/									// uint32_t                     bufferCount
	};

	VkCommandBuffer cb;
	if (vkAllocateCommandBuffers(dev_.device_, &cmd_buffer_allocate_info, &cb) != VK_SUCCESS) {
		log_error("Could not allocate command buffers!\n");
		return false;
	}

	return new RHICmdBufVk(cb/*, qfi, cmd_pool*/);

}

bool RHIDeviceVk::BeginFrame() {
	VkResult result = vkAcquireNextImageKHR(dev_.device_, dev_.swap_chain_.swap_chain_, UINT64_MAX,
											dev_.img_avail_sem_, VK_NULL_HANDLE, &cur_swap_chain_img_idx_);
	switch( result ) {
		case VK_SUCCESS:
		case VK_SUBOPTIMAL_KHR:
			break;
		case VK_ERROR_OUT_OF_DATE_KHR:
			log_warning("VK_ERROR_OUT_OF_DATE_KHR, probab;y need to resize window!\n");
			return false;
		default:
			log_error("Problem occurred during swap chain image acquisition!\n");
			return false;
	}

	// setup cur framebuffer pointer
	fb_.SetImage(dev_.swap_chain_.images_[cur_swap_chain_img_idx_]);

	cur_frame_++;
	between_begin_frame = true;


	return true;
}

bool RHIDeviceVk::Submit(IRHICmdBuf* cb_in, RHIQueueType queue_type) {

	RHICmdBufVk* cb = ResourceCast(cb_in);

	VkPipelineStageFlags wait_dst_stage_mask = VK_PIPELINE_STAGE_TRANSFER_BIT;
	VkCommandBuffer cbs[] = { cb->Handle() };
	VkSubmitInfo submit_info = {
		VK_STRUCTURE_TYPE_SUBMIT_INFO, nullptr, 
		1, &dev_.img_avail_sem_, 
		&wait_dst_stage_mask, 
		countof(cbs), cbs, 
		1, &dev_.rendering_finished_sem_
	};

	VkQueue queue = RHIQueueType::kGraphics == queue_type ? dev_.graphics_queue_ : dev_.present_queue_;
	if (vkQueueSubmit(queue, 1, &submit_info, VK_NULL_HANDLE) != VK_SUCCESS) {
		log_error("vkQueueSubmit: failed\n");
		return false;
	}
	return true;
}

bool RHIDeviceVk::Present() {

	VkPresentInfoKHR present_info = {
		VK_STRUCTURE_TYPE_PRESENT_INFO_KHR, // VkStructureType              sType
		nullptr,							// const void                  *pNext
		1,									// uint32_t                     waitSemaphoreCount
		&dev_.rendering_finished_sem_,		// const VkSemaphore           *pWaitSemaphores
		1,									// uint32_t                     swapchainCount
		&dev_.swap_chain_.swap_chain_,		// const VkSwapchainKHR        *pSwapchains
		&cur_swap_chain_img_idx_,			// const uint32_t              *pImageIndices
		nullptr								// VkResult                    *pResults
	};

	VkResult result = vkQueuePresentKHR(dev_.present_queue_, &present_info);
	switch (result) {
	case VK_SUCCESS:
		break;
	case VK_ERROR_OUT_OF_DATE_KHR:
	case VK_SUBOPTIMAL_KHR:
		log_error("vkQueuePresentKHR: VK_SUBOPTIMAL_KHR\n");
		return false; 
	default:
		log_error("vkQueuePresentKHR: Problem occurred during image presentation!\n");
		return false;
	}

	return true;
}

bool RHIDeviceVk::EndFrame() {
	assert(prev_frame_ == cur_frame_ - 1);
	prev_frame_ = cur_frame_;
	between_begin_frame = false;
	return true;
}
