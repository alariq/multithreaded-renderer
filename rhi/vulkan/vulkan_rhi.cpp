#include "vulkan_rhi.h"
#include "rhi.h"
#include "gos_render.h"
#include "gos_render_priv.h"
#include "utils/logging.h"
#include "utils/macros.h"
#include "utils/vec.h"

#include "SDL2/SDL_vulkan.h"
#include "vulkan/vulkan.h"
#include <cassert>
#include <vector>
#include <algorithm>
//#include <optional> -std=17

namespace rhi_vulkan {
	struct QueueFamilies {
		uint32_t graphics_ = 0xffffffff;
		uint32_t compute_ = 0xffffffff;
		uint32_t transfer_ = 0xffffffff;
		uint32_t family_bits_ = 0;

		uint32_t present_ = 0xffffffff;

		bool has_graphics() {
			return 0 != (family_bits_ & VK_QUEUE_GRAPHICS_BIT);
		}

		bool has_present() {
			return 0xffffffff != present_;
		}
	};

	struct SwapChainData {
		VkSurfaceCapabilitiesKHR capabilities_;
		std::vector<VkSurfaceFormatKHR> formats_;
	    std::vector<VkPresentModeKHR> present_modes_;
	};

	struct SwapChain {
		VkSwapchainKHR swap_chain_ = VK_NULL_HANDLE;
		VkFormat format_ = VK_FORMAT_UNDEFINED;
		VkExtent2D extent_{ 0, 0 };
		std::vector<VkImage> images_;
		std::vector<VkImageView> image_views_;
	};
}

using SwapChainData = rhi_vulkan::SwapChainData;
using SwapChain = rhi_vulkan::SwapChain;
using QueueFamilies = rhi_vulkan::QueueFamilies;

struct RHIVulkan {
	// TODO: move from here to avoid accidentally using those in functions
	// all parameter should be passed explicitly tho the functions
	VkInstance instance_;
	VkDebugUtilsMessengerEXT debug_messenger_;
	VkPhysicalDevice phys_device_ = VK_NULL_HANDLE;
	SwapChainData swap_chain_data_;
	SwapChain swap_chain_;
	QueueFamilies queue_families_;
	VkDevice device_ = VK_NULL_HANDLE;
	VkQueue graphics_queue_ = VK_NULL_HANDLE;
	VkQueue present_queue_ = VK_NULL_HANDLE;
	VkSurfaceKHR surface_;

	VkAllocationCallbacks* pallocator_ = nullptr;
};

namespace rhi_vulkan {
	
	RHIVulkan rhi_;

	static const char* const s_validation_layers[] = {
		"VK_LAYER_KHRONOS_validation",
		"VK_LAYER_LUNARG_standard_validation"
	};

	const std::vector<const char*> req_device_ext= {
		VK_KHR_SWAPCHAIN_EXTENSION_NAME
	};

	static const bool s_enable_validation_layers = !M_IS_DEFINED(NDEBUG);

	std::vector<const char*> get_validation_layers() {
		uint32_t count;
		vkEnumerateInstanceLayerProperties(&count, nullptr);
		std::vector<VkLayerProperties> available_layers(count);
		vkEnumerateInstanceLayerProperties(&count, available_layers.data());

		std::vector<const char*> actual_layer_names2enable;

		for (auto name: s_validation_layers) {
			bool b_found = false;

			for (const auto & props: available_layers) {
				if (strcmp(name, props.layerName) == 0) {
					b_found = true;
					actual_layer_names2enable.push_back(name);
					break;
				}
			}

			if (!b_found) {
				log_warning("Validation layer %s is not available, will not be enabled\n", name);
			}
		}
		return actual_layer_names2enable;
	}

	bool get_required_extensions(graphics::RenderWindowHandle rw_handle, std::vector<const char*>& ext) {

		uint32_t count;
		if (!graphics::get_required_extensions(rw_handle, &count, nullptr))
			return false;

		ext.resize(count);
		if (!graphics::get_required_extensions(rw_handle, &count, ext.data()))
			return false;

		if(s_enable_validation_layers) {
			ext.push_back(VK_EXT_DEBUG_REPORT_EXTENSION_NAME);
			ext.push_back(VK_EXT_DEBUG_UTILS_EXTENSION_NAME);
		};
		return true;
	}

	bool check_device_extensions(VkPhysicalDevice phys_device, const std::vector<const char*> req_ext) {

		uint32_t count = 0;
		vkEnumerateDeviceExtensionProperties(phys_device, nullptr, &count, nullptr);

		std::vector<VkExtensionProperties> extensions(count);
		if (vkEnumerateDeviceExtensionProperties(phys_device, nullptr, &count, extensions.data()) != VK_SUCCESS) {
			return false;
		}
		// first print all extensions	
		log_info("Device extensions:\n");
		for (const auto& e : extensions) {
			log_info("%s : %d\n", e.extensionName, e.specVersion);
		}

		for (const auto re : req_ext) {
			if (extensions.end() ==
				std::find_if(extensions.begin(), extensions.end(),
							 [re](VkExtensionProperties prop) { return 0 == strcmp(prop.extensionName, re); }))
				return false;
		}

		return true;
	}


	static VKAPI_ATTR VkBool32 VKAPI_CALL
	debugCallback(VkDebugUtilsMessageSeverityFlagBitsEXT messageSeverity,
				  VkDebugUtilsMessageTypeFlagsEXT /*messageType*/,
				  const VkDebugUtilsMessengerCallbackDataEXT *pCallbackData, void * /*pUserData*/) {

		if (messageSeverity >= VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT) {
			log_error("validation layer: %s\n", pCallbackData->pMessage);
		}
		else {
			log_info("validation layer: %s\n", pCallbackData->pMessage);
		}
		return VK_FALSE;
	}

	VkDebugUtilsMessengerCreateInfoEXT setup_debug_messenger() {
		VkDebugUtilsMessengerCreateInfoEXT create_info{};
		create_info.sType = VK_STRUCTURE_TYPE_DEBUG_UTILS_MESSENGER_CREATE_INFO_EXT;
		create_info.messageSeverity = VK_DEBUG_UTILS_MESSAGE_SEVERITY_VERBOSE_BIT_EXT |
									 VK_DEBUG_UTILS_MESSAGE_SEVERITY_INFO_BIT_EXT |
									 VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT |
									 VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT;
		create_info.messageType = VK_DEBUG_UTILS_MESSAGE_TYPE_GENERAL_BIT_EXT |
								 VK_DEBUG_UTILS_MESSAGE_TYPE_VALIDATION_BIT_EXT |
								 VK_DEBUG_UTILS_MESSAGE_TYPE_PERFORMANCE_BIT_EXT;
		create_info.pfnUserCallback = debugCallback;
		create_info.pUserData = nullptr; // Optional
		return create_info;
	}

	VkResult create_debug_utils_messenger_EXT(VkInstance instance,
											  const VkDebugUtilsMessengerCreateInfoEXT *pCreateInfo,
											  const VkAllocationCallbacks *pAllocator,
											  VkDebugUtilsMessengerEXT *pDebugMessenger) {
		auto func = (PFN_vkCreateDebugUtilsMessengerEXT)vkGetInstanceProcAddr(
			instance, "vkCreateDebugUtilsMessengerEXT");
		if (func != nullptr) {
			return func(instance, pCreateInfo, pAllocator, pDebugMessenger);
		} else {
			return VK_ERROR_EXTENSION_NOT_PRESENT;
		}
	}

	void destroy_debug_utils_messenger_EXT(VkInstance instance, VkDebugUtilsMessengerEXT debugMessenger,
									   const VkAllocationCallbacks *pAllocator) {
		auto func = (PFN_vkDestroyDebugUtilsMessengerEXT)vkGetInstanceProcAddr(
			instance, "vkDestroyDebugUtilsMessengerEXT");
		if (func != nullptr) {
			func(instance, debugMessenger, pAllocator);
		}
	}

	bool create_instance(graphics::RenderWindowHandle rw_handle, VkInstance& instance) {

		VkApplicationInfo application_info = {
			VK_STRUCTURE_TYPE_APPLICATION_INFO,	// VkStructureType            sType
			nullptr,					// const void                *pNext
			"mt-renderer",				// const char *pApplicationName
			VK_MAKE_VERSION(1, 0, 0),	// uint32_t                   applicationVersion
			"engine-001",				// const char                *pEngineName
			VK_MAKE_VERSION(1, 0, 0),	// uint32_t                   engineVersion
			VK_API_VERSION_1_2			// uint32_t                   apiVersion
		};

		const auto layers = get_validation_layers();
		std::vector<const char*> ext;
		if (!get_required_extensions(rw_handle, ext))
			return false;

		VkInstanceCreateInfo instance_create_info = {
			VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO, // VkStructureType            sType
			nullptr,								// const void*                pNext
			0,										// VkInstanceCreateFlags      flags
			&application_info,						// const VkApplicationInfo   *pApplicationInfo
			(uint32_t)layers.size(), layers.data(),
			(uint32_t)ext.size(), ext.data()
		};

		// in order to catch errors during instance creation / destruction
		// because debug messenger is not available yet
		auto msg_creation_info = setup_debug_messenger();
		if (s_enable_validation_layers) {
			instance_create_info.pNext = &msg_creation_info;
		}

		if (vkCreateInstance(&instance_create_info, rhi_.pallocator_, &instance) != VK_SUCCESS) {
			log_error("Could not create Vulkan instance!\n");
			return false;
		}

		return true;
	}

	bool create_surface(graphics::RenderWindowHandle rw_handle, VkInstance instance, VkSurfaceKHR& surface) {
		graphics::RenderWindow* rw = (graphics::RenderWindow*)rw_handle;
	    assert(rw);
		if(!SDL_Vulkan_CreateSurface(rw->window_, instance, &surface)) {
			log_error("SDL_Vulkan_CreateSurfac failed: %s", SDL_GetError());
			return false;
		}
		return true;
	}

	QueueFamilies find_queue_families(VkPhysicalDevice device, VkSurfaceKHR surface) {
		QueueFamilies indices = {};

		uint32_t count = 0;
		vkGetPhysicalDeviceQueueFamilyProperties(device, &count, nullptr);
		uint32_t found_queue = 0;
		std::vector<VkQueueFamilyProperties> families(count);
		vkGetPhysicalDeviceQueueFamilyProperties(device, &count, families.data());
		int idx = 0;
		for (auto& f : families) {
			bool is_graphics = 0 != (f.queueFlags & VK_QUEUE_GRAPHICS_BIT);
			bool is_compute = 0 != (f.queueFlags & VK_QUEUE_COMPUTE_BIT);
			bool is_transfer = 0 != (f.queueFlags & VK_QUEUE_TRANSFER_BIT);
			log_info("Queue family: G:%d C:%d T:%d count: %d\n",
				is_graphics, is_compute, is_transfer, f.queueCount);

			if (is_graphics && !(found_queue&VK_QUEUE_GRAPHICS_BIT)) {
				indices.graphics_ = idx;
				found_queue |= VK_QUEUE_GRAPHICS_BIT;
				indices.family_bits_ |= VK_QUEUE_GRAPHICS_BIT;
			}
			if (is_compute && !(found_queue&VK_QUEUE_COMPUTE_BIT)) {
				indices.compute_ = idx;
				found_queue |= VK_QUEUE_COMPUTE_BIT;
				indices.family_bits_ |= VK_QUEUE_COMPUTE_BIT;
			}
			if (is_transfer && !(found_queue&VK_QUEUE_TRANSFER_BIT)) {
				indices.transfer_ = idx;
				found_queue |= VK_QUEUE_TRANSFER_BIT;
				indices.family_bits_ |= VK_QUEUE_TRANSFER_BIT;
			}

			VkBool32 present_support = false;
			vkGetPhysicalDeviceSurfaceSupportKHR(device, idx, surface, &present_support);
			if (present_support) {
				indices.present_ = idx;
			}

			idx++;
		}

	    return indices;
	}

	bool query_swapchain_data(VkPhysicalDevice phys_device, VkSurfaceKHR surface, SwapChainData& swap_chain) {

		if (VK_SUCCESS != vkGetPhysicalDeviceSurfaceCapabilitiesKHR(phys_device, surface, &swap_chain.capabilities_))
			return false;

		uint32_t format_count;
		if (VK_SUCCESS != vkGetPhysicalDeviceSurfaceFormatsKHR(phys_device, surface, &format_count, nullptr))
			return false;
		if (format_count != 0) {
			swap_chain.formats_.resize(format_count);
			vkGetPhysicalDeviceSurfaceFormatsKHR(phys_device, surface, &format_count, swap_chain.formats_.data());
		}

		uint32_t present_mode_count;
		if (VK_SUCCESS != vkGetPhysicalDeviceSurfacePresentModesKHR(phys_device, surface, &present_mode_count, nullptr))
			return false;
		if (present_mode_count != 0) {
			swap_chain.present_modes_.resize(present_mode_count);
			vkGetPhysicalDeviceSurfacePresentModesKHR(phys_device, surface, &present_mode_count, swap_chain.present_modes_.data());
		}

		return true;
	}


	bool is_device_suitable(VkPhysicalDevice device, VkPhysicalDeviceProperties props,
							VkPhysicalDeviceFeatures features, VkSurfaceKHR surface) {
		// some arbitrary checks
		auto qf = find_queue_families(device, surface);
		SwapChainData swap_chain;
		if (!query_swapchain_data(device, surface, swap_chain)) {
			return false;
		}
		bool swap_chain_ok =!swap_chain.formats_.empty() && !swap_chain.present_modes_.empty(); 
		return props.deviceType == VK_PHYSICAL_DEVICE_TYPE_DISCRETE_GPU &&
			features.geometryShader && qf.has_graphics() &&
			check_device_extensions(device, req_device_ext) && swap_chain_ok;
	}

	bool pick_phys_device(VkInstance instance, VkSurfaceKHR surface, VkPhysicalDevice &phys_device) {
		uint32_t count = 0;
		vkEnumeratePhysicalDevices(instance, &count, nullptr);
		if (0 == count) {
			log_error("vkEnumeratePhysicalDevices: returned 0 available devices\n");
			return false;
		}
		std::vector<VkPhysicalDevice> phys_devices(count);
		vkEnumeratePhysicalDevices(instance, &count, phys_devices.data());
		const char* picked_name = nullptr;
		for (auto device : phys_devices) {
			VkPhysicalDeviceProperties props;
			VkPhysicalDeviceFeatures features;
			vkGetPhysicalDeviceProperties(device, &props);
			vkGetPhysicalDeviceFeatures(device, &features);
			if (is_device_suitable(device, props, features, surface) && !picked_name) {
				phys_device = device;
				picked_name = props.deviceName;
			}
			log_info("Phys device: %s\n", props.deviceName);
		}
		if (picked_name) {
			log_info("Picked phys device: %s\n", picked_name);
		}
		return nullptr!=picked_name;
	}


	bool create_logical_device(VkPhysicalDevice phys_device, QueueFamilies qf, VkDevice& device) {

		VkDeviceQueueCreateInfo qci[2] = {}; // graphics + present
		uint32_t qf_indices[2] = { qf.graphics_, qf.present_ };
		const int qci_count = qf.graphics_ == qf.present_ ? 1 : 2;

		float prio = 1.0f;
		for(int i=0;i<qci_count;++i) {
			qci[i].sType = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO;
			qci[i].queueFamilyIndex = qf_indices[i];
			qci[i].queueCount = 1;
			qci[i].pQueuePriorities = &prio;
		}

		// fill as necessary later
		VkPhysicalDeviceFeatures features{};

		VkDeviceCreateInfo device_create_info{};
		device_create_info.sType = VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO;
		device_create_info.pQueueCreateInfos = &qci[0];
		device_create_info.queueCreateInfoCount = qci_count;
		device_create_info.pEnabledFeatures = &features;
		device_create_info.enabledExtensionCount = 1;
		device_create_info.ppEnabledExtensionNames = req_device_ext.data();
	
		if (vkCreateDevice(phys_device, &device_create_info, rhi_.pallocator_, &device) != VK_SUCCESS) {
			log_error("vkCreateDevice: Failed to create logical device!");
			return false;
		}
		return true;
	}

	bool create_swap_chain(SwapChainData swap_chain_data, VkDevice device, VkSurfaceKHR surface,
						   QueueFamilies qfi, VkAllocationCallbacks* pallocator, SwapChain& swap_chain) {

		// select format 
		VkSurfaceFormatKHR format = { VK_FORMAT_UNDEFINED,VK_COLOR_SPACE_MAX_ENUM_KHR };
		for (const auto& fmt: swap_chain_data.formats_) {
			if (fmt.format == VK_FORMAT_B8G8R8A8_SRGB && fmt.colorSpace == VK_COLOR_SPACE_SRGB_NONLINEAR_KHR) {
				format = fmt;
				break;
			}
		}

		if (format.format == VK_FORMAT_UNDEFINED)
			return false;

		VkPresentModeKHR present_mode = VK_PRESENT_MODE_FIFO_KHR;
		for (const auto& pmode: swap_chain_data.present_modes_) {
			if (pmode == VK_PRESENT_MODE_MAILBOX_KHR) {
				present_mode = pmode;
				break;
			}
		}

		//int drawable_width;
		//int drawable_height;
		//SDL_Vulkan_GetDrawableSize(rw->window_, &drawable_width, &drawable_height);

		VkExtent2D extent = {1024, 768};
		if (swap_chain_data.capabilities_.currentExtent.width != UINT32_MAX) {
	        extent = swap_chain_data.capabilities_.currentExtent;
		} else {
			extent.width = (uint32_t)clamp(
				extent.width, (float)swap_chain_data.capabilities_.minImageExtent.width,
				(float)swap_chain_data.capabilities_.maxImageExtent.width);
			extent.height = (uint32_t)clamp(
				extent.height, (float)swap_chain_data.capabilities_.minImageExtent.height,
				(float)swap_chain_data.capabilities_.maxImageExtent.height);
		}

		// +1 to avoid waiting fpr a driver 
		uint32_t image_count = min(swap_chain_data.capabilities_.minImageCount + 1, swap_chain_data.capabilities_.maxImageCount);

		VkSwapchainCreateInfoKHR create_info{};
		create_info.sType = VK_STRUCTURE_TYPE_SWAPCHAIN_CREATE_INFO_KHR;
		create_info.surface = surface;
		create_info.minImageCount = image_count;
		create_info.imageFormat = format.format;
		create_info.imageColorSpace = format.colorSpace;
		create_info.imageExtent = extent;
		create_info.imageArrayLayers = 1;
		create_info.imageUsage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;

		uint32_t qf_indices[2] = { qfi.graphics_ , qfi.present_ };
		if (qfi.graphics_ != qfi.present_) {
			// slower than VK_SHARING_MODE_EXCLUSIVE but requires explicit transfering of images from a queue to a queue
			create_info.imageSharingMode = VK_SHARING_MODE_CONCURRENT;
			create_info.queueFamilyIndexCount = 2;
			create_info.pQueueFamilyIndices = qf_indices;
		} else {
			create_info.imageSharingMode = VK_SHARING_MODE_EXCLUSIVE;
			create_info.queueFamilyIndexCount = 0; // Optional
			create_info.pQueueFamilyIndices = nullptr; // Optional
		}

		create_info.preTransform = swap_chain_data.capabilities_.currentTransform;
		create_info.compositeAlpha = VK_COMPOSITE_ALPHA_OPAQUE_BIT_KHR;
		create_info.presentMode = present_mode;
		// do not care about pixels obscured by another window as we are not going to read from a window
		create_info.clipped = VK_TRUE; 
		create_info.oldSwapchain = VK_NULL_HANDLE;

		if (vkCreateSwapchainKHR(device, &create_info, pallocator, &swap_chain.swap_chain_) != VK_SUCCESS) {
		    log_error("vkCreateSwapchainKHR: failed to create swap chain!");
			return false;
		}
		swap_chain.extent_ = extent;
		swap_chain.format_ = format.format;

		uint32_t img_count;
		vkGetSwapchainImagesKHR(device, swap_chain.swap_chain_, &img_count, nullptr);
		swap_chain.images_.resize(img_count);
		swap_chain.image_views_.resize(img_count);
		vkGetSwapchainImagesKHR(device, swap_chain.swap_chain_, &img_count, swap_chain.images_.data());

		int idx = 0;
		for (auto img : swap_chain.images_) {
			VkImageViewCreateInfo ci{};
			ci.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
			ci.image = img;
			ci.viewType = VK_IMAGE_VIEW_TYPE_2D;
			ci.format = swap_chain.format_;
			ci.components.r = VK_COMPONENT_SWIZZLE_IDENTITY;
			ci.components.g = VK_COMPONENT_SWIZZLE_IDENTITY;
			ci.components.b = VK_COMPONENT_SWIZZLE_IDENTITY;
			ci.components.a = VK_COMPONENT_SWIZZLE_IDENTITY;
			ci.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
			ci.subresourceRange.baseMipLevel = 0;
			ci.subresourceRange.levelCount = 1;
			ci.subresourceRange.baseArrayLayer = 0;
			ci.subresourceRange.layerCount = 1;

			if (vkCreateImageView(device, &ci, pallocator, &swap_chain.image_views_[idx]) != VK_SUCCESS) {
				log_error("vkCreateImageView: failed to create image views!\n");
				return false;
			}
			++idx;
		}
		return true;
	}
	
	bool initialize(graphics::RenderWindowHandle rw_handle) {

		uint32_t extensionCount = 0;
		vkEnumerateInstanceExtensionProperties(nullptr, &extensionCount, nullptr);

		std::vector<VkExtensionProperties> extensions(extensionCount);
		if (vkEnumerateInstanceExtensionProperties(nullptr, &extensionCount, extensions.data()) != VK_SUCCESS) {
			return false;
		}
		// place to check for required extensions
		for (auto& ext : extensions) {
			log_info("%s : %d\n", ext.extensionName, ext.specVersion);
		}

		if (!create_instance(rw_handle, rhi_.instance_)) {
			return false;
		}

		VkApplicationInfo application_info = {
			VK_STRUCTURE_TYPE_APPLICATION_INFO,	// VkStructureType            sType
			nullptr,					// const void                *pNext
			"mt-renderer",				// const char *pApplicationName
			VK_MAKE_VERSION(1, 0, 0),	// uint32_t                   applicationVersion
			"engine-001",				// const char                *pEngineName
			VK_MAKE_VERSION(1, 0, 0),	// uint32_t                   engineVersion
			VK_API_VERSION_1_2			// uint32_t                   apiVersion
		};

		const auto layers = get_validation_layers();
		std::vector<const char*> ext;
		if (!get_required_extensions(rw_handle, ext))
			return false;

		VkInstanceCreateInfo instance_create_info = {
			VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO, // VkStructureType            sType
			nullptr,								// const void*                pNext
			0,										// VkInstanceCreateFlags      flags
			&application_info,						// const VkApplicationInfo   *pApplicationInfo
			(uint32_t)layers.size(), layers.data(),
			(uint32_t)ext.size(), ext.data()
		};

		// in order to catch errors during instance creation / destruction
		// because debug messenger is not available yet
		auto msg_creation_info = setup_debug_messenger();
		if (s_enable_validation_layers) {
			instance_create_info.pNext = &msg_creation_info;
		}

		if (vkCreateInstance(&instance_create_info, rhi_.pallocator_, &rhi_.instance_) != VK_SUCCESS) {
			log_error("Could not create Vulkan instance!\n");
			return false;
		}

		if (s_enable_validation_layers &&
			create_debug_utils_messenger_EXT(rhi_.instance_, &msg_creation_info, rhi_.pallocator_,
											 &rhi_.debug_messenger_) != VK_SUCCESS) {
			log_warning("Failed to set up debug messenger!\n");
		}

		if (!create_surface(rw_handle, rhi_.instance_, rhi_.surface_)) {
			return false;
		}

		if (!pick_phys_device(rhi_.instance_, rhi_.surface_, rhi_.phys_device_)) {
			return false;
		}
		
		if (!query_swapchain_data(rhi_.phys_device_, rhi_.surface_, rhi_.swap_chain_data_)) {
			return false;
		}

		rhi_.queue_families_ = find_queue_families(rhi_.phys_device_, rhi_.surface_);

		if (!create_logical_device(rhi_.phys_device_, rhi_.queue_families_, rhi_.device_)) {
			return false;
		}
		
		vkGetDeviceQueue(rhi_.device_, rhi_.queue_families_.graphics_, 0, &rhi_.graphics_queue_);
		vkGetDeviceQueue(rhi_.device_, rhi_.queue_families_.present_, 0, &rhi_.present_queue_);

		if (!create_swap_chain(rhi_.swap_chain_data_, rhi_.device_, rhi_.surface_, rhi_.queue_families_, rhi_.pallocator_, rhi_.swap_chain_)) {
			return false;
		}

		return true;
	}


	uint32_t get_window_flags(void) {
		return SDL_WINDOW_VULKAN;
	}

	void make_current_context() {}

	bool finalize() {

		for (auto image_view: rhi_.swap_chain_.image_views_) {
			vkDestroyImageView(rhi_.device_, image_view, rhi_.pallocator_);
		}

		vkDestroySwapchainKHR(rhi_.device_, rhi_.swap_chain_.swap_chain_, rhi_.pallocator_);

		vkDeviceWaitIdle(rhi_.device_);
		vkDestroyDevice(rhi_.device_, rhi_.pallocator_);

		vkDestroySurfaceKHR(rhi_.instance_, rhi_.surface_, rhi_.pallocator_);

		if (s_enable_validation_layers) {
			destroy_debug_utils_messenger_EXT(rhi_.instance_, rhi_.debug_messenger_, rhi_.pallocator_);
		}
		vkDestroyInstance(rhi_.instance_, nullptr);
		return true;
	}
} // namespace rhi_vulkan

bool CreateRHI_Vulkan(struct rhi* backend) {
    assert(backend);
    backend->initialize_rhi = rhi_vulkan::initialize;
    backend->finalize_rhi = rhi_vulkan::finalize;
    backend->make_current_context = rhi_vulkan::make_current_context;
    backend->get_window_flags = rhi_vulkan::get_window_flags;
    return true;
}
