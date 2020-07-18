#include "vulkan_rhi.h"
#include "rhi.h"
#include "gos_render.h"
#include "utils/logging.h"
#include "utils/macros.h"

#include "SDL2/SDL_vulkan.h"
#include "vulkan/vulkan.h"
#include <cassert>
#include <vector>
//#include <optional> -std=17

namespace rhi_vulkan {

	struct queue_families {
		uint32_t graphics_;
		uint32_t compute_;
		uint32_t transfer_;
		uint32_t family_bits;

		bool has_graphics() {
			return 0 != (family_bits & VK_QUEUE_GRAPHICS_BIT);
		}
	};

	// TODO: move from here to avoid accidentally using those in functions
	// all parameter should be passed explicitly tho the functions
	VkInstance instance_;
	VkDebugUtilsMessengerEXT debug_messenger_;
	VkPhysicalDevice phys_device_ = VK_NULL_HANDLE;
	queue_families queue_families_;
	VkDevice device_ = VK_NULL_HANDLE;
	VkQueue graphics_queue_ = VK_NULL_HANDLE;

	static const char* const s_validation_layers[] = {
		"VK_LAYER_KHRONOS_validation",
		"VK_LAYER_LUNARG_standard_validation"
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

		if (vkCreateInstance(&instance_create_info, nullptr, &instance) != VK_SUCCESS) {
			log_error("Could not create Vulkan instance!\n");
			return false;
		}

		return true;
	}

	queue_families find_queue_families(VkPhysicalDevice device) {
		queue_families indices = {};

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
			}
			if (is_compute && !(found_queue&VK_QUEUE_COMPUTE_BIT)) {
				indices.compute_ = idx;
				found_queue |= VK_QUEUE_COMPUTE_BIT;
			}
			if (is_transfer && !(found_queue&VK_QUEUE_TRANSFER_BIT)) {
				indices.transfer_ = idx;
				found_queue |= VK_QUEUE_TRANSFER_BIT;
			}
			idx++;
		}

	    return indices;
	}

	bool is_device_suitable(VkPhysicalDevice device, VkPhysicalDeviceProperties props,
							VkPhysicalDeviceFeatures features) {
		// some arbitrarry checks
		auto qf = find_queue_families(device);
		return props.deviceType == VK_PHYSICAL_DEVICE_TYPE_DISCRETE_GPU && features.geometryShader && qf.has_graphics();
	}

	bool pick_phys_device(VkInstance instance, VkPhysicalDevice &phys_device) {
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
			if (is_device_suitable(device, props, features) && !picked_name) {
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


	bool create_logical_device(VkPhysicalDevice phys_device, queue_families qf, VkDevice& device) {

		VkDeviceQueueCreateInfo queue_create_info{};
		queue_create_info.sType = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO;
		queue_create_info.queueFamilyIndex = qf.graphics_;
		queue_create_info.queueCount = 1;
		float prio = 1.0f;
		queue_create_info.pQueuePriorities = &prio;

		// fill as necessary later
		VkPhysicalDeviceFeatures features{};

		VkDeviceCreateInfo device_create_info{};
		device_create_info.sType = VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO;
		device_create_info.pQueueCreateInfos = &queue_create_info;
		device_create_info.queueCreateInfoCount = 1;
		device_create_info.pEnabledFeatures = &features;
	
		if (vkCreateDevice(phys_device, &device_create_info, nullptr, &device) != VK_SUCCESS) {
			log_error("vkCreateDevice: Failed to create logical device!");
			return false;
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

		if (!create_instance(rw_handle, instance_)) {
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

		if (vkCreateInstance(&instance_create_info, nullptr, &instance_) != VK_SUCCESS) {
			log_error("Could not create Vulkan instance!\n");
			return false;
		}

		if (s_enable_validation_layers &&
			create_debug_utils_messenger_EXT(instance_, &msg_creation_info, nullptr,
											 &debug_messenger_) != VK_SUCCESS) {
			log_warning("Failed to set up debug messenger!\n");
		}

		if (!pick_phys_device(instance_, phys_device_)) {
			return false;
		}

		queue_families_ = find_queue_families(phys_device_);

		if (!create_logical_device(phys_device_, queue_families_, device_)) {
			return false;
		}
		
		vkGetDeviceQueue(device_, queue_families_.graphics_, 0, &graphics_queue_);

		return true;
	}

	uint32_t get_window_flags(void) {
		return SDL_WINDOW_VULKAN;
	}

	void make_current_context() {}

	bool finalize() {

		vkDestroyDevice(device_, nullptr);

		if (s_enable_validation_layers) {
			destroy_debug_utils_messenger_EXT(instance_, debug_messenger_, nullptr);
		}
		vkDestroyInstance(instance_, nullptr);
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
