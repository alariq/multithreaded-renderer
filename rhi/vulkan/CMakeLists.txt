cmake_minimum_required (VERSION 3.12)
project(rhi_vulkan)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake")

set(VULKAN_SOURCES ${VULKAN_SOURCES}
    vulkan_device.cpp
    vulkan_device.h
    vulkan_rhi.cpp
    vulkan_rhi.h
    vulkan_state.cpp
    vulkan_state.h
    )
source_group("vulkan" FILES ${VULKAN_SOURCES})

find_path(VULKAN_INCLUDE "vulkan/vulkan.h")
#find_package(Vulkan REQUIRED PATHS "/opt" NO_DEFAULT_PATH)
include_directories(${VULKAN_INCLUDE})
include_directories(".")
include_directories("../")
include_directories("../../")
include_directories("${CMAKE_SOURCE_DIR}/engine")
include_directories("${CMAKE_SOURCE_DIR}/platform/include")


add_library(rhi_vulkan ${VULKAN_SOURCES})
link_libraries(rhi_vulkan)

