cmake_minimum_required (VERSION 3.12)
project(rhi)

set(RHI_SOURCES ${RHI_SOURCES}
    rhi.cpp
    rhi.h
    )

source_group("rhi" FILES ${RHI_SOURCES})

include_directories(".")
include_directories("../engine")


add_subdirectory(opengl)
add_subdirectory(vulkan)

