cmake_minimum_required (VERSION 3.12)
project(rhi_opengl)

set(OPENGL_SOURCES ${OPENGL_SOURCES}
    opengl_rhi.cpp
    opengl_rhi.h
    )

source_group("opengl" FILES ${OPENGL_SOURCES})

find_package(GLEW REQUIRED)

include_directories(".")
include_directories("../")
include_directories("../../engine")

add_library(rhi_opengl ${OPENGL_SOURCES})
link_libraries(rhi_opengl)

