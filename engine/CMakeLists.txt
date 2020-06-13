cmake_minimum_required (VERSION 2.8)
project(engine)

set(SOURCES ${SOURCES}
    gameos.cpp
    gameos.hpp
    gameosmain.cpp
    gameos_graphics.cpp
    #gameos_res.cpp
    #gameos_fileio.cpp
    gameos_input.cpp
    gameos_debugging.cpp
    gameos_sound.cpp
    gos_render.cpp
    gos_render.h
    gos_font.cpp
    gos_font.h
    gos_input.cpp
    gos_input.h

    utils/stream.cpp
    utils/stream.h
    utils/camera.cpp
    utils/camera.h
    utils/gl_fbo.cpp
    utils/gl_utils.cpp
    utils/gl_utils.h
    utils/gl_render_constants.cpp
    utils/gl_render_constants.h
    utils/Image.cpp
    utils/Image.h
    utils/intersection.h
    utils/logging.cpp
    utils/logging.h
    utils/shader_builder.cpp
    utils/shader_builder.h
    utils/matrix.cpp
    utils/matrix.h
    utils/math_utils.h
    utils/vec.cpp
    utils/vec.h
    utils/quaternion.cpp
    utils/quaternion.h
    utils/frustum.cpp
    utils/frustum.h
    utils/timing.cpp
    utils/timing.h
    utils/ts_queue.h
    utils/string_utils.cpp
    utils/string_utils.h
    utils/file_utils.cpp
    utils/file_utils.h
    utils/threading.cpp
    utils/threading.h
    utils/obj_loader.cpp
    utils/obj_loader.h
    utils/hashing.cpp
    utils/hashing.h
    Remotery/lib/Remotery.c
    Remotery/lib/Remotery.h
    )

#find_package(SDL2 REQUIRED PATHS "/opt" NO_DEFAULT_PATH)
#include_directories("${SDL2_PREFIX}/include")

include_directories(".")
include_directories("../platform/include")

add_library(engine ${SOURCES})

