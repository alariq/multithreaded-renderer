cmake_minimum_required (VERSION 3.10)
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

    utils/adt.h
    utils/defines.h
    utils/stream.cpp
    utils/stream.h
    utils/camera.cpp
    utils/camera.h
    utils/camera_utils.h
    utils/gl_fbo.cpp
    utils/gl_utils.cpp
    utils/gl_utils.h
    utils/gl_render_constants.cpp
    utils/gl_render_constants.h
    utils/Image.cpp
    utils/Image.h
    utils/intersection.h
    utils/kernels.h
    utils/kernels.cpp
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
    utils/spline.cpp
    utils/spline.h
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
    #utils/simple_quadrature.h
    #utils/simple_quadrature.cpp
    utils/marshingcubes.h
    utils/marshingcubes.cpp
    utils/sdf.h
    utils/sdf.cpp
    profiler/profiler.h
    profiler/tracy_wrapper.cpp
    )

set (REMOTERY_SOURCES
    "../third_party/Remotery/lib/Remotery.c"
    "../third_party/Remotery/lib/Remotery.h"
)

if(USE_TRACY)
    message(" -- Compiling with Remotery")
    list(APPEND SOURCES ${REMOTERY_SOURCES})
endif()

include_directories(".")
include_directories("../platform/include")
include_directories("../third_party/tracy/public")

add_library(engine ${SOURCES})
target_compile_definitions(engine PUBLIC $<$<CONFIG:Debug>:_ARMOR _DEBUG BUGLOG LAB_ONLY>)
# Ensure Tracy compile definitions propagate to engine sources that include Tracy headers.
if(USE_TRACY)
    target_link_libraries(engine PUBLIC Tracy::TracyClient)
endif()
# all users of this library will have this path added to their includes (but this lib will not have it added)
# ref. official cmake tutorial
# ref. https://leimao.github.io/blog/CMake-Public-Private-Interface/
target_include_directories(engine INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/..)
# so that we can use: engine/xxx/xxx.h in the engine project as well
target_include_directories(engine PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/..)
