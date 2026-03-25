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

set(THIRD_PARTY_DIR "../third_party")

set (REMOTERY_SOURCES
    "${THIRD_PARTY_DIR}/Remotery/lib/Remotery.c"
    "${THIRD_PARTY_DIR}/Remotery/lib/Remotery.h"
)

if(USE_TRACY)
    message(" -- Compiling with Remotery")
    list(APPEND SOURCES ${REMOTERY_SOURCES})
endif()

# on Windows custom FindSDL2 is used so probably need to use SDL2main (as recommended, but it works with just "SDL2" as well just with a warning)
find_package(SDL2 REQUIRED)
find_package(SDL2_image REQUIRED)
message("SDL2 includes: ${SDL2_INCLUDE_DIRS}")

if(NOT WIN32)
    # atomic to link atomic support for 128bit data
    set(ADDITIONAL_LIBS dl pthread atomic SDL2::SDL2 SDL2::SDL2main SDL2_image::SDL2_image)
	message("Not win32 system")
else()
	# winmm for timeGetTime, maybe switch to GetTickCount to remove this dependency
    set(ADDITIONAL_LIBS winmm SDL2::Main)
endif()

include_directories(".")
include_directories("../platform/include")
include_directories("${THIRD_PARTY_DIR}/tracy/public")

if(USE_IMGUI)
    set(IMGUI_DIR "${THIRD_PARTY_DIR}/imgui/")
    include_directories(${IMGUI_DIR})
    include_directories(${IMGUI_DIR}backends)
    include_directories(${THIRD_PARTY_DIR}/imgui_club)

    file(GLOB IMGUI_SRC ${IMGUI_DIR}/*.cpp ${IMGUI_DIR}backends/imgui_impl_opengl3.cpp ${IMGUI_DIR}backends/imgui_impl_sdl2.cpp)
    file(GLOB IMGUI_INC ${IMGUI_DIR}/*.h ${IMGUI_DIR}backends/imgui_impl_opengl3.h ${IMGUI_DIR}backends/imgui_impl_sdl2.h)
    list(APPEND SOURCES ${IMGUI_SRC})
    list(APPEND SOURCES ${IMGUI_INC})
    message("-- With Dear ImGui")
endif()

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

if(USE_IMGUI)
    target_compile_definitions(engine PUBLIC USE_IMGUI)
endif()

target_link_libraries(engine PUBLIC ${ADDITIONAL_LIBS})

