cmake_minimum_required (VERSION 3.10)
cmake_policy(SET CMP0072 NEW) # cmake --help-policy CMP0072
project(mt-renderer)

include(CMakeToolsHelpers OPTIONAL)

# get FindSDL2 from here https://github.com/aminosbh/sdl2-cmake-modules
# and put it into 3rdparty
list(APPEND CMAKE_MODULE_PATH ${CMAKE_PREFIX_PATH}/cmake/sdl2)
set(SDL2_PATH ${CMAKE_PREFIX_PATH})

message("CMAKE library architecture: ${CMAKE_LIBRARY_ARCHITECTURE}")

if("${CMAKE_SIZEOF_VOID_P}" EQUAL "8")
	message(STATUS "Target is 64 bits")
	set_property(GLOBAL PROPERTY FIND_LIBRARY_USE_LIB64_PATHS TRUE)
else("${CMAKE_SIZEOF_VOID_P}" EQUAL "8")
	message(STATUS "Target is 32 bits")
endif("${CMAKE_SIZEOF_VOID_P}" EQUAL "8")

set(CMAKE_EXPORT_COMPILE_COMMANDS "ON")
 
if(NOT MSVC)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -ggdb")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror=return-type -Wno-unused-local-typedefs -Wno-unused-function -Werror=array-bounds -Wno-switch -Wwrite-strings -Wno-unknown-warning-option -Wunused-variable -Wuninitialized -fno-exceptions" ) # -ftime-trace -fsanitize=address -fno-omit-frame-pointer")
    set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -O2")
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O2")
endif()
# -Wno-unused-local-typedefs - to disable "typedef was ignored on this declaration"


# to support windows sln file generator as well
#set(CMAKE_CXX_FLAGS_RELWITHDEBINFO" ${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -DSOME_STUFF")
if(MSVC)
    # 4514 - unreferenced inline function has been removed
    # 4121 - alignment of a member was sensitive to packing
    # 4099 - type name first seen using 'struct' now seen using 'class'
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /std:c++14 /W4 /wd4514 /wd4121 /we4099")
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /MTd")
endif()

if(MSVC)
    # error C2338: related to alignment used in my allocator
    # VS had a bug
    add_definitions(-D_ENABLE_EXTENDED_ALIGNED_STORAGE)

    # to compile on newer Windows SDKs
    add_definitions(-DWINDOWS_IGNORE_PACKING_MISMATCH)

	add_definitions(-DPLATFORM_WINDOWS)
	add_definitions(-D_CRT_SECURE_NO_WARNINGS)
	message("Compiling on M$ Windows")
endif()

add_definitions(-DUSE_ASSEMBLER_CODE=0)
add_definitions(-DLINUX_BUILD)
add_definitions(-DRMT_USE_OPENGL=1)

find_path(GLEW_INCLUDE "GL/glew.h")

# on Windows custom FindSDL2 is used so probably need to use SDL2main (as recommended, but it works with just "SDL2" as well just with a warning)
find_package(SDL2 REQUIRED)
find_package(GLEW REQUIRED)

get_filename_component(COM_PATH1 "./GameOS/include" ABSOLUTE)
get_filename_component(COM_PATH2 "./GameOS/gameos" ABSOLUTE)

set(COMMON_INCLUDE_DIRS  ${COM_PATH1} ${COM_PATH2})

include_directories(SDL2::SDL2)
include_directories(${GLEW_INCLUDE})

add_subdirectory("./engine" "./out/engine")
add_subdirectory("./platform" "./out/platform")

set(SOURCES 
    "main.cpp"
    "shadow_renderer.cpp"
    "particle_system.cpp"
    "debug_renderer.cpp"
    "deferred_renderer.cpp"
    "obj_id_renderer.cpp"
    "renderer.cpp"
    "renderer.h"
    "render_utils.h"
    "render_utils.cpp"
    "test_fixed_block_allocator.cpp"
    "scene.cpp"
    "obj_model.cpp"
    "res_man.cpp"
    "editor.cpp"
    )

#find_library(GLEW NAMES "GLEW" "glew32")

find_package(OpenGL REQUIRED)

message("OPENGL library found in ${OPENGL}")
message("GLEW library found in ${GLEW}")

include_directories(${COMMON_INCLUDE_DIRS} "./platform/include" "./engine" )

message("OpenGL library linking: ${OPENGL_gl_LIBRARY}")
message("OpenGL library include: ${OPENGL_INCLUDE_DIR}")

#if(NOT ${CMAKE_SYSTEM_NAME} STREQUAL "Windows")
if(NOT WIN32)
    # atomic to link atomic support for 128bit data
    set(ADDITIONAL_LIBS dl pthread atomic SDL2::SDL2 SDL2::SDL2main)
	message("Not win32 system")
else()
	# winmm for timeGetTime, maybe switch to GetTickCount to remove this dependency
    set(ADDITIONAL_LIBS winmm SDL2::Main)
endif()

add_executable(mt-renderer ${SOURCES})
target_link_libraries(mt-renderer engine platform GLEW::GLEW ${ADDITIONAL_LIBS} OpenGL::GL)

