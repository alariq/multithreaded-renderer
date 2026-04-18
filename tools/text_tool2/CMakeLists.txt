add_definitions(-DUSE_ASSEMBLER_CODE=0)
add_definitions(-DLINUX_BUILD)

find_package(SDL3 REQUIRED)
find_package(SDL3_ttf REQUIRED)
find_package(SDL3_image REQUIRED) # save to tga
find_package(OpenGL REQUIRED)

set(TEXT_TOOL_SOURCES "main.cpp")

include_directories(${CMAKE_PREFIX_PATH}/include)
include_directories(${CMAKE_SOURCE_DIR}/engine)
include_directories(${CMAKE_SOURCE_DIR}/platform/include)
include_directories(${CMAKE_SOURCE_DIR}/third_party)

add_executable(text_tool2 ${TEXT_TOOL_SOURCES})
target_link_libraries(text_tool2 ${ASAN_LIB} engine platform ${SDL3_LIBRARIES} SDL3_ttf::SDL3_ttf SDL3_image::SDL3_image GLEW::GLEW OpenGL::GL)
