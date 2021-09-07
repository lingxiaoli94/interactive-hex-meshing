file(GLOB vkoo_srcs
        src/*.cpp
        src/core/*.cpp
        src/st/*.cpp
        src/st/components/*.cpp
        src/st/scripts/*.cpp
        src/st/hittables/*.cpp)

add_library(vkoo ${vkoo_srcs})
target_compile_options(vkoo PRIVATE ${cxx_warning_flags})
target_include_directories(vkoo PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(vkoo Vulkan::Vulkan stb spirv-cross-glsl imgui SPIRV glslang glslang-default-resource-limits glfw glm spdlog)
target_compile_definitions(vkoo PRIVATE SHADER_PATH="${CMAKE_CURRENT_SOURCE_DIR}/shaders" ASSET_PATH="${CMAKE_CURRENT_SOURCE_DIR}/assets")
