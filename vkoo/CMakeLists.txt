# Two source lists, one target. A separate vkoo-core/vkoo-render pair was
# considered and rejected: nothing links the core alone, and splitting would
# change the ON build's link line for no benefit.
if (HEX_ENABLE_VULKAN)
    file(GLOB vkoo_srcs
            src/*.cpp
            src/core/*.cpp
            src/st/*.cpp
            src/st/components/*.cpp
            src/st/scripts/*.cpp
            src/st/hittables/*.cpp)
else ()
    # Explicit, not a glob: the point of this list is that adding a renderer file
    # must NOT silently join the Vulkan-free build. This is the renderer-free
    # dependency closure: the scene graph plus the two Vulkan-free odds and
    # ends.
    #
    # core/InputEvent.cpp is here despite living in core/: InputEvent.h has no
    # includes at all -- no GLFW, no Vulkan -- and every stage signature uses it.
    set(vkoo_srcs
            src/utils.cpp                       # FindMemoryType is #if'd out
            src/ObjParser.cpp
            src/core/InputEvent.cpp
            src/st/Node.cpp
            src/st/Scene.cpp
            src/st/Transform.cpp
            src/st/components/Camera.cpp
            src/st/components/ComponentBase.cpp
            src/st/components/Light.cpp
            src/st/components/Script.cpp
            src/st/components/Tracing.cpp
            src/st/hittables/AABB.cpp
            src/st/hittables/Cylinder.cpp
            src/st/hittables/Plane.cpp
            src/st/hittables/Sphere.cpp
            src/st/hittables/Triangle.cpp
            src/st/scripts/ArcBallCameraScript.cpp)
    # Deliberately absent: everything else under src/core/, src/common.cpp (100%
    # Vulkan), and st/{Image,Material,Texture,ShaderProgram}.cpp +
    # st/components/Mesh.cpp, which are reachable only from views/.
endif ()

add_library(vkoo ${vkoo_srcs})
target_compile_options(vkoo PRIVATE ${cxx_warning_flags})
target_include_directories(vkoo PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

# PUBLIC, unlike hex's HEX_ENABLE_CUDA: vkoo/common.h is a public header and its
# Vulkan block is behind this guard, so every consumer must see the same value.
target_compile_definitions(vkoo PUBLIC HEX_ENABLE_VULKAN=$<BOOL:${HEX_ENABLE_VULKAN}>)

if (HEX_ENABLE_VULKAN)
    target_link_libraries(vkoo Vulkan::Vulkan stb spirv-cross-glsl imgui SPIRV glslang glslang-default-resource-limits glfw glm spdlog)
else ()
    # Verification I checks this list stays this short: no Vulkan, no glfw, no
    # imgui, no glslang/SPIRV/spirv-cross, no stb.
    target_link_libraries(vkoo glm spdlog)
endif ()

target_compile_definitions(vkoo PRIVATE SHADER_PATH="${CMAKE_CURRENT_SOURCE_DIR}/shaders" ASSET_PATH="${CMAKE_CURRENT_SOURCE_DIR}/assets")
