# hdf5
find_package(HDF5 COMPONENTS CXX REQUIRED)

file(GLOB hex_srcs
        ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/cli/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/models/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/optim/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/views/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/controllers/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/controllers/stages/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/serialization/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/utility/*.cpp)

if (NOT HEX_ENABLE_VULKAN)
    # Remove the renderer from the target rather than #if-ing out whole files:
    # every view, the cuboid editor, the ImGui helpers, the Vulkan application
    # and each stage's GUI translation unit. Phase A proved every one of these
    # is unreachable on the script path, which is what makes the removal safe
    # rather than hopeful.
    #
    # The *Gui.cpp files additionally carry their own `#if HEX_ENABLE_VULKAN`,
    # so a future glob change cannot quietly pull them back in.
    list(FILTER hex_srcs EXCLUDE REGEX "/src/views/")
    list(FILTER hex_srcs EXCLUDE REGEX "/src/controllers/CuboidEditingController\\.cpp$")
    list(FILTER hex_srcs EXCLUDE REGEX "/src/utility/ImGuiEx\\.cpp$")
    list(FILTER hex_srcs EXCLUDE REGEX "/src/HexMeshingApp\\.cpp$")
    list(FILTER hex_srcs EXCLUDE REGEX "Gui\\.cpp$")
    # ShortcutController is NOT excluded: GlobalController holds one by value,
    # so its constructor must exist. Only its ImGui menu is #if'd out.
endif ()

add_executable(hex ${hex_srcs})
target_compile_options(hex PRIVATE ${cxx_warning_flags})
set_target_properties(hex PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/bin/$<CONFIG>")

# For some reason linking with HDF5 will automatically add D_FORTIFY_SOURCE=2 which may cause warnings in Debug mode.
target_link_libraries(hex vkoo ${TORCH_LIBRARIES} eigen geomlib OpenMP::OpenMP_CXX yaml-cpp::yaml-cpp HDF5::HDF5)
target_include_directories(hex PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_compile_definitions(hex PRIVATE IMPORT_PATH="${PROJECT_SOURCE_DIR}/assets")
# Numeric on purpose -- see the option() in the top-level CMakeLists.txt.
target_compile_definitions(hex PRIVATE HEX_ENABLE_CUDA=$<BOOL:${HEX_ENABLE_CUDA}>)


if (MSVC)
  file(GLOB TORCH_DLLS "${TORCH_INSTALL_PREFIX}/lib/*.dll")
  add_custom_command(TARGET hex
                     POST_BUILD
                     COMMAND ${CMAKE_COMMAND} -E copy_if_different
                     ${TORCH_DLLS}
                     $<TARGET_FILE_DIR:hex>)
endif (MSVC)
