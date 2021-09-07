# hdf5
find_package(HDF5 COMPONENTS CXX REQUIRED)

file(GLOB hex_srcs
        ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/models/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/optim/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/views/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/controllers/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/controllers/stages/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/serialization/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/utility/*.cpp)

add_executable(hex ${hex_srcs})
target_compile_options(hex PRIVATE ${cxx_warning_flags})
set_target_properties(hex PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/bin/$<CONFIG>")

# For some reason linking with HDF5 will automatically add D_FORTIFY_SOURCE=2 which may cause warnings in Debug mode.
target_link_libraries(hex vkoo ${TORCH_LIBRARIES} eigen geomlib OpenMP::OpenMP_CXX yaml-cpp::yaml-cpp HDF5::HDF5)
target_include_directories(hex PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_compile_definitions(hex PRIVATE IMPORT_PATH="${PROJECT_SOURCE_DIR}/assets")


if (MSVC)
  file(GLOB TORCH_DLLS "${TORCH_INSTALL_PREFIX}/lib/*.dll")
  add_custom_command(TARGET hex
                     POST_BUILD
                     COMMAND ${CMAKE_COMMAND} -E copy_if_different
                     ${TORCH_DLLS}
                     $<TARGET_FILE_DIR:hex>)
endif (MSVC)
