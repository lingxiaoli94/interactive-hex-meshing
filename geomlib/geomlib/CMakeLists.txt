# Host sources, compiled by the C++ compiler in every configuration.
file(GLOB geomlib_src *.cpp io/*.cpp autograd/*.cpp)

if (HEX_ENABLE_CUDA)
    set(CMAKE_CUDA_STANDARD 17)
    set(CMAKE_CUDA_ARCHITECTURES 75)

    # Note: Libtorch only supports the style of findCUDA.
    find_package(CUDA REQUIRED)
    find_package(CUDAToolkit REQUIRED)

    set(CUDA_SEPARABLE_COMPILATION ON)

    # Listed explicitly rather than globbed: in a CPU-only build plain g++ must
    # never be handed a .cu, and a glob would silently reintroduce one.
    set(geomlib_cuda_src
            generalized_projection_cuda.cu
            point_tet_mesh_test_cuda.cu)

    cuda_add_library(geomlib ${geomlib_src} ${geomlib_cuda_src})

    target_link_libraries(geomlib eigen CUDA::cudart CUDA::cusolver CUDA::cupti ${TORCH_LIBRARIES} OpenMP::OpenMP_CXX spdlog)
else ()
    add_library(geomlib ${geomlib_src})

    target_link_libraries(geomlib eigen ${TORCH_LIBRARIES} OpenMP::OpenMP_CXX spdlog)
endif ()

target_include_directories(geomlib PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/../")

# PUBLIC: geomlib's public headers carry conditional declarations, so consumers
# must see the same value. Numeric -- see the option() in the top-level file.
target_compile_definitions(geomlib PUBLIC HEX_ENABLE_CUDA=$<BOOL:${HEX_ENABLE_CUDA}>)
