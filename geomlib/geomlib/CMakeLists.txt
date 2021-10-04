set(CMAKE_CUDA_STANDARD 17)
set(CMAKE_CUDA_ARCHITECTURES 75)

# Note: Libtorch only supports the style of findCUDA.
find_package(CUDA REQUIRED)
find_package(CUDAToolkit REQUIRED)

set(CUDA_SEPARABLE_COMPILATION ON)

file(GLOB geomlib_src *.cpp io/*.cpp *.cu autograd/*.cpp)

cuda_add_library(geomlib ${geomlib_src})

target_include_directories(geomlib PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/../")
target_link_libraries(geomlib eigen CUDA::cudart CUDA::cusolver CUDA::cupti ${TORCH_LIBRARIES} OpenMP::OpenMP_CXX spdlog)
