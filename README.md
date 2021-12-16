# Interactive All-Hex Meshing via Cuboid Decomposition

![teaser](https://user-images.githubusercontent.com/38452438/144882120-359ea450-03f8-4267-93f4-6a78f9ffff80.png)
**[Video demonstration](https://www.dropbox.com/s/v687elfwgjnzfx4/demo.mp4?dl=0)**

This repository contains an interactive software to the PolyCube-based hex-meshing problem. You can solve hex meshing by playing minecraft!

Features include:
- a 4-stage interactive pipeline that can robustly generate high-quality hex meshes from an input tetrahedral mesh;
- extensive user control over each stage, such as editing the voxelized PolyCube, positioning surface vertices, and exploring the trade-off among competing quality metrics;
- automatic alternatives based on GPU-powered continuous optimization that can run at interactive speed.

It is the original implementation of the SIGGRAPH Asia 2021 paper "Interactive All-Hex Meshing via Cuboid Decomposition" by 
Lingxiao Li, Paul Zhang, Dmitriy Smirnov, Mazdak Abulnaga, Justin Solomon.
**Check out our [paper](https://arxiv.org/pdf/2109.06279.pdf) for a complete description of our pipeline!**


## Organization
There are three main components of the project.
- The `geomlib` folder contains a standalone C++ library with GPU-based geometric operations including point-triangle projection (in arbitrary dimensions), point-tetrahedron projection (in arbitrary dimensions), point-in-tet-mesh inclusion testing, sampling on a triangular mesh, capable of handling tens of thousands of point queries on large meshes in milliseconds.
- The `vkoo` folder contains a standalone object-oriented Vulkan graphics engine that is built based on the [official Vulkan samples code](https://github.com/KhronosGroup/Vulkan-Samples) with a lot of simplification and modification for the purpose of this project.
- The `hex` folder contains the application-specific code for our interactive PolyCube-based hex meshing software, and should be most relevant for learning about the implementation details of our paper.

In addition,
- The `assets` folder contains a small number of tetrahedral meshes to test on, but you can include your own meshes easily (if you only have triangular meshes, try using TetGen or [this](https://github.com/wildmeshing/fTetWild) to mesh the interior first).
- The `external` folder contains additional dependencies that are included in the repo.

## Dependencies
Main dependencies that are not included in the repo and should be installed first:
- CMake
- CUDA (tested with 11.2, 11.3, 11.4, 11.5) and cuDNN
- Pytorch C++ frontend (tested with 1.7, 1.8, 1.9, 1.10)
- Vulkan SDK
- Python3
- HDF5

There are additional dependencies in `external` and should be built correctly with the provided CMake hierarchy:
- Eigen
- glfw
- glm
- glslang
- imgui
- spdlog
- spirv-cross
- stb
- yaml-cpp

## Linux Instruction
The instruction is slightly different on various Linux distributions. We have tested on Arch Linux and Ubuntu 20.04.
First install all dependencies above using the respective package manager. 
Then download and unzip [Pytorch C++ frontend](https://pytorch.org/get-started/locally/) for Linux (tested with cxx11 ABI) -- it should be under the tab `Libtorch > C++/Java > CUDA 11.x`.
Add `Torch_DIR=<unzipped folder>` to your environment variable lists (or add your unzipped folder to `CMAKE_PREFIX_PATH`).
Then clone the repo (**be sure to use `--recursive` to clone the submodules as well**).
Next run the usual cmake/make commands to build target `hex` in Debug or Release mode:
```
mkdir -p build/Release
cd build/Release
cmake ../.. -DCMAKE_BUILD_TYPE=Release
make hex -j
```
This should generate an executable named `hex` under `bin/Release/hex` which can be run directly.
See `CMakeLists.txt` for more information.


## Windows Instruction
Compiling on Windows is trickier than on Linux. The following procedure has been tested to work on multiple Windows machines.
- Download and install Visual Studio 2019
- Download and install the newest CUDA Toolkit (tested with 11.2)
- Download and install cuDNN for Windows (this amounts to copying a bunch of `dll`'s to the CUDA path)
- Download and install the newest Vulkan SDK binary for Windows
- Download and install Python3
- Download and unzip [Pytorch C++ frontend](https://pytorch.org/get-started/locally/) for Windows. Then add `TORCH_DIR=<unzipped folder>` to your environment variable lists.
- Download and install HDF5 for Windows
- In VS2019, install CMake tools, and then build the project following [this](https://docs.microsoft.com/en-us/cpp/build/cmake-projects-in-visual-studio?view=msvc-160)
This should generate an executable under `bin/Debug` or `bin/Release`.
