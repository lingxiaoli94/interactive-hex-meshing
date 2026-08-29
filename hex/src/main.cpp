#include <iostream>
#include <string>

#include "HeadlessSession.h"
#include "cli/PipelineScriptRunner.h"
#include "optim/torch_utils.h"
#if HEX_ENABLE_VULKAN
#include "HexMeshingApp.h"
#endif

using namespace hex;

namespace {
// Single source of truth for what this binary was built to do, so the default
// device, the --device validation and the --help text cannot drift apart.
#if HEX_ENABLE_CUDA
constexpr bool kCudaBuild = true;
#else
constexpr bool kCudaBuild = false;
#endif
constexpr const char* kDefaultDevice = kCudaBuild ? "cuda" : "cpu";
constexpr const char* kDeviceChoices = kCudaBuild ? "cpu|cuda" : "cpu";
constexpr const char* kDeviceChoicesProse =
    kCudaBuild ? "'cpu' or 'cuda'" : "'cpu'";

// A HEX_ENABLE_VULKAN=OFF binary contains no renderer at all: --headless and
// --no-vulkan are accepted as idempotent no-ops so the same command lines and
// scripts keep working, and the GUI simply does not exist.
#if HEX_ENABLE_VULKAN
constexpr bool kVulkanBuild = true;
#else
constexpr bool kVulkanBuild = false;
#endif
}  // namespace

int main(int argc, char** argv) {
  std::string script_path;
  [[maybe_unused]] bool exit_after = false;
  bool headless = false;
  bool no_vulkan = false;
  // cuda where the build has it; a CUDA-free build can only ever be cpu.
  std::string device = kDefaultDevice;
  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    if (a == "--script" && i + 1 < argc) {
      script_path = argv[++i];
    } else if (a == "--exit-after") {
      exit_after = true;
    } else if (a == "--headless") {
      headless = true;
    } else if (a == "--no-vulkan") {
      no_vulkan = true;
    } else if (a == "--device" && i + 1 < argc) {
      device = argv[++i];
    } else if (a == "--help" || a == "-h") {
      std::cout << "Usage: hex [--script <yaml>] [--exit-after] [--headless] "
                   "[--no-vulkan] [--device "
                << kDeviceChoices << "]\n"
                << "  --script <yaml>   run scripted pipeline from YAML config\n";
      if (kVulkanBuild) {
        std::cout << "  --exit-after      close the window after the script "
                     "finishes\n"
                  << "  --headless        run with no GUI window/surface at "
                     "all (no X11);\n"
                  << "                    runs the script and exits. Implies "
                     "--exit-after.\n"
                  << "  --no-vulkan       make no Vulkan API call at all: no "
                     "instance, device,\n"
                  << "                    sampler or view is created, so no "
                     "ICD or graphics driver\n"
                  << "                    is needed. Implies --headless. The "
                     "binary still links\n"
                  << "                    the Vulkan loader.\n";
      } else {
        std::cout << "  --exit-after      accepted, no effect (there is no "
                     "window to close)\n"
                  << "  --headless        accepted, no effect (every run is "
                     "headless)\n"
                  << "  --no-vulkan       accepted, no effect (this binary "
                     "contains no Vulkan)\n";
      }
      std::cout << "  --device " << kDeviceChoices << "\n"
                << "                    compute device for the pipeline math "
                   "(default "
                << kDefaultDevice << ").\n"
                << "                    cpu runs without an NVIDIA GPU"
                << (kVulkanBuild ? "; pair with --headless.\n" : ".\n");
      if (!kCudaBuild) {
        std::cout << "\nThis binary was built without CUDA "
                     "(-DHEX_ENABLE_CUDA=OFF): it links no CUDA at\n"
                     "all and cpu is the only available device.\n";
      }
      if (!kVulkanBuild) {
        std::cout << "\nThis binary was built without the renderer "
                     "(-DHEX_ENABLE_VULKAN=OFF): it links no\n"
                     "Vulkan, imgui or glfw at all. There is no GUI, so "
                     "--script is required and\n"
                     "--headless / --no-vulkan are accepted but do nothing.\n";
      }
      return 0;
    }
  }

  // One-way implication: there is no second render backend, so a GUI without
  // Vulkan is not a thing that can exist. In a Vulkan-free build every run is
  // headless whether or not the flag was passed.
  if (!kVulkanBuild) {
    headless = true;
  } else if (no_vulkan && !headless) {
    headless = true;
    std::cout << "--no-vulkan implies --headless (there is no non-Vulkan "
                 "render backend).\n";
  }

  if (headless && script_path.empty()) {
    if (!kVulkanBuild) {
      std::cerr << "This hex was built without the renderer "
                   "(-DHEX_ENABLE_VULKAN=OFF), so there is no\n"
                   "GUI to open: --script <yaml> is required.\n";
    } else {
      std::cerr << (no_vulkan ? "--no-vulkan" : "--headless")
                << " requires --script <yaml> (nothing to display, so there is "
                   "nothing to do without a script).\n";
    }
    return 1;
  }

  if (device == "cpu") {
    hex::SetComputeDevice(torch::Device(torch::kCPU));
  } else if (device == "cuda" && kCudaBuild) {
    hex::SetComputeDevice(torch::Device(torch::kCUDA));
  } else if (device == "cuda") {
    std::cerr << "--device cuda is unavailable: this hex was built without "
                 "CUDA (-DHEX_ENABLE_CUDA=OFF). Use --device cpu.\n";
    return 1;
  } else {
    std::cerr << "--device must be " << kDeviceChoicesProse << " (got '"
              << device << "').\n";
    return 1;
  }

  // No Vulkan wanted (or none available): run on the headless session owner,
  // which has no vkoo::Application under it at all. This is the OFF build's
  // only path, and running the ON build's --no-vulkan through it too means the
  // OFF build's code is exercised by every no-Vulkan test in either variant.
  if (!kVulkanBuild || no_vulkan) {
    HeadlessSession session;
    PipelineScriptRunner runner(session.GetGlobalController());
    return runner.LoadAndRun(script_path) ? 0 : 1;
  }

#if HEX_ENABLE_VULKAN
  // The two remaining combinations, resolved to one value so no downstream code
  // has to re-derive them. See vkoo/core/DisplayMode.h.
  const vkoo::DisplayMode display_mode = headless
                                             ? vkoo::DisplayMode::HeadlessVulkan
                                             : vkoo::DisplayMode::Gui;

  HexMeshingApp app{true};
  app.Prepare(display_mode);

  if (!script_path.empty()) {
    PipelineScriptRunner runner(app.GetGlobalController());
    bool ok = runner.LoadAndRun(script_path);
    if (!ok) return 1;
    // Headless never created a window, so there is no main loop to enter.
    if (headless || exit_after || runner.RequestedExit()) return 0;
  }

  app.MainLoop();
#endif
  return 0;
}
