#include <iostream>
#include <string>

#include "HexMeshingApp.h"
#include "cli/PipelineScriptRunner.h"
#include "optim/torch_utils.h"

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
}  // namespace

int main(int argc, char** argv) {
  std::string script_path;
  bool exit_after = false;
  bool headless = false;
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
    } else if (a == "--device" && i + 1 < argc) {
      device = argv[++i];
    } else if (a == "--help" || a == "-h") {
      std::cout << "Usage: hex [--script <yaml>] [--exit-after] [--headless] "
                   "[--device "
                << kDeviceChoices << "]\n"
                << "  --script <yaml>   run scripted pipeline from YAML config\n"
                << "  --exit-after      close the window after the script "
                   "finishes\n"
                << "  --headless        run with no GUI window/surface at all "
                   "(no X11);\n"
                << "                    runs the script and exits. Implies "
                   "--exit-after.\n"
                << "  --device " << kDeviceChoices << "\n"
                << "                    compute device for the pipeline math "
                   "(default "
                << kDefaultDevice << ").\n"
                << "                    cpu runs without an NVIDIA GPU; pair with "
                   "--headless.\n";
      if (!kCudaBuild) {
        std::cout << "\nThis binary was built without CUDA "
                     "(-DHEX_ENABLE_CUDA=OFF): it links no CUDA at\n"
                     "all and cpu is the only available device.\n";
      }
      return 0;
    }
  }

  if (headless && script_path.empty()) {
    std::cerr << "--headless requires --script <yaml> (nothing to display, so "
                 "there is nothing to do without a script).\n";
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

  HexMeshingApp app{true};
  app.Prepare(headless);

  if (!script_path.empty()) {
    PipelineScriptRunner runner(app);
    bool ok = runner.LoadAndRun(script_path);
    if (!ok) return 1;
    // Headless never created a window, so there is no main loop to enter.
    if (headless || exit_after || runner.RequestedExit()) return 0;
  }

  app.MainLoop();
  return 0;
}
