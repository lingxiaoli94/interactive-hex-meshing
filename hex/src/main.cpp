#include <iostream>
#include <string>

#include "HexMeshingApp.h"
#include "cli/PipelineScriptRunner.h"
#include "optim/torch_utils.h"

using namespace hex;

int main(int argc, char** argv) {
  std::string script_path;
  bool exit_after = false;
  bool headless = false;
  std::string device = "cuda";  // default preserves original behavior
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
                   "[--device cpu|cuda]\n"
                << "  --script <yaml>   run scripted pipeline from YAML config\n"
                << "  --exit-after      close the window after the script "
                   "finishes\n"
                << "  --headless        run with no GUI window/surface at all "
                   "(no X11);\n"
                << "                    runs the script and exits. Implies "
                   "--exit-after.\n"
                << "  --device cpu|cuda compute device for the pipeline math "
                   "(default cuda).\n"
                << "                    cpu runs without an NVIDIA GPU; pair with "
                   "--headless.\n";
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
  } else if (device == "cuda") {
    hex::SetComputeDevice(torch::Device(torch::kCUDA));
  } else {
    std::cerr << "--device must be 'cpu' or 'cuda' (got '" << device << "').\n";
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
