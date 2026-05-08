#include <iostream>
#include <string>

#include "HexMeshingApp.h"
#include "cli/PipelineScriptRunner.h"

using namespace hex;

int main(int argc, char** argv) {
  std::string script_path;
  bool exit_after = false;
  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    if (a == "--script" && i + 1 < argc) {
      script_path = argv[++i];
    } else if (a == "--exit-after") {
      exit_after = true;
    } else if (a == "--help" || a == "-h") {
      std::cout << "Usage: hex [--script <yaml>] [--exit-after]\n"
                << "  --script <yaml>   run scripted pipeline from YAML config\n"
                << "  --exit-after      close the window after the script "
                   "finishes\n";
      return 0;
    }
  }

  HexMeshingApp app{true};
  app.Prepare();

  if (!script_path.empty()) {
    PipelineScriptRunner runner(app);
    bool ok = runner.LoadAndRun(script_path);
    if (!ok) return 1;
    if (exit_after || runner.RequestedExit()) return 0;
  }

  app.MainLoop();
  return 0;
}
