#pragma once

#include <string>

namespace hex {
class HexMeshingApp;

// Reads a YAML pipeline-automation script, runs the requested stages by
// dispatching to the existing stage controllers, and writes per-stage HDF5
// snapshots and an optional final .mesh export to the run directory.
//
// This is *scripted GUI automation*, not a headless CLI: the host application
// (Vulkan + window) must already be initialized before LoadAndRun is called.
class PipelineScriptRunner {
 public:
  explicit PipelineScriptRunner(HexMeshingApp& app);

  // Returns true on success, false on any error (logged + printed to stderr).
  bool LoadAndRun(const std::string& yaml_path);

  // True if the YAML asked us to close the window after running. main.cpp
  // also honors --exit-after on top of this.
  bool RequestedExit() const { return requested_exit_; }

 private:
  HexMeshingApp& app_;
  bool requested_exit_{false};
};
}  // namespace hex
