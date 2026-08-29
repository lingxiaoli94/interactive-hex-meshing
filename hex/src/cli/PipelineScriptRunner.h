#pragma once

#include <string>

namespace hex {
class GlobalController;

// Reads a YAML pipeline-automation script, runs the requested stages by
// dispatching to the existing stage controllers, and writes per-stage HDF5
// snapshots and an optional final .mesh export to the run directory.
//
// Takes the GlobalController directly. It used to take HexMeshingApp& purely to
// call GetGlobalController() on it, which made the script runner -- the one
// component that is pure automation -- depend on the Vulkan application type
// and prevented a HEX_ENABLE_VULKAN=OFF build from containing it at all.
class PipelineScriptRunner {
 public:
  explicit PipelineScriptRunner(GlobalController& global_controller);

  // Returns true on success, false on any error (logged + printed to stderr).
  bool LoadAndRun(const std::string& yaml_path);

  // True if the YAML asked us to close the window after running. main.cpp
  // also honors --exit-after on top of this.
  bool RequestedExit() const { return requested_exit_; }

 private:
  GlobalController& gc_;
  bool requested_exit_{false};
};
}  // namespace hex
