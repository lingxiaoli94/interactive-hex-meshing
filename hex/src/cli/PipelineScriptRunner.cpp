#include "cli/PipelineScriptRunner.h"

#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <system_error>
#include <vector>

#include "cli/MetricsDumper.h"
#include "controllers/GlobalController.h"
#include "controllers/stages/DecompositionStage.h"
#include "controllers/stages/DeformationStage.h"
#include "controllers/stages/DiscretizationStage.h"
#include "controllers/stages/HexahedralizationStage.h"
#include "logging.h"
#include "models/GlobalState.h"
#include "serialization/Serializer.h"

namespace fs = std::filesystem;

namespace hex {
namespace {

// Maps a stage name from the YAML to a numeric index used in the saved file
// name. Matches the pipeline numbering documented in PIPELINE_NOTES.md.
int StageIndex(const std::string& name) {
  if (name == "deformation") return 0;
  if (name == "decomposition") return 1;
  if (name == "discretization") return 2;
  if (name == "hexahedralization") return 3;
  return -1;
}

std::string StageFileName(int idx, const std::string& name) {
  return "stage_" + std::to_string(idx) + "_" + name + ".hdf5";
}

// Resolves a file path relative to a base directory. If `path` is already
// absolute, returns it unchanged.
std::string ResolvePath(const std::string& base, const std::string& path) {
  fs::path p(path);
  if (p.is_absolute()) return p.string();
  return (fs::path(base) / p).string();
}

fs::path NormalizePathForComparison(const fs::path& path) {
  std::error_code error;
  fs::path normalized = fs::weakly_canonical(path, error);
  if (!error) return normalized;

  error.clear();
  normalized = fs::absolute(path, error);
  return error ? path.lexically_normal() : normalized.lexically_normal();
}

bool RefersToSameFileOrPath(const fs::path& first, const fs::path& second) {
  std::error_code error;
  if (fs::exists(first, error)) {
    error.clear();
    if (fs::exists(second, error)) {
      error.clear();
      if (fs::equivalent(first, second, error) && !error) return true;
    }
  }
  return NormalizePathForComparison(first) ==
         NormalizePathForComparison(second);
}

struct StageSpec {
  std::string name;
  YAML::Node config;
  int index;
};

struct OutputDestination {
  std::string label;
  fs::path path;
};

}  // namespace

PipelineScriptRunner::PipelineScriptRunner(GlobalController& global_controller)
    : gc_(global_controller) {}

bool PipelineScriptRunner::LoadAndRun(const std::string& yaml_path) {
  YAML::Node script;
  try {
    script = YAML::LoadFile(yaml_path);
  } catch (const std::exception& e) {
    std::cerr << "[script] failed to parse YAML " << yaml_path << ": "
              << e.what() << std::endl;
    return false;
  }

  // Decode the complete stage list before touching the input. The first stage
  // determines the HDF5 restart boundary, so loading first would already expose
  // controller/view constructors to stale downstream state.
  if (!script["stages"] || !script["stages"].IsSequence() ||
      script["stages"].size() == 0) {
    std::cerr << "[script] missing, empty, or invalid 'stages' list"
              << std::endl;
    return false;
  }

  std::vector<StageSpec> stages;
  for (const auto& entry : script["stages"]) {
    if (!entry.IsMap() || entry.size() != 1) {
      std::cerr << "[script] each stage entry must be a single-key map "
                   "(e.g., '- discretization: {...}')"
                << std::endl;
      return false;
    }

    try {
      auto it = entry.begin();
      std::string stage_name = it->first.as<std::string>();
      int stage_index = StageIndex(stage_name);
      if (stage_index < 0) {
        std::cerr << "[script] unknown stage: " << stage_name << std::endl;
        return false;
      }
      if (!stages.empty() && stage_index != stages.back().index + 1) {
        std::cerr
            << "[script] multiple stages must be contiguous and in strictly "
               "increasing pipeline order; skipped, backward, or repeated "
               "stages require a separate fresh run"
            << std::endl;
        return false;
      }
      stages.push_back(StageSpec{stage_name, it->second, stage_index});
    } catch (const std::exception& e) {
      std::cerr << "[script] invalid stage entry: " << e.what() << std::endl;
      return false;
    }
  }

  // ---- 1. Load input at the first requested stage boundary ----
  if (!script["input"]) {
    std::cerr << "[script] missing 'input' section in " << yaml_path
              << std::endl;
    return false;
  }
  std::string input_type;
  std::string input_path;
  try {
    input_type = script["input"]["type"]
                     ? script["input"]["type"].as<std::string>()
                     : "hdf5";
    if (!script["input"]["path"]) {
      throw std::runtime_error("input.path is required");
    }
    input_path = script["input"]["path"].as<std::string>();
  } catch (const std::exception& e) {
    std::cerr << "[script] invalid input section: " << e.what() << std::endl;
    return false;
  }
  LOGI("[script] loading input ({}): {}", input_type, input_path);
  if (input_type == "hdf5") {
    LOGI("[script] fresh restart boundary: stage {} ({})", stages.front().index,
         stages.front().name);
  }

  try {
    if (input_type == "hdf5") {
      gc_.OpenProjectForStage(input_path, stages.front().index);
    } else if (input_type == "mesh") {
      if (stages.front().index != 0) {
        throw std::runtime_error(
            "raw .mesh/.vtk input can only start with deformation");
      }
      gc_.LoadTargetMesh(input_path);
    } else {
      std::cerr << "[script] unknown input.type: " << input_type << std::endl;
      return false;
    }
  } catch (const std::exception& e) {
    std::cerr << "[script] failed to load input: " << e.what() << std::endl;
    return false;
  }

  // ---- 2. Resolve run directory ----
  std::string run_dir;
  std::vector<std::string> stage_files;
  std::optional<std::string> export_mesh_path;
  std::optional<std::string> export_metrics_path;
  std::vector<OutputDestination> output_destinations;
  try {
    if (script["output"] && script["output"]["run_dir"]) {
      run_dir = script["output"]["run_dir"].as<std::string>();
      fs::create_directories(run_dir);
    } else {
      // Default to the YAML's own directory if no run_dir provided.
      run_dir = fs::path(yaml_path).parent_path().string();
    }

    for (const auto& stage : stages) {
      stage_files.push_back(
          ResolvePath(run_dir, StageFileName(stage.index, stage.name)));
      output_destinations.push_back(
          {"stage " + std::to_string(stage.index) + " output",
           stage_files.back()});
    }
    if (script["output"] && script["output"]["export_mesh"]) {
      export_mesh_path = ResolvePath(
          run_dir, script["output"]["export_mesh"].as<std::string>());
      output_destinations.push_back({"export_mesh", *export_mesh_path});
    }
    if (script["output"] && script["output"]["export_metrics"]) {
      export_metrics_path = ResolvePath(
          run_dir, script["output"]["export_metrics"].as<std::string>());
      output_destinations.push_back(
          {"export_metrics", *export_metrics_path});
    }
  } catch (const std::exception& e) {
    std::cerr << "[script] invalid output section: " << e.what() << std::endl;
    return false;
  }
  LOGI("[script] run_dir: {}", run_dir);

  // Stage saves and final exports all truncate their destinations. Direct YAML
  // users are not protected by run.sh's input copy, so reject every collision
  // (including existing symlinks/hardlinks) before the first stage runs.
  for (size_t i = 0; i < output_destinations.size(); i++) {
    const auto& destination = output_destinations[i];
    if (RefersToSameFileOrPath(input_path, destination.path)) {
      std::cerr << "[script] refusing to overwrite input file with "
                << destination.label << ": " << destination.path << std::endl;
      return false;
    }
    for (size_t j = 0; j < i; j++) {
      const auto& previous = output_destinations[j];
      if (RefersToSameFileOrPath(previous.path, destination.path)) {
        std::cerr << "[script] output destinations collide ("
                  << previous.label << " and " << destination.label
                  << "): " << destination.path << std::endl;
        return false;
      }
    }
  }

  // ---- 3. Run each stage ----
  bool ran_hexahedralization = false;
  for (size_t stage_position = 0; stage_position < stages.size();
       stage_position++) {
    const auto& stage = stages[stage_position];
    const std::string& stage_name = stage.name;
    const YAML::Node& stage_cfg = stage.config;
    int idx = stage.index;

    LOGI("[script] === running stage {} ({}) ===", idx, stage_name);
    try {
      if (stage_name == "deformation") {
        gc_.GetDeformationStage().RunFromScript(stage_cfg);
      } else if (stage_name == "decomposition") {
        gc_.GetDecompositionStage().RunFromScript(stage_cfg);
      } else if (stage_name == "discretization") {
        gc_.GetDiscretizationStage().RunFromScript(stage_cfg);
      } else if (stage_name == "hexahedralization") {
        gc_.GetHexahedralizationStage().RunFromScript(stage_cfg);
        ran_hexahedralization = true;
      } else {
        std::cerr << "[script] unknown stage name '" << stage_name << "'"
                  << std::endl;
        return false;
      }
    } catch (const std::exception& e) {
      std::cerr << "[script] stage '" << stage_name << "' failed: " << e.what()
                << std::endl;
      return false;
    }

    // Save intermediate HDF5 after this stage.
    const std::string& stage_file = stage_files[stage_position];
    LOGI("[script] saving {}", stage_file);
    try {
      gc_.SaveProject(stage_file);
    } catch (const std::exception& e) {
      std::cerr << "[script] failed to save " << stage_file << ": " << e.what()
                << std::endl;
      return false;
    }
  }

  // ---- 4. Optional final mesh export ----
  if (ran_hexahedralization && export_mesh_path) {
    LOGI("[script] exporting final hex mesh to {}", *export_mesh_path);
    try {
      gc_.ExportTargetComplex(*export_mesh_path);
    } catch (const std::exception& e) {
      std::cerr << "[script] export_mesh failed: " << e.what() << std::endl;
      return false;
    }
  }

  // ---- 4b. Optional quality-metrics sidecar ----
  if (ran_hexahedralization && export_metrics_path) {
    try {
      if (!gc_.GetGlobalState().HasResultMesh()) {
        throw std::runtime_error(
            "no result_mesh in global state; hexahedralization did not "
            "produce a final hex mesh");
      }
      DumpQualityMetrics(gc_.GetGlobalState().GetResultMesh(),
                         *export_metrics_path);
    } catch (const std::exception& e) {
      std::cerr << "[script] export_metrics failed: " << e.what() << std::endl;
      return false;
    }
  }

  // ---- 5. Window behavior ----
  if (script["keep_window_open"]) {
    requested_exit_ = !script["keep_window_open"].as<bool>();
  }

  LOGI("[script] done.");
  return true;
}

}  // namespace hex
