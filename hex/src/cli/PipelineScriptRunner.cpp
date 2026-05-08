#include "cli/PipelineScriptRunner.h"

#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <iostream>
#include <stdexcept>

#include "HexMeshingApp.h"
#include "controllers/GlobalController.h"
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

}  // namespace

PipelineScriptRunner::PipelineScriptRunner(HexMeshingApp& app) : app_(app) {}

bool PipelineScriptRunner::LoadAndRun(const std::string& yaml_path) {
  YAML::Node script;
  try {
    script = YAML::LoadFile(yaml_path);
  } catch (const std::exception& e) {
    std::cerr << "[script] failed to parse YAML " << yaml_path << ": "
              << e.what() << std::endl;
    return false;
  }

  auto& gc = app_.GetGlobalController();

  // ---- 1. Load input ----
  if (!script["input"]) {
    std::cerr << "[script] missing 'input' section in " << yaml_path
              << std::endl;
    return false;
  }
  std::string input_type =
      script["input"]["type"] ? script["input"]["type"].as<std::string>()
                              : "hdf5";
  std::string input_path = script["input"]["path"].as<std::string>();
  LOGI("[script] loading input ({}): {}", input_type, input_path);

  try {
    if (input_type == "hdf5") {
      gc.OpenProject(input_path);
    } else if (input_type == "mesh") {
      gc.LoadTargetMesh(input_path);
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
  if (script["output"] && script["output"]["run_dir"]) {
    run_dir = script["output"]["run_dir"].as<std::string>();
    fs::create_directories(run_dir);
  } else {
    // Default to the YAML's own directory if no run_dir provided.
    run_dir = fs::path(yaml_path).parent_path().string();
  }
  LOGI("[script] run_dir: {}", run_dir);

  // ---- 3. Run each stage ----
  if (!script["stages"] || !script["stages"].IsSequence()) {
    std::cerr << "[script] missing or invalid 'stages' list" << std::endl;
    return false;
  }

  bool ran_hexahedralization = false;
  for (const auto& entry : script["stages"]) {
    if (!entry.IsMap() || entry.size() != 1) {
      std::cerr << "[script] each stage entry must be a single-key map "
                   "(e.g., '- discretization: {...}')"
                << std::endl;
      return false;
    }
    auto it = entry.begin();
    std::string stage_name = it->first.as<std::string>();
    YAML::Node stage_cfg = it->second;
    int idx = StageIndex(stage_name);
    if (idx < 0) {
      std::cerr << "[script] unknown stage: " << stage_name << std::endl;
      return false;
    }

    LOGI("[script] === running stage {} ({}) ===", idx, stage_name);
    try {
      if (stage_name == "discretization") {
        gc.GetDiscretizationStage().RunFromScript(stage_cfg);
      } else if (stage_name == "hexahedralization") {
        gc.GetHexahedralizationStage().RunFromScript(stage_cfg);
        ran_hexahedralization = true;
      } else {
        std::cerr << "[script] stage '" << stage_name
                  << "' is not yet supported by the script runner (only "
                     "discretization and hexahedralization in this build)"
                  << std::endl;
        return false;
      }
    } catch (const std::exception& e) {
      std::cerr << "[script] stage '" << stage_name << "' failed: " << e.what()
                << std::endl;
      return false;
    }

    // Save intermediate HDF5 after this stage.
    std::string stage_file =
        ResolvePath(run_dir, StageFileName(idx, stage_name));
    LOGI("[script] saving {}", stage_file);
    try {
      gc.SaveProject(stage_file);
    } catch (const std::exception& e) {
      std::cerr << "[script] failed to save " << stage_file << ": " << e.what()
                << std::endl;
      return false;
    }
  }

  // ---- 4. Optional final mesh export ----
  if (ran_hexahedralization && script["output"] &&
      script["output"]["export_mesh"]) {
    std::string mesh_rel = script["output"]["export_mesh"].as<std::string>();
    std::string mesh_path = ResolvePath(run_dir, mesh_rel);
    LOGI("[script] exporting final hex mesh to {}", mesh_path);
    try {
      gc.ExportTargetComplex(mesh_path);
    } catch (const std::exception& e) {
      std::cerr << "[script] export_mesh failed: " << e.what() << std::endl;
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
