#pragma once

#include "models/GlobalState.h"

namespace hex {
class Serializer {
 public:
  static void SaveState(const GlobalState& state, const std::string& file_path);

  static std::unique_ptr<GlobalState> LoadState(const std::string& file_path);

  // Loads only the upstream dependency closure needed to rerun `stage_index`.
  // The requested stage's previous output and every downstream output are
  // deliberately omitted. Unlike loading everything and pruning afterward,
  // this prevents stale or malformed downstream groups from being consumed
  // while controllers and views are rebuilt.
  static std::unique_ptr<GlobalState> LoadStateForStage(
      const std::string& file_path, int stage_index);

  static Eigen::MatrixXf LoadSingleMatrixXf(const std::string& file_path,
                                            const std::string& name);

  static void SaveHexMesh(
      const HexahedralMesh& mesh, const std::string& file_path,
      std::function<Vector3f(Vector3f)> vertex_map = [](Vector3f v) {
        return v;
      });
  static std::unique_ptr<HexahedralMesh> LoadHexMesh(
      const std::string& file_path);
};
}  // namespace hex
