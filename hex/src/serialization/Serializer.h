#pragma once

#include "models/GlobalState.h"

namespace hex {
class Serializer {
 public:
  static void SaveState(const GlobalState& state, const std::string& file_path);

  static std::unique_ptr<GlobalState> LoadState(const std::string& file_path);

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
