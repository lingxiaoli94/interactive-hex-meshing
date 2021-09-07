#pragma once

#include "geomlib/TetMesh.h"

namespace geomlib {
class TetMeshParser {
 public:
  static std::unique_ptr<TetMesh> Parse(const std::string& file_path);
  static std::unique_ptr<TetMesh> ParseMeditToMesh(
      const std::string& file_path);
  static std::unique_ptr<TetMesh> ParseVtkToMesh(const std::string& file_path);

 private:
};
}  // namespace geomlib
