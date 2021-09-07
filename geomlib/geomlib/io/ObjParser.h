#pragma once

#include "geomlib/TriMesh.h"
#include "geomlib/common.h"

namespace geomlib {
class ObjParser {
 public:
  struct ParsedData {
    std::unique_ptr<Eigen::MatrixXf> positions;
    std::unique_ptr<Eigen::MatrixXf> normals;
    std::unique_ptr<Eigen::MatrixXi> faces;
    std::unique_ptr<Eigen::MatrixXf> tex_coords;
  };

  static ParsedData Parse(const std::string& file_path);
  static std::unique_ptr<TriMesh> ParseToMesh(const std::string& file_path);
};
}  // namespace geomlib
