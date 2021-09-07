#pragma once

#include "common.h"

namespace vkoo {
struct MeshGroup {
  std::string name;
  size_t start_face_index;
  size_t num_indices;
  std::string material_name;
};

struct ParsedData {
  std::unique_ptr<std::vector<glm::vec3>> positions;
  std::unique_ptr<std::vector<glm::vec3>> normals;
  std::unique_ptr<std::vector<uint32_t>> indices;
  std::unique_ptr<std::vector<glm::vec2>> tex_coords;

  std::vector<MeshGroup> groups;
};

class ObjParser {
 public:
  static ParsedData Parse(const std::string& file_path, bool& success);
};
}  // namespace vkoo
