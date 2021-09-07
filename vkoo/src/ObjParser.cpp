#include "vkoo/ObjParser.h"

#include "vkoo/utils.h"

namespace vkoo {
ParsedData ObjParser::Parse(const std::string& file_path, bool& success) {
  success = false;
  std::fstream fs(file_path);
  if (!fs) {
    std::cerr << "ERROR: Unable to open OBJ file " + file_path + "!"
              << std::endl;
    return {};
  }

  ParsedData data;

  MeshGroup current_group;
  std::string line;
  while (std::getline(fs, line)) {
    std::stringstream ss(line);
    std::string command;
    ss >> command;
    if (command == "#" || command == "") {
      continue;
    } else if (command == "v") {
      glm::vec3 p;
      ss >> p.x >> p.y >> p.z;
      if (data.positions == nullptr)
        data.positions = std::make_unique<std::vector<glm::vec3>>();
      data.positions->emplace_back(std::move(p));
    } else if (command == "vn") {
      glm::vec3 n;
      ss >> n.x >> n.y >> n.z;
      if (data.normals == nullptr)
        data.normals = std::make_unique<std::vector<glm::vec3>>();
      data.normals->emplace_back(std::move(n));
    } else if (command == "vt") {
      glm::vec2 uv;
      ss >> uv.s >> uv.t;
      if (data.tex_coords == nullptr)
        data.tex_coords = std::make_unique<std::vector<glm::vec2>>();
      data.tex_coords->emplace_back(std::move(uv));
    } else if (command == "f") {
      if (data.indices == nullptr)
        data.indices = std::make_unique<std::vector<uint32_t>>();
      for (int t = 0; t < 3; t++) {
        std::string str;
        ss >> str;
        unsigned int idx;
        if (str.find('/') == std::string::npos) {
          idx = std::stoul(str);
        } else {
          idx = std::stoul(Split(str, '/')[0]);
        }
        // Minus 1 because OBJ indices start with 1.
        data.indices->push_back(idx - 1);
      }
    } else if (command == "g") {
      if (current_group.name != "") {
        current_group.num_indices =
            data.indices->size() - current_group.start_face_index;
        data.groups.push_back(std::move(current_group));
      }
      ss >> current_group.name;
      if (data.indices == nullptr)
        current_group.start_face_index = 0;
      else
        current_group.start_face_index = data.indices->size();
    } else if (command == "usemtl") {
      ss >> current_group.material_name;
    } else if (command == "mtllib") {
      std::string mtl_file;
      ss >> mtl_file;
      // TODO: parse material.
    } else if (command == "o" || command == "s") {
      std::cout << "Skipped command: " << command << std::endl;
    } else {
      std::cerr << "Unknown obj command: " << command << std::endl;
      success = false;
      continue;
    }
  }

  if (current_group.name != "") {
    current_group.num_indices =
        data.indices->size() - current_group.start_face_index;
    data.groups.push_back(std::move(current_group));
  }

  success = true;
  return data;
}
}  // namespace vkoo
