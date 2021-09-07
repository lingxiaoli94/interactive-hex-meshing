#include "ObjParser.h"
#include "geomlib/logging.h"

namespace geomlib {
ObjParser::ParsedData ObjParser::Parse(const std::string& file_path) {
  std::fstream fs(file_path);
  if (!fs) {
    throw std::runtime_error(
        fmt::format("ERROR: unable to open OBJ file {}!", file_path));
  }

  ParsedData data;

  std::vector<float> positions_flatten;
  std::vector<float> normals_flatten;
  std::vector<int> indices_flatten;
  std::vector<float> tex_coords_flatten;

  std::string line;
  while (std::getline(fs, line)) {
    std::stringstream ss(line);
    std::string command;
    ss >> command;
    if (command[0] == '#' || command == "") {
      continue;
    } else if (command == "v") {
      float x, y, z;
      ss >> x >> y >> z;
      positions_flatten.insert(positions_flatten.end(), {x, y, z});
    } else if (command == "vn") {
      float x, y, z;
      ss >> x >> y >> z;
      normals_flatten.insert(normals_flatten.end(), {x, y, z});
    } else if (command == "vt") {
      float u, v;
      ss >> u >> v;
      tex_coords_flatten.insert(tex_coords_flatten.end(), {u, v});
    } else if (command == "f") {
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
        indices_flatten.push_back(static_cast<int>(idx - 1));
      }
    } else if (command == "g") {
      continue;  // skip groups
    } else if (command == "usemtl" || command == "mtllib") {
      continue;  // skip materials
    } else if (command == "o" || command == "s") {
      std::cout << "Skipped command: " << command << std::endl;
    } else {
      std::cerr << "Unknown obj command: " << command << std::endl;
      continue;
    }
  }

  // Put things into Eigen structures.
  data.positions = std::make_unique<Eigen::MatrixXf>(
      Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic,
                               Eigen::RowMajor>>(
          positions_flatten.data(), positions_flatten.size() / 3, 3));
  data.normals = std::make_unique<Eigen::MatrixXf>(
      Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic,
                               Eigen::RowMajor>>(
          normals_flatten.data(), normals_flatten.size() / 3, 3));
  data.faces = std::make_unique<Eigen::MatrixXi>(
      Eigen::Map<
          Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
          indices_flatten.data(), indices_flatten.size() / 3, 3));
  data.tex_coords = std::make_unique<Eigen::MatrixXf>(
      Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic,
                               Eigen::RowMajor>>(
          tex_coords_flatten.data(), tex_coords_flatten.size() / 2, 2));

  return data;
}

std::unique_ptr<TriMesh> ObjParser::ParseToMesh(const std::string& file_path) {
  auto parsed_data = ObjParser::Parse(file_path);
  return std::make_unique<TriMesh>(*parsed_data.positions, *parsed_data.faces);
}
}  // namespace geomlib
