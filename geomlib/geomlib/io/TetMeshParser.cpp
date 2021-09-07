#include "TetMeshParser.h"

#include <filesystem>
#include <stdexcept>

#include "geomlib/logging.h"

namespace fs = std::filesystem;

namespace geomlib {
std::unique_ptr<TetMesh> TetMeshParser::Parse(const std::string& file_path) {
  auto ext = fs::path(file_path).extension();
  if (ext == ".mesh") {
    return ParseMeditToMesh(file_path);
  } else if (ext == ".vtk") {
    return ParseVtkToMesh(file_path);
  } else {
    throw std::runtime_error(
        fmt::format("Unknown tetrahedral mesh format: {}", ext.string()));
  }
}

std::unique_ptr<TetMesh> TetMeshParser::ParseMeditToMesh(
    const std::string& file_path) {
  std::fstream fs(file_path);
  if (!fs) {
    throw std::runtime_error(
        fmt::format("ERROR: unable to open MEDIT MESH file {}", file_path));
  }

  Eigen::MatrixXf vertices;
  Eigen::MatrixXi tets;

  std::string line;
  while (std::getline(fs, line))
    if (!line.empty()) {
      std::stringstream ss(line);
      std::string command;
      ss >> command;
      if (command == "MeshVersionFormatted" || command == "Dimension" ||
          command == "End") {
        continue;
      } else if (command == "Vertices") {
        int n;
        fs >> n;
        vertices = Eigen::MatrixXf(n, 3);
        for (int i = 0; i < n; i++) {
          Vector3f p;
          float tmp;
          fs >> p.x() >> p.y() >> p.z() >> tmp;
          vertices.row(i) = p;
        }
      } else if (command == "Triangles") {
        // Skip.
        int n;
        fs >> n;
        for (int i = 0; i < n; i++) {
          int tmp;
          fs >> tmp >> tmp >> tmp >> tmp;
        }
      } else if (command == "Tetrahedra") {
        int n;
        fs >> n;
        tets = Eigen::MatrixXi(n, 4);
        for (int i = 0; i < n; i++) {
          Vector4i t;
          int tmp;
          for (int k = 0; k < 4; k++) {
            fs >> t(k);
            t(k)--;  // .mesh's index starts at 1
          }
          fs >> tmp;
          tets.row(i) = t;
        }
      }
    }
  return std::make_unique<TetMesh>(vertices, tets);
}

std::unique_ptr<TetMesh> TetMeshParser::ParseVtkToMesh(
    const std::string& file_path) {
  std::fstream fs(file_path);
  if (!fs) {
    throw std::runtime_error(
        fmt::format("ERROR: unable to open VTK file {}", file_path));
  }

  Eigen::MatrixXf vertices;
  Eigen::MatrixXi tets;

  std::string line;

  // Skip file version and header lines.
  std::getline(fs, line);
  std::getline(fs, line);

  fs >> line;
  if (line != "ASCII") {
    throw std::runtime_error("Cannot parse binary VTK format!");
  }

  while (std::getline(fs, line))
    if (!line.empty()) {
      std::stringstream ss(line);
      std::string command;
      ss >> command;
      if (command == "DATASET") {
        ss >> command;
        if (command != "UNSTRUCTURED_GRID") {
          throw std::runtime_error(
              "Cannot parse non-unstructured-grid dataset!");
        }
      } else if (command == "POINTS") {
        int n;
        ss >> n;
        std::string type;
        ss >> type;
        if (type != "float" && type != "double") {
          throw std::runtime_error("Cannot parse non-floating-point type!");
        }
        vertices = Eigen::MatrixXf(n, 3);
        for (int i = 0; i < n; i++) {
          Vector3f p;
          fs >> p.x() >> p.y() >> p.z();
          vertices.row(i) = p;
        }
      } else if (command == "CELLS") {
        int n, m;
        ss >> n >> m;
        if (m != 5 * n) {
          throw std::runtime_error("CELLS must contain all tetrahedrons!");
        }

        tets = Eigen::MatrixXi(n, 4);
        for (int i = 0; i < n; i++) {
          Vector4i t;
          int tmp;
          fs >> tmp;
          for (int k = 0; k < 4; k++) {
            fs >> t(k);  // index starts at 0
          }
          tets.row(i) = t;
        }
      } else if (command == "CELL_TYPES") {
        int n;
        ss >> n;
        if (n != tets.rows()) {
          throw std::runtime_error(
              "CELL_TYPES must be provided for each CELL!");
        }
        for (int i = 0; i < n; i++) {
          int tmp;
          fs >> tmp;
          if (tmp != 10) {
            throw std::runtime_error("All CELL_TYPES must be 10 (tet)!");
          }
        }
      }
    }
  return std::make_unique<TetMesh>(vertices, tets);
}
}  // namespace geomlib
