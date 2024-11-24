#include <fstream>

#include "H5Cpp.h"
#include "Serializer.h"
#include "logging.h"

using namespace H5;

namespace hex {
namespace {
const int kHexMeditOrder[8] = {1, 5, 7, 3, 0, 4, 6, 2};
const std::vector<std::string> kFloatKeys = {"input_scale"};
const std::vector<std::string> kMatrixXfKeys = {"input_center"};
void CreateMatrixXfDataset(H5::Group& group, const std::string& name,
                           const Eigen::MatrixXf& matrix) {
  hsize_t dims[2];
  dims[0] = matrix.cols();  // Matrix is column-major
  dims[1] = matrix.rows();

  DataSpace dataspace{2, dims};

  DataSet dataset{group.createDataSet(name, PredType::IEEE_F32BE, dataspace)};
  dataset.write(matrix.data(), PredType::NATIVE_FLOAT);
}

Eigen::MatrixXf LoadMatrixXfFromDataset(const DataSet& dataset) {
  DataSpace space = dataset.getSpace();
  hsize_t dims[2];
  space.getSimpleExtentDims(dims);

  Eigen::MatrixXf matrix{dims[1], dims[0]};
  dataset.read(matrix.data(), PredType::NATIVE_FLOAT);
  return matrix;
}

void CreateMatrixXiDataset(H5::Group& group, const std::string& name,
                           const Eigen::MatrixXi& matrix) {
  hsize_t dims[2];
  dims[0] = matrix.cols();  // Matrix is column-major
  dims[1] = matrix.rows();

  DataSpace dataspace{2, dims};

  DataSet dataset{group.createDataSet(name, PredType::STD_I32BE, dataspace)};
  dataset.write(matrix.data(), PredType::NATIVE_INT32);
}

Eigen::MatrixXi LoadMatrixXiFromDataset(const DataSet& dataset) {
  DataSpace space = dataset.getSpace();
  hsize_t dims[2];
  space.getSimpleExtentDims(dims);

  Eigen::MatrixXi matrix{dims[1], dims[0]};
  dataset.read(matrix.data(), PredType::NATIVE_INT32);
  return matrix;
}

void CreateArrayIntDataset(H5::Group& group, const std::string& name,
                           const std::vector<int>& array) {
  hsize_t dims[1];
  dims[0] = array.size();

  DataSpace dataspace{1, dims};

  DataSet dataset{group.createDataSet(name, PredType::STD_I32BE, dataspace)};
  dataset.write(array.data(), PredType::NATIVE_INT32);
}

std::vector<int> LoadArrayIntFromDataset(const DataSet& dataset) {
  DataSpace space = dataset.getSpace();
  hsize_t dims[1];
  space.getSimpleExtentDims(dims);

  std::vector<int> array(dims[0]);
  dataset.read(array.data(), PredType::NATIVE_INT32);
  return array;
}

void CreateFixedLengthStringArrayDataset(H5::Group& group,
                                         const std::string& name,
                                         const std::vector<const char*>& arr,
                                         int buf_size) {
  std::vector<char> char_buf_2d(arr.size() * buf_size);
  for (int i = 0; i < static_cast<int>(arr.size()); i++) {
    std::memcpy(char_buf_2d.data() + i * buf_size, arr[i],
                buf_size * sizeof(char));
  }

  hsize_t dims[2];
  dims[0] = arr.size();
  dims[1] = buf_size;

  DataSpace dataspace{2, dims};
  DataSet dataset{group.createDataSet(name, PredType::STD_U8BE, dataspace)};
  dataset.write(char_buf_2d.data(), PredType::NATIVE_CHAR);
}

std::vector<std::string> LoadFixedLengthStringArrayDataset(
    const DataSet& dataset) {
  DataSpace space = dataset.getSpace();
  hsize_t dims[2];
  space.getSimpleExtentDims(dims);

  std::vector<char> array(dims[0] * dims[1]);
  dataset.read(array.data(), PredType::NATIVE_CHAR);

  std::vector<std::string> result;
  for (int i = 0; i < dims[0]; i++) {
    // Assuming the char arrays are null-terminated.
    result.push_back(array.data() + i * dims[1]);
  }

  return result;
}

template <class IntType>
void CreateNestedArrayIntDataset(
    H5::Group& group, const std::string& name,
    const std::vector<std::vector<IntType>>& arrays, int special_value = -1) {
  std::vector<int> array_concat;
  for (auto& arr : arrays) {
    for (auto i : arr) {
      array_concat.push_back(static_cast<int>(i));
    }
    array_concat.push_back(special_value);
  }
  CreateArrayIntDataset(group, name, array_concat);
}

template <class IntType>
std::vector<std::vector<IntType>> LoadNestedArrayIntFromDataset(
    const DataSet& dataset, int special_value = -1) {
  auto array_concat = LoadArrayIntFromDataset(dataset);

  std::vector<std::vector<IntType>> batches;
  std::vector<IntType> batch;
  for (auto i : array_concat) {
    if (i == special_value) {
      batches.push_back(batch);
      batch.clear();
    } else {
      batch.push_back(static_cast<IntType>(i));
    }
  }
  return batches;
}

void CreateTetrahedralMeshGroup(H5::Group& group, const TetrahedralMesh& mesh) {
  CreateMatrixXfDataset(group, "vertices", mesh.GetVertices());
  CreateMatrixXiDataset(group, "tets", mesh.GetTets());
  if (mesh.HasAnchors()) {
    CreateMatrixXfDataset(group, "anchors", mesh.GetAnchors());
  }
  if (mesh.HasDistanceField()) {
    CreateMatrixXfDataset(group, "sdf", mesh.GetDistanceField());
  }
}

std::unique_ptr<TetrahedralMesh> LoadTetrahedralMeshFromGroup(
    const H5::Group& group) {
  auto vertices = LoadMatrixXfFromDataset(group.openDataSet("vertices"));
  auto tets = LoadMatrixXiFromDataset(group.openDataSet("tets"));
  auto mesh = std::make_unique<TetrahedralMesh>(vertices, tets);
  if (group.nameExists("anchors")) {
    mesh->SetAnchors(LoadMatrixXfFromDataset(group.openDataSet("anchors")));
  }
  if (group.nameExists("sdf")) {
    mesh->SetDistanceField(LoadMatrixXfFromDataset(group.openDataSet("sdf")));
  }

  return mesh;
}

void CreateHexahedralMeshGroup(H5::Group& group, const HexahedralMesh& mesh) {
  CreateMatrixXfDataset(group, "vertices",
                        ArrayVector3fToMatrixXf(mesh.GetVertices()));
  CreateNestedArrayIntDataset(group, "hexes", mesh.GetHexes());
}

std::unique_ptr<HexahedralMesh> LoadHexahedralMeshFromGroup(
    const H5::Group& group) {
  auto vertices = LoadMatrixXfFromDataset(group.openDataSet("vertices"));
  auto hexes =
      LoadNestedArrayIntFromDataset<size_t>(group.openDataSet("hexes"));
  auto mesh = std::make_unique<HexahedralMesh>(
      MatrixXfToArrayVector3f(vertices), hexes);
  return mesh;
}

void CreatePolycubeGroup(H5::Group& group, const Polycube& polycube) {
  CreateMatrixXfDataset(group, "params", polycube.ToMatrix());
}

std::unique_ptr<Polycube> LoadPolycubeFromGroup(const H5::Group& group) {
  Eigen::MatrixXf params = LoadMatrixXfFromDataset(group.openDataSet("params"));
  return std::make_unique<Polycube>(params);
}

void CreatePolycubeInfoGroup(H5::Group& group, const PolycubeInfo& info) {
  CreateArrayIntDataset(group, "ordering", info.ordering);
  CreateArrayIntDataset(group, "locked", info.locked);

  std::vector<const char*> names;
  for (int i = 0; i < static_cast<int>(info.names.size()); i++) {
    names.push_back(&info.names[i].buf[0]);
  }
  CreateFixedLengthStringArrayDataset(group, "names", names, kMaxNameLength);
}

std::unique_ptr<PolycubeInfo> LoadPolycubeInfoFromGroup(
    const H5::Group& group) {
  auto polycube_info = std::make_unique<PolycubeInfo>();
  polycube_info->ordering =
      LoadArrayIntFromDataset(group.openDataSet("ordering"));
  polycube_info->locked = LoadArrayIntFromDataset(group.openDataSet("locked"));
  std::vector<std::string> names =
      LoadFixedLengthStringArrayDataset(group.openDataSet("names"));
  for (auto& name : names) {
    FixedLengthStr str;
    std::memcpy(str.buf, name.data(), name.size() * sizeof(char));
    assert(name.size() <= kMaxNameLength);
    str.buf[name.size()] = '\0';
    polycube_info->names.push_back(str);
  }

  return polycube_info;
}

void CreateHexComplexGroup(H5::Group& group, const HexComplex& hex_complex) {
  CreateMatrixXfDataset(group, "vertices",
                        ArrayVector3fToMatrixXf(hex_complex.GetVertices()));
  CreateMatrixXiDataset(group, "quads",
                        ArrayVector4iToMatrixXi(hex_complex.GetQuads()));

  CreateNestedArrayIntDataset(group, "patches", hex_complex.GetPatches());
  CreateNestedArrayIntDataset(group, "hexes", hex_complex.GetHexes());
}

std::unique_ptr<HexComplex> LoadHexComplexFromGroup(const H5::Group& group) {
  auto vertices = LoadMatrixXfFromDataset(group.openDataSet("vertices"));
  auto quads = LoadMatrixXiFromDataset(group.openDataSet("quads"));
  auto patches =
      LoadNestedArrayIntFromDataset<size_t>(group.openDataSet("patches"));
  auto hexes =
      LoadNestedArrayIntFromDataset<size_t>(group.openDataSet("hexes"));

  return std::make_unique<HexComplex>(MatrixXfToArrayVector3f(vertices),
                                      MatrixXiToArrayVector4i(quads), patches,
                                      hexes);
}
}  // namespace

void Serializer::SaveState(const GlobalState& state,
                           const std::string& file_path) {
  H5File file(file_path, H5F_ACC_TRUNC);
  if (state.HasTargetVolumeMesh()) {
    Group group(file.createGroup("/target_volume_mesh"));
    CreateTetrahedralMeshGroup(group, state.GetTargetVolumeMesh());
  }
  if (state.HasDeformedVolumeMesh()) {
    Group group(file.createGroup("/deformed_volume_mesh"));
    CreateTetrahedralMeshGroup(group, state.GetDeformedVolumeMesh());
  }
  if (state.HasPolycube()) {
    Group group(file.createGroup("/polycube"));
    CreatePolycubeGroup(group, state.GetPolycube());
  }
  if (state.HasPolycubeInfo()) {
    Group group(file.createGroup("/polycube_info"));
    CreatePolycubeInfoGroup(group, state.GetPolycubeInfo());
  }
  if (state.HasPolycubeComplex()) {
    Group group(file.createGroup("/polycube_complex"));
    CreateHexComplexGroup(group, state.GetPolycubeComplex());
  }
  if (state.HasTargetComplex()) {
    Group group(file.createGroup("/target_complex"));
    CreateHexComplexGroup(group, state.GetTargetComplex());
  }
  if (state.HasResultMesh()) {
    Group group(file.createGroup("/result_mesh"));
    CreateHexahedralMeshGroup(group, state.GetResultMesh());
  }

  // Write scalars and small matrices as attributes to the root file.
  auto& float_dict = state.GetFloatDict();
  for (auto& kv : float_dict) {
    auto att = file.createAttribute(kv.first, PredType::IEEE_F32BE,
                                    DataSpace(H5S_SCALAR));
    att.write(PredType::NATIVE_FLOAT, &kv.second);
  }

  auto& matrix_xf_dict = state.GetMatrixXfDict();
  for (auto& kv : matrix_xf_dict) {
    auto& mat = kv.second;
    hsize_t dims[2];
    dims[0] = mat.cols();
    dims[1] = mat.rows();

    DataSpace dataspace{2, dims};
    auto att = file.createAttribute(kv.first, PredType::IEEE_F32BE, dataspace);
    att.write(PredType::NATIVE_FLOAT, mat.data());
  }
}

std::unique_ptr<GlobalState> Serializer::LoadState(
    const std::string& file_path) {
  H5File file(file_path, H5F_ACC_RDONLY);

  std::unique_ptr<GlobalState> state = std::make_unique<GlobalState>();

  if (file.nameExists("/target_volume_mesh")) {
    state->SetTargetVolumeMesh(
        LoadTetrahedralMeshFromGroup(file.openGroup("/target_volume_mesh")));
  }
  if (file.nameExists("/deformed_volume_mesh")) {
    state->SetDeformedVolumeMesh(
        LoadTetrahedralMeshFromGroup(file.openGroup("/deformed_volume_mesh")));
  }
  if (file.nameExists("/polycube")) {
    state->SetPolycube(LoadPolycubeFromGroup(file.openGroup("/polycube")));
  }
  if (file.nameExists("/polycube_info")) {
    state->SetPolycubeInfo(
        LoadPolycubeInfoFromGroup(file.openGroup("/polycube_info")));
  }
  if (file.nameExists("/polycube_complex")) {
    state->SetPolycubeComplex(
        LoadHexComplexFromGroup(file.openGroup("/polycube_complex")));
  }
  if (file.nameExists("/target_complex")) {
    state->SetTargetComplex(
        LoadHexComplexFromGroup(file.openGroup("/target_complex")));
  }
  if (file.nameExists("/result_mesh")) {
    state->SetResultMesh(
        LoadHexahedralMeshFromGroup(file.openGroup("/result_mesh")));
  }

  // Load attributes.
  for (auto& key : kFloatKeys) {
    if (file.attrExists(key)) {
      auto att = file.openAttribute(key);
      float f;
      att.read(PredType::NATIVE_FLOAT, &f);
      state->SetFloat(key, f);
      LOGI("Load float {}: {}", key, f);
    }
  }

  for (auto& key : kMatrixXfKeys) {
    if (file.attrExists(key)) {
      auto att = file.openAttribute(key);

      DataSpace space = att.getSpace();
      hsize_t dims[2];
      space.getSimpleExtentDims(dims);

      Eigen::MatrixXf matrix{dims[1], dims[0]};
      att.read(PredType::NATIVE_FLOAT, matrix.data());
      state->SetMatrixXf(key, matrix);

      LOGI("Load matrix {}: ({}, {}, {})", key, matrix(0), matrix(1),
           matrix(2));
    }
  }
  return state;
}

Eigen::MatrixXf Serializer::LoadSingleMatrixXf(const std::string& file_path,
                                               const std::string& name) {
  H5File file(file_path, H5F_ACC_RDONLY);
  Eigen::MatrixXf result;
  if (file.nameExists(name)) {
    result = LoadMatrixXfFromDataset(file.openDataSet(name));
  } else {
    LOGW("Dataset {} does not exist in {}!", name, file_path);
  }
  return result;
}

void Serializer::SaveHexMesh(const HexahedralMesh& mesh,
                             const std::string& file_path,
                             std::function<Vector3f(Vector3f)> vertex_map) {
  std::ofstream ofs(file_path);
  ofs << "MeshVersionFormatted 1\nDimension 3\n\n";
  auto& vertices = mesh.GetVertices();
  ofs << fmt::format("Vertices\n{}\n", vertices.size());
  for (int i = 0; i < static_cast<int>(vertices.size()); i++) {
    auto v = vertex_map(vertices[i]);
    ofs << fmt::format("{:6f} {:6f} {:6f} 0\n", v(0), v(1), v(2));
  }
  auto& hexes = mesh.GetHexes();
  ofs << fmt::format("Hexahedra\n{}\n", hexes.size());
  for (int i = 0; i < static_cast<int>(hexes.size()); i++) {
    std::stringstream ss;
    for (int k = 0; k < 8; k++) {
      ss << hexes[i][kHexMeditOrder[k]] + 1;  // plus 1 since index starts at 1
      if (k < 7) {
        ss << ' ';
      }
    }
    ofs << ss.str() << " 0\n";
  }
  ofs << "End\n";
}

std::unique_ptr<HexahedralMesh> Serializer::LoadHexMesh(
    const std::string& file_path) {
  int inv_order[8];
  for (int k = 0; k < 8; k++) {
    inv_order[kHexMeditOrder[k]] = k;
  }

  std::fstream fs(file_path);
  if (!fs) {
    throw std::runtime_error(
        fmt::format("ERROR: unable to open MEDIT MESH file {}", file_path));
  }

  std::vector<Vector3f> vertices;
  std::vector<HexahedralMesh::Hex> hexes;

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
        ss >> n;
        vertices.resize(n);
        for (int i = 0; i < n; i++) {
          Vector3f p;
          float tmp;
          fs >> p.x() >> p.y() >> p.z() >> tmp;
          vertices[i] = p;
        }
      } else if (command == "Hexahedra") {
        int n;
        ss >> n;
        hexes.resize(n);
        for (int i = 0; i < n; i++) {
          HexahedralMesh::Hex hex(8);
          int tmp;
          for (int k = 0; k < 8; k++) {
            int a;
            fs >> a;
            a--;
            hex[kHexMeditOrder[k]] = a;
          }
          fs >> tmp;
          hexes[i] = hex;
        }
      }
    }
  LOGI("Loaded {} vertices and {} hexes", vertices.size(), hexes.size());
  return std::make_unique<HexahedralMesh>(vertices, hexes);
}

}  // namespace hex
