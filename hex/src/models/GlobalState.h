#pragma once

#include "common.h"

#include "models/HexComplex.h"
#include "models/Polycube.h"
#include "models/PolycubeInfo.h"
#include "models/TetrahedralMesh.h"
#include "models/TriangularMesh.h"

namespace hex {
class GlobalState {
 public:
  void SaveToFile(const std::string& file_path);
  static std::unique_ptr<GlobalState> LoadFromFile(
      const std::string& file_path);

  void SetTargetVolumeMesh(std::unique_ptr<TetrahedralMesh> volume_mesh);
  void SetDeformedVolumeMesh(std::unique_ptr<TetrahedralMesh> volume_mesh);
  void SetPolycube(std::unique_ptr<Polycube> polycube);
  void SetPolycubeInfo(std::unique_ptr<PolycubeInfo> polycube_info);
  void SetPolycubeComplex(std::unique_ptr<HexComplex> polycube_complex);
  void SetTargetComplex(std::unique_ptr<HexComplex> target_complex);
  void SetResultMesh(std::unique_ptr<HexahedralMesh> result_mesh);
  void SetFloat(const std::string& key, float val);
  void SetMatrixXf(const std::string& key, const Eigen::MatrixXf& matrix);

  bool HasTargetVolumeMesh() const;
  bool HasDeformedVolumeMesh() const;
  bool HasPolycube() const;
  bool HasPolycubeInfo() const;
  bool HasPolycubeComplex() const;
  bool HasTargetComplex() const;
  bool HasResultMesh() const;
  bool HasFloat(const std::string& key) const;
  bool HasMatrixXf(const std::string& key) const;

  TetrahedralMesh& GetTargetVolumeMesh();
  TetrahedralMesh& GetDeformedVolumeMesh();
  Polycube& GetPolycube();
  PolycubeInfo& GetPolycubeInfo();
  HexComplex& GetPolycubeComplex();
  HexComplex& GetTargetComplex();
  HexahedralMesh& GetResultMesh();
  float GetFloat(const std::string& key);
  const Eigen::MatrixXf& GetMatrixXf(const std::string& key) const;
  const std::unordered_map<std::string, float> GetFloatDict() const;
  const std::unordered_map<std::string, Eigen::MatrixXf> GetMatrixXfDict()
      const;

  const TetrahedralMesh& GetTargetVolumeMesh() const;
  const TetrahedralMesh& GetDeformedVolumeMesh() const;
  const Polycube& GetPolycube() const;
  const PolycubeInfo& GetPolycubeInfo() const;
  const HexComplex& GetPolycubeComplex() const;
  const HexComplex& GetTargetComplex() const;
  const HexahedralMesh& GetResultMesh() const;

  bool CheckPolycubeConsistency() const;

 private:
  std::unique_ptr<TetrahedralMesh> target_volume_mesh_;
  std::unique_ptr<TetrahedralMesh> deformed_volume_mesh_;
  std::unique_ptr<Polycube> polycube_;
  std::unique_ptr<PolycubeInfo> polycube_info_;
  std::unique_ptr<HexComplex> polycube_complex_;
  std::unique_ptr<HexComplex> target_complex_;
  std::unique_ptr<HexahedralMesh> result_mesh_;

  std::unordered_map<std::string, float> float_dict_;
  std::unordered_map<std::string, Eigen::MatrixXf> matrix_xf_dict_;
};
}  // namespace hex
