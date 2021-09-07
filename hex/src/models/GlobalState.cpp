#include "GlobalState.h"
#include "serialization/Serializer.h"

namespace hex {
void GlobalState::SetPolycube(std::unique_ptr<Polycube> polycube) {
  polycube_ = std::move(polycube);
}

bool GlobalState::HasPolycube() const { return polycube_ != nullptr; }

Polycube& GlobalState::GetPolycube() {
  assert(HasPolycube());
  return *polycube_;
}

void GlobalState::SetPolycubeInfo(std::unique_ptr<PolycubeInfo> info) {
  polycube_info_ = std::move(info);
}

bool GlobalState::HasPolycubeInfo() const { return polycube_info_ != nullptr; }

PolycubeInfo& GlobalState::GetPolycubeInfo() {
  assert(HasPolycubeInfo());
  return *polycube_info_;
}

void GlobalState::SetPolycubeComplex(
    std::unique_ptr<HexComplex> polycube_complex) {
  polycube_complex_ = std::move(polycube_complex);
}

HexComplex& GlobalState::GetPolycubeComplex() {
  assert(HasPolycubeComplex());
  return *polycube_complex_;
}

bool GlobalState::HasPolycubeComplex() const {
  return polycube_complex_ != nullptr;
}

void GlobalState::SetTargetComplex(std::unique_ptr<HexComplex> target_complex) {
  target_complex_ = std::move(target_complex);
}

bool GlobalState::HasTargetComplex() const {
  return target_complex_ != nullptr;
}

HexComplex& GlobalState::GetTargetComplex() {
  assert(HasTargetComplex());
  return *target_complex_;
}

void GlobalState::SaveToFile(const std::string& file_path) {
  Serializer::SaveState(*this, file_path);
}

const Polycube& GlobalState::GetPolycube() const { return *polycube_; }
const PolycubeInfo& GlobalState::GetPolycubeInfo() const {
  return *polycube_info_;
}
const HexComplex& GlobalState::GetPolycubeComplex() const {
  return *polycube_complex_;
}
const HexComplex& GlobalState::GetTargetComplex() const {
  return *target_complex_;
}

std::unique_ptr<GlobalState> GlobalState::LoadFromFile(
    const std::string& file_path) {
  return Serializer::LoadState(file_path);
}

void GlobalState::SetTargetVolumeMesh(
    std::unique_ptr<TetrahedralMesh> volume_mesh) {
  target_volume_mesh_ = std::move(volume_mesh);
}

void GlobalState::SetDeformedVolumeMesh(
    std::unique_ptr<TetrahedralMesh> volume_mesh) {
  deformed_volume_mesh_ = std::move(volume_mesh);
}

bool GlobalState::HasTargetVolumeMesh() const {
  return target_volume_mesh_ != nullptr;
}

bool GlobalState::HasDeformedVolumeMesh() const {
  return deformed_volume_mesh_ != nullptr;
}

TetrahedralMesh& GlobalState::GetTargetVolumeMesh() {
  assert(HasTargetVolumeMesh());
  return *target_volume_mesh_;
}

TetrahedralMesh& GlobalState::GetDeformedVolumeMesh() {
  assert(HasDeformedVolumeMesh());
  return *deformed_volume_mesh_;
}

const TetrahedralMesh& GlobalState::GetTargetVolumeMesh() const {
  return *target_volume_mesh_;
}

const TetrahedralMesh& GlobalState::GetDeformedVolumeMesh() const {
  return *deformed_volume_mesh_;
}

void GlobalState::SetResultMesh(std::unique_ptr<HexahedralMesh> result_mesh) {
  result_mesh_ = std::move(result_mesh);
}

bool GlobalState::HasResultMesh() const { return result_mesh_ != nullptr; }

HexahedralMesh& GlobalState::GetResultMesh() { return *result_mesh_; }

const HexahedralMesh& GlobalState::GetResultMesh() const {
  return *result_mesh_;
}

bool GlobalState::CheckPolycubeConsistency() const {
  assert(HasPolycube() && HasPolycubeInfo());
  return polycube_->GetCuboidCount() == polycube_info_->names.size() &&
         polycube_info_->names.size() == polycube_info_->ordering.size();
}

const std::unordered_map<std::string, float> GlobalState::GetFloatDict() const {
  return float_dict_;
}

void GlobalState::SetFloat(const std::string& key, float val) {
  float_dict_[key] = val;
}

float GlobalState::GetFloat(const std::string& key) {
  return float_dict_.at(key);
}

bool GlobalState::HasFloat(const std::string& key) const {
  return float_dict_.count(key) > 0;
}

const std::unordered_map<std::string, Eigen::MatrixXf>
GlobalState::GetMatrixXfDict() const {
  return matrix_xf_dict_;
}

void GlobalState::SetMatrixXf(const std::string& key,
                              const Eigen::MatrixXf& val) {
  matrix_xf_dict_[key] = val;
}

const Eigen::MatrixXf& GlobalState::GetMatrixXf(const std::string& key) const {
  return matrix_xf_dict_.at(key);
}

bool GlobalState::HasMatrixXf(const std::string& key) const {
  return matrix_xf_dict_.count(key) > 0;
}
}  // namespace hex
