#include "cli/MetricsDumper.h"

#include <yaml-cpp/yaml.h>

#include <Eigen/Core>
#include <cmath>
#include <fstream>
#include <stdexcept>

#include "logging.h"
#include "models/HexMeshQuality.h"
#include "models/HexahedralMesh.h"

namespace hex {
namespace {

struct Stats {
  float min;
  float max;
  float mean;
  float std;
};

Stats Summarize(const Eigen::VectorXf& v) {
  Stats s{0, 0, 0, 0};
  if (v.size() == 0) return s;
  s.min = v.minCoeff();
  s.max = v.maxCoeff();
  s.mean = v.mean();
  s.std = std::sqrt((v.array() - s.mean).square().mean());
  return s;
}

void EmitStats(YAML::Emitter& em, const std::string& key, const Stats& s) {
  em << YAML::Key << key << YAML::Value << YAML::BeginMap
     << YAML::Key << "min" << YAML::Value << s.min
     << YAML::Key << "max" << YAML::Value << s.max
     << YAML::Key << "mean" << YAML::Value << s.mean
     << YAML::Key << "std" << YAML::Value << s.std
     << YAML::EndMap;
}

}  // namespace

void DumpQualityMetrics(const HexahedralMesh& mesh,
                        const std::string& out_path) {
  const auto& quality = mesh.GetMeshQuality();
  const Eigen::VectorXf& sj = quality.GetScaledJacobians();
  const Eigen::VectorXf& jac = quality.GetJacobians();

  Stats sj_s = Summarize(sj);
  Stats jac_s = Summarize(jac);
  int inverted = static_cast<int>((sj.array() < 0.0f).count());

  YAML::Emitter em;
  em << YAML::BeginMap;
  em << YAML::Key << "total_hexes" << YAML::Value
     << static_cast<int>(sj.size());
  em << YAML::Key << "inverted_count" << YAML::Value << inverted;
  EmitStats(em, "scaled_jacobian", sj_s);
  EmitStats(em, "jacobian", jac_s);
  em << YAML::EndMap;

  std::ofstream f(out_path);
  if (!f) {
    throw std::runtime_error("cannot open metrics file for writing: " +
                             out_path);
  }
  f << em.c_str();
  f << "\n";
  LOGI(
      "[script] wrote quality metrics to {}: total_hexes={}, "
      "inverted_count={}, scaled_jac min/max={:.4f}/{:.4f}",
      out_path, static_cast<int>(sj.size()), inverted, sj_s.min, sj_s.max);
}

}  // namespace hex
