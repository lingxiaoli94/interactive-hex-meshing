#include "Settings.h"

#include <vkoo/st/components/Light.h>
#include <yaml-cpp/node/type.h>
#include <yaml-cpp/yaml.h>

#include <filesystem>

#include "logging.h"

namespace fs = std::filesystem;
namespace YAML {
template <>
struct convert<glm::vec3> {
  static Node encode(const glm::vec3& rhs) {
    Node node;
    node.push_back(rhs.x);
    node.push_back(rhs.y);
    node.push_back(rhs.z);
    return node;
  }

  static bool decode(const Node& node, glm::vec3& rhs) {
    if (!node.IsSequence() || node.size() != 3) {
      return false;
    }

    rhs.x = node[0].as<float>();
    rhs.y = node[1].as<float>();
    rhs.z = node[2].as<float>();
    return true;
  }
};

template <>
struct convert<glm::quat> {
  static Node encode(const glm::quat& rhs) {
    Node node;
    node.push_back(rhs.x);
    node.push_back(rhs.y);
    node.push_back(rhs.z);
    node.push_back(rhs.w);
    return node;
  }

  static bool decode(const Node& node, glm::quat& rhs) {
    if (!node.IsSequence() || node.size() != 4) {
      return false;
    }

    rhs.x = node[0].as<float>();
    rhs.y = node[1].as<float>();
    rhs.z = node[2].as<float>();
    rhs.w = node[3].as<float>();
    return true;
  }
};

template <>
struct convert<hex::Settings::CameraSpec> {
  static Node encode(const hex::Settings::CameraSpec& rhs) {
    YAML::Node node;
    node["position"] = rhs.position;
    node["anchor"] = rhs.anchor;
    node["up_direction"] = rhs.up_direction;
    return node;
  }

  static bool decode(const Node& node, hex::Settings::CameraSpec& rhs) {
    rhs.position = node["position"].as<glm::vec3>();
    rhs.anchor = node["anchor"].as<glm::vec3>();
    rhs.up_direction = node["up_direction"].as<glm::vec3>();
    return true;
  }
};

template <>
struct convert<vkoo::st::LightProperties> {
  static Node encode(const vkoo::st::LightProperties& rhs) {
    YAML::Node node;
    node["color"] = rhs.color;
    node["direction"] = rhs.direction;
    node["intensity"] = rhs.intensity;
    node["range"] = rhs.range;
    return node;
  }

  static bool decode(const Node& node, vkoo::st::LightProperties& rhs) {
    rhs.color = node["color"].as<glm::vec3>();
    rhs.direction = node["direction"].as<glm::vec3>();
    rhs.intensity = node["intensity"].as<float>();
    rhs.range = node["range"].as<float>();
    return true;
  }
};

template <>
struct convert<hex::Settings::LightSpec> {
  static Node encode(const hex::Settings::LightSpec& rhs) {
    YAML::Node node;
    if (rhs.type == vkoo::st::LightType::Directional) {
      node["type"] = "directional";
    } else {
      node["type"] = "point";
    }

    node["properties"] = rhs.properties;
    node["position"] = rhs.position;

    return node;
  };

  static bool decode(const Node& node, hex::Settings::LightSpec& rhs) {
    rhs.type = (node["type"].as<std::string>() == "directional")
                   ? vkoo::st::LightType::Directional
                   : vkoo::st::LightType::Point;

    rhs.properties = node["properties"].as<vkoo::st::LightProperties>();
    rhs.position = node["position"].as<glm::vec3>();
    return true;
  }
};
}  // namespace YAML

namespace hex {
Settings::Settings(const std::string& file_path, double save_freq)
    : file_path_{file_path}, save_freq_{save_freq} {
  if (!file_path.empty() && fs::exists(file_path)) {
    Load();
  } else {
    CreateDefaultLights();
  }
  last_save_time_ = Clock::now();
}

void Settings::Load() { Load(file_path_); }

void Settings::Load(const std::string& file_path) {
  YAML::Node config = YAML::LoadFile(file_path);
  if (config["expert_mode"]) {
    expert_mode = config["expert_mode"].as<bool>();
  }
  if (config["window_width"]) {
    window_width = config["window_width"].as<int>();
  }
  if (config["window_height"]) {
    window_height = config["window_height"].as<int>();
  }
  if (config["msaa_on"]) {
    msaa_on = config["msaa_on"].as<bool>();
  }
  if (config["ssao"]) {
    ssao_on = config["ssao_on"].as<bool>();
  }
  if (config["camera"]) {
    camera = config["camera"].as<CameraSpec>();
  }
  if (config["lights"]) {
    size_t num_lights = config["lights"].size();
    lights.clear();
    for (size_t i = 0; i < num_lights; i++) {
      lights.push_back(config["lights"][i].as<LightSpec>());
    }
  } else {
    CreateDefaultLights();
  }
}

void Settings::CreateDefaultLights() {
  lights.clear();
  {
    vkoo::st::LightProperties light_properties;
    light_properties.direction = glm::normalize(glm::vec3(0.0f, -1.0f, -1.0f));
    light_properties.color = glm::vec3(1.0f, 1.0f, 1.0f);
    light_properties.intensity = 0.3f;
    lights.push_back({vkoo::st::LightType::Directional,
                      light_properties,
                      {0.0f, 0.0f, 0.0f}});
  }
  {
    vkoo::st::LightProperties light_properties;
    light_properties.direction = glm::normalize(glm::vec3(-1.0f, 0.0f, -1.0f));
    light_properties.color = glm::vec3(1.0f, 1.0f, 1.0f);
    light_properties.intensity = 0.1f;
    lights.push_back({vkoo::st::LightType::Directional,
                      light_properties,
                      {0.0f, 0.0f, 0.0f}});
  }
  {
    vkoo::st::LightProperties light_properties;
    light_properties.direction = glm::normalize(glm::vec3(-1.0f, -1.0f, 0.0f));
    light_properties.color = glm::vec3(1.0f, 1.0f, 1.0f);
    light_properties.intensity = 0.2f;
    lights.push_back({vkoo::st::LightType::Directional,
                      light_properties,
                      {0.0f, 0.0f, 0.0f}});
  }
}

void Settings::Save(const std::string& file_path) {
  YAML::Node node;

  node["window_width"] = window_width;
  node["window_height"] = window_height;

  node["expert_mode"] = expert_mode;
  node["msaa_on"] = msaa_on;
  node["ssao_on"] = ssao_on;
  node["camera"] = camera;

  node["lights"] = YAML::Node(YAML::NodeType::Sequence);
  for (size_t i = 0; i < lights.size(); i++) {
    node["lights"].push_back(lights[i]);
  }

  std::ofstream ofs(file_path);
  ofs << node;
}

void Settings::Save() { Save(file_path_); }

void Settings::TrySave() {
  auto current_time = Clock::now();
  auto duration_sec =
      std::chrono::duration<double>(current_time - last_save_time_);
  if (duration_sec.count() >= save_freq_) {
    Save();
    last_save_time_ = current_time;
  }
}

}  // namespace hex
