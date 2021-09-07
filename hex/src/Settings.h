#pragma once

#include "common.h"

#include <vkoo/st/components/Light.h>

namespace hex {
struct Settings {
  using Clock = std::chrono::steady_clock;

  Settings(const std::string& file_path, double save_freq);
  void TrySave();
  void Load();
  void Load(const std::string& file_path);
  void Save();
  void Save(const std::string& file_path);

  // Global settings.
  int window_width{1280};
  int window_height{720};
  bool expert_mode{false};
  bool msaa_on{true};
  bool ssao_on{true};
  bool show_advanced{false};
  bool show_demo{false};
  bool show_gui{true};
  int fps_limit{60};
  bool export_unit_scale{true};

  // Scene related.
  struct CameraSpec {
    glm::vec3 position{0.0f, 0.0f, 5.0f};
    glm::vec3 anchor{0.0f, 0.0f, 0.0f};
    glm::vec3 up_direction{0.0f, 1.0f, 0.0f};
  } camera;

  struct LightSpec {
    vkoo::st::LightType type;
    vkoo::st::LightProperties properties;
    glm::vec3 position;
  };

  std::vector<LightSpec> lights;

 private:
  void CreateDefaultLights();

  std::string file_path_;
  double save_freq_;  // in seconds
  std::chrono::time_point<Clock> last_save_time_;
};
}  // namespace hex
