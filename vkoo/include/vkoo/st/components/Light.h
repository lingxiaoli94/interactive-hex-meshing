#pragma once

#include "ComponentBase.h"

namespace vkoo {
namespace st {

enum LightType { Directional = 0, Point = 1, Max };

struct LightProperties {
  glm::vec3 direction{0.0f, 0.0f, -1.0f};
  glm::vec3 color{1.0f, 1.0f, 1.0f};
  float intensity{1.0f};
  float range{0.0f};
};

class Light : public ComponentBase {
 public:
  std::type_index GetType() const final;
  LightType GetLightType() const;
  void SetType(LightType type);
  const LightProperties& GetProperties() const;
  void SetProperties(const LightProperties& properties);

 private:
  LightType type_;
  LightProperties properties_;
};
}  // namespace st
}  // namespace vkoo
