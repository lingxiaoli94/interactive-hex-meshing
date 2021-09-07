#include "vkoo/st/components/Light.h"

namespace vkoo {
namespace st {
const LightProperties& Light::GetProperties() const { return properties_; }

LightType Light::GetLightType() const { return type_; }

void Light::SetProperties(const LightProperties& properties) {
  properties_ = properties;
}

void Light::SetType(LightType type) { type_ = type; }

std::type_index Light::GetType() const { return typeid(Light); }

}  // namespace st
}  // namespace vkoo