#pragma once

#include "vkoo/st/Texture.h"
#include "vkoo/st/components/ComponentBase.h"

namespace vkoo {
namespace st {
class Material {
 public:
  std::unordered_map<std::string, Texture*> textures;
  std::unordered_map<std::string, glm::vec4> colors;
};
}  // namespace st
}  // namespace vkoo
