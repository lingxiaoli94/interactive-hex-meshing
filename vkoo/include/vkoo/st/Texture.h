#pragma once

#include "Image.h"
#include "vkoo/core/Sampler.h"

namespace vkoo {
namespace st {
class Texture {
 public:
  Image* GetImage() const { return image_; }
  void SetImage(Image* image) { image_ = image; }
  core::Sampler* GetSampler() const { return sampler_; }
  void SetSampler(core::Sampler* sampler) { sampler_ = sampler; }

 private:
  Image* image_{nullptr};
  core::Sampler* sampler_{nullptr};
};
}  // namespace st
}  // namespace vkoo
