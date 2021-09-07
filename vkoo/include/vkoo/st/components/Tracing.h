#pragma once

#include "ComponentBase.h"
#include "vkoo/st/hittables/HittableBase.h"

namespace vkoo {
namespace st {
class Tracing : public ComponentBase {
 public:
  Tracing(std::shared_ptr<HittableBase> hittable);
  const HittableBase& GetHittable() const { return *hittable_; }
  void SetHittable(std::shared_ptr<HittableBase> hittable);
  std::type_index GetType() const final;

 private:
  std::shared_ptr<HittableBase> hittable_;
};
}  // namespace st
}  // namespace vkoo
