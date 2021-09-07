#include "vkoo/st/components/Tracing.h"

namespace vkoo {
namespace st {
Tracing::Tracing(std::shared_ptr<HittableBase> hittable)
    : hittable_{hittable} {}

std::type_index Tracing::GetType() const { return typeid(Tracing); }

void Tracing::SetHittable(std::shared_ptr<HittableBase> hittable) {
  hittable_ = hittable;
}
}  // namespace st
}  // namespace vkoo
