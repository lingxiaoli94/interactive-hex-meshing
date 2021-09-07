#include "vkoo/st/components/Script.h"

namespace vkoo {
namespace st {
Script::Script(Node& node) { SetNode(&node); }

std::type_index Script::GetType() const { return typeid(Script); }
}  // namespace st
}  // namespace vkoo
