#pragma once

#include "common.h"

namespace hex {
class GlobalController;
class ShortcutController {
 public:
  ShortcutController(GlobalController& global_controller);
  void DrawMenu();

 private:
};
}  // namespace hex
