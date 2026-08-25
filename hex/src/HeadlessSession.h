#pragma once

#include <vkoo/st/Scene.h>

#include <memory>
#include <string>

#include "Settings.h"
#include "controllers/GlobalController.h"

namespace hex {
// Owns everything a scripted run needs and nothing a renderer needs: the
// settings, a Vulkan-free scene (camera + lights, which the stages and the
// arc-ball script read), and the GlobalController.
//
// This is the headless application boundary. Without it, a
// HEX_ENABLE_VULKAN=OFF build would have to keep HexMeshingApp -- a class that
// inherits the Vulkan-heavy vkoo::Application and whose header is full of
// VkFormat, RenderPipeline and Sampler -- purely to hold a GlobalController.
// Surrounding
// those members with scattered preprocessor guards was rejected; a separate
// owner gives the OFF build a real compile-time boundary and stops renderer
// dependencies leaking back in.
//
// Compiled in both variants. In the ON build it is what `--no-vulkan` runs on,
// so the OFF build's only code path is exercised by every ON-build no-Vulkan
// test rather than being reachable in one configuration alone.
class HeadlessSession {
 public:
  HeadlessSession();
  ~HeadlessSession();

  HeadlessSession(const HeadlessSession&) = delete;
  HeadlessSession& operator=(const HeadlessSession&) = delete;

  GlobalController& GetGlobalController() { return *global_controller_; }
  Settings& GetSettings() { return settings_; }

 private:
  void SetupScene();

  Settings settings_;
  std::unique_ptr<vkoo::st::Scene> scene_;
  std::unique_ptr<GlobalController> global_controller_;
};
}  // namespace hex
