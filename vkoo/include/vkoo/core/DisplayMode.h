#pragma once

namespace vkoo {
// What an Application is allowed to talk to.
//
//   Gui               window + surface + swapchain + render context. The
//                     unchanged interactive path.
//   HeadlessVulkan    no window, surface or swapchain, but the Vulkan instance
//                     and logical device still exist so views and GPU buffers
//                     can be allocated. Needs an ICD and a graphics driver.
//   HeadlessNoVulkan  no Vulkan object of any kind is created and no Vulkan
//                     entry point is called, so no ICD, driver, layer or
//                     VK_ICD_FILENAMES is needed. The binary still LINKS the
//                     Vulkan loader (libvulkan.so.1); removing that requires a
//                     HEX_ENABLE_VULKAN=OFF build.
//
// One parameter instead of two bools on purpose: `headless=false,
// no_vulkan=true` is not a state the renderer can be in, and an enum cannot
// express it. --no-vulkan implies --headless, one way.
enum class DisplayMode {
  Gui,
  HeadlessVulkan,
  HeadlessNoVulkan,
};

// Whether the mode permits creating an Instance/Device. This describes the
// *request*; ask Application::HasGraphicsDevice() for what actually exists.
constexpr bool ModeHasGraphicsDevice(DisplayMode mode) {
  return mode != DisplayMode::HeadlessNoVulkan;
}

constexpr bool ModeHasWindow(DisplayMode mode) {
  return mode == DisplayMode::Gui;
}
}  // namespace vkoo
