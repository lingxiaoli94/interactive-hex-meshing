#pragma once

#define GLFW_INCLUDE_VULKAN
#include <GLFW/glfw3.h>

#include "Device.h"
#include "InputEvent.h"
#include "Instance.h"
#include "PhysicalDevice.h"
#include "RenderContext.h"
#include "RenderPipeline.h"
#include "vkoo/st/Scene.h"

namespace vkoo {
class Gui;

class Application {
 public:
  Application(bool enable_validation_layer);
  virtual ~Application();

  virtual void Prepare();
  virtual void Update(float delta_time);
  virtual void Draw(CommandBuffer& command_buffer, RenderTarget& render_target);
  virtual void DrawRenderPasses(CommandBuffer& command_buffer,
                                RenderTarget& render_target) = 0;
  virtual void MainLoop();

  RenderContext& GetRenderContext() { return *render_context_; }
  float GetContentScaleFactor() const;
  virtual float GetDpiFactor() const;
  void GetWindowSize(uint32_t& width, uint32_t& height);
  float GetWindowAspectRatio();
  GLFWwindow* GetGLFWWindow() { return window_; }

  bool IsKeyPressed(int key);
  void GetMousePosition(float& x_pos, float& y_pos);
  Device& GetDevice();
  st::Scene& GetScene();

 protected:
  virtual void PrepareRenderContext();
  void SetViewPortAndScissor(CommandBuffer& command_buffer,
                             const VkExtent2D& extent) const;
  void ChangeWindowSize(uint32_t new_width, uint32_t new_height);
  virtual void UpdateScene(float delta_time);
  virtual void UpdateGui(float delta_time);
  virtual void DrawGui() {}
  virtual void FramebufferSizeCallback(int width, int height);
  virtual void HandleInputEvent(const InputEvent& event);

  std::unique_ptr<Instance> instance_;
  std::unique_ptr<Device> device_;

  // render_context_ manages the swapchain and a list of RenderFrame.
  std::unique_ptr<RenderContext> render_context_;

  std::string name_;

  std::unique_ptr<st::Scene> scene_;

  std::unique_ptr<Gui> gui_;
  GLFWwindow* window_;

  uint32_t starter_width_{1280};
  uint32_t starter_height_{720};

 private:
  void PrepareWindow();
  void CreateSurface();
  void CursorPositionCallback(float x_pos, float y_pos);
  void MouseButtonCallback(int button, int action, float x_pos, float y_pos);
  void KeyCallback(int key, int action);
  void CharCallback(unsigned int codepoint);
  void ScrollCallback(double x_offset, double y_offset);

  bool enable_validation_layer_;
  VkSurfaceKHR surface_;
};
}  // namespace vkoo
