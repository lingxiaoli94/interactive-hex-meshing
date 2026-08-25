#include "vkoo/core/Application.h"

#include <cassert>

#include "vkoo/core/Gui.h"
#include "vkoo/logging.h"
#include "vkoo/st/components/Script.h"

namespace vkoo {
Application::Application(bool enable_validation_layer)
    : enable_validation_layer_(enable_validation_layer) {}

void Application::PrepareWindow() {
  glfwInit();

  glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);

  window_ = glfwCreateWindow(starter_width_, starter_height_, name_.c_str(),
                             nullptr, nullptr);
  glfwSetWindowUserPointer(window_, this);

  // Register a bunch of callbacks.
  glfwSetFramebufferSizeCallback(
      window_, +[](GLFWwindow* window, int width, int height) {
        static_cast<Application*>(glfwGetWindowUserPointer(window))
            ->FramebufferSizeCallback(width, height);
      });
  glfwSetCursorPosCallback(
      window_, +[](GLFWwindow* window, double xpos, double ypos) {
        static_cast<Application*>(glfwGetWindowUserPointer(window))
            ->CursorPositionCallback(static_cast<float>(xpos),
                                     static_cast<float>(ypos));
      });
  glfwSetMouseButtonCallback(
      window_, +[](GLFWwindow* window, int button, int action, int /*modes*/) {
        double xpos, ypos;
        glfwGetCursorPos(window, &xpos, &ypos);
        static_cast<Application*>(glfwGetWindowUserPointer(window))
            ->MouseButtonCallback(button, action, static_cast<float>(xpos),
                                  static_cast<float>(ypos));
      });
  glfwSetScrollCallback(
      window_, +[](GLFWwindow* window, double x_offset, double y_offset) {
        static_cast<Application*>(glfwGetWindowUserPointer(window))
            ->ScrollCallback(static_cast<float>(x_offset),
                             static_cast<float>(y_offset));
      });
  glfwSetCharCallback(
      window_, +[](GLFWwindow* window, unsigned int codepoint) {
        static_cast<Application*>(glfwGetWindowUserPointer(window))
            ->CharCallback(codepoint);
      });
  glfwSetKeyCallback(
      window_, +[](GLFWwindow* window, int key, int /*scancode*/, int action,
                   int /*mods*/) {
        static_cast<Application*>(glfwGetWindowUserPointer(window))
            ->KeyCallback(key, action);
      });
}

void Application::ChangeWindowSize(uint32_t new_width, uint32_t new_height) {
  assert(window_);
  glfwSetWindowSize(window_, new_width, new_height);
}

void Application::Prepare(DisplayMode mode) {
  display_mode_ = mode;

  // The GLFW window is the only true X11/display dependency. In headless mode
  // it (and the surface/swapchain built from it) is skipped entirely.
  if (ModeHasWindow(display_mode_)) {
    PrepareWindow();
  } else if (display_mode_ == DisplayMode::HeadlessVulkan) {
    LOGI(
        "vkoo: headless mode - skipping window, surface and swapchain "
        "(surfaceless Vulkan).");
  }

  if (!ModeHasGraphicsDevice(display_mode_)) {
    // Close the tap. No Instance, no GetSuitableGPU(), no Device, no
    // RenderContext -- so not one Vulkan entry point is called and no ICD,
    // driver or validation layer needs to be present. instance_, device_,
    // surface_ and render_context_ all stay null by construction.
    LOGI(
        "vkoo: no-Vulkan mode - skipping instance, physical device, logical "
        "device and render context; no Vulkan call will be made.");
    return;
  }

  std::vector<const char*> instance_extensions;
  if (!IsHeadless()) {
    // Surface extensions are only needed when we have a window to present to.
    uint32_t glfw_extension_count = 0;
    const char** glfw_extensions =
        glfwGetRequiredInstanceExtensions(&glfw_extension_count);
    instance_extensions.assign(glfw_extensions,
                               glfw_extensions + glfw_extension_count);
  }
  instance_extensions.push_back(
      VK_KHR_GET_PHYSICAL_DEVICE_PROPERTIES_2_EXTENSION_NAME);  // to get depth
                                                                // resolve mode

  instance_ = std::make_unique<Instance>(name_, instance_extensions,
                                         enable_validation_layer_);

  if (!IsHeadless()) {
    CreateSurface();
  }

  auto& gpu = instance_->GetSuitableGPU();

  // VK_KHR_swapchain depends on the VK_KHR_surface instance extension, so it is
  // only requested when not headless. surface_ stays VK_NULL_HANDLE in headless
  // mode; Device queries present support with IsPresentSupported(), which
  // already no-ops on a null surface.
  std::vector<const char*> device_extensions = {
      VK_KHR_DEPTH_STENCIL_RESOLVE_EXTENSION_NAME,
      VK_KHR_CREATE_RENDERPASS_2_EXTENSION_NAME,
      VK_KHR_MAINTENANCE2_EXTENSION_NAME, VK_KHR_MULTIVIEW_EXTENSION_NAME};
  if (!IsHeadless()) {
    device_extensions.push_back(VK_KHR_SWAPCHAIN_EXTENSION_NAME);
  }
  device_ = std::make_unique<Device>(gpu, surface_, device_extensions);

  if (!IsHeadless()) {
    render_context_ = std::make_unique<RenderContext>(
        *device_, surface_, starter_width_, starter_height_);
    PrepareRenderContext();
  }
}

bool Application::HasAnyVulkanObject() const {
  return instance_ != nullptr || device_ != nullptr ||
         surface_ != VK_NULL_HANDLE || render_context_ != nullptr;
}

void Application::PrepareRenderContext() { render_context_->Prepare(); }

void Application::CreateSurface() {
  VK_CHECK(glfwCreateWindowSurface(instance_->GetHandle(), window_, nullptr,
                                   &surface_));
}

void Application::Update(float delta_time) {
  UpdateScene(delta_time);

  UpdateGui(delta_time);

  CommandBuffer& command_buffer = render_context_->Begin();
  command_buffer.Begin(VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT);
  Draw(command_buffer, render_context_->GetActiveFrame().GetRenderTarget());
  command_buffer.End();
  render_context_->Submit(command_buffer);
}

void Application::UpdateScene(float delta_time) {
  if (scene_) {
    auto scripts = scene_->GetRoot().GetComponentsRecursive<st::Script>();
    for (auto script : scripts) {
      script->Update(delta_time);
    }
  }
}

void Application::Draw(CommandBuffer& command_buffer,
                       RenderTarget& render_target) {
  DrawRenderPasses(command_buffer, render_target);
}

void Application::SetViewPortAndScissor(CommandBuffer& command_buffer,
                                        const VkExtent2D& extent) const {
  VkViewport viewport{};
  viewport.width = static_cast<float>(extent.width);
  viewport.height = static_cast<float>(extent.height);
  viewport.minDepth = 0.0f;
  viewport.maxDepth = 1.0f;
  command_buffer.SetViewport(0, {viewport});

  VkRect2D scissor{};
  scissor.extent = extent;
  command_buffer.SetScissor(0, {scissor});
}

Application::~Application() {
  // Invariant: a surface can only have been created from an instance. The two
  // independent null checks below are correct only while that holds, so state
  // it rather than relying on them happening to line up.
  assert(surface_ == VK_NULL_HANDLE || instance_ != nullptr);

  if (device_) {
    device_->WaitIdle();
  }
  scene_.reset();
  gui_.reset();

  render_context_.reset();
  device_.reset();
  if (surface_ != VK_NULL_HANDLE) {
    vkDestroySurfaceKHR(instance_->GetHandle(), surface_, nullptr);
  }
  instance_.reset();

  // In headless mode no window was ever created (window_ stays nullptr and GLFW
  // was never initialized), so skip the window/GLFW teardown.
  if (window_ != nullptr) {
    glfwDestroyWindow(window_);
    glfwTerminate();
  }
}

void Application::MainLoop() {
  using Clock = std::chrono::high_resolution_clock;
  using TimePoint =
      std::chrono::time_point<Clock, std::chrono::duration<double>>;
  TimePoint last_tick_time = Clock::now();
  while (!glfwWindowShouldClose(window_)) {
    glfwPollEvents();
    TimePoint current_tick_time = Clock::now();
    double delta_time = (current_tick_time - last_tick_time).count();
    last_tick_time = current_tick_time;

    Update(delta_time);
  }
}

void Application::FramebufferSizeCallback(int width, int height) {
  if (gui_) {
    gui_->Resize(width, height);
  }

  if (scene_) {
    auto scripts = scene_->GetRoot().GetComponentsRecursive<st::Script>();
    for (auto script : scripts) {
      script->OnWindowResize(static_cast<uint32_t>(width),
                             static_cast<uint32_t>(height));
    }
  }
}

void Application::UpdateGui(float delta_time) {
  if (gui_) {
    gui_->NewFrame();
    DrawGui();
    gui_->Update(delta_time);
  }
}

void Application::CursorPositionCallback(float x_pos, float y_pos) {
  HandleInputEvent(MouseButtonInputEvent{MouseButton::Unknown,
                                         MouseAction::Move, x_pos, y_pos});
}

void Application::MouseButtonCallback(int button, int action, float x_pos,
                                      float y_pos) {
  MouseButton translated_button;
  if (button == GLFW_MOUSE_BUTTON_LEFT) {
    translated_button = MouseButton::Left;
  } else if (button == GLFW_MOUSE_BUTTON_RIGHT) {
    translated_button = MouseButton::Right;
  } else if (button == GLFW_MOUSE_BUTTON_MIDDLE) {
    translated_button = MouseButton::Middle;
  } else {
    translated_button = MouseButton::Unknown;
  }

  MouseAction translated_action;
  if (action == GLFW_PRESS) {
    translated_action = MouseAction::Down;
  } else if (action == GLFW_RELEASE) {
    translated_action = MouseAction::Up;
  } else {
    translated_action = MouseAction::Unknown;
  }

  HandleInputEvent(MouseButtonInputEvent{translated_button, translated_action,
                                         x_pos, y_pos});
}

void Application::KeyCallback(int key, int action) {
  KeyAction translated_action;
  if (action == GLFW_PRESS) {
    translated_action = KeyAction::Down;
  } else if (action == GLFW_RELEASE) {
    translated_action = KeyAction::Up;
  } else if (action == GLFW_REPEAT) {
    translated_action = KeyAction::Repeat;
  } else {
    translated_action = KeyAction::Unknown;
  }

  HandleInputEvent(KeyInputEvent{key, translated_action});
}

void Application::CharCallback(unsigned int codepoint) {
  if (gui_) {
    gui_->HandleCharCallback(codepoint);
  }
}

void Application::ScrollCallback(double x_offset, double y_offset) {
  HandleInputEvent(ScrollInputEvent{x_offset, y_offset});
}

void Application::HandleInputEvent(const InputEvent& event) {
  bool captured_by_gui = false;
  if (gui_) {
    captured_by_gui = gui_->HandleInputEvent(event);
  }
  if (!captured_by_gui && scene_) {
    auto scripts = scene_->GetRoot().GetComponentsRecursive<st::Script>();
    for (auto script : scripts) {
      script->HandleInputEvent(event);
    }
  }
}

bool Application::IsKeyPressed(int key) {
  return glfwGetKey(window_, key) == GLFW_PRESS;
}
void Application::GetMousePosition(float& x_pos, float& y_pos) {
  double x_pos_d;
  double y_pos_d;
  glfwGetCursorPos(window_, &x_pos_d, &y_pos_d);
  x_pos = static_cast<float>(x_pos_d);
  y_pos = static_cast<float>(y_pos_d);
}

Device& Application::GetDevice() {
  assert(device_ != nullptr);
  return *device_;
}

st::Scene& Application::GetScene() {
  assert(scene_ != nullptr);
  return *scene_;
}

float Application::GetContentScaleFactor() const {
  int fb_width, fb_height;
  glfwGetFramebufferSize(window_, &fb_width, &fb_height);
  int win_width, win_height;
  glfwGetWindowSize(window_, &win_width, &win_height);

  return static_cast<float>(fb_width) / win_width;
}

float Application::GetDpiFactor() const {
  auto primary_monitor = glfwGetPrimaryMonitor();
  auto vidmode = glfwGetVideoMode(primary_monitor);

  int width_mm, height_mm;
  glfwGetMonitorPhysicalSize(primary_monitor, &width_mm, &height_mm);

  static const float inch_to_mm = 25.0f;
  static const float win_base_density = 96.0f;

  auto dpi = static_cast<uint32_t>(vidmode->width / (width_mm / inch_to_mm));
  auto dpi_factor = dpi / win_base_density;

  return dpi_factor;
}

void Application::GetWindowSize(uint32_t& width, uint32_t& height) {
  if (render_context_) {
    VkExtent2D extent = render_context_->GetSurfaceExtent();
    width = extent.width;
    height = extent.height;
  } else {
    // Headless: no swapchain/surface. Report the nominal starter size so
    // callers (e.g. the scene camera aspect ratio) have sane values.
    width = starter_width_;
    height = starter_height_;
  }
}

float Application::GetWindowAspectRatio() {
  uint32_t width;
  uint32_t height;
  GetWindowSize(width, height);
  return static_cast<float>(width) / static_cast<float>(height);
}

}  // namespace vkoo
