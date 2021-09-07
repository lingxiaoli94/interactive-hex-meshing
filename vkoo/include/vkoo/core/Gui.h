#pragma once

#include <imgui.h>

#include "Buffer.h"
#include "CommandBuffer.h"
#include "Image.h"
#include "ImageView.h"
#include "InputEvent.h"
#include "RenderFrame.h"
#include "Sampler.h"
#include "vkoo/common.h"

namespace vkoo {
class Application;
struct Font {
 public:
  Font(const std::string& name, float size);

 private:
  ImFont* handle_{nullptr};
  std::string name_;
  std::vector<uint8_t> data_;
};

class Gui {
 public:
  Gui(Application& application, float font_size);
  ~Gui();
  void Resize(uint32_t width, uint32_t height);
  void Draw(CommandBuffer& command_buffer);
  void NewFrame();
  void Update(float delta_time);
  bool HandleInputEvent(const InputEvent& event);
  void HandleCharCallback(unsigned int codepoint);

 private:
  void PrepareFont();
  void UpdateBuffers(CommandBuffer& command_buffer, RenderFrame& render_frame);

  Application& application_;

  std::vector<Font> fonts_;
  float font_size_;

  std::unique_ptr<core::Image> font_image_;
  std::unique_ptr<core::ImageView> font_image_view_;
  std::unique_ptr<core::Sampler> sampler_;

  PipelineLayout* pipeline_layout_;

  float content_scale_factor_;
  float dpi_factor_;

  const std::string kDefaultFont = "NunitoSans-Regular";
};
}  // namespace vkoo
