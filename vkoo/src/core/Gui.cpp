#include "vkoo/core/Gui.h"

#include "imgui.h"
#include "vkoo/core/Application.h"
#include "vkoo/core/ShaderModule.h"
#include "vkoo/logging.h"
#include "vkoo/utils.h"

namespace vkoo {
namespace {
static const char* ImGui_ImplGlfw_GetClipboardText(void* user_data) {
  return glfwGetClipboardString((GLFWwindow*)user_data);
}

static void ImGui_ImplGlfw_SetClipboardText(void* user_data, const char* text) {
  glfwSetClipboardString((GLFWwindow*)user_data, text);
}

void UploadRawData(ImDrawData* draw_data, const uint8_t* vertex_data,
                   const uint8_t* index_data) {
  ImDrawVert* vtx_dst = (ImDrawVert*)vertex_data;
  ImDrawIdx* idx_dst = (ImDrawIdx*)index_data;

  for (int n = 0; n < draw_data->CmdListsCount; n++) {
    const ImDrawList* cmd_list = draw_data->CmdLists[n];
    memcpy(vtx_dst, cmd_list->VtxBuffer.Data,
           cmd_list->VtxBuffer.Size * sizeof(ImDrawVert));
    memcpy(idx_dst, cmd_list->IdxBuffer.Data,
           cmd_list->IdxBuffer.Size * sizeof(ImDrawIdx));
    vtx_dst += cmd_list->VtxBuffer.Size;
    idx_dst += cmd_list->IdxBuffer.Size;
  }
}
}  // namespace

Gui::Gui(Application& application, float font_size)
    : application_{application},
      font_size_{font_size},
      content_scale_factor_{application.GetContentScaleFactor()},
      dpi_factor_{application.GetDpiFactor()} {
  ImGui::CreateContext();

  // Styles.
  ImGui::StyleColorsDark();
  ImGuiStyle& style = ImGui::GetStyle();
  style.WindowPadding = {12.0f, 12.0f};
  style.WindowRounding = 8.0f;
  style.FrameRounding = 6.0f;
  style.GrabRounding = style.FrameRounding;
  style.GrabMinSize = 8.0f;
  style.IndentSpacing = 1.0f;
  {
    ImVec4* colors = ImGui::GetStyle().Colors;
    colors[ImGuiCol_TitleBgActive] = ImVec4(0.05f, 0.26f, 0.25f, 1.00f);
    colors[ImGuiCol_TitleBgCollapsed] = ImVec4(0.14f, 0.13f, 0.13f, 0.51f);
  }

  style.WindowMenuButtonPosition = ImGuiDir_Right;
  style.WindowBorderSize = 0.0f;
  style.ScaleAllSizes(1.0f * dpi_factor_);

  // Dimensions.
  ImGuiIO& io = ImGui::GetIO();
  auto extent = application_.GetRenderContext().GetSurfaceExtent();
  io.DisplaySize.x = static_cast<float>(extent.width);
  io.DisplaySize.y = static_cast<float>(extent.height);
  io.FontGlobalScale = 1.0f;
  io.DisplayFramebufferScale = ImVec2(1.0f, 1.0f);

  // io.MouseDoubleClickTime = 1.0f;

  // Enable keyboard navigation.
  io.KeyMap[ImGuiKey_Tab] = GLFW_KEY_TAB;
  io.KeyMap[ImGuiKey_LeftArrow] = GLFW_KEY_LEFT;
  io.KeyMap[ImGuiKey_RightArrow] = GLFW_KEY_RIGHT;
  io.KeyMap[ImGuiKey_UpArrow] = GLFW_KEY_UP;
  io.KeyMap[ImGuiKey_DownArrow] = GLFW_KEY_DOWN;
  io.KeyMap[ImGuiKey_PageUp] = GLFW_KEY_PAGE_UP;
  io.KeyMap[ImGuiKey_PageDown] = GLFW_KEY_PAGE_DOWN;
  io.KeyMap[ImGuiKey_Home] = GLFW_KEY_HOME;
  io.KeyMap[ImGuiKey_End] = GLFW_KEY_END;
  io.KeyMap[ImGuiKey_Insert] = GLFW_KEY_INSERT;
  io.KeyMap[ImGuiKey_Delete] = GLFW_KEY_DELETE;
  io.KeyMap[ImGuiKey_Backspace] = GLFW_KEY_BACKSPACE;
  io.KeyMap[ImGuiKey_Space] = GLFW_KEY_SPACE;
  io.KeyMap[ImGuiKey_Enter] = GLFW_KEY_ENTER;
  io.KeyMap[ImGuiKey_Escape] = GLFW_KEY_ESCAPE;
  io.KeyMap[ImGuiKey_KeyPadEnter] = GLFW_KEY_KP_ENTER;

  io.SetClipboardTextFn = ImGui_ImplGlfw_SetClipboardText;
  io.GetClipboardTextFn = ImGui_ImplGlfw_GetClipboardText;
  io.ClipboardUserData = application_.GetGLFWWindow();

  io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;

  PrepareFont();

  Device& device = application_.GetRenderContext().GetDevice();

  // Prepare shaders.
  std::vector<ShaderModule*> shader_modules;
  shader_modules.push_back(&device.GetResourceCache().RequestShaderModule(
      VK_SHADER_STAGE_VERTEX_BIT,
      ShaderSource{GetShaderPath() + "/imgui.vert", "main"}));
  shader_modules.push_back(&device.GetResourceCache().RequestShaderModule(
      VK_SHADER_STAGE_FRAGMENT_BIT,
      ShaderSource{GetShaderPath() + "/imgui.frag", "main"}));
  pipeline_layout_ =
      &device.GetResourceCache().RequestPipelineLayout(shader_modules);

  // Create sampler.
  VkSamplerCreateInfo sampler_info{VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO};
  sampler_info.maxAnisotropy = 1.0f;
  sampler_info.magFilter = VK_FILTER_LINEAR;
  sampler_info.minFilter = VK_FILTER_LINEAR;
  sampler_info.mipmapMode = VK_SAMPLER_MIPMAP_MODE_NEAREST;
  sampler_info.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
  sampler_info.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
  sampler_info.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
  sampler_info.borderColor = VK_BORDER_COLOR_FLOAT_OPAQUE_WHITE;

  sampler_ = std::make_unique<core::Sampler>(device, sampler_info);
}

Gui::~Gui() { ImGui::DestroyContext(); }

void Gui::PrepareFont() {
  // Default font.
  fonts_.emplace_back(kDefaultFont, font_size_ * dpi_factor_);

  // Create font texture.
  unsigned char* font_data;
  int tex_width, tex_height;
  ImGuiIO& io = ImGui::GetIO();
  io.Fonts->GetTexDataAsRGBA32(&font_data, &tex_width, &tex_height);
  size_t upload_size = tex_width * tex_height * 4 * sizeof(char);

  Device& device = application_.GetRenderContext().GetDevice();

  // Create target image for copy.
  VkExtent2D font_extent{static_cast<uint32_t>(tex_width),
                         static_cast<uint32_t>(tex_height)};
  font_image_ = std::make_unique<core::Image>(
      device, font_extent, VK_FORMAT_R8G8B8A8_UNORM,
      VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT,
      VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
  font_image_view_ =
      std::make_unique<core::ImageView>(*font_image_, VK_IMAGE_VIEW_TYPE_2D);

  // Upload font data into the vulkan image memory.
  {
    core::Buffer staging_buffer{device, upload_size,
                                VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                                VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
                                    VK_MEMORY_PROPERTY_HOST_COHERENT_BIT};
    staging_buffer.Update({font_data, font_data + upload_size});

    auto& command_buffer = device.RequestCommandBuffer();

    FencePool fence_pool{device};

    command_buffer.Begin(VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT);

    {
      ImageMemoryBarrier memory_barrier{};
      memory_barrier.old_layout = VK_IMAGE_LAYOUT_UNDEFINED;
      memory_barrier.new_layout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
      memory_barrier.src_access_mask = 0;
      memory_barrier.dst_access_mask = VK_ACCESS_TRANSFER_WRITE_BIT;
      memory_barrier.src_stage_mask = VK_PIPELINE_STAGE_HOST_BIT;
      memory_barrier.dst_stage_mask = VK_PIPELINE_STAGE_TRANSFER_BIT;

      command_buffer.InsertImageMemoryBarrier(*font_image_view_,
                                              memory_barrier);
    }

    VkBufferImageCopy buffer_copy_region{};
    buffer_copy_region.imageSubresource.layerCount =
        font_image_view_->GetSubresourceRange().layerCount;
    buffer_copy_region.imageSubresource.aspectMask =
        font_image_view_->GetSubresourceRange().aspectMask;
    buffer_copy_region.imageExtent.width = font_image_->GetExtent().width;
    buffer_copy_region.imageExtent.height = font_image_->GetExtent().height;
    buffer_copy_region.imageExtent.depth = 1;

    command_buffer.CopyBufferToImage(staging_buffer, *font_image_,
                                     {buffer_copy_region});

    {
      ImageMemoryBarrier memory_barrier{};
      memory_barrier.old_layout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
      memory_barrier.new_layout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
      memory_barrier.src_access_mask = 0;
      memory_barrier.dst_access_mask = VK_ACCESS_SHADER_READ_BIT;
      memory_barrier.src_stage_mask = VK_PIPELINE_STAGE_TRANSFER_BIT;
      memory_barrier.dst_stage_mask = VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT;

      command_buffer.InsertImageMemoryBarrier(*font_image_view_,
                                              memory_barrier);
    }

    command_buffer.End();

    auto& queue = device.GetQueueByFlags(VK_QUEUE_GRAPHICS_BIT, 0);

    queue.Submit(command_buffer, device.RequestFence());

    device.GetFencePool().Wait();
    device.GetFencePool().Reset();
    device.GetCommandPool().ResetPool();
    device.WaitIdle();
  }
}

void Gui::Draw(CommandBuffer& command_buffer) {
  // Vertex input state.
  VkVertexInputBindingDescription vertex_input_binding{};
  vertex_input_binding.stride = static_cast<uint32_t>(sizeof(ImDrawVert));

  // Location 0: Position.
  VkVertexInputAttributeDescription pos_attr{};
  pos_attr.format = VK_FORMAT_R32G32_SFLOAT;
  pos_attr.offset = static_cast<uint32_t>(offsetof(ImDrawVert, pos));

  // Location 1: UV.
  VkVertexInputAttributeDescription uv_attr{};
  uv_attr.location = 1;
  uv_attr.format = VK_FORMAT_R32G32_SFLOAT;
  uv_attr.offset = static_cast<uint32_t>(offsetof(ImDrawVert, uv));

  // Location 2: Color.
  VkVertexInputAttributeDescription col_attr{};
  col_attr.location = 2;
  col_attr.format = VK_FORMAT_R8G8B8A8_UNORM;
  col_attr.offset = static_cast<uint32_t>(offsetof(ImDrawVert, col));

  VertexInputState vertex_input_state{};
  vertex_input_state.bindings = {vertex_input_binding};
  vertex_input_state.attributes = {pos_attr, uv_attr, col_attr};

  command_buffer.SetVertexInputState(vertex_input_state);

  // Blend state.
  ColorBlendAttachmentState color_attachment{};
  color_attachment.blend_enable = VK_TRUE;
  color_attachment.color_write_mask =
      VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT |
      VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
  color_attachment.src_color_blend_factor = VK_BLEND_FACTOR_SRC_ALPHA;
  color_attachment.dst_color_blend_factor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
  color_attachment.src_alpha_blend_factor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
  color_attachment.src_alpha_blend_factor = VK_BLEND_FACTOR_ONE;
  color_attachment.dst_alpha_blend_factor = VK_BLEND_FACTOR_ONE;
  color_attachment.alpha_blend_op = VK_BLEND_OP_MAX;

  ColorBlendState blend_state{};
  blend_state.attachments = {color_attachment};

  command_buffer.SetColorBlendState(blend_state);

  RasterizationState rasterization_state{};
  rasterization_state.cull_mode = VK_CULL_MODE_NONE;
  command_buffer.SetRasterizationState(rasterization_state);

  DepthStencilState depth_state{};
  depth_state.depth_test_enable = VK_FALSE;
  depth_state.depth_write_enable = VK_FALSE;
  command_buffer.SetDepthStencilState(depth_state);

  // Bind pipeline layout.
  command_buffer.BindPipelineLayout(*pipeline_layout_);

  command_buffer.BindImage(*font_image_view_, *sampler_, 0, 0, 0);

  // Pre-rotation: none on desktops.
  ImGuiIO& io = ImGui::GetIO();
  auto push_transform = glm::mat4(1.0f);

  // GUI coordinate space to screen space.
  push_transform =
      glm::translate(push_transform, glm::vec3(-1.0f, -1.0f, 0.0f));
  push_transform = glm::scale(
      push_transform,
      glm::vec3(2.0f / io.DisplaySize.x, 2.0f / io.DisplaySize.y, 0.0f));

  // Push constants.
  command_buffer.PushConstants(push_transform);

  UpdateBuffers(command_buffer,
                application_.GetRenderContext().GetActiveFrame());

  // Render commands.
  ImDrawData* draw_data = ImGui::GetDrawData();
  int32_t vertex_offset = 0;
  uint32_t index_offset = 0;

  if (!draw_data || draw_data->CmdListsCount == 0) {
    return;
  }

  for (int32_t i = 0; i < draw_data->CmdListsCount; i++) {
    const ImDrawList* cmd_list = draw_data->CmdLists[i];
    for (int32_t j = 0; j < cmd_list->CmdBuffer.Size; j++) {
      const ImDrawCmd* cmd = &cmd_list->CmdBuffer[j];
      VkRect2D scissor_rect;
      scissor_rect.offset.x =
          std::max(static_cast<int32_t>(cmd->ClipRect.x), 0);
      scissor_rect.offset.y =
          std::max(static_cast<int32_t>(cmd->ClipRect.y), 0);
      scissor_rect.extent.width =
          static_cast<uint32_t>(cmd->ClipRect.z - cmd->ClipRect.x);
      scissor_rect.extent.height =
          static_cast<uint32_t>(cmd->ClipRect.w - cmd->ClipRect.y);

      command_buffer.SetScissor(0, {scissor_rect});
      command_buffer.DrawIndexed(cmd->ElemCount, 1, index_offset, vertex_offset,
                                 0);
      index_offset += cmd->ElemCount;
    }
    vertex_offset += cmd_list->VtxBuffer.Size;
  }
}

void Gui::UpdateBuffers(CommandBuffer& command_buffer,
                        RenderFrame& render_frame) {
  ImDrawData* draw_data = ImGui::GetDrawData();

  if (!draw_data) {
    return;
  }

  size_t vertex_buffer_size = draw_data->TotalVtxCount * sizeof(ImDrawVert);
  size_t index_buffer_size = draw_data->TotalIdxCount * sizeof(ImDrawIdx);

  if ((vertex_buffer_size == 0) || (index_buffer_size == 0)) {
    return;
  }

  std::vector<uint8_t> vertex_data(vertex_buffer_size);
  std::vector<uint8_t> index_data(index_buffer_size);

  UploadRawData(draw_data, vertex_data.data(), index_data.data());

  core::Buffer& vertex_buffer =
      application_.GetRenderContext().GetActiveFrame().AllocateBuffer(
          VK_BUFFER_USAGE_VERTEX_BUFFER_BIT, vertex_buffer_size);

  vertex_buffer.Update(vertex_data);

  command_buffer.BindVertexBuffers(0, {&vertex_buffer}, {0});

  core::Buffer& index_buffer =
      application_.GetRenderContext().GetActiveFrame().AllocateBuffer(
          VK_BUFFER_USAGE_INDEX_BUFFER_BIT, index_buffer_size);

  index_buffer.Update(index_data);

  command_buffer.BindIndexBuffer(index_buffer, 0, VK_INDEX_TYPE_UINT16);
}

void Gui::Update(float delta_time) {
  // Update imGui.
  ImGuiIO& io = ImGui::GetIO();
  auto extent = application_.GetRenderContext().GetSurfaceExtent();
  Resize(extent.width, extent.height);
  io.DeltaTime = delta_time;

  // Render to generate draw buffers.
  ImGui::Render();
}

void Gui::NewFrame() { ImGui::NewFrame(); }

void Gui::Resize(uint32_t width, uint32_t height) {
  ImGuiIO& io = ImGui::GetIO();
  io.DisplaySize.x = static_cast<float>(width);
  io.DisplaySize.y = static_cast<float>(height);

  content_scale_factor_ = application_.GetContentScaleFactor();
}

Font::Font(const std::string& name, float size)
    : name_{name},
      data_{ReadBinaryFile(GetAssetPath() + "/fonts/" + name + ".ttf")} {
  // Keep ownership of the font data to avoid a double delete.
  ImFontConfig font_config{};
  font_config.FontDataOwnedByAtlas = false;

  ImGuiIO& io = ImGui::GetIO();
  handle_ = io.Fonts->AddFontFromMemoryTTF(
      data_.data(), static_cast<int>(data_.size()), size, &font_config);
}

bool Gui::HandleInputEvent(const InputEvent& event) {
  ImGuiIO& io = ImGui::GetIO();
  bool captured = false;
  if (event.GetSource() == EventSource::Mouse) {
    auto& mouse_event = static_cast<const MouseButtonInputEvent&>(event);
    io.MousePos = ImVec2{mouse_event.GetXPos() * content_scale_factor_,
                         mouse_event.GetYPos() * content_scale_factor_};
    // Direct casting button id is compatible.
    auto button_id = static_cast<int>(mouse_event.GetButton());
    if (mouse_event.GetAction() == MouseAction::Down) {
      io.MouseDown[button_id] = true;
      // FIXME: right now don't capture mouse down event if it is not hovered
      // over any window or any popup. This is a bit hacky.
      captured =
          ImGui::IsWindowHovered(ImGuiHoveredFlags_AnyWindow |
                                 ImGuiHoveredFlags_AllowWhenBlockedByPopup);
    } else if (mouse_event.GetAction() == MouseAction::Up) {
      io.MouseDown[button_id] = false;
      captured =
          ImGui::IsWindowHovered(ImGuiHoveredFlags_AnyWindow |
                                 ImGuiHoveredFlags_AllowWhenBlockedByPopup);
    } else if (mouse_event.GetAction() == MouseAction::Move) {
      captured = io.WantCaptureMouse;
    }
  } else if (event.GetSource() == EventSource::Keyboard) {
      auto& key_event = static_cast<const KeyInputEvent&>(event);
      if (key_event.GetAction() == KeyAction::Down) {
        io.KeysDown[key_event.GetCode()] = true;
      }
      if (key_event.GetAction() == KeyAction::Up) {
        io.KeysDown[key_event.GetCode()] = false;
      }
      io.KeyCtrl = io.KeysDown[GLFW_KEY_LEFT_CONTROL] ||
                   io.KeysDown[GLFW_KEY_RIGHT_CONTROL];
      io.KeyShift =
          io.KeysDown[GLFW_KEY_LEFT_SHIFT] || io.KeysDown[GLFW_KEY_RIGHT_SHIFT];
      io.KeyAlt =
          io.KeysDown[GLFW_KEY_LEFT_ALT] || io.KeysDown[GLFW_KEY_RIGHT_ALT];
      io.KeySuper =
          io.KeysDown[GLFW_KEY_LEFT_SUPER] || io.KeysDown[GLFW_KEY_RIGHT_SUPER];
      if (io.WantCaptureKeyboard | io.WantTextInput) {
        captured = true;
      }
  } else if (event.GetSource() == EventSource::Scroll) {
    auto& scroll_event = static_cast<const ScrollInputEvent&>(event);
    if (io.WantCaptureMouse) {
      io.MouseWheelH += (float)scroll_event.GetXOffset();
      io.MouseWheel += (float)scroll_event.GetYOffset();
      captured = true;
    }
  }
  return captured;
}

void Gui::HandleCharCallback(unsigned int codepoint) {
  ImGuiIO& io = ImGui::GetIO();
  io.AddInputCharacter(codepoint);
}

}  // namespace vkoo
