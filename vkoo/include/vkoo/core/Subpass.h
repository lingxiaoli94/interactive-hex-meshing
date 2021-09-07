#pragma once

#include "CommandBuffer.h"
#include "RenderContext.h"
#include "ShaderModule.h"
#include "vkoo/st/Node.h"
#include "vkoo/st/Scene.h"
#include "vkoo/st/ShaderProgram.h"
#include "vkoo/st/Transform.h"
#include "vkoo/st/components/Light.h"

namespace vkoo {
glm::mat4 VulkanStyleProjection(const glm::mat4& proj);

struct alignas(16) Light {
  glm::vec4 position;   // position.w represents type of light
  glm::vec4 color;      // color.w represents light intensity
  glm::vec4 direction;  // direction.w represents range
};

class Subpass {
 public:
  Subpass(RenderContext& render_context,
          const st::ShaderProgram& shader_program, st::Scene& scene);
  virtual ~Subpass() = default;

  virtual void Prepare() = 0;
  virtual void Draw(CommandBuffer& command_buffer) = 0;

  const std::vector<uint32_t> GetInputAttachments() const {
    return input_attachments_;
  }

  const std::vector<uint32_t> GetOutputAttachments() const {
    return output_attachments_;
  }

  const std::vector<uint32_t> GetColorResolveAttachments() const {
    return color_resolve_attachments_;
  }

  uint32_t GetDepthStencilResolveAttachment() const {
    return depth_stencil_resolve_attachment_;
  }

  VkResolveModeFlagBits GetDepthStencilResolveMode() const {
    return depth_stencil_resolve_mode_;
  }

  void SetInputAttachments(const std::vector<uint32_t>& input_attachments) {
    input_attachments_ = input_attachments;
  }

  void SetOutputAttachments(const std::vector<uint32_t>& output_attachments) {
    output_attachments_ = output_attachments;
  }

  void SetColorResolveAttachments(
      const std::vector<uint32_t>& color_resolve_attachments) {
    color_resolve_attachments_ = color_resolve_attachments;
  }

  void SetDepthStencilResolveAttachment(
      uint32_t depth_stencil_resolve_attachment) {
    depth_stencil_resolve_attachment_ = depth_stencil_resolve_attachment;
  }

  void SetDepthStencilResolveMode(
      VkResolveModeFlagBits depth_stencil_resolve_mode) {
    depth_stencil_resolve_mode_ = depth_stencil_resolve_mode;
  }

  bool IsDisableDepthStencilAttachment() const {
    return disable_depth_stencil_attachment_;
  }

  void SetDisableDepthStencilAttachment(bool disable_depth_stencil_attachment) {
    disable_depth_stencil_attachment_ = disable_depth_stencil_attachment;
  }

  void SetSampleCount(VkSampleCountFlagBits sample_count) {
    sample_count_ = sample_count;
  }

 protected:
  template <class T>
  void AggregateLights(T& light_ubo,
                       const std::vector<st::Light*>& scene_lights) {
    light_ubo.directional_light_count = 0;
    light_ubo.point_light_count = 0;
    for (auto& scene_light : scene_lights) {
      const auto& properties = scene_light->GetProperties();
      const auto& transform = scene_light->GetNode()->GetTransform();

      Light light{
          {transform.GetWorldPosition(),
           static_cast<float>(scene_light->GetLightType())},
          {properties.color, properties.intensity},
          {glm::vec3(transform.GetLocalToWorldMatrix() *
                     glm::vec4(properties.direction, 0.0f)),
           properties.range},
      };
      if (scene_light->GetLightType() == st::LightType::Directional) {
        light_ubo.directional_lights[light_ubo.directional_light_count++] =
            light;
      } else if (scene_light->GetLightType() == st::LightType::Point) {
        light_ubo.point_lights[light_ubo.point_light_count++] = light;
      } else {
        throw std::runtime_error("Unsupported light type.");
      }
    }
  }

  RenderContext& render_context_;
  st::ShaderProgram shader_program_;
  st::Scene& scene_;

  VkSampleCountFlagBits sample_count_{VK_SAMPLE_COUNT_1_BIT};

  std::vector<uint32_t> input_attachments_ = {};
  std::vector<uint32_t> output_attachments_ = {0};
  std::vector<uint32_t> color_resolve_attachments_ = {};
  uint32_t depth_stencil_resolve_attachment_{VK_ATTACHMENT_UNUSED};
  VkResolveModeFlagBits depth_stencil_resolve_mode_{VK_RESOLVE_MODE_NONE};

  bool disable_depth_stencil_attachment_{false};
};
}  // namespace vkoo
