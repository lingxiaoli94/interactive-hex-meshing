#include "vkoo/core/GeometrySubpass.h"

#include "vkoo/st/Material.h"
#include "vkoo/st/Scene.h"
#include "vkoo/st/Texture.h"
#include "vkoo/st/components/Camera.h"

namespace vkoo {
namespace {
void GatherVisibleMeshesInTree(const st::Node& node,
                               std::vector<st::Mesh*>& result) {
  if (!node.IsVisible()) {
    return;
  }
  auto mesh = node.GetComponentPtr<st::Mesh>();
  if (mesh != nullptr) {
    result.push_back(mesh);
  }
  size_t child_count = node.GetChildrenCount();
  for (size_t i = 0; i < child_count; i++) {
    const st::Node& child = node.GetChild(i);
    GatherVisibleMeshesInTree(child, result);
  }
}

std::vector<st::Mesh*> GetVisibleMeshesInTree(const st::Node& root) {
  std::vector<st::Mesh*> result;
  GatherVisibleMeshesInTree(root, result);
  return result;
}
}  // namespace

GeometrySubpass::GeometrySubpass(RenderContext& render_context,
                                 const st::ShaderProgram& shader_program,
                                 st::Scene& scene,
                                 TransparencyMode transparency_mode)
    : Subpass{render_context, shader_program, scene},
      transparency_mode_{transparency_mode} {}

void GeometrySubpass::Prepare() {}

void GeometrySubpass::Draw(CommandBuffer& command_buffer) {
  auto meshes = GetVisibleMeshesInTree(scene_.GetRoot());
  std::vector<st::Mesh*> solid_meshes;
  std::vector<st::Mesh*> transparent_meshes_depth;
  std::vector<st::Mesh*> transparent_meshes_no_depth;
  for (st::Mesh* mesh : meshes) {
    if (mesh->IsTransparent()) {
      if (mesh->AllowDepthTesting()) {
        transparent_meshes_depth.push_back(mesh);
      } else {
        transparent_meshes_no_depth.push_back(mesh);
      }
    } else {
      solid_meshes.push_back(mesh);
    }
  }

  if (transparency_mode_ != TransparencyMode::TransparentOnly) {
    // By default, depth test/write is enabled for solid objects.
    for (st::Mesh* mesh : solid_meshes) {
      UpdateUniform(command_buffer, *mesh->GetNode());
      DrawSubmesh(command_buffer, *mesh);
    }
  }

  if (transparency_mode_ != TransparencyMode::SolidOnly) {
    // For transparent objects, they never write depth.
    // But they can either allow/disallow depth testing.
    DepthStencilState depth_stencil_state{};
    depth_stencil_state.depth_write_enable = VK_FALSE;

    // Enable alpha blending.
    ColorBlendAttachmentState color_blend_attachment{};
    color_blend_attachment.blend_enable = VK_TRUE;
    color_blend_attachment.src_color_blend_factor = VK_BLEND_FACTOR_SRC_ALPHA;
    color_blend_attachment.dst_color_blend_factor =
        VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
    color_blend_attachment.src_alpha_blend_factor = VK_BLEND_FACTOR_ONE;
    color_blend_attachment.dst_alpha_blend_factor = VK_BLEND_FACTOR_ONE;

    ColorBlendState color_blend_state{};
    color_blend_state.attachments.resize(GetOutputAttachments().size());
    for (auto& it : color_blend_state.attachments) {
      it = color_blend_attachment;
    }
    command_buffer.SetColorBlendState(color_blend_state);

    // Draw transparent meshes that require depth testing first.
    depth_stencil_state.depth_test_enable = VK_TRUE;
    command_buffer.SetDepthStencilState(depth_stencil_state);
    for (st::Mesh* mesh : transparent_meshes_depth) {
      UpdateUniform(command_buffer, *mesh->GetNode());
      DrawSubmesh(command_buffer, *mesh);
    }

    depth_stencil_state.depth_test_enable = VK_FALSE;
    command_buffer.SetDepthStencilState(depth_stencil_state);
    for (st::Mesh* mesh : transparent_meshes_no_depth) {
      UpdateUniform(command_buffer, *mesh->GetNode());
      DrawSubmesh(command_buffer, *mesh);
    }
  }
}

void GeometrySubpass::PreparePipelineState(CommandBuffer& command_buffer,
                                           st::Mesh& mesh) {
  InputAssemblyState input_assembly_state{};
  if (mesh.GetPrimitiveTopology() == st::PrimitiveTopology::LineList) {
    input_assembly_state.topology = VK_PRIMITIVE_TOPOLOGY_LINE_LIST;
  } else if (mesh.GetPrimitiveTopology() ==
             st::PrimitiveTopology::TriangleList) {
    input_assembly_state.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
  } else if (mesh.GetPrimitiveTopology() == st::PrimitiveTopology::PointList) {
    input_assembly_state.topology = VK_PRIMITIVE_TOPOLOGY_POINT_LIST;
  }
  command_buffer.SetInputAssemblyState(input_assembly_state);

  RasterizationState rasterization_state{};
  if (mesh.GetPolygonMode() == st::PolygonMode::Line) {
    rasterization_state.polygon_mode = VK_POLYGON_MODE_LINE;
    rasterization_state.cull_mode = VK_CULL_MODE_NONE;
  } else if (mesh.GetPolygonMode() == st::PolygonMode::Fill) {
    rasterization_state.polygon_mode = VK_POLYGON_MODE_FILL;
    if (transparency_mode_ == TransparencyMode::TransparentOnly) {
      rasterization_state.cull_mode = VK_CULL_MODE_NONE;
    }
  }
  rasterization_state.line_width = mesh.GetLineWidth();

  command_buffer.SetRasterizationState(rasterization_state);
}

PipelineLayout& GeometrySubpass::GetPipelineLayout(
    CommandBuffer& command_buffer,
    const std::vector<ShaderModule*>& shader_modules) {
  return command_buffer.GetDevice().GetResourceCache().RequestPipelineLayout(
      shader_modules);
}

void GeometrySubpass::DrawSubmesh(CommandBuffer& command_buffer,
                                  st::Mesh& mesh) {
  auto& device = command_buffer.GetDevice();

  PreparePipelineState(command_buffer, mesh);

  MultisampleState multisample_state{};
  multisample_state.rasterization_samples = sample_count_;
  command_buffer.SetMultisampleState(multisample_state);

  auto& vert_shader_module = device.GetResourceCache().RequestShaderModule(
      VK_SHADER_STAGE_VERTEX_BIT, shader_program_.vertex_shader_source,
      mesh.GetShaderVariant());
  auto& frag_shader_module = device.GetResourceCache().RequestShaderModule(
      VK_SHADER_STAGE_FRAGMENT_BIT, shader_program_.fragment_shader_source,
      mesh.GetShaderVariant());

  std::vector<ShaderModule*> shader_modules{&vert_shader_module,
                                            &frag_shader_module};

  PipelineLayout& pipeline_layout =
      GetPipelineLayout(command_buffer, shader_modules);

  command_buffer.BindPipelineLayout(pipeline_layout);

  DescriptorSetLayout& descriptor_set_layout =
      pipeline_layout.GetDescriptorSetLayout(0);

  // Bind material.
  MaterialUniform material_uniform{glm::vec4{1.0f}, false};

  const st::Material* material = mesh.GetMaterial();
  if (material != nullptr) {
    auto it = material->textures.find("diffuse_texture");
    if (it != material->textures.end()) {
      if (auto layout_binding =
              descriptor_set_layout.GetLayoutBinding("diffuse_texture")) {
        command_buffer.BindImage(it->second->GetImage()->GetVkImageView(),
                                 *it->second->GetSampler(), 0,
                                 layout_binding->binding, 0);
        material_uniform.has_texture = 1U;
      } else {
        throw std::runtime_error(
            "Shaders for the GeometrySubpass does not have "
            "\"diffuse_texture\"");
      }
    } else {
      auto color_it = material->colors.find("diffuse_color");
      if (color_it != material->colors.end()) {
        material_uniform.diffuse_color = color_it->second;
        material_uniform.has_texture = 0U;
      }
    }
  }
  command_buffer.PushConstants(material_uniform);

  std::vector<ShaderResource> vertex_input_resources =
      pipeline_layout.GetResources(ShaderResourceType::Input,
                                   VK_SHADER_STAGE_VERTEX_BIT);

  VertexInputState vertex_input_state;

  for (auto& input_resource : vertex_input_resources) {
    if (auto attribute = mesh.GetAttribute(input_resource.name)) {
      VkVertexInputAttributeDescription vertex_attribute{};
      vertex_attribute.binding = input_resource.location;
      vertex_attribute.format = attribute->format;
      vertex_attribute.location = input_resource.location;
      vertex_attribute.offset = attribute->offset;

      vertex_input_state.attributes.push_back(vertex_attribute);

      VkVertexInputBindingDescription vertex_binding{};
      vertex_binding.binding = input_resource.location;
      vertex_binding.stride = attribute->stride;

      vertex_input_state.bindings.push_back(vertex_binding);
    }
  }

  command_buffer.SetVertexInputState(vertex_input_state);

  for (auto& input_resource : vertex_input_resources) {
    core::Buffer* buffer =
        mesh.GetVertexObject().FindResource(input_resource.name);
    if (buffer != nullptr) {
      command_buffer.BindVertexBuffers(input_resource.location, {buffer}, {0});
    } else {
      throw std::runtime_error("Cannot find resource " + input_resource.name +
                               " in the mesh!");
    }
  }

  DrawSubmeshCommand(command_buffer, mesh);
}

void GeometrySubpass::DrawSubmeshCommand(CommandBuffer& command_buffer,
                                         st::Mesh& mesh) {
  command_buffer.BindIndexBuffer(mesh.GetVertexObject().GetIndexBuffer(),
                                 mesh.GetIndexOffset(), VK_INDEX_TYPE_UINT32);
  command_buffer.DrawIndexed(mesh.GetIndexCount(), 1, 0, 0, 0);
}

void GeometrySubpass::UpdateUniform(CommandBuffer& command_buffer,
                                    st::Node& node) {
  GlobalUniform global_uniform;
  const st::Camera& camera = *scene_.GetActiveCameraPtr();

  global_uniform.view_proj_mat =
      VulkanStyleProjection(camera.GetProjectionMatrix()) *
      camera.GetViewMatrix();
  global_uniform.camera_position =
      camera.GetNode()->GetTransform().GetWorldPosition();

  st::Transform& transform = node.GetTransform();

  global_uniform.model_mat = transform.GetLocalToWorldMatrix();
  global_uniform.normal_mat =
      glm::transpose(glm::inverse(glm::mat3(global_uniform.model_mat)));

  RenderFrame& render_frame = render_context_.GetActiveFrame();
  core::Buffer& buffer = render_frame.AllocateBuffer(
      VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, sizeof(GlobalUniform));
  buffer.ConvertAndUpdate(global_uniform);

  // Global uniform is always at set = 0, binding = 1.
  command_buffer.BindBuffer(buffer, 0, buffer.GetSize(), 0, 1, 0);
}

}  // namespace vkoo
