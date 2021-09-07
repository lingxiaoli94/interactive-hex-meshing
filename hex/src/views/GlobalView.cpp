#include "GlobalView.h"

#include <glm/gtx/color_space.hpp>
#include <imgui.h>
#include <vkoo/st/components/Mesh.h>

#include <limits>
#include <stdexcept>

#include "logging.h"
#include "models/HexConvention.h"

namespace hex {
namespace {
std::string FlagToName(GlobalView::VisibilityFlag flag) {
  if (flag == GlobalView::VisibilityInputMesh) {
    return "Input mesh";
  } else if (flag == GlobalView::VisibilityDeformedMesh) {
    return "Deformed mesh";
  } else if (flag == GlobalView::VisibilityDecomposition) {
    return "Cuboid decomposition";
  } else if (flag == GlobalView::VisibilityPolycubeComplex) {
    return "Polycube quad surface";
  } else if (flag == GlobalView::VisibilityResultFilteredMesh) {
    return "Result mesh (filtered)";
  } else if (flag == GlobalView::VisibilityAnchors) {
    return "Anchors";
  }

  throw std::runtime_error(fmt::format("Unrecognized flag: {}", flag));
}

const std::vector<glm::vec3> kAlignedFaceHSVColors = {
    glm::vec3{1.5f / 7.0f, 0.8f, 0.8f}, glm::vec3{2.5f / 7.0f, 0.8f, 0.8f},
    glm::vec3{3.5f / 7.0f, 0.8f, 0.8f}, glm::vec3{4.5f / 7.0f, 0.8f, 0.8f},
    glm::vec3{5.5f / 7.0f, 0.8f, 0.8f}, glm::vec3{6.5f / 7.0f, 0.8f, 0.8f}};
const std::vector<glm::vec4> kFilteredMeshColors = {
    {252.f / 255.f, 169.f / 255.f, 3.f / 255.f, 1.0f},
    {1.0f, 1.0f, 1.0f, 1.0f}};
}  // namespace

GlobalView::GlobalView(vkoo::Device& device, vkoo::st::Scene& scene)
    : device_{device}, scene_{scene} {
  CreateDefaultMaterials();
  SetMeshOptions();
  RegisterVisibilityInfos();
}

GlobalView::~GlobalView() {
  // Cleanup child views first, which may remove nodes under wrappers.
  // Otherwise double-delete might occur.
  if (polycube_view_) {
    polycube_view_->Cleanup();
  }

  // Next remove nodes from the scene.
  for (auto& info : visibility_infos_) {
    if (info.node != nullptr) {
      info.node->RemoveFromParent();
    }
  }
}

void GlobalView::CreateDefaultMaterials() {
  anchors_materials_[0].colors["diffuse_color"] = {1.0f, 0.0f, 0.0f, 1.0f};
  anchors_materials_[1].colors["diffuse_color"] = {0.0f, 1.0f, 0.0f, 1.0f};
}

void GlobalView::SetMeshOptions() {
  input_mesh_options_.transparent_color = {52.f / 255.f, 219.f / 255.f,
                                           235.f / 255.f, 0.3f};
  deformed_mesh_options_.transparent_color = {0.964f, 0.278f, 1.f, 0.5f};
}

void GlobalView::RegisterVisibilityInfos() {
  visibility_infos_.push_back(
      {VisibilityInputMesh, input_mesh_node_, visibility_.input_mesh});
  visibility_infos_.push_back(
      {VisibilityDeformedMesh, deformed_mesh_node_, visibility_.deformed_mesh});
  visibility_infos_.push_back(
      {VisibilityDecomposition, polycube_node_, visibility_.decomposition});
  visibility_infos_.push_back({VisibilityPolycubeComplex,
                               polycube_complex_node_,
                               visibility_.polycube_complex});
  visibility_infos_.push_back({VisibilityResultFilteredMesh,
                               result_filtered_mesh_node_,
                               visibility_.result_filtered_mesh});
  visibility_infos_.push_back(
      {VisibilityAnchors, anchors_node_, visibility_.anchors});
}

void GlobalView::UpdateAllViews(GlobalState& global_state) {
  unsigned long visibility_mask = GetVisibility();

  UpdateInputMeshView(global_state);
  UpdateDeformedMeshView(global_state);
  UpdateAnchorsView(global_state);
  UpdatePolycubeView(global_state);
  UpdatePolycubeComplexView(global_state);
  UpdateFilteredHexView(global_state, FilteringSetting());

  SetVisibility(visibility_mask);
}

void GlobalView::UpdateInputMeshView(GlobalState& global_state) {
  if (input_mesh_node_ != nullptr) {
    input_mesh_node_->RemoveFromParent();
    input_mesh_node_ = nullptr;
  }
  if (global_state.HasTargetVolumeMesh()) {
    input_mesh_view_ = std::make_unique<TriSurfaceView>(
        device_, scene_.GetRoot(), input_mesh_options_);
    input_mesh_view_->Update(
        global_state.GetTargetVolumeMesh().GetSurfaceMesh());
    input_mesh_node_ = input_mesh_view_->GetWrapperNode();
  }
}

void GlobalView::UpdateDeformedMeshView(GlobalState& global_state) {
  if (deformed_mesh_node_ != nullptr) {
    deformed_mesh_node_->RemoveFromParent();
    deformed_mesh_node_ = nullptr;
  }
  if (global_state.HasDeformedVolumeMesh()) {
    deformed_mesh_view_ = std::make_unique<TriSurfaceView>(
        device_, scene_.GetRoot(), deformed_mesh_options_);
    deformed_mesh_view_->Update(
        global_state.GetDeformedVolumeMesh().GetSurfaceMesh());
    deformed_mesh_node_ = deformed_mesh_view_->GetWrapperNode();
  }
}

void GlobalView::UpdateAnchorsView(GlobalState& global_state) {
  if (anchors_node_ != nullptr) {
    anchors_node_->RemoveFromParent();
    anchors_node_ = nullptr;
  }
  if (global_state.HasDeformedVolumeMesh()) {
    auto& deformed_mesh = global_state.GetDeformedVolumeMesh();
    if (deformed_mesh.HasAnchors() && deformed_mesh.HasDistanceField()) {
      auto vbos = deformed_mesh.CreateAnchorVBOs(device_);
      auto node = std::make_unique<vkoo::st::Node>();
      for (int i = 0; i < 2; i++) {
        auto subnode = std::make_unique<vkoo::st::Node>();
        auto& mesh_component = subnode->CreateComponent<vkoo::st::Mesh>(
            vbos[i], vbos[i]->GetIndexCount());
        mesh_component.SetMaterial(anchors_materials_[i]);
        mesh_component.SetPrimitiveTopology(
            vkoo::st::PrimitiveTopology::PointList);
        node->AddChild(std::move(subnode));
      }
      anchors_node_ = node.get();
      scene_.GetRoot().AddChild(std::move(node));
    }
  }
}

void GlobalView::UpdatePolycubeComplexView(const QuadComplex& quad_complex,
                                           bool color_by_patch,
                                           bool use_axis_color) {
  CleanupPolycubeComplexView();
  auto node = std::make_unique<vkoo::st::Node>();
  polycube_complex_view_ = std::make_unique<QuadSurfaceView>(
      device_, *node, QuadSurfaceView::Options());
  if (color_by_patch) {
    // Update patch colors to the newest.
    patch_colors_ = std::make_unique<std::vector<glm::vec4>>(
        CreatePolycubePatchColors(quad_complex, use_axis_color));
    polycube_complex_view_->Update(quad_complex, *patch_colors_);
  } else {
    polycube_complex_view_->Update(*quad_complex.ExtractQuadMesh());
  }
  polycube_complex_node_ = node.get();
  scene_.GetRoot().AddChild(std::move(node));
}

std::vector<glm::vec4> GlobalView::CreatePolycubePatchColors(
    const QuadComplex& quad_complex, bool use_axis_color) {
  std::vector<glm::vec4> result;
  auto& vertices = quad_complex.GetVertices();
  auto& kFaceOffsets = HexConvention::GetFaceOffsets();
  auto rand_eng = std::default_random_engine(42);
  std::uniform_real_distribution<float> dis(0.0f, 1.0f);
  for (size_t i = 0; i < quad_complex.GetPatches().size(); i++) {
    glm::vec3 rgb_color;
    if (use_axis_color) {
      auto& patch = quad_complex.GetPatches()[i];
      assert(!patch.empty());
      auto& quad = quad_complex.GetQuads()[patch[0]];
      Vector3f v[4];
      for (int k = 0; k < 4; k++) {
        v[k] = vertices[quad[k]];
      }
      auto n = (v[1] - v[0]).cross(v[2] - v[0]).normalized();
      int closest = 0;
      float closest_dot = std::numeric_limits<float>::min();
      for (size_t j = 0; j < 6; j++) {
        float tmp = kFaceOffsets[j].cast<float>().dot(n);
        if (closest_dot < tmp) {
          closest_dot = tmp;
          closest = j;
        }
      }
      auto hsv_color = kAlignedFaceHSVColors[closest];
      ImGui::ColorConvertHSVtoRGB(hsv_color.x, hsv_color.y, hsv_color.z,
                                  rgb_color.x, rgb_color.y, rgb_color.z);
    } else {
      rgb_color = glm::vec3{(float)dis(rand_eng), (float)dis(rand_eng),
                            (float)dis(rand_eng)};
    }
    result.emplace_back(rgb_color.x, rgb_color.y, rgb_color.z, 1.0f);
  }
  return result;
}

void GlobalView::UpdatePolycubeComplexView(GlobalState& global_state) {
  CleanupPolycubeComplexView();
  if (global_state.HasPolycubeComplex()) {
    UpdatePolycubeComplexView(
        global_state.GetPolycubeComplex().GetQuadComplex(), true, false);
  }
}

void GlobalView::CleanupPolycubeComplexView() {
  if (polycube_complex_node_ != nullptr) {
    polycube_complex_node_->RemoveFromParent();
    polycube_complex_node_ = nullptr;
  }
}

void GlobalView::DrawNodeVisibilityGui(bool shift_pressed) {
  bool changed = false;
  for (int i = 0; i < (int)visibility_infos_.size(); i++) {
    auto& info = visibility_infos_[i];
    if (info.node != nullptr) {
      if (ImGui::Selectable(FlagToName(info.flag).c_str(), info.visibility)) {
        if (!shift_pressed) {
          ClearVisibility();
        }
        info.visibility ^= 1;
        changed = true;
      }
      if (info.flag == VisibilityFlag::VisibilityInputMesh ||
          info.flag == VisibilityFlag::VisibilityDeformedMesh) {
        // Popups for tri surfaces.
        if (ImGui::BeginPopupContextItem()) {
          bool changed = false;
          if (ImGui::RadioButton("solid", !info.transparent)) {
            info.transparent = false;
            changed = true;
          }
          if (ImGui::RadioButton("transparent", info.transparent)) {
            info.transparent = true;
            info.wireframe = false;
            changed = true;
          }

          if (ImGui::Checkbox("wireframe", &info.wireframe)) {
            changed = true;
          }
          if (changed) {
            UpdateMeshViewByInfo(info);
          }
          ImGui::EndPopup();
        }
      }
    }
  }

  if (changed) {
    UpdateVisibility();
  }
}

void GlobalView::UpdateVisibility() {
  for (auto& info : visibility_infos_) {
    if (info.node != nullptr) {
      info.node->SetVisible(info.visibility);
    }
  }
}

void GlobalView::SetVisibility(unsigned long flags) {
  ClearVisibility();
  for (auto& info : visibility_infos_) {
    if (info.node != nullptr && (info.flag & flags)) {
      info.visibility = true;
    }
  }
  UpdateVisibility();
}

void GlobalView::AddVisibility(unsigned long flags) {
  SetVisibility(GetVisibility() | flags);
}

void GlobalView::RemoveVisibility(unsigned long flags) {
  SetVisibility(GetVisibility() & ~flags);
}

unsigned long GlobalView::GetVisibility() {
  unsigned long mask = 0;
  for (auto& info : visibility_infos_) {
    if (info.node != nullptr && info.visibility) {
      mask |= info.flag;
    }
  }
  return mask;
}

void GlobalView::ClearVisibility() {
  for (auto& info : visibility_infos_) {
    if (info.node != nullptr) {
      info.visibility = false;
    }
  }
}

PolycubeView& GlobalView::GetPolycubeView() {
  assert(polycube_view_ != nullptr);
  return *polycube_view_;
}

void GlobalView::UpdatePolycubeView(GlobalState& global_state) {
  // This always recreates the entire polycube view.
  if (polycube_node_ != nullptr) {
    polycube_node_->RemoveFromParent();
    polycube_node_ = nullptr;
  }
  if (global_state.HasPolycube()) {
    assert(global_state.HasPolycubeInfo());
    auto polycube_node = std::make_unique<vkoo::st::Node>();
    polycube_node_ = polycube_node.get();
    scene_.GetRoot().AddChild(std::move(polycube_node));
    polycube_view_ = std::make_unique<PolycubeView>(device_, polycube_node_);
    polycube_view_->Update(global_state.GetPolycube(),
                           global_state.GetPolycubeInfo());
  }
}

void GlobalView::UpdateFilteredHexView(
    GlobalState& global_state, const FilteringSetting& filtering_setting) {
  if (result_filtered_mesh_node_ != nullptr) {
    result_filtered_mesh_node_->RemoveFromParent();
    result_filtered_mesh_node_ = nullptr;
  }

  if (global_state.HasResultMesh()) {
    // Note: FilteredHexMesh only stores const references of hex mesh so there
    // is no expensive copy involved.
    filtered_hex_mesh_ =
        std::make_unique<FilteredHexMesh>(global_state.GetResultMesh());
    filtered_hex_mesh_->Filter(filtering_setting);

    result_filtered_view_ = std::make_unique<QuadSurfaceView>(
        device_, scene_.GetRoot(), QuadSurfaceView::Options());
    auto extract_result = filtered_hex_mesh_->ExtractFilteredSurfaceMesh(true);
    auto& quad_mesh = *extract_result.quad_mesh;
    QuadComplex quad_complex{quad_mesh.GetVertices(), quad_mesh.GetQuads(),
                             extract_result.patches};
    result_filtered_view_->Update(quad_complex, kFilteredMeshColors);
    result_filtered_mesh_node_ = result_filtered_view_->GetWrapperNode();
  }
}

void GlobalView::ChangeTriMeshAppearance(VisibilityFlag flag, bool transparent,
                                         bool wireframe) {
  auto& info = GetVisibilityInfo(flag);
  info.transparent = transparent;
  info.wireframe = wireframe;
  UpdateMeshViewByInfo(info);
}

GlobalView::VisibilityInfo& GlobalView::GetVisibilityInfo(VisibilityFlag flag) {
  for (auto& info : visibility_infos_) {
    if (info.flag == flag) {
      return info;
    }
  }
  throw std::runtime_error(
      fmt::format("Unidentifiable visibility flag: {}", flag));
}

void GlobalView::UpdateMeshViewByInfo(const VisibilityInfo& info) {
  TriSurfaceView* view;
  if (info.flag == VisibilityFlag::VisibilityInputMesh) {
    view = input_mesh_view_.get();
  } else {
    assert(info.flag == VisibilityFlag::VisibilityDeformedMesh);
    view = deformed_mesh_view_.get();
  }
  view->SetWireframeVisible(info.wireframe);
  view->SetTransparency(info.transparent);
}

}  // namespace hex
