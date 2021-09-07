#include "DiscretizationStage.h"

#include <imgui.h>
#include <models/PolycubeGraph.h>

#include "GLFW/glfw3.h"
#include "controllers/GlobalController.h"
#include "logging.h"
#include "utility/ImGuiEx.h"
#include "utility/StopWatch.h"
#include "vkoo/core/InputEvent.h"

namespace hex {
namespace {
const glm::vec4 kDiggingHexColor = {0.8f, 0.2f, 0.1f, 0.3f};
const glm::vec4 kExtrusionHexColor = {0.2f, 0.8f, 0.2f, 0.3f};
}
DiscretizationStage::DiscretizationStage(GlobalController& global_controller)
    : PipelineStage(global_controller, GlobalView::VisibilityDecomposition) {}

void hex::DiscretizationStage::DrawStageWindow() {
  ImGui::SetNextWindowSize(ImVec2{300, 200}, ImGuiCond_Once);
  ImGui::Begin("Discretization");

  ImGui::SetNextItemWidth(100);
  {
    ImGuiFrameColorGuard frame_cg{2.0f / 7.0f};
    ImGui::DragFloat("hex size", &hex_size_, 0.001f, 0.01f, FLT_MAX, "%.3f");
  }
  ImGui::SameLine();
  ImGuiHelpMarker(
      "Edge length of a hex element in the discretized polycube. Smaller value "
      "leads to greater number of hexes.");

  ImGui::SameLine();
  ImGui::Checkbox("round", &round_to_nearest_);
  ImGui::SameLine();
  ImGuiHelpMarker(
      "Whether to round to nearest integer during discretization. If not, each "
      "cuboid will be expanded.");

  bool pressed = ImGui::Button("Discretize");
  ImGui::SameLine();
  ImGuiHelpMarker("Discretize the polycube with given hex element size.");
  if (pressed) {
    DiscretizePolycube();
  }
  ImGui::SameLine();
  ImGui::Checkbox("colored", &color_by_patch_);
  ImGui::SameLine();
  ImGuiHelpMarker(
      "Show each patch using a color corresponding to its orientation.");

  if (current_graph_) {
    {
      if (ImGui::RadioButton("Dig", mode_ == 0)) {
        mode_ = 0;
      };
      ImGui::SameLine();
      if (ImGui::RadioButton("Extrude", mode_ == 1)) {
        mode_ = 1;
      };
      ImGui::SameLine();
      ImGuiHelpMarker(
          "Edit the polycube by holding LEFT-SHIFT and click and drag on the "
          "surface. Dig mode will destroy existing hexes while Extrude mode "
          "will add a new layer of hexes.\n You can press Z to toggle as a "
          "shortcut.\n You may additionally hold LEFT-CTRL to edit an entire "
          "patch.");
    }

    pressed = ImGui::Button("Finalize polycube");
    ImGui::SameLine();
    ImGuiHelpMarker(
        "Build a hex mesh for the polycube that will later be mapped to the "
        "input mesh's surface.");
    ImGui::SameLine();
    ImGui::Checkbox("padding", &padding_);
    ImGui::SameLine();
    ImGuiHelpMarker(
        "Determine whether to add an extra layer of global padding. This can "
        "improve final mesh quality in many instances.");

    if (pressed) {
      FinalizePolycube();
    }
  }
  ImGui::End();
}

bool DiscretizationStage::CanSwitchTo() {
  auto& global_state = global_controller_.GetGlobalState();
  if (!global_state.HasPolycube()) {
    LOGW("Polycube must exist before discretization stage!");
    return false;
  }
  return true;
}

void DiscretizationStage::DiscretizePolycube() {
  auto& global_state = global_controller_.GetGlobalState();
  auto& polycube = global_state.GetPolycube();
  current_graph_ =
      std::make_unique<PolycubeGraph>(polycube, hex_size_, round_to_nearest_);
  current_quad_complex_ = current_graph_->GenerateQuadComplex();
  current_surface_mesh_ = current_quad_complex_->ExtractQuadMesh();

  auto& global_view = global_controller_.GetGlobalView();
  global_view.UpdatePolycubeComplexView(*current_quad_complex_, color_by_patch_,
                                        true);
  global_view.SetVisibility(GlobalView::VisibilityPolycubeComplex);
}

void DiscretizationStage::FinalizePolycube() {
  assert(current_graph_);
  auto& global_state = global_controller_.GetGlobalState();
  global_state.SetPolycubeComplex(current_graph_->GenerateHexComplex(padding_));

  auto& global_view = global_controller_.GetGlobalView();
  global_view.UpdatePolycubeComplexView(*current_graph_->GenerateQuadComplex(),
                                        color_by_patch_, false);
  global_view.SetVisibility(GlobalView::VisibilityPolycubeComplex);

  // Invalidate target complex and hex mesh.
  global_state.SetTargetComplex(nullptr);
  global_controller_.ResetHexahedralizationStage();
}

void DiscretizationStage::SwitchTo() { PipelineStage::SwitchTo(); }
void DiscretizationStage::SwitchFrom() { PipelineStage::SwitchFrom(); }

bool DiscretizationStage::HandleInputEvent(const vkoo::InputEvent& event) {
  static StopWatch stop_watch;
  static double time_elapsed = 0;

  bool handled = false;
  if (!current_surface_mesh_) {
    return handled;
  }
  if (event.GetSource() == vkoo::EventSource::Keyboard) {
    auto& key_event = static_cast<const vkoo::KeyInputEvent&>(event);
    if (key_event.GetAction() == vkoo::KeyAction::Down &&
        key_event.GetCode() == GLFW_KEY_Z) {
      mode_ ^= 1;
      handled = true;
    }
  }
  if (event.GetSource() == vkoo::EventSource::Mouse) {
    bool shift_pressed = global_controller_.IsKeyPressed(GLFW_KEY_LEFT_SHIFT);
    auto& mouse_event = static_cast<const vkoo::MouseButtonInputEvent&>(event);
    if (status_ == Status::Idle) {
      if (mouse_event.GetAction() == vkoo::MouseAction::Down && shift_pressed) {
        handled = true;
        time_elapsed = 0;
        if (mode_ == 0) {
          status_ = Status::Digging;
          pending_view_ = std::make_unique<HexCollectionView>(
              global_controller_.GetDevice(),
              global_controller_.GetScene().GetRoot(),
              current_graph_->GetHexSize(), kDiggingHexColor);
        } else {
          status_ = Status::Extrusion;
          pending_view_ = std::make_unique<HexCollectionView>(
              global_controller_.GetDevice(),
              global_controller_.GetScene().GetRoot(),
              current_graph_->GetHexSize(), kExtrusionHexColor);
        }
      }
    }

    if (status_ != Status::Idle) {
      if (shift_pressed &&
          (mouse_event.GetAction() == vkoo::MouseAction::Move ||
           mouse_event.GetAction() == vkoo::MouseAction::Down)) {
        Ray world_ray = global_controller_.ShootRayAtMousePosition(
            mouse_event.GetXPos(), mouse_event.GetYPos());

        QuadrilateralMesh::HitRecord hit_record;
        stop_watch.Tic();
        bool intersected =
            current_surface_mesh_->IntersectWithRay(world_ray, hit_record);
        time_elapsed += stop_watch.Toc().count();
        if (intersected) {
          HandleQuadClicked(hit_record.quad_id);
          handled = true;
        }
      }

      if (!shift_pressed || mouse_event.GetAction() == vkoo::MouseAction::Up) {
        ApplyPendingChanges();
        status_ = Status::Idle;
        handled = true;
        // LOGI("Ray tracing takes {} secs", time_elapsed);
      }
    }
  }
  return handled;
}

void DiscretizationStage::HandleQuadClicked(size_t quad_id) {
  std::vector<Vector3i> hex_coords;
  if (status_ == Status::Digging) {
    if (global_controller_.IsKeyPressed(GLFW_KEY_LEFT_CONTROL)) {
      auto hex_ids = current_graph_->GetHexesOnSamePatch(quad_id);
      for (auto hex_id : hex_ids) {
        hex_coords.push_back(current_graph_->GetHexCoord(hex_id));
      }
    } else {
      size_t hex_id = current_graph_->GetHexIdFromQuadId(quad_id);
      hex_coords.push_back(current_graph_->GetHexCoord(hex_id));
    }

  } else {
    assert(status_ == Status::Extrusion);
    if (global_controller_.IsKeyPressed(GLFW_KEY_LEFT_CONTROL)) {
      auto new_hex_coords = current_graph_->GetExtrusionPatch(quad_id);
      hex_coords.insert(hex_coords.end(), new_hex_coords.begin(),
                        new_hex_coords.end());
    } else {
      hex_coords.push_back(current_graph_->GetExtrusionCoord(quad_id));
    }
  }

  for (auto& hex_coord : hex_coords) {
    auto itr = pending_hex_coords_.find(hex_coord);
    if (itr == pending_hex_coords_.end()) {
      pending_hex_coords_.insert(hex_coord);
      pending_view_->AddHex(hex_coord, false, true);
    }
  }
}

void DiscretizationStage::ApplyPendingChanges() {
  if (pending_hex_coords_.empty()) {
    return;
  }
  // LOGI("Applying {} pending changes to ", pending_hex_coords_.size());
  assert(status_ != Status::Idle);

  auto old_hex_coords = current_graph_->GetAllHexCoords();
  std::unordered_set<Vector3i, Vector3iHasher> new_hex_coords{
      old_hex_coords.begin(), old_hex_coords.end()};
  for (auto& c : pending_hex_coords_) {
    auto itr = new_hex_coords.find(c);
    if (status_ == Status::Digging) {
      assert(itr != new_hex_coords.end());

      new_hex_coords.erase(itr);
    } else {
      assert(itr == new_hex_coords.end());
      new_hex_coords.insert(c);
    }
  }

  // Rebuild everything.
  pending_view_.reset();

  float old_hex_size = current_graph_->GetHexSize();
  current_graph_ = std::make_unique<PolycubeGraph>(
      std::vector<Vector3i>(new_hex_coords.begin(), new_hex_coords.end()),
      old_hex_size);
  current_quad_complex_ = current_graph_->GenerateQuadComplex();
  current_surface_mesh_ = current_quad_complex_->ExtractQuadMesh();
  auto& global_view = global_controller_.GetGlobalView();
  global_view.UpdatePolycubeComplexView(*current_quad_complex_, color_by_patch_,
                                        true);
  global_view.AddVisibility(GlobalView::VisibilityPolycubeComplex);

  pending_hex_coords_.clear();
}
}  // namespace hex
