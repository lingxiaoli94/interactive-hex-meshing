#include "HexahedralizationStage.h"

// GUI-only half of HexahedralizationStage; excluded when HEX_ENABLE_VULKAN=OFF.
#if HEX_ENABLE_VULKAN

#include <GLFW/glfw3.h>
#include <imgui.h>

#include <limits>

#include "controllers/GlobalController.h"
#include "logging.h"
#include "utility/ImGuiEx.h"
#include "views/GlobalView.h"
#include "vkoo/core/InputEvent.h"

namespace hex {
void HexahedralizationStage::DrawStageWindow() {
  auto& global_state = global_controller_.GetGlobalState();
  DrawHexahedralizationWindow();
  if (global_state.HasResultMesh()) {
    DrawFilteringWindow();
  }
}

void HexahedralizationStage::DrawHexahedralizationWindow() {
  if (hex_complex_deformer_ && hex_complex_deformer_->GetStatus() ==
                                   HexComplexDeformer::Status::Running) {
    return;
  }

  ImGui::SetNextWindowSize(ImVec2{400, 500}, ImGuiCond_Once);
  ImGui::Begin("Hexahedralization");
  bool pressed;

  pressed = ImGui::Button("Init/Reset final hex mesh");
  ImGui::SameLine();
  ImGuiHelpMarker(
      "Initialize the final hex mesh according to the "
      "polycube hex mesh and the input volumetric mesh.");

  static bool use_pullback = true;
  static bool indirect = true;
  static bool transport = true;
  if (global_controller_.IsExpertMode()) {
    ImGui::Checkbox("pullback", &use_pullback);
    if (use_pullback) {
      ImGui::Checkbox("indirect", &indirect);
      ImGui::SameLine();
      ImGui::Checkbox("gradual", &transport);
    }
    if (in_transition_) {
      if (ImGui::Button("Morph")) {
        OptimizeHexDeformation(100);
      }
      if (ImGui::Button("Finish morphing")) {
        in_transition_ = false;
        hex_complex_deformer_.reset();
      }
    }
  } else {
    ImGui::SameLine();
    static bool inversion_free = true;
    ImGui::Checkbox("inversion-free", &inversion_free);
    ImGui::SameLine();
    ImGuiHelpMarker(
        "A more involved initialization strategy that uses a smooth "
        "deformation to avoid inverting hexes. This might take a while.");
    use_pullback = true;
    indirect = inversion_free;
    transport = inversion_free;
  }

  if (pressed) {
    InitTargetComplex(use_pullback, indirect, transport);
  }

  auto& global_state = global_controller_.GetGlobalState();
  if (global_state.HasTargetComplex() && !in_transition_) {
    if (!is_editing_) {
      if (ImGui::Button("Enter edit mode")) {
        EnterEditMode();
      }
    } else {
      if (ImGui::Button("Leave edit mode")) {
        LeaveEditMode();
      }
    }
    ImGui::SameLine();
    ImGuiHelpMarker(
        "Click to enter/leave edit mode. In edit mode, you can mark vertices "
        "as "
        "landmarks by holding SHIFT and clicking on them. Clicking a second "
        "time will remove a landmark. Landmarks will be fixed during "
        "optimization. You can also drag a landmark along the input mesh "
        "surface while holding down CTRL to reposition it.");
    ImGui::SameLine();
    if (ImGui::Button("Clear landmarks")) {
      ClearLandmarks();
    }

    DrawDistortionEnergyChildWindow(distortion_options_, true, false);

    DrawQualityParametersChildWindow();

    DrawOptimizationControlWindow();

    DrawOptimizerOptionsNode(
        hex_deformer_options_.learning_rate, hex_deformer_options_.adam_betas,
        global_controller_.IsExpertMode() ? &hex_deformer_options_.snapshot_freq
                                          : nullptr);

    pressed = ImGui::Button("Init/Reset optimizer");
    ImGui::SameLine();
    ImGuiHelpMarker(
        "Create a new deformation optimizer based on the current set of "
        "parameters. If an optimizer has been created already, it will be "
        "overriden and all previous optimizer state (e.g. momentum of "
        "gradients) "
        "will be lost, which may or may not be the desired outcome.");
    if (pressed) {
      PrepareHexDeformation();
    }

    if (hex_complex_deformer_) {
      pressed = ImGui::Button("Reoptimize");
      ImGui::SameLine();
      static int opt_steps = 100;
      ImGui::SetNextItemWidth(100);
      if (ImGui::InputInt("steps", &opt_steps)) {
        if (opt_steps < 0) {
          opt_steps = 0;
        }
      }
      if (pressed) {
        OptimizeHexDeformation(opt_steps);
      }
    }
  }
  ImGui::End();
}

void HexahedralizationStage::DrawFilteringWindow() {
  FilteringSetting old_setting = filtering_setting_;

  ImGui::SetNextWindowPos(ImVec2{100, 500}, ImGuiCond_Once);
  ImGui::SetNextWindowSize(ImVec2{400, 300}, ImGuiCond_Once);
  ImGui::Begin("Filters");
  bool pressed = ImGui::Button("Calibrate");
  ImGui::SameLine();
  ImGuiHelpMarker(
      "Set the normal of the filtering plane to be in the same direction as "
      "the camera.");
  if (pressed) {
    filtering_setting_.slice_normal = ToEigen(global_controller_.GetScene()
                                                  .GetActiveCameraPtr()
                                                  ->GetFrontDirection());
  }

  ImGui::SameLine();

  ImGui::SetNextItemWidth(120);
  {
    ImGuiFrameColorGuard frame_cg{305.f / 360.f};
    ImGui::SliderFloat("slice", &filtering_setting_.slice_dist, 0.0f, 1.0f,
                       "%.3f");
  }
  ImGui::SameLine();
  ImGuiHelpMarker("Move the filtering plane forward or backward.");

  const char* labels[] = {"scaled jacobian", "jacobian"};
  const char* tooltips[] = {
      "Scaled Jacobian computes the determinant of the Jacobian matrix after "
      "normalizing each row of the Jacobian. Hence it will not penalize "
      "hexes that are perfect cuboids.",
      "Jacobian computes the determinant of the Jacobian matrix without "
      "normalization, so it measures how volume-preserving each hex element "
      "is."};
  static int item_current = 0;
  const char* combo_label = labels[item_current];

  ImGui::SetNextItemWidth(180);
  if (ImGui::BeginCombo("metric", combo_label, ImGuiComboFlags_None)) {
    for (int n = 0; n < IM_ARRAYSIZE(labels); n++) {
      const bool is_selected = (item_current == n);
      if (ImGui::Selectable(labels[n], is_selected)) {
        item_current = n;
      }

      if (is_selected) ImGui::SetItemDefaultFocus();
      if (ImGui::IsItemHovered()) {
        ImGui::BeginTooltip();
        ImGui::Text("%s", tooltips[n]);
        ImGui::EndTooltip();
      }
    }
    ImGui::EndCombo();
  }
  if (item_current == 0) {
    filtering_setting_.quality = HexQualityType::ScaledJacobian;
  } else {
    filtering_setting_.quality = HexQualityType::Jacobian;
  }
  ImGui::SameLine();
  ImGuiHelpMarker("Choose which metric to show/filter.");
  ImGui::SetNextItemWidth(180);
  {
    ImGuiFrameColorGuard frame_cg{123.f / 360.f};
    ImGui::SliderFloat("quality", &filtering_setting_.quality_cutoff, 0.0f,
                       1.0f, "%.3f");
  }
  ImGui::SameLine();
  ImGuiHelpMarker(
      "Show only hex elements with selected mesh quality >= this value, "
      "normalized to [0, 1].");

  if (filtering_setting_ != old_setting) {
    UpdateFilteredMeshView();
  }

  auto& global_state = global_controller_.GetGlobalState();
  if (global_state.HasResultMesh()) {
    auto& result_mesh = global_state.GetResultMesh();
    auto& quality = result_mesh.GetMeshQuality();
    auto& numbers = quality.GetQuality(filtering_setting_.quality);
    ImGui::BeginChild("Quality", ImVec2(0, 100), false);

    const char* fmt_by_type[] = {"%.3f", "%.3e"};
    const char* fmt =
        fmt_by_type[filtering_setting_.quality == HexQualityType::ScaledJacobian
                        ? 0
                        : 1];
    ImGui::Text(
        fmt::format("Min: {},  Avg: {}, Std: {}", fmt, fmt, fmt).c_str(),
        numbers.minCoeff(), numbers.mean(),
        std::sqrt((numbers.array() - numbers.mean()).square().sum() /
                  std::max((int)numbers.size() - 1, 1)));
    ImGui::Text("#Vertices: %d, #Hexes: %d",
                (int)result_mesh.GetVertices().size(),
                (int)result_mesh.GetHexes().size());
    ImGui::Text("Hausdorff distance (x1000):\n Max: %.3f, Mean: %.3f",
                1000.0f * std::get<0>(hausdorff_distance_),
                1000.0f * std::get<1>(hausdorff_distance_));
    ImGui::EndChild();
  }
  ImGui::End();
}

void HexahedralizationStage::Update([[maybe_unused]] float delta_time) {
  if (hex_complex_deformer_ && hex_complex_deformer_->HasFreshSnapshot()) {
    FetchSnapshot();
  }

  // GUI-only: Update() runs from the frame loop, which headless never enters.
  if (landmarks_view_ &&
      !global_controller_.IsKeyPressed(GLFW_KEY_LEFT_SHIFT) &&
      !global_controller_.IsKeyPressed(GLFW_KEY_LEFT_CONTROL)) {
    landmarks_view_->RemoveHighlightLandmark();
  }
}

void HexahedralizationStage::DrawQualityParametersChildWindow() {
  ImGuiTreeNodeWithTooltip(
      "Mesh quality parameters", ImGuiTreeNodeFlags_DefaultOpen,
      "These parameters control the quality of the resulting hex mesh, "
      "together with distortion parameters.",
      [&] {
        {
          ImGui::SetNextItemWidth(80);
          ImGuiFrameColorGuard frame_cg{5.0f / 7.0f};
          ImGui::DragFloat("smoothness",
                           &hex_deformer_options_.smoothness_weight, 1e-3f,
                           0.0f, 1.0f, "%.3f");
          ImGui::SameLine();
          ImGuiHelpMarker(
              "This determines how the smooth we want the resulting mesh's "
              "surface to be.");
        }
        if (global_controller_.IsExpertMode()) {
          ImGui::SameLine();
          ImGui::SetNextItemWidth(80);
          ImGuiFrameColorGuard frame_cg{5.5f / 7.0f};
          ImGui::DragFloat("wire", &hex_deformer_options_.fairness_weight,
                           1e-3f, 0.0f, 1.0f, "%.3f");
          ImGui::SameLine();
          ImGuiHelpMarker(
              "This determines how much we want to preserve cube structure.");
        }
        {
          ImGui::SetNextItemWidth(80);
          ImGuiFrameColorGuard frame_cg{6.0f / 7.0f};
          ImGui::DragFloat("projection",
                           &hex_deformer_options_.projection_weight, 1.0f, 0.0f,
                           1000.f, "%.2f", ImGuiSliderFlags_Logarithmic);
          ImGui::SameLine();
          ImGuiHelpMarker(
              "This determines how close we want the surface of the resulting "
              "mesh "
              "to be with the surface of the input mesh. This is always in "
              "log scale.");
        }
        ImGui::SameLine();
        {
          ImGui::SetNextItemWidth(80);
          ImGuiFrameColorGuard frame_cg{6.5f / 7.0f};
          ImGui::DragFloat("details", &hex_deformer_options_.hausdorff_weight,
                           1.0f, 0.0f, 1000.f, "%.2f",
                           ImGuiSliderFlags_Logarithmic);
          ImGui::SameLine();
          ImGuiHelpMarker(
              "This determines how close we want the input surface to be "
              "within the resulting mesh "
              "This is always in log scale.");
        }

        {
          const char* labels[] = {"none", "scaled jacobian"};
          const char* tooltips[] = {"No custom energy.",
                                    "Scaled Jacobian computes the determinant "
                                    "of the Jacobian matrix after "
                                    "normalizing each row of the Jacobian. "
                                    "Hence it will not penalize "
                                    "hexes that are perfect cuboids."};
          static int item_current = 0;
          const char* combo_label = labels[item_current];

          ImGui::SetNextItemWidth(120);
          if (ImGui::BeginCombo("custom", combo_label, ImGuiComboFlags_None)) {
            for (int n = 0; n < IM_ARRAYSIZE(labels); n++) {
              const bool is_selected = (item_current == n);
              if (ImGui::Selectable(labels[n], is_selected)) {
                item_current = n;
              }

              if (is_selected) ImGui::SetItemDefaultFocus();
              if (ImGui::IsItemHovered()) {
                ImGui::BeginTooltip();
                ImGui::Text("%s", tooltips[n]);
                ImGui::EndTooltip();
              }
            }
            ImGui::EndCombo();
          }

          ImGui::SameLine();
          ImGui::SetNextItemWidth(70);
          ImGui::DragFloat("weight", &hex_deformer_options_.custom_weight,
                           1e-3f, 0.0f, 1.0f, "%.3f");
          ImGui::SameLine();
          ImGuiHelpMarker("Weight of the custom energy.");

          ImGui::SetNextItemWidth(100);
          ImGui::Checkbox("exp scaling",
                          &hex_deformer_options_.custom_exp_scaling);
          ImGui::SameLine();
          ImGuiHelpMarker(
              "Whether to apply exponential scaling to custom metric in "
              "order "
              "to improve worst element.");

          HexComplexDeformer::CustomQuality custom_quality;
          if (item_current == 0) {
            custom_quality = HexComplexDeformer::CustomQuality::None;
          } else if (item_current == 1) {
            custom_quality = HexComplexDeformer::CustomQuality::ScaledJacobian;
          } else {
            throw std::runtime_error("Unknown custom quality");
          }
          hex_deformer_options_.custom_quality = custom_quality;
        }
      });
}

void HexahedralizationStage::DrawOptimizationControlWindow() {
  {
    const char* labels[] = {"free", "fixed", "constrained"};
    const char* tooltips[] = {
        "Allow surface vertices to move off the input mesh surface during "
        "optimization, with an additional term penalizing the deviation.",
        "Fix the position of the surface vertices during optimization.",
        "Reparameterize the positions of surface vertices so that they always "
        "stay on the surface during optimization.\nInternally the surface "
        "vertices are allowed to move off the surface, and then a "
        "differentiable projection is applied."};
    static int item_current = 0;
    HexComplexDeformer::Mode mode;
    ImGui::SetNextItemWidth(160);
    const char* combo_label = labels[item_current];
    if (ImGui::BeginCombo("surface representation", combo_label,
                          ImGuiComboFlags_None)) {
      for (int n = 0; n < IM_ARRAYSIZE(labels); n++) {
        const bool is_selected = (item_current == n);
        if (ImGui::Selectable(labels[n], is_selected)) {
          item_current = n;
        }

        if (is_selected) ImGui::SetItemDefaultFocus();
        if (ImGui::IsItemHovered()) {
          ImGui::BeginTooltip();
          ImGui::Text("%s", tooltips[n]);
          ImGui::EndTooltip();
        }
      }
      ImGui::EndCombo();
    }
    ImGui::SameLine();
    ImGuiHelpMarker(
        "Choose how the surface vertices are represented in optimization.");
    if (item_current == 0) {
      mode = HexComplexDeformer::Mode::Projection;
    } else if (item_current == 1) {
      mode = HexComplexDeformer::Mode::FixedBoundary;
    } else {
      mode = HexComplexDeformer::Mode::LiftedProjection;
    }
    hex_deformer_options_.mode = mode;
  }
}

bool HexahedralizationStage::HandleInputEvent(const vkoo::InputEvent& event) {
  bool handled = false;

  if (event.GetSource() == vkoo::EventSource::Keyboard) {
    auto& key_event = static_cast<const vkoo::KeyInputEvent&>(event);
    if (key_event.GetAction() == vkoo::KeyAction::Down &&
        key_event.GetCode() == GLFW_KEY_ESCAPE) {
      LOGI("ESC pressed!");
      StopAndJoinOptThread();
    }
  }

  // The rest is handling events in editing mode.
  if (!is_editing_) {
    return handled;
  }

  bool shift_pressed = global_controller_.IsKeyPressed(GLFW_KEY_LEFT_SHIFT);
  bool ctrl_pressed = global_controller_.IsKeyPressed(GLFW_KEY_LEFT_CONTROL);
  if (event.GetSource() == vkoo::EventSource::Mouse) {
    auto& mouse_event = static_cast<const vkoo::MouseButtonInputEvent&>(event);

    if (shift_pressed ^ ctrl_pressed) {
      // Trigger this when exactly one of SHIFT, CTRL is pressed.
      handled = true;

      if (mouse_event.GetAction() == vkoo::MouseAction::Move) {
        // Terminate early if lagged behind to avoid the expensive ray tracing.
        float cur_x_pos;
        float cur_y_pos;
        global_controller_.GetMousePosition(cur_x_pos, cur_y_pos);
        Vector2f delta =
            Vector2f{mouse_event.GetXPos(), mouse_event.GetYPos()} -
            Vector2f{cur_x_pos, cur_y_pos};
        if (delta.squaredNorm() > 1e-8f) {
          return handled;
        }
      }
      Ray world_ray = global_controller_.ShootRayAtMousePosition(
          mouse_event.GetXPos(), mouse_event.GetYPos());
      size_t v;

      bool intersected = false;
      if (landmarks_view_->IntersectWithRay(world_ray, v)) {
        // Prioritize intersecting landmark balls.
        intersected = true;
      } else {
        QuadrilateralMesh::HitRecord hit_record;
        intersected =
            current_surface_mesh_->IntersectWithRay(world_ray, hit_record);

        if (intersected) {
          v = FindClosestVertexOnQuad(world_ray.At(hit_record.t),
                                      hit_record.quad_id);
        }
      }

      if (intersected) {
        auto& result_mesh_vertices =
            global_controller_.GetGlobalState().GetResultMesh().GetVertices();

        if (shift_pressed) {
          landmarks_view_->HighlightLandmark(result_mesh_vertices.at(v));

          if (mouse_event.GetAction() == vkoo::MouseAction::Down) {
            if (landmarks_set_.count(v)) {
              landmarks_view_->RemoveLandmark(v);
              landmarks_set_.erase(v);
            } else {
              landmarks_view_->AddLandmark(v, result_mesh_vertices.at(v));
              landmarks_set_.insert(v);
            }
          }
        } else if (ctrl_pressed && !is_being_dragged_) {
          if (landmarks_set_.count(v)) {
            landmarks_view_->HighlightLandmark(result_mesh_vertices.at(v));

            if (mouse_event.GetAction() == vkoo::MouseAction::Down) {
              is_being_dragged_ = true;
              dragged_vtx_id_ = v;
              global_controller_.GetGlobalView()->ChangeTriMeshAppearance(
                  VisibilityInputMesh, false,
                  false);
              global_controller_.AddVisibility(
                  VisibilityInputMesh);
              pickable_surface_view_->GetWrapperNode()->SetVisible(false);
            }
          }
        }
      }

      if (ctrl_pressed && is_being_dragged_) {
        auto& target_mesh = global_controller_.GetGlobalState()
                                .GetTargetVolumeMesh()
                                .GetSurfaceMesh();
        TriangularMesh::HitRecord hit_record;
        bool intersected = target_mesh.IntersectWithRay(world_ray, hit_record);
        if (intersected) {
          auto intersection = world_ray.At(hit_record.t);
          dragged_vtx_position_ = intersection;
          landmarks_view_->MoveLandmark(dragged_vtx_id_, intersection);
        }

        if (mouse_event.GetAction() == vkoo::MouseAction::Up) {
          is_being_dragged_ = false;
          auto& target_complex =
              global_controller_.GetGlobalState().GetTargetComplex();
          target_complex.UpdateSingleVertex(dragged_vtx_id_,
                                            dragged_vtx_position_);
          deformer_dirty_ = true;
          UpdateOnTargetComplexChange();
          global_controller_.RemoveVisibility(
              VisibilityInputMesh);
          pickable_surface_view_->GetWrapperNode()->SetVisible(true);
        }
      }
    }
  }
  return handled;
}

size_t HexahedralizationStage::FindClosestVertexOnQuad(const Vector3f& p,
                                                       size_t quad_id) {
  auto& vertices = current_surface_mesh_->GetVertices();
  auto& quad = current_surface_mesh_->GetQuads().at(quad_id);

  size_t result;
  float min_d = std::numeric_limits<float>::max();
  for (size_t k = 0; k < 4; k++) {
    float d = (p - vertices.at(quad[k])).norm();
    if (d < min_d) {
      min_d = d;
      result = quad[k];
    }
  }
  auto& target_complex = global_controller_.GetGlobalState().GetTargetComplex();
  return target_complex.GetSurfaceIndices().at(result);
}

void HexahedralizationStage::EnterEditMode() {
  assert(current_surface_mesh_);
  assert(!is_editing_);
  is_editing_ = true;
  // First hide everything. All four functions below are GUI-only (edit mode is
  // entered from the stage window), so the view is guaranteed to exist.
  saved_visibility_flag_ = global_controller_.GetVisibility();
  pickable_surface_view_->GetWrapperNode()->SetVisible(true);
  landmarks_view_->GetWrapperNode()->SetVisible(true);

  global_controller_.GetGlobalView()->ChangeTriMeshAppearance(
      VisibilityInputMesh, true, false);
  global_controller_.AddVisibility(
      VisibilityInputMesh);
  global_controller_.RemoveVisibility(
      VisibilityResultFilteredMesh);
}

void HexahedralizationStage::LeaveEditMode() {
  assert(is_editing_);
  is_editing_ = false;
  pickable_surface_view_->GetWrapperNode()->SetVisible(false);
  landmarks_view_->GetWrapperNode()->SetVisible(false);
  global_controller_.AddVisibility(
      VisibilityResultFilteredMesh);
}

void HexahedralizationStage::ShowGuideSurface() {
  global_controller_.GetGlobalView()->ChangeTriMeshAppearance(
      VisibilityInputMesh, true, false);
  global_controller_.AddVisibility(
      VisibilityInputMesh);
}

void HexahedralizationStage::HideGuideSurface() {
  global_controller_.RemoveVisibility(
      VisibilityInputMesh);
}
}  // namespace hex

#endif  // HEX_ENABLE_VULKAN
