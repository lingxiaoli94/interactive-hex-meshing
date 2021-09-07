#include <GLFW/glfw3.h>
#include <imgui.h>
#include <vkoo/st/components/Tracing.h>

#include "DecompositionStage.h"
#include "controllers/GlobalController.h"
#include "logging.h"
#include "utility/ImGuiEx.h"
#include "vkoo/core/InputEvent.h"

namespace hex {
namespace {
const float kDuplicateOffset = 0.01f;
const float kEps = 1e-8f;

bool IsCuboidBoundsEmpty(const std::pair<Vector3f, Vector3f>& bounds) {
  return !(bounds.first(0) + kEps < bounds.second(0) &&
           bounds.first(1) + kEps < bounds.second(1) &&
           bounds.first(2) + kEps < bounds.second(2));
}
}  // namespace
DecompositionStage::DecompositionStage(GlobalController& global_controller)
    : PipelineStage(global_controller, GlobalView::VisibilityDeformedMesh),
      cuboid_editing_controller_{global_controller, false} {
  polycube_optimizer_ =
      std::make_unique<PolycubeOptimizer>(PolycubeOptimizer::Options());
}

void DecompositionStage::DrawStageWindow() {
  if (!polycube_optimizer_->IsIdle()) {
    return;
  }
  ImGui::SetNextWindowSize(ImVec2(400, 400), ImGuiCond_Once);
  ImGui::Begin("Decomposition");
  DrawAnchorsSdfChildWindow();
  DrawPolycubeControlChildWindow();
  ImGui::End();

  if (cuboid_editing_controller_.GetState() !=
      CuboidEditingController::State::Idle) {
    CuboidEditingController::WindowState cuboid_state;
    auto& polycube_info = global_controller_.GetGlobalState().GetPolycubeInfo();
    cuboid_state.lock_cuboid = polycube_info.locked.at(focused_cuboid_index_);
    cuboid_editing_controller_.DrawWindow(cuboid_state);
    if (cuboid_state.lock_cuboid) {
      // This means the button is pressed, so toggle.
      polycube_info.ToggleLock(focused_cuboid_index_);
    }

    if (cuboid_state.delete_cuboid) {
      assert(focused_cuboid_index_ != -1);
      DeleteCuboid(focused_cuboid_index_);
    } else if (cuboid_state.duplicate_cuboid) {
      assert(focused_cuboid_index_ != -1);
      DuplicateCuboid(focused_cuboid_index_);
    }
  } else if (subtract_editing_controller_) {
    assert(subtract_editing_controller_->GetState() !=
           CuboidEditingController::State::Idle);
    CuboidEditingController::WindowState cuboid_state;
    subtract_editing_controller_->DrawWindow(cuboid_state);
    if (cuboid_state.subtract_cuboid) {
      PerformSubtraction();
    }
  }
  DrawPolycubeInfoWindow();
}

void DecompositionStage::DrawPolycubeInfoWindow() {
  if (!global_controller_.GetGlobalState().HasPolycube() ||
      global_controller_.GetGlobalState().GetPolycube().GetCuboidCount() == 0) {
    return;
  }

  ImGui::SetNextWindowPos(ImVec2{100, 400}, ImGuiCond_Once);
  ImGui::SetNextWindowSize(ImVec2{250, 240}, ImGuiCond_Once);
  ImGui::Begin("Polycube Info");

  auto& global_state = global_controller_.GetGlobalState();
  auto& polycube_info = global_state.GetPolycubeInfo();

  if (ImGui::Button("Up")) {
    if (focused_cuboid_index_ != -1) {
      polycube_info.MoveItem(focused_cuboid_index_, -1);
    }
  }
  ImGui::SameLine();
  if (ImGui::Button("Down")) {
    if (focused_cuboid_index_ != -1) {
      polycube_info.MoveItem(focused_cuboid_index_, 1);
    }
  }
  ImGui::SameLine();
  if (ImGui::Button("Lock/Unlock")) {
    if (focused_cuboid_index_ != -1) {
      polycube_info.ToggleLock(focused_cuboid_index_);
    }
  }
  ImGui::SameLine();
  ImGuiHelpMarker("Lock a cuboid to prevent it from being optimized.");

  ImGui::BeginChild("CuboidList", ImVec2{0, 0}, true);
  for (int i = 0; i < polycube_info.names.size(); i++) {
    ImGui::PushID(i);
    int j = polycube_info.ordering[i];
    bool clicked = ImGui::SelectableInput(
        "entry", j == focused_cuboid_index_, ImGuiSelectableFlags_None,
        polycube_info.names[j].buf, IM_ARRAYSIZE(polycube_info.names[j].buf));
    ImGui::SameLine(200);
    ImGui::Text("%s", polycube_info.locked[j] ? "L" : " ");
    if (clicked && j != focused_cuboid_index_) {
      FocusCuboid(j);
    }
    ImGui::PopID();
  }
  ImGui::EndChild();
  ImGui::End();
}

bool DecompositionStage::CanSwitchTo() {
  auto& global_state = global_controller_.GetGlobalState();
  if (!global_state.HasDeformedVolumeMesh()) {
    LOGW(
        "Deformed mesh is required for cuboid "
        "decomposition stage!");
    return false;
  }
  return true;
}

void DecompositionStage::SwitchTo() { PipelineStage::SwitchTo(); }

void DecompositionStage::Update([[maybe_unused]] float delta_time) {
  if (polycube_optimizer_->GetStatus() == PolycubeOptimizer::Status::Running ||
      polycube_optimizer_->GetStatus() ==
          PolycubeOptimizer::Status::Completed) {
    auto& global_state = global_controller_.GetGlobalState();
    global_state.SetPolycube(
        std::make_unique<Polycube>(polycube_optimizer_->GetSnapshot()));

    auto& polycube_view = global_controller_.GetGlobalView().GetPolycubeView();
    polycube_view.Update(global_state.GetPolycube(),
                         global_state.GetPolycubeInfo());

    if (polycube_optimizer_->GetStatus() ==
        PolycubeOptimizer::Status::Completed) {
      polycube_optimizer_->SetIdle();
    }
  }
}

void DecompositionStage::Reoptimize(size_t num_steps) {
  UnfocusCuboid();

  auto& deformed_mesh =
      global_controller_.GetGlobalState().GetDeformedVolumeMesh();
  auto& global_state = global_controller_.GetGlobalState();
  auto& polycube = global_state.GetPolycube();
  auto& polycube_info = global_state.GetPolycubeInfo();
  opt_thread_ = std::thread([&, num_steps]() {
    polycube_optimizer_->Optimize(deformed_mesh, polycube, num_steps,
                                  polycube_info.locked);
  });
}

void DecompositionStage::SuggestNewCuboid(
    PolycubeOptimizer::SuggestStrategy strategy) {
  UnfocusCuboid();

  auto& deformed_mesh =
      global_controller_.GetGlobalState().GetDeformedVolumeMesh();
  auto& polycube = global_controller_.GetGlobalState().GetPolycube();
  Cuboid new_cuboid;
  bool success = polycube_optimizer_->SuggestNewCuboid(deformed_mesh, polycube,
                                                       strategy, new_cuboid);
  if (success) {
    AddNewCuboid(new_cuboid);
  } else {
    LOGI("Failed to suggest a new cuboid!");
  }
}

void DecompositionStage::AddNewCuboid(const Cuboid& cuboid) {
  auto& global_state = global_controller_.GetGlobalState();
  if (!global_state.HasPolycube()) {
    LOGW("You must initialize the polycube first!");
    return;
  }

  assert(global_state.CheckPolycubeConsistency());
  auto& polycube = global_state.GetPolycube();
  polycube.AddCuboid(cuboid);

  auto& polycube_info = global_state.GetPolycubeInfo();
  polycube_info.Push();

  auto& polycube_view = global_controller_.GetGlobalView().GetPolycubeView();
  polycube_view.AddCuboid(cuboid);

  FocusCuboid(polycube.GetCuboidCount() - 1);
}

void DecompositionStage::DuplicateCuboid(int i) {
  auto& global_state = global_controller_.GetGlobalState();
  auto& polycube = global_state.GetPolycube();

  Cuboid cuboid = polycube.GetCuboid(i);
  // Add in slight offset.
  cuboid.center += kDuplicateOffset * Vector3f::Ones();
  AddNewCuboid(cuboid);
}

void DecompositionStage::DeleteCuboid(int i) {
  if (i == focused_cuboid_index_) {
    UnfocusCuboid();
  }
  auto& global_state = global_controller_.GetGlobalState();

  auto& polycube = global_state.GetPolycube();
  polycube.DeleteCuboid(i);

  auto& polycube_info = global_state.GetPolycubeInfo();
  polycube_info.Delete(i);

  auto& polycube_view = global_controller_.GetGlobalView().GetPolycubeView();
  polycube_view.DeleteCuboid(i);
}

void DecompositionStage::ResetPolycube() {
  UnfocusCuboid();
  global_controller_.GetGlobalState().SetPolycube(std::make_unique<Polycube>());
  global_controller_.GetGlobalState().SetPolycubeInfo(
      std::make_unique<PolycubeInfo>());
  global_controller_.GetGlobalView().UpdatePolycubeView(
      global_controller_.GetGlobalState());
  global_controller_.GetGlobalView().SetVisibility(
      GlobalView::VisibilityDeformedMesh | GlobalView::VisibilityDecomposition);
}

bool DecompositionStage::HandleInputEvent(const vkoo::InputEvent& event) {
  if (!polycube_optimizer_->IsIdle()) {
    return false;
  }
  bool handled = false;
  if (cuboid_editing_controller_.GetState() !=
      CuboidEditingController::State::Idle) {
    handled = cuboid_editing_controller_.HandleInputEvent(event);
  } else if (subtract_editing_controller_ != nullptr) {
    handled = subtract_editing_controller_->HandleInputEvent(event);
  } else {
    if (event.GetSource() == vkoo::EventSource::Mouse) {
      auto& mouse_event =
          static_cast<const vkoo::MouseButtonInputEvent&>(event);

      // Ray in world space.
      vkoo::st::Ray world_ray =
          global_controller_
              .ShootRayAtMousePosition(mouse_event.GetXPos(),
                                       mouse_event.GetYPos())
              .ToGlm();

      auto traceables = global_controller_.GetScene()
                            .GetRoot()
                            .GetComponentsRecursive<vkoo::st::Tracing>(true);
      vkoo::st::HitRecord hit_record{};
      vkoo::st::Tracing* hit_obj = nullptr;
      for (auto traceable : traceables) {
        vkoo::st::Ray local_ray = world_ray;
        local_ray.ApplyTransform(
            traceable->GetNode()->GetTransform().GetWorldToLocalMatrix());
        if (traceable->GetHittable().Intersect(local_ray, hit_record)) {
          hit_obj = traceable;
        }
      }

      if (hit_obj != nullptr) {
        auto& cuboid_nodes = global_controller_.GetGlobalView()
                                 .GetPolycubeView()
                                 .GetCuboidNodes();
        // Check if hit object is a cuboid.
        auto it = std::find(cuboid_nodes.begin(), cuboid_nodes.end(),
                            hit_obj->GetNode());
        if (it != cuboid_nodes.end()) {
          if (mouse_event.GetAction() == vkoo::MouseAction::Down &&
              mouse_event.GetButton() == vkoo::MouseButton::Left) {
            FocusCuboid(std::distance(cuboid_nodes.begin(), it));
            handled = true;
          }
        } else {
          // If anything other than a cuboid is hit, then treat as if not
          // hit.
          hit_obj = nullptr;
        }
      }
    }
  }
  if (event.GetSource() == vkoo::EventSource::Keyboard) {
    auto& key_event = static_cast<const vkoo::KeyInputEvent&>(event);
    if (key_event.GetAction() == vkoo::KeyAction::Down) {
      if (key_event.GetCode() == GLFW_KEY_ESCAPE) {
        UnfocusCuboid();
      } else if (key_event.GetCode() == GLFW_KEY_BACKSPACE) {
        if (focused_cuboid_index_ != -1) {
          DeleteCuboid(focused_cuboid_index_);
          assert(cuboid_editing_controller_.GetState() ==
                 CuboidEditingController::State::Idle);
        }
      } else if (key_event.GetCode() == GLFW_KEY_L) {
        if (focused_cuboid_index_ != -1) {
          auto& polycube_info =
              global_controller_.GetGlobalState().GetPolycubeInfo();
          polycube_info.ToggleLock(focused_cuboid_index_);
        }
      }
    }
  }
  return handled;
}

void DecompositionStage::FocusCuboid(int id) {
  if (focused_cuboid_index_ != -1) {
    UnfocusCuboid();
  }

  focused_cuboid_index_ = id;
  auto& cuboid_node =
      global_controller_.GetGlobalView().GetPolycubeView().GetCuboidNode(id);
  cuboid_node.UpdateMode(CuboidNode::Mode::Focused);
  cuboid_editing_controller_.Focus(
      cuboid_node,
      global_controller_.GetGlobalState().GetPolycube().GetCuboid(id));

  global_controller_.GetGlobalView().AddVisibility(
      GlobalView::VisibilityDecomposition);
}

void DecompositionStage::UnfocusCuboid() {
  if (opt_thread_.joinable()) {
    opt_thread_.join();
  }

  if (subtract_editing_controller_) {
    subtract_editing_controller_.reset();  // destructor unfocuses automatically

    // Note: deletion of tmp cuboid must happen before calling the destructor of
    // subtract_editing_controller_!
    global_controller_.GetGlobalView().GetPolycubeView().DeleteTmpCuboid();
  }

  if (focused_cuboid_index_ == -1) {
    return;
  }
  cuboid_editing_controller_.Unfocus();
  auto& cuboid_node =
      global_controller_.GetGlobalView().GetPolycubeView().GetCuboidNode(
          focused_cuboid_index_);
  bool locked = global_controller_.GetGlobalState().GetPolycubeInfo().locked.at(
      focused_cuboid_index_);
  cuboid_node.UpdateMode(locked ? CuboidNode::Mode::Locked
                                : CuboidNode::Mode::Free);
  focused_cuboid_index_ = -1;
}

DecompositionStage::~DecompositionStage() {
  CuboidNode::Cleanup();  // TODO: fix this
  if (opt_thread_.joinable()) {
    polycube_optimizer_->Stop();
    opt_thread_.join();
  }
}

void DecompositionStage::SwitchFrom() {
  PipelineStage::SwitchFrom();
  if (opt_thread_.joinable()) {
    polycube_optimizer_->Stop();
    opt_thread_.join();
  }
  UnfocusCuboid();
}

void DecompositionStage::CreateSdfAndAnchors() {
  LOGI("Creating anchors and sdf for the deformed mesh...");
  auto& global_state = global_controller_.GetGlobalState();
  if (!global_state.HasDeformedVolumeMesh()) {
    LOGW("Cannot create anchors and sdfs without a deformed mesh!");
    return;
  }
  // Note: order matters here; otherwise distance field will be built on a
  // different set of anchors due to randomness!
  global_state.GetDeformedVolumeMesh().CreateAnchors(
      anchors_options_.grid_size, anchors_options_.inside_only,
      anchors_options_.bbox_padding, anchors_options_.surface_samples,
      anchors_options_.perturbation);
  global_state.GetDeformedVolumeMesh().CreateDistanceField();

  global_controller_.GetGlobalView().UpdateAnchorsView(global_state);
  global_controller_.GetGlobalView().SetVisibility(
      GlobalView::VisibilityDeformedMesh | GlobalView::VisibilityAnchors);
}

void DecompositionStage::DrawAnchorsSdfChildWindow() {
  ImGuiTreeNodeWithTooltip(
      "Anchors creation parameters", ImGuiTreeNodeFlags_None,
      "Change how anchors are created. Anchors are the set of points that we "
      "discretize the signed distance field on with respect to a shape (input "
      "mesh or polycube)."
      "A larger number will lead to slower computation time, but more "
      "accurate approximation of the distance field.",
      [&] {
        ImGui::SetNextItemWidth(80);
        {
          ImGuiFrameColorGuard frame_cg{5.0f / 7.0f};
          ImGui::DragInt("grid size", &anchors_options_.grid_size, 1, 4, 64,
                         "%d");
        }
        ImGui::SameLine();
        ImGuiHelpMarker("Size of the uniform grid used as anchors.");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(80);
        {
          ImGui::Checkbox("inside only", &anchors_options_.inside_only);
          ImGui::SameLine();
          ImGuiHelpMarker("Whether to create uniform grid inside the mesh.");
        }
        ImGui::SetNextItemWidth(80);
        {
          ImGuiFrameColorGuard frame_cg{4.0f / 7.0f};
          ImGui::DragFloat("bbox padding", &anchors_options_.bbox_padding,
                           1e-3f, 0.0f, 0.5f, "%.3f");
          ImGui::SameLine();
          ImGuiHelpMarker(
              "How much padding to add when creating the uniform grid "
              "anchors.");
        }
        ImGui::SetNextItemWidth(80);
        {
          ImGuiFrameColorGuard frame_cg{4.0f / 7.0f};
          ImGui::DragInt("surface samples", &anchors_options_.surface_samples,
                         1, 1, 100, "%d");
          ImGui::SameLine();
          ImGuiHelpMarker(
              "A multiple of this many surface points will be included in the "
              "anchors with random jittering.");
        }
        ImGui::SameLine();
        ImGui::SetNextItemWidth(80);
        {
          ImGuiFrameColorGuard frame_cg{4.0f / 7.0f};
          ImGui::DragFloat("jitter", &anchors_options_.perturbation, 1e-3f,
                           0.0f, 0.5f, "%.3f");
          ImGui::SameLine();
          ImGuiHelpMarker("How much do we perturb surface points?");
        }
      });
  if (ImGui::Button("Create anchors and signed distance fields")) {
    CreateSdfAndAnchors();
  }
  ImGui::SameLine();
  ImGuiHelpMarker(
      "Create anchors with prescribed parameters, and compute signed "
      "distance field with respect to the input shape.");
}

void DecompositionStage::DrawPolycubeControlChildWindow() {
  DrawPolycubeOptimizerTreeNode();
  DrawPolycubeManipulationTreeNode();
}

void DecompositionStage::DrawPolycubeManipulationTreeNode() {
  if (ImGui::TreeNodeEx("Polycube manipulation",
                        ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::SetNextItemWidth(100);
    {
      ImGuiButtonColorGuard cg{0.05f};
      if (ImGui::Button("Init/Reset polycube")) {
        ResetPolycube();
      }
    }
    ImGui::SameLine();
    ImGuiHelpMarker(
        "Initialize/reset an empty polycube. You must initialize a polycube "
        "before "
        "adding cuboids!\nWARNING: this will remove all existing cuboids, so "
        "save your progress first.");

    if (global_controller_.GetGlobalState().HasPolycube()) {
      {
        const char* suggest_strat_strs[] = {"distance-based", "volume-based"};
        const char* suggest_tooltips[] = {
            "Put a cuboid at a point uncovered by any cuboid that is furthest "
            "away "
            "from any cuboid.",
            "Create a cuboid that has largest volume while being inside the "
            "input "
            "mesh and outside of any existing cuboid."};
        static int item_current = 0;
        if (ImGui::Button("Add a new cuboid")) {
          PolycubeOptimizer::SuggestStrategy strat;
          if (item_current == 0) {
            strat = PolycubeOptimizer::SuggestStrategy::Simple;
          } else {
            strat = PolycubeOptimizer::SuggestStrategy::Largest;
          }
          SuggestNewCuboid(strat);
        }
        ImGui::SameLine();
        ImGuiHelpMarker(
            "Add a new cuboid according to the strategy specified on the "
            "right.");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(140);
        {
          const char* combo_label = suggest_strat_strs[item_current];
          if (ImGui::BeginCombo("strategy", combo_label,
                                ImGuiComboFlags_None)) {
            for (int n = 0; n < IM_ARRAYSIZE(suggest_strat_strs); n++) {
              const bool is_selected = (item_current == n);
              if (ImGui::Selectable(suggest_strat_strs[n], is_selected)) {
                item_current = n;
              }

              if (is_selected) ImGui::SetItemDefaultFocus();
              ImGuiHoveredTooltip(suggest_tooltips[n]);
            }
            ImGui::EndCombo();
          }
        }
      }
      if (global_controller_.GetGlobalState().GetPolycube().GetCuboidCount() >
          0) {
        if (ImGui::Button("Subtract a cuboid")) {
          StartSubtractCuboid();
        }
        ImGui::SameLine();
        ImGuiHelpMarker(
            "Subtract a cuboid from the polycube. This may break up each "
            "intersecting cuboid into up to 8 new cuboids.");
      }

      if (global_controller_.GetGlobalState().GetPolycube().GetCuboidCount() >
          0) {
        bool pressed = ImGui::Button("Reoptimize");
        ImGui::SameLine();
        ImGuiHelpMarker(
            "Optimize all unlocked cuboids' parameters jointly. You can choose "
            "to lock a cuboid to prevent its parameters from being changed.");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(120);
        if (ImGui::InputInt("steps", &reoptimize_steps_)) {
          if (reoptimize_steps_ < 0) {
            reoptimize_steps_ = 0;
          }
        }

        if (pressed) {
          Reoptimize(reoptimize_steps_);
        }
      }
    }
    ImGui::TreePop();
  }
}

void DecompositionStage::DrawPolycubeOptimizerTreeNode() {
  ImGuiTreeNodeWithTooltip(
      "Coverage parameters", ImGuiTreeNodeFlags_DefaultOpen,
      "Tweak how much to penalize over/under coverage of input shape by "
      "cuboids.",
      [&] {
        ImGui::SetNextItemWidth(70);
        {
          ImGuiFrameColorGuard frame_cg{4.5f / 7.0f};
          ImGui::DragFloat("+ weight", &options_.positive_l2_weight, 1e-3f,
                           0.0f, 1.0f, "%.3f");
        }
        ImGui::SameLine();
        ImGuiHelpMarker(
            "Weight of the positive part of the l2 loss (i.e. integration over "
            "the positive sdf region of input shape); higher value "
            "disencourages over-coverage of the input shape by cuboids.");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(70);
        {
          ImGuiFrameColorGuard frame_cg{6.5f / 7.0f};
          ImGui::DragFloat("- weight", &options_.negative_l2_weight, 1e-3f,
                           0.0f, 1.0f, "%.3f");
        }

        ImGui::SameLine();
        ImGuiHelpMarker(
            "Weight of the negative part of the l2 loss (i.e. integration over "
            "the positive sdf region of input shape)(i.e. integration over the "
            "negative sdf region of input shape); higher value "
            "disencourages under-coverage of the input shape by cuboids.");
      });

  DrawOptimizerOptionsNode(options_.learning_rate, options_.adam_betas,
                           &options_.snapshot_freq);
  if (ImGui::Button("Init/Reset optimizer")) {
    polycube_optimizer_ = std::make_unique<PolycubeOptimizer>(options_);
  }
  ImGui::SameLine();
  ImGuiHelpMarker("Recreate optimizer to use the prescribed parameters.");
}

void DecompositionStage::StartSubtractCuboid() {
  UnfocusCuboid();
  auto& deformed_mesh =
      global_controller_.GetGlobalState().GetDeformedVolumeMesh();
  auto& polycube = global_controller_.GetGlobalState().GetPolycube();
  subtract_cuboid_ = std::make_unique<Cuboid>();
  bool success = polycube_optimizer_->SuggestSubtractCuboid(
      deformed_mesh, polycube, *subtract_cuboid_);
  if (success) {
    subtract_editing_controller_ =
        std::make_unique<CuboidEditingController>(global_controller_, true);
    auto& polycube_view = global_controller_.GetGlobalView().GetPolycubeView();
    polycube_view.CreateTmpCuboid(*subtract_cuboid_);
    auto& cuboid_node = polycube_view.GetTmpCuboidNode();
    cuboid_node.UpdateMode(CuboidNode::Mode::Subtract);
    subtract_editing_controller_->Focus(cuboid_node, *subtract_cuboid_);
  } else {
    LOGI(
        "Failed to subtract cuboid: likely the polycube is contained entirely "
        "inside the input mesh!");
  }
}

void DecompositionStage::PerformSubtraction() {
  LOGI("Perform subtraction!");

  UnfocusCuboid();

  auto subtract_bounds = subtract_cuboid_->GetBound();
  subtract_cuboid_.reset();

  auto& global_state = global_controller_.GetGlobalState();
  assert(global_state.HasPolycube());

  auto& polycube = global_state.GetPolycube();
  auto& polycube_info = global_state.GetPolycubeInfo();
  auto& polycube_view = global_controller_.GetGlobalView().GetPolycubeView();

  size_t n = polycube.GetCuboidCount();

  struct ChangeEntry {
    int index;
    std::string name;
    std::vector<Cuboid> split_cuboids;
  };

  std::vector<ChangeEntry> change_list;
  for (size_t i = 0; i < n; i++) {
    auto& cuboid = polycube.GetCuboid(i);

    auto bounds = cuboid.GetBound();
    auto intersection = subtract_bounds;
    intersection.first = intersection.first.cwiseMax(bounds.first);
    intersection.second = intersection.second.cwiseMin(bounds.second);

    if (IsCuboidBoundsEmpty(intersection)) {
      continue;  // only add non-empty ones to change list
    }

    ChangeEntry entry;
    entry.index = i;
    entry.name = polycube_info.names[i].ToString();
    // Create a most 6 non-disjoint cuboids for bounds minus intersection.
    for (int k = 0; k < 3; k++) {
      for (int t = 0; t <= 1; t++) {
        auto new_bounds = bounds;

        if (t == 0) {
          new_bounds.second(k) = intersection.first(k);
        } else {
          new_bounds.first(k) = intersection.second(k);
        }
        if (!IsCuboidBoundsEmpty(new_bounds)) {
          entry.split_cuboids.emplace_back(
              Cuboid::FromBounds(new_bounds.first, new_bounds.second));
        }
      }
    }
    change_list.push_back(entry);
  }

  LOGI("Applying {} subtraction changes ...", change_list.size());
  // Now apply changes to polycube, polycube info, and polycube view.
  // For loop must be in reverse order to guarantee correctness!
  for (int j = static_cast<int>(change_list.size()) - 1; j >= 0; j--) {
    auto& entry = change_list[j];
    int i = entry.index;
    int old_pos = polycube_info.GetPosition(i);
    polycube.DeleteCuboid(i);
    polycube_info.Delete(i);
    polycube_view.DeleteCuboid(i);

    int move_delta = static_cast<int>(polycube_info.ordering.size()) - old_pos;
    int count = 0;
    for (auto& cuboid : entry.split_cuboids) {
      polycube.AddCuboid(cuboid);
      polycube_view.AddCuboid(cuboid);
      polycube_info.Push(fmt::format("[split #{}] {}", count, entry.name));
      polycube_info.MoveItem(polycube.GetCuboidCount() - 1, -move_delta);
      count++;
    }
  }
}
}  // namespace hex
// namespace hex
