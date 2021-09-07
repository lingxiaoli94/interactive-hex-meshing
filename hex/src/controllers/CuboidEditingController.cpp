#include "CuboidEditingController.h"

#include <vkoo/st/components/Mesh.h>

#include <limits>

#include "GLFW/glfw3.h"
#include "GlobalController.h"
#include "imgui.h"
#include "utility/ImGuiEx.h"
#include "vkoo/core/Gui.h"
#include "vkoo/core/InputEvent.h"
#include "vkoo/core/PrimitiveFactory.h"
#include "vkoo/st/hittables/AABB.h"

namespace hex {
namespace {
const float kDragSensitivity = 5e-3f;
const float kStickyRadius = 3e-2f;
const Vector3f kAxes[3] = {
    {1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
}  // namespace

bool CuboidEditingController::HandleInputEvent(const vkoo::InputEvent& event) {
  if (state_ == State::Idle) {
    return false;
  }
  bool handled = false;
  if (event.GetSource() == vkoo::EventSource::Mouse) {
    auto& mouse_event = static_cast<const vkoo::MouseButtonInputEvent&>(event);

    if (state_ == State::Focused) {
      // Ray in world space.
      vkoo::st::Ray world_ray =
          global_controller_
              .ShootRayAtMousePosition(mouse_event.GetXPos(),
                                       mouse_event.GetYPos())
              .ToGlm();

      auto traceables =
          wrapper_node_ptr_->GetComponentsRecursive<vkoo::st::Tracing>();
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
        drag_hit_obj_ = hit_obj;
        // TODO: highlight when mouse is hovering over.

        if (mouse_event.GetButton() == vkoo::MouseButton::Left &&
            mouse_event.GetAction() == vkoo::MouseAction::Down) {
          drag_start_position_.x = mouse_event.GetXPos();
          drag_start_position_.y = mouse_event.GetYPos();
          drag_start_cuboid_ = *focused_cuboid_;
          state_ = State::Dragged;
          handled = true;
        }
      }
    }

    if (state_ == State::Dragged) {
      if (sticky_highlight_node_ == nullptr) {
        // Always detach highlight node at start.
        sticky_highlight_node_ =
            sticky_highlight_node_ptr_->GetParentPtr()->RemoveChild(
                sticky_highlight_node_ptr_);
      }

      if (mouse_event.GetAction() == vkoo::MouseAction::Move) {
        ResolveDrag(glm::vec2{mouse_event.GetXPos(), mouse_event.GetYPos()});
        handled = true;
      }

      if (mouse_event.GetAction() == vkoo::MouseAction::Up) {
        state_ = State::Focused;
        drag_hit_obj_ = nullptr;
        sticky_infos_ = nullptr;
        handled = true;
      }
    }
  }

  if (event.GetSource() == vkoo::EventSource::Keyboard) {
    if (state_ == State::Focused) {
      auto& key_event = static_cast<const vkoo::KeyInputEvent&>(event);
      if (key_event.GetAction() == vkoo::KeyAction::Down &&
          key_event.GetCode() == GLFW_KEY_LEFT_ALT) {
        is_move_ = !is_move_;
        AttachAxes();
      }
    }
  }
  return handled;
}

void CuboidEditingController::ComputeStickyCoordinates(int ax_id) {
  sticky_infos_ = std::make_unique<std::vector<StickyInfo>>();
  auto& polycube = global_controller_.GetGlobalState().GetPolycube();
  int num_cuboids = polycube.GetCuboidCount();

  for (int i = 0; i < num_cuboids; i++) {
    auto& cuboid = polycube.GetCuboid(i);
    if (&cuboid == focused_cuboid_) {
      continue;
    }

    auto bound = cuboid.GetBound();

    sticky_infos_->push_back({bound.first(ax_id), cuboid.GetFace(ax_id, -1)});
    sticky_infos_->push_back({bound.second(ax_id), cuboid.GetFace(ax_id, 1)});
  }
}

int CuboidEditingController::RetrieveStickyInfo(float position) {
  int result = -1;
  for (int i = 0; i < static_cast<int>(sticky_infos_->size()); i++) {
    if (result == -1 ||
        std::abs(sticky_infos_->at(i).coordinate - position) <
            std::abs(sticky_infos_->at(result).coordinate - position)) {
      result = i;
    }
  }
  return result;
}

void CuboidEditingController::ResolveDrag(const glm::vec2& current_position) {
  assert(drag_hit_obj_ != nullptr);
  if (is_move_) {
    ResolveDragMove(current_position);
  } else {
    ResolveDragStretch(current_position);
  }
}

void CuboidEditingController::ResolveDragMove(
    const glm::vec2& current_position) {
  int ax_id = -1;
  for (int i = 0; i < 3; i++) {
    if (ax_node_ptrs_[i]->GetComponentPtr<vkoo::st::Tracing>() ==
        drag_hit_obj_) {
      ax_id = i;
      break;
    }
  }
  assert(ax_id != -1);
  if (sticky_infos_ == nullptr) {
    ComputeStickyCoordinates(ax_id);
  }
  auto ax = kAxes[ax_id];
  float displacement = CalculateDisplacement(ax, current_position);

  Vector3f new_center =
      drag_start_cuboid_.center + kDragSensitivity * displacement * ax;

  if (global_controller_.IsKeyPressed(GLFW_KEY_LEFT_CONTROL) ||
      global_controller_.IsKeyPressed(GLFW_KEY_RIGHT_CONTROL)) {
    float sticky_cost = std::numeric_limits<float>::max();
    float perturbed_pos = new_center(ax_id);
    int sticky_k = -1;
    for (int s = -1; s <= 1; s += 2) {
      float bound =
          new_center(ax_id) + s * drag_start_cuboid_.halflengths(ax_id);
      int k = RetrieveStickyInfo(bound);
      if (k != -1) {
        float coord = sticky_infos_->at(k).coordinate;
        float cost = std::abs(bound - coord);
        if (cost < sticky_cost) {
          sticky_k = k;
          sticky_cost = cost;
          perturbed_pos = new_center(ax_id) - bound + coord;
        }
      }
    }
    if (sticky_cost < kStickyRadius) {
      assert(sticky_k != -1);

      new_center(ax_id) = perturbed_pos;
      AttachHighlightNode(sticky_infos_->at(sticky_k).quads);
    }
  }
  focused_cuboid_->center = new_center;
  UpdateView();
}

void CuboidEditingController::AttachHighlightNode(
    const std::vector<Vector3f>& quad) {
  assert(sticky_highlight_node_);
  assert(quad.size() == 4);

  sticky_highlight_node_ = CreateStickyHighlightNode();  // FIXME
  sticky_highlight_node_ptr_ = sticky_highlight_node_.get();

  auto& mesh = *(sticky_highlight_node_->GetComponentPtr<vkoo::st::Mesh>());
  auto& vbo = mesh.GetVertexObject();

  std::vector<glm::vec3> positions;

  for (int i = 0; i < 4; i++) {
    positions.push_back({quad[i].x(), quad[i].y(), quad[i].z()});
  }

  // Normals don't matter for transparent objects.
  vbo.Update("position", positions);

  // Add to root so the position is absolute.
  global_controller_.GetScene().GetRoot().AddChild(
      std::move(sticky_highlight_node_));
}

void CuboidEditingController::ResolveDragStretch(
    const glm::vec2& current_position) {
  int sign = -1;
  int ax_id = -1;
  for (int s = 0; s <= 1; s++) {
    for (int i = 0; i < 3; i++) {
      if (bi_ax_node_ptrs_[s][i]->GetComponentPtr<vkoo::st::Tracing>() ==
          drag_hit_obj_) {
        sign = s;
        ax_id = i;
        break;
      }
    }
  }
  assert(ax_id != -1);
  if (sticky_infos_ == nullptr) {
    ComputeStickyCoordinates(ax_id);
  }
  auto ax = kAxes[ax_id];
  float displacement = CalculateDisplacement(ax, current_position);

  auto new_bound = drag_start_cuboid_.GetBound();
  if (sign == 0) {
    // 0 corresponds to +.
    new_bound.second += kDragSensitivity * displacement * ax;
    if (new_bound.second(ax_id) < new_bound.first(ax_id)) {
      new_bound.second(ax_id) = new_bound.first(ax_id);
    }
  } else {
    new_bound.first += kDragSensitivity * displacement * ax;
    if (new_bound.first(ax_id) > new_bound.second(ax_id)) {
      new_bound.first(ax_id) = new_bound.second(ax_id);
    }
  }

  if (global_controller_.IsKeyPressed(GLFW_KEY_LEFT_CONTROL) ||
      global_controller_.IsKeyPressed(GLFW_KEY_RIGHT_CONTROL)) {
    float* perturbed_ptr =
        sign == 0 ? &new_bound.second(ax_id) : &new_bound.first(ax_id);
    int sticky_k = -1;
    float sticky_cost = std::numeric_limits<float>::max();
    float perturbed_pos = *perturbed_ptr;
    int k = RetrieveStickyInfo(perturbed_pos);
    if (k != -1) {
      float coord = sticky_infos_->at(k).coordinate;
      float cost = std::abs(perturbed_pos - coord);
      if (cost < sticky_cost) {
        sticky_k = k;
        sticky_cost = cost;
        perturbed_pos = coord;
      }
    }
    if (sticky_cost < kStickyRadius) {
      assert(sticky_k != -1);

      *perturbed_ptr = perturbed_pos;
      AttachHighlightNode(sticky_infos_->at(sticky_k).quads);
    }
  }

  // Prevent negative halflengths again.
  if (sign == 0) {
    if (new_bound.second(ax_id) < new_bound.first(ax_id)) {
      new_bound.second(ax_id) = new_bound.first(ax_id);
    }
  } else {
    if (new_bound.first(ax_id) > new_bound.second(ax_id)) {
      new_bound.first(ax_id) = new_bound.second(ax_id);
    }
  }
  focused_cuboid_->SetBound(new_bound.first, new_bound.second);
  UpdateView();
}

float CuboidEditingController::CalculateDisplacement(
    const Vector3f& ax, const glm::vec2& current_position) {
  // Transform to screen space.
  auto world_to_screen_mat = global_controller_.GetWorldToScreenMatrix();
  auto origin_screen = world_to_screen_mat * glm::vec4{0.0f, 0.0f, 0.0f, 1.0f};
  origin_screen /= origin_screen.w;
  auto ax_screen =
      world_to_screen_mat * glm::vec4{ax.x(), ax.y(), ax.z(), 1.0f};
  ax_screen /= ax_screen.w;
  auto mouse_delta = current_position - drag_start_position_;
  mouse_delta = glm::vec2{mouse_delta.x, -mouse_delta.y};
  float displacement =
      glm::dot(mouse_delta, glm::normalize(ax_screen - origin_screen).xy());
  return displacement;
}

void CuboidEditingController::Focus(CuboidNode& node, Cuboid& cuboid) {
  focused_cuboid_ = &cuboid;
  focused_node_ = &node;
  focused_node_->AddChild(std::move(wrapper_node_));
  is_move_ = true;  // default to move
  AttachAxes();

  state_ = State::Focused;
}

void CuboidEditingController::AttachAxes() {
  if (is_move_) {
    if (axes_node_ != nullptr) {
      if (bi_axes_node_ == nullptr) {
        bi_axes_node_ = wrapper_node_ptr_->RemoveChild(bi_axes_node_ptr_);
      }
      wrapper_node_ptr_->AddChild(std::move(axes_node_));
    }
  } else {
    if (bi_axes_node_ != nullptr) {
      if (axes_node_ == nullptr) {
        axes_node_ = wrapper_node_ptr_->RemoveChild(axes_node_ptr_);
      }
      wrapper_node_ptr_->AddChild(std::move(bi_axes_node_));
    }
  }
}

void CuboidEditingController::Unfocus() {
  if (focused_node_) {
    wrapper_node_ = focused_node_->RemoveChild(wrapper_node_ptr_);
    assert(wrapper_node_.get() == wrapper_node_ptr_);
  }
  focused_cuboid_ = nullptr;
  focused_node_ = nullptr;
  state_ = State::Idle;
}

CuboidEditingController::~CuboidEditingController() { Unfocus(); }

void CuboidEditingController::DrawWindow(WindowState& window_state) {
  if (focused_cuboid_ != nullptr && focused_node_ != nullptr) {
    ImGui::SetNextWindowPos(ImVec2(800, 500), ImGuiCond_Once);
    ImGui::SetNextWindowSize(ImVec2(300, 150), ImGuiCond_Once);
    ImGui::Begin("Cuboid Editor", nullptr,
                 ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoNav);
    Vector3f center = focused_cuboid_->center;
    Vector3f halflengths = focused_cuboid_->halflengths;
    ImGui::SetNextItemWidth(200);
    bool center_dragged = ImGui::DragFloat3("Center", center.data(), 0.01f);
    ImGui::SetNextItemWidth(200);
    bool halflengths_dragged = ImGui::DragFloat3(
        "Halflengths", halflengths.data(), 0.01f, 0.0f, FLT_MAX);

    ImGui::Spacing();
    if (is_subtraction_) {
      {
        ImGuiButtonColorGuard btn_cg{0.5f};
        window_state.subtract_cuboid = ImGui::Button("Subtract");
      }
    } else {
      {
        ImGuiButtonColorGuard btn_cg{0.f};
        window_state.delete_cuboid = ImGui::Button("Delete");
      }
      ImGui::SameLine();
      {
        ImGuiButtonColorGuard btn_cg{0.3f};
        window_state.duplicate_cuboid = ImGui::Button("Duplicate");
      }
      ImGui::SameLine();
      {
        ImGuiButtonColorGuard btn_cg{0.5f};
        window_state.lock_cuboid =
            ImGui::Button(window_state.lock_cuboid ? "Unlock" : "Lock");
      }
    }
    ImGui::End();
    bool updated = false;
    if (center_dragged) {
      focused_cuboid_->center = center;
      updated = true;
    }
    if (halflengths_dragged) {
      focused_cuboid_->halflengths = halflengths;
      updated = true;
    }
    if (updated) {
      UpdateView();
    }
  }
}

void CuboidEditingController::UpdateView() {
  focused_node_->UpdateView(*focused_cuboid_);
}

CuboidEditingController::CuboidEditingController(
    GlobalController& global_controller, bool is_subtraction)
    : global_controller_{global_controller}, is_subtraction_{is_subtraction} {
  // Build the axes node.
  wrapper_node_ = std::make_unique<vkoo::st::Node>();
  wrapper_node_ptr_ = wrapper_node_.get();

  axes_materials_[0] = std::make_unique<vkoo::st::Material>();
  axes_materials_[0]->colors["diffuse_color"] =
      glm::vec4{1.0f, 0.0f, 0.0f, 0.6f};
  axes_materials_[1] = std::make_unique<vkoo::st::Material>();
  axes_materials_[1]->colors["diffuse_color"] =
      glm::vec4{0.0f, 1.0f, 0.0f, 0.6f};
  axes_materials_[2] = std::make_unique<vkoo::st::Material>();
  axes_materials_[2]->colors["diffuse_color"] =
      glm::vec4{0.0f, 0.0f, 1.0f, 0.6f};

  std::shared_ptr<vkoo::VertexObject> ax_vtx_obj =
      vkoo::PrimitiveFactory::CreateCylinder(global_controller_.GetDevice(),
                                             1.0f, 1.0f, 24);

  axes_hittable_ = std::make_shared<vkoo::st::Cylinder>(1.0f, 1.0f);

  // Build axes_node_.
  axes_node_ = std::make_unique<vkoo::st::Node>();
  axes_node_ptr_ = axes_node_.get();
  for (int i = 0; i < 3; i++) {
    auto ax_node = std::make_unique<vkoo::st::Node>();
    auto& mesh = ax_node->CreateComponent<vkoo::st::Mesh>(
        ax_vtx_obj, ax_vtx_obj->GetIndexCount());
    mesh.SetMaterial(*axes_materials_[i]);
    mesh.SetTransparent(true);
    mesh.SetAllowDepthTesting(false);
    ax_node->GetTransform().SetScale({kAxesRadius, 1.0f, 0.05f});
    ax_node->CreateComponent<vkoo::st::Tracing>(axes_hittable_);

    if (i == 0) {
      ax_node->GetTransform().SetRotation({0.0f, 0.0f, -1.0f},
                                          vkoo::kPi / 2.0f);
    } else if (i == 2) {
      ax_node->GetTransform().SetRotation({1.0f, 0.0f, 0.0f}, vkoo::kPi / 2.0f);
    }
    ax_node_ptrs_[i] = ax_node.get();
    axes_node_->AddChild(std::move(ax_node));
  }

  // Build bi_axes_node_.
  bi_axes_node_ = std::make_unique<vkoo::st::Node>();
  bi_axes_node_ptr_ = bi_axes_node_.get();
  for (int s = 0; s <= 1; s++)
    for (int i = 0; i < 3; i++) {
      auto ax_node = std::make_unique<vkoo::st::Node>();
      auto& mesh = ax_node->CreateComponent<vkoo::st::Mesh>(
          ax_vtx_obj, ax_vtx_obj->GetIndexCount());
      mesh.SetMaterial(*axes_materials_[i]);
      mesh.SetTransparent(true);
      mesh.SetAllowDepthTesting(false);
      ax_node->GetTransform().SetScale({kAxesRadius, 1.0f, 0.05f});
      ax_node->CreateComponent<vkoo::st::Tracing>(axes_hittable_);

      if (s == 0 && i == 0) {
        ax_node->GetTransform().SetRotation({0.0f, 0.0f, -1.0f},
                                            vkoo::kPi / 2.0f);
      } else if (s == 0 && i == 1) {
      } else if (s == 0 && i == 2) {
        ax_node->GetTransform().SetRotation({1.0f, 0.0f, 0.0f},
                                            vkoo::kPi / 2.0f);
      } else if (s == 1 && i == 0) {
        ax_node->GetTransform().SetRotation({0.0f, 0.0f, 1.0f},
                                            vkoo::kPi / 2.0f);
      } else if (s == 1 && i == 1) {
        ax_node->GetTransform().SetRotation({1.0f, 0.0f, 0.0f}, vkoo::kPi);
      } else if (s == 1 && i == 2) {
        ax_node->GetTransform().SetRotation({-1.0f, 0.0f, 0.0f},
                                            vkoo::kPi / 2.0f);
      }
      bi_ax_node_ptrs_[s][i] = ax_node.get();
      bi_axes_node_->AddChild(std::move(ax_node));
    }

  wrapper_node_->GetTransform().SetScale(
      {0.25f, 0.25f, 0.25f});  // TODO: better scaling scheme

  sticky_highlight_node_ = CreateStickyHighlightNode();
  sticky_highlight_node_ptr_ = sticky_highlight_node_.get();
}

std::unique_ptr<vkoo::st::Node>
CuboidEditingController::CreateStickyHighlightNode() {
  sticky_highlight_material_ = std::make_unique<vkoo::st::Material>();
  sticky_highlight_material_->colors["diffuse_color"] =
      glm::vec4{0.3f, 0.2f, 0.9f, 0.5f};

  auto node = std::make_unique<vkoo::st::Node>();

  std::shared_ptr<vkoo::VertexObject> vtx_obj =
      vkoo::PrimitiveFactory::CreateQuad(global_controller_.GetDevice());
  auto& mesh =
      node->CreateComponent<vkoo::st::Mesh>(vtx_obj, vtx_obj->GetIndexCount());
  mesh.SetMaterial(*sticky_highlight_material_);
  mesh.SetTransparent(true);
  mesh.SetAllowDepthTesting(false);
  // TODO: double-facet.

  return node;
}
}  // namespace hex
