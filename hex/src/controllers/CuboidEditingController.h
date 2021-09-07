#pragma once

#include "models/Cuboid.h"
#include "views/CuboidNode.h"
#include "vkoo/core/InputEvent.h"
#include "vkoo/st/Node.h"
#include "vkoo/st/components/Tracing.h"
#include "vkoo/st/hittables/Cylinder.h"

namespace hex {
class GlobalController;

class CuboidEditingController {
 public:
  enum class State { Idle, Focused, Dragged };

  CuboidEditingController(GlobalController& global_controller,
                          bool is_subtraction);
  ~CuboidEditingController();
  State GetState() const { return state_; }
  bool HandleInputEvent(const vkoo::InputEvent& event);
  void Focus(CuboidNode& node, Cuboid& cuboid);
  void Unfocus();

  struct WindowState {
    bool delete_cuboid{false};
    bool duplicate_cuboid{false};
    bool subtract_cuboid{false};
    bool lock_cuboid{false};
  };
  void DrawWindow(WindowState& window_state);

 private:
  void UpdateView();
  void ResolveDrag(const glm::vec2& current_position);
  void ResolveDragMove(const glm::vec2& current_position);
  void ResolveDragStretch(const glm::vec2& current_position);
  void AttachAxes();
  float CalculateDisplacement(const Vector3f& ax,
                              const glm::vec2& current_position);
  void ComputeStickyCoordinates(int ax_id);
  int RetrieveStickyInfo(float position);
  std::unique_ptr<vkoo::st::Node> CreateStickyHighlightNode();
  void AttachHighlightNode(const std::vector<Vector3f>& quad);

  State state_{State::Idle};
  bool is_move_{true};

  CuboidNode* focused_node_{nullptr};
  Cuboid* focused_cuboid_{nullptr};

  // Look-and-feel.
  std::unique_ptr<vkoo::st::Material> axes_materials_[3];
  std::unique_ptr<vkoo::st::Material> sticky_highlight_material_;

  std::shared_ptr<vkoo::st::Cylinder> axes_hittable_;

  // Nodes.
  std::unique_ptr<vkoo::st::Node> wrapper_node_;
  vkoo::st::Node* wrapper_node_ptr_{nullptr};

  // ax_node_ is for drag-move, while bi_ax_node_ is for drag-stretch.
  std::unique_ptr<vkoo::st::Node> axes_node_;
  vkoo::st::Node* axes_node_ptr_{nullptr};
  vkoo::st::Node* ax_node_ptrs_[3];
  std::unique_ptr<vkoo::st::Node> bi_axes_node_;
  vkoo::st::Node* bi_axes_node_ptr_{nullptr};
  vkoo::st::Node* bi_ax_node_ptrs_[2][3];

  glm::vec2 drag_start_position_;
  Cuboid drag_start_cuboid_;
  vkoo::st::Tracing* drag_hit_obj_{nullptr};

  struct StickyInfo {
    float coordinate;
    std::vector<Vector3f> quads;
  };

  // Variables for sticky drag.
  std::unique_ptr<std::vector<StickyInfo>> sticky_infos_;
  std::unique_ptr<vkoo::st::Node> sticky_highlight_node_;
  vkoo::st::Node* sticky_highlight_node_ptr_{nullptr};

  GlobalController& global_controller_;
  bool is_subtraction_;

  const float kAxesRadius = 0.05f;
};
}  // namespace hex
