#pragma once

#include <vkoo/core/Device.h>
#include <vkoo/st/Material.h>
#include <vkoo/st/Scene.h>
#include <vkoo/st/components/Mesh.h>

#include "FilteredHexMesh.h"
#include "PolycubeView.h"
#include "QuadSurfaceView.h"
#include "TriSurfaceView.h"
#include "models/GlobalState.h"

namespace hex {
class GlobalView {
 public:
  enum VisibilityFlag : unsigned long {
    VisibilityNone = 0x00,
    VisibilityInputMesh = 0x01,
    VisibilityDeformedMesh = 0x02,
    VisibilityDecomposition = 0x04,
    VisibilityPolycubeComplex = 0x08,
    VisibilityResultFilteredMesh = 0x20,
    VisibilityAnchors = 0x40,
  };
  GlobalView(vkoo::Device& device, vkoo::st::Scene& scene);
  GlobalView(const GlobalView&) = delete;
  ~GlobalView();
  GlobalView& operator=(const GlobalView&) = delete;

  void DrawNodeVisibilityGui(bool ctrl_pressed);
  void UpdateAllViews(GlobalState& global_state);
  void UpdateInputMeshView(GlobalState& global_state);
  void UpdateDeformedMeshView(GlobalState& global_state);
  void UpdateAnchorsView(GlobalState& global_state);
  void UpdatePolycubeView(GlobalState& global_state);
  void UpdatePolycubeComplexView(GlobalState& global_state);
  void UpdatePolycubeComplexView(const QuadComplex& quad_complex,
                                 bool color_by_patch, bool use_axis_color);
  void UpdateFilteredHexView(GlobalState& global_state,
                             const FilteringSetting& filtering_setting);
  void ChangeTriMeshAppearance(VisibilityFlag flag, bool transparent,
                               bool wireframe);

  void SetVisibility(unsigned long flags);
  void AddVisibility(unsigned long flags);
  void RemoveVisibility(unsigned long flags);
  unsigned long GetVisibility();
  PolycubeView& GetPolycubeView();

 private:
  struct VisibilityInfo {
    VisibilityFlag flag;
    vkoo::st::Node*& node;
    bool& visibility;
    bool transparent{false};  // only used by input/deformed meshes
    bool wireframe{true};     // only used by input/deformed meshes
  };

  void CreateDefaultMaterials();
  void SetMeshOptions();
  void RegisterVisibilityInfos();
  void UpdateVisibility();
  void ClearVisibility();
  VisibilityInfo& GetVisibilityInfo(VisibilityFlag flag);
  void UpdateMeshViewByInfo(const VisibilityInfo& info);

  void CleanupPolycubeComplexView();
  std::vector<glm::vec4> CreatePolycubePatchColors(
      const QuadComplex& quad_complex, bool use_axis_color);

  vkoo::Device& device_;
  vkoo::st::Scene& scene_;

  vkoo::st::Node* input_mesh_node_{nullptr};
  vkoo::st::Node* deformed_mesh_node_{nullptr};
  vkoo::st::Node* anchors_node_{nullptr};
  vkoo::st::Node* polycube_node_{nullptr};
  vkoo::st::Node* polycube_complex_node_{nullptr};
  vkoo::st::Node* result_filtered_mesh_node_{nullptr};

  struct {
    bool input_mesh{false};
    bool deformed_mesh{false};
    bool anchors{false};
    bool decomposition{false};
    bool polycube_complex{false};
    bool result_filtered_mesh{false};
  } visibility_;

  std::vector<VisibilityInfo> visibility_infos_;

  vkoo::st::Material anchors_materials_[2];

  TriSurfaceView::Options input_mesh_options_;
  TriSurfaceView::Options deformed_mesh_options_;

  std::unique_ptr<TriSurfaceView> input_mesh_view_;
  std::unique_ptr<TriSurfaceView> deformed_mesh_view_;
  std::unique_ptr<PolycubeView> polycube_view_;
  std::unique_ptr<QuadSurfaceView> polycube_complex_view_;
  std::unique_ptr<QuadSurfaceView> result_filtered_view_;

  std::unique_ptr<FilteredHexMesh> filtered_hex_mesh_;

  std::unique_ptr<std::vector<glm::vec4>> patch_colors_;
};
}  // namespace hex
