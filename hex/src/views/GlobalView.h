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
#include "models/Visibility.h"

namespace hex {
class GlobalView {
 public:
  GlobalView(vkoo::Device& device, vkoo::st::Scene& scene);
  GlobalView(const GlobalView&) = delete;
  ~GlobalView();
  GlobalView& operator=(const GlobalView&) = delete;

  // Returns true when the user changed the mask, which the caller then applies.
  // The mask is passed in rather than owned here: it is non-visual state and
  // lives in GlobalController, so it survives with no renderer present.
  bool DrawNodeVisibilityGui(bool ctrl_pressed, unsigned long& mask);
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

  // Shows exactly the nodes whose flag is set in `mask` (and that exist).
  // GlobalController is the owner of `mask` and the only caller.
  void ApplyVisibility(unsigned long mask);
  PolycubeView& GetPolycubeView();

 private:
  struct VisibilityInfo {
    VisibilityFlag flag;
    vkoo::st::Node*& node;
    bool transparent{false};  // only used by input/deformed meshes
    bool wireframe{true};     // only used by input/deformed meshes
  };

  void CreateDefaultMaterials();
  void SetMeshOptions();
  void RegisterVisibilityInfos();
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
