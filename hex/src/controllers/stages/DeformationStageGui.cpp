#include "DeformationStage.h"

// GUI-only half of DeformationStage; excluded when HEX_ENABLE_VULKAN=OFF.
#if HEX_ENABLE_VULKAN

#include <imgui.h>

#include "controllers/GlobalController.h"
#include "utility/ImGuiEx.h"

namespace hex {
void DeformationStage::DrawStageWindow() {
  ImGui::SetNextWindowSize(ImVec2(400, 360), ImGuiCond_Once);
  ImGui::Begin("Deformation");
  bool pressed;

  ImGui::PushItemWidth(50);
  pressed = ImGui::Button("Init/Reset deformed mesh");
  ImGui::SameLine();
  ImGuiHelpMarker(
      "Initialize or reset the deformed mesh to be the same as the input "
      "mesh. This must be pressed first after loading a tetrahedral mesh.");
  if (pressed) {
    InitDeformedMesh();
  }
  ImGui::PopItemWidth();

  DrawDistortionEnergyChildWindow(distortion_options_, false, false);

  ImGuiTreeNodeWithTooltip(
      "L1 energy parameters", ImGuiTreeNodeFlags_DefaultOpen,
      "These parameters control how close the deformed shape resemble an "
      "axis-aligned polycube.",
      [&] {
        ImGui::SetNextItemWidth(80);
        {
          ImGuiFrameColorGuard frame_cg{5.0f / 7.0f};
          ImGui::DragFloat("cubeness", &deformer_options_.cubeness_weight,
                           1e-2f, 0.0f, 100.0f, "%.2f");
        }
        ImGui::SameLine();
        ImGuiHelpMarker(
            "This measures how much we want each face normal to be "
            "axis-aligned.");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(80);
        {
          ImGuiFrameColorGuard frame_cg{6.0f / 7.0f};
          ImGui::DragFloat("smoothness", &deformer_options_.smoothness_weight,
                           1e-3f, 0.0f, 1.0f, "%.3f");
          ImGui::SameLine();
          ImGuiHelpMarker(
              "This measures how well smooth normals on neighboring faces need "
              "to "
              "be.");
        }

        if (global_controller_.IsExpertMode()) {
          ImGui::SetNextItemWidth(120);

          static bool is_logarithm = true;
          {
            ImGuiFrameColorGuard frame_cg{0.0f / 7.0f};
            ImGui::DragFloat("eps", &deformer_options_.norm_eps, 3e-4f, 1e-6f,
                             1.0f, "%.6f",
                             is_logarithm ? ImGuiSliderFlags_Logarithmic
                                          : ImGuiSliderFlags_None);
          }
          ImGui::SameLine();
          ImGuiHelpMarker(
              "This is the regularization constant used to give a smoothed "
              "version of l1 loss. ");

          ImGui::SameLine();
          ImGui::Checkbox("log-scale", &is_logarithm);
          ImGui::SameLine();
          ImGuiHelpMarker(
              "Determines Whether dragging eps will change its value in log "
              "scale "
              "(recommended) "
              "or not.");
        }
      });

  DrawOptimizerOptionsNode(deformer_options_.learning_rate,
                           deformer_options_.adam_betas, nullptr);

  pressed = ImGui::Button("Init/Reset optimizer");
  ImGui::SameLine();
  ImGuiHelpMarker(
      "Create a new deformation optimizer based on the current set of "
      "parameters. If an optimizer has been created already, it will be "
      "overriden and all previous optimizer state (e.g. momentum of gradients) "
      "will be lost, which may or may not be the desired outcome.");
  if (pressed) {
    PrepareVolumetricDeformation();
  }

  if (cubic_volumetric_deformer_) {
    pressed = ImGui::Button("Reoptimize");
    ImGui::SameLine();
    ImGuiHelpMarker(
        "Reoptimize for a given number of steps. This will block the program "
        "until finish.");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(100);
    if (ImGui::InputInt("steps", &reopt_steps_)) {
      if (reopt_steps_ < 0) {
        reopt_steps_ = 0;
      }
    }
    if (pressed) {
      Reoptimize(reopt_steps_);
    }
  }

  ImGui::End();
}
}  // namespace hex

#endif  // HEX_ENABLE_VULKAN
