#include "ImGuiEx.h"

#include <imgui_internal.h>

#include "imgui.h"

namespace ImGui {
bool SelectableInput(const char* str_id, bool selected,
                     ImGuiSelectableFlags flags, char* buf, size_t buf_size) {
  ImGuiContext& g = *GImGui;
  ImGuiWindow* window = g.CurrentWindow;
  ImVec2 pos_before = window->DC.CursorPos;

  PushID(str_id);
  PushStyleVar(ImGuiStyleVar_ItemSpacing,
               ImVec2(g.Style.ItemSpacing.x, g.Style.FramePadding.y * 2.0f));
  bool ret = Selectable("##Selectable", selected,
                        flags | ImGuiSelectableFlags_AllowDoubleClick |
                            ImGuiSelectableFlags_AllowItemOverlap);
  PopStyleVar();

  ImGuiID id = window->GetID("##Input");
  bool temp_input_is_active = TempInputIsActive(id);
  bool temp_input_start = ret ? IsMouseDoubleClicked(0) : false;

  if (temp_input_start) SetActiveID(id, window);

  if (temp_input_is_active || temp_input_start) {
    ImVec2 pos_after = window->DC.CursorPos;
    window->DC.CursorPos = pos_before;
    ret = TempInputText(window->DC.LastItemRect, id, "##Input", buf,
                        (int)buf_size, ImGuiInputTextFlags_None);
    window->DC.CursorPos = pos_after;
  } else {
    window->DrawList->AddText(pos_before, GetColorU32(ImGuiCol_Text), buf);
  }

  PopID();
  return ret;
}
}  // namespace ImGui

namespace hex {
namespace {
void DrawDistortionEnergyContent(DistortionOptions& options, bool show_amips) {
  ImGui::SetNextItemWidth(80);
  {
    ImGuiFrameColorGuard frame_cg{1.0f / 7.0f};
    ImGui::DragFloat("conformal", &options.conformal_weight, 1e-3f, 0.0f, 1.0f,
                     "%.3f");
    ImGui::SameLine();
    ImGuiHelpMarker("This measures how well the deformation preserves angles.");
  }
  ImGui::SameLine();
  ImGui::SetNextItemWidth(80);
  {
    ImGuiFrameColorGuard frame_cg{2.0f / 7.0f};
    ImGui::DragFloat("authalic", &options.authalic_weight, 1e-3f, 0.0f, 1.0f,
                     "%.3f");
    ImGui::SameLine();
    ImGuiHelpMarker(
        "This measures how well the deformation preserves volumes.");
  }

  ImGui::SetNextItemWidth(120);

  static bool is_logarithm = true;
  {
    ImGuiFrameColorGuard frame_cg{0.0f / 7.0f};
    ImGui::DragFloat(
        "eps", &options.regularizer_eps, 3e-4f, 1e-6f, 1.0f, "%.6f",
        is_logarithm ? ImGuiSliderFlags_Logarithmic : ImGuiSliderFlags_None);
  }
  ImGui::SameLine();
  ImGuiHelpMarker(
      "This is the regularization constant. It is recommended to start with "
      "a "
      "bigger value and gradually decrease it to a smaller value after the "
      "optimization reaches a stable state.");

  ImGui::SameLine();
  ImGui::Checkbox("log-scale", &is_logarithm);
  ImGui::SameLine();
  ImGuiHelpMarker(
      "Determines whether dragging eps will change its value in log scale "
      "(recommended) or not.");

  if (show_amips) {
    ImGui::Checkbox("exponential scaling", &options.amips);
    ImGui::SameLine();
    ImGuiHelpMarker(
        "Determines whether exponential scaling will be used to penalize worst "
        "elements more. Note: this may not be stable if the mesh quality is "
        "too bad to start with.");
  }
}
}  // namespace

ImGuiFrameColorGuard::ImGuiFrameColorGuard(float H) {
  ImGui::PushStyleColor(ImGuiCol_FrameBg, (ImVec4)ImColor::HSV(H, 0.5f, 0.5f));
  ImGui::PushStyleColor(ImGuiCol_FrameBgHovered,
                        (ImVec4)ImColor::HSV(H, 0.6f, 0.5f));
  ImGui::PushStyleColor(ImGuiCol_FrameBgActive,
                        (ImVec4)ImColor::HSV(H, 0.7f, 0.5f));
  ImGui::PushStyleColor(ImGuiCol_SliderGrab,
                        (ImVec4)ImColor::HSV(H, 0.8f, 0.3f));
  ImGui::PushStyleColor(ImGuiCol_SliderGrabActive,
                        (ImVec4)ImColor::HSV(H, 0.9f, 0.3f));
}

ImGuiFrameColorGuard::~ImGuiFrameColorGuard() { ImGui::PopStyleColor(5); }

ImGuiButtonColorGuard::ImGuiButtonColorGuard(float H) {
  ImGui::PushStyleColor(ImGuiCol_Button, (ImVec4)ImColor::HSV(H, 0.5f, 0.5f));
  ImGui::PushStyleColor(ImGuiCol_ButtonHovered,
                        (ImVec4)ImColor::HSV(H, 0.6f, 0.5f));
  ImGui::PushStyleColor(ImGuiCol_ButtonActive,
                        (ImVec4)ImColor::HSV(H, 0.7f, 0.5f));
}

ImGuiButtonColorGuard::~ImGuiButtonColorGuard() { ImGui::PopStyleColor(3); }

void ImGuiHoveredTooltip(const char* desc) {
  if (ImGui::IsItemHovered()) {
    ImGui::BeginTooltip();
    ImGui::PushTextWrapPos(ImGui::GetFontSize() * 25.0f);
    ImGui::TextUnformatted(desc);
    ImGui::PopTextWrapPos();
    ImGui::EndTooltip();
  }
}

void ImGuiHelpMarker(const char* desc) {
  ImGui::TextDisabled("(?)");
  ImGuiHoveredTooltip(desc);
}

void ImGuiTreeNodeWithTooltip(const char* label, ImGuiTreeNodeFlags flags,
                              const char* desc, std::function<void()> closure) {
  if (ImGui::TreeNodeEx(label, flags)) {
    ImGuiHoveredTooltip(desc);
    closure();
    ImGui::TreePop();
  } else {
    ImGuiHoveredTooltip(desc);
  }
}

void DrawDistortionEnergyChildWindow(DistortionOptions& options,
                                     bool show_amips, bool default_open) {
  ImGuiTreeNodeWithTooltip(
      "Distortion energy parameters",
      default_open ? ImGuiTreeNodeFlags_DefaultOpen : ImGuiTreeNodeFlags_None,
      "These parameters control how much distortion (angle-preserving and "
      "volume-preserving) is allowed in the deformation. Together they can "
      "create foldover-free volumetric maps.",
      [&] { DrawDistortionEnergyContent(options, show_amips); });
}

void DrawOptimizerOptionsNode(float& learning_rate, Vector2f& adam_betas,
                              int* snapshot_freq, bool default_open) {
  ImGuiTreeNodeWithTooltip(
      "Optimizer options",
      default_open ? ImGuiTreeNodeFlags_DefaultOpen : ImGuiTreeNodeFlags_None,
      "Options for Adam optimizer.", [&] {
        ImGui::SetNextItemWidth(60);
        {
          ImGuiFrameColorGuard frame_cg{2.5f / 7.0f};
          ImGui::DragFloat("learning rate", &learning_rate, 1e-3f, 0.0f, 1.0f,
                           "%.4f", ImGuiSliderFlags_Logarithmic);
        }
        ImGui::SameLine();
        ImGuiHelpMarker(
            "Learning rate of the Adam optimizer. A smaller value will result "
            "in smaller update steps being taken.");

        ImGui::SameLine();
        ImGui::SetNextItemWidth(100);
        {
          ImGuiFrameColorGuard frame_cg{2.0f / 7.0f};
          ImGui::DragFloat2("betas", adam_betas.data(), 1e-3f, 0.0f, 1.0f,
                            "%.3f");
        }
        ImGui::SameLine();
        ImGuiHelpMarker("Betas of Adam optimizer.");

        if (snapshot_freq != nullptr) {
          ImGui::SetNextItemWidth(100);
          if (ImGui::InputInt("snapshot frequency", snapshot_freq)) {
            if (*snapshot_freq < 0) {
              *snapshot_freq = 0;
            }
          }
          ImGui::SameLine();
          ImGuiHelpMarker(
              "How often do we fetch result from the optimizer to visualize "
              "intermediate steps?");
        }
      });
}
}  // namespace hex
