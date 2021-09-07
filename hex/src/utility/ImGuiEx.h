#pragma once

#include "common.h"

#include <imgui.h>

#include "optim/DistortionEnergy.h"

namespace ImGui {
bool SelectableInput(const char* str_id, bool selected,
                     ImGuiSelectableFlags flags, char* buf, size_t buf_size);
}

namespace hex {
struct ImGuiFrameColorGuard {
  ImGuiFrameColorGuard(float H);
  ~ImGuiFrameColorGuard();
};
struct ImGuiButtonColorGuard {
  ImGuiButtonColorGuard(float H);
  ~ImGuiButtonColorGuard();
};

void ImGuiHelpMarker(const char* desc);
void ImGuiHoveredTooltip(const char* desc);
void ImGuiTreeNodeWithTooltip(const char* label, ImGuiTreeNodeFlags flags,
                              const char* desc, std::function<void()> closure);

void DrawDistortionEnergyChildWindow(DistortionOptions& options,
                                     bool show_amips, bool default_open = true);
void DrawOptimizerOptionsNode(float& learning_rate, Vector2f& adam_betas,
                              int* snapshot_freq = nullptr,
                              bool default_open = false);

}  // namespace hex
