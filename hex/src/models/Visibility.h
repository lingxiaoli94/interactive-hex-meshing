#pragma once

namespace hex {
// Which pipeline objects the user has chosen to see.
//
// This is non-visual state -- a bitmask that outlives any renderer -- and it is
// owned by GlobalController (Phase A, Step A2). The flags live here rather than
// inside GlobalView because GlobalView.h pulls in vkoo::Device, st::Material and
// st::Mesh, none of which exist in a HEX_ENABLE_VULKAN=OFF build, while the
// stages that read and write the mask are compiled in both.
enum VisibilityFlag : unsigned long {
  VisibilityNone = 0x00,
  VisibilityInputMesh = 0x01,
  VisibilityDeformedMesh = 0x02,
  VisibilityDecomposition = 0x04,
  VisibilityPolycubeComplex = 0x08,
  VisibilityResultFilteredMesh = 0x20,
  VisibilityAnchors = 0x40,
};
}  // namespace hex
