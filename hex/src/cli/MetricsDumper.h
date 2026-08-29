#pragma once

#include <string>

namespace hex {
class HexahedralMesh;

// Computes min/max/mean/std of scaled-Jacobian and Jacobian over the mesh,
// counts inverted hexes (scaled_jac < 0), and writes a YAML sidecar at
// out_path. Reuses HexMeshQuality's lazy-cached vectors — same data the GUI
// quality filter uses.
void DumpQualityMetrics(const HexahedralMesh& mesh,
                        const std::string& out_path);
}  // namespace hex
