#include "ShortcutController.h"

#include <imgui.h>

#include <random>

#include "GlobalController.h"
#include "logging.h"
#include "optim/LargestCuboidSolver.h"
#include "utility/StopWatch.h"

namespace hex {
namespace {
void TestLargestCuboid() {
  /* int occ_raw[] = {1, 0, 1, 0, 0, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1,
   * 0, */
  /*                  1, 0, 1, 0, 0, 1, 0, 1, 1, 1, 1, 1, 0, 1, 1, 1, 0, 0, 1,
   * 1}; */
  /* Array3D<int> occ(2, 4, 5); // answer should be 8 */

  std::default_random_engine gen(
      std::chrono::system_clock().now().time_since_epoch().count());
  std::uniform_int_distribution<int> dis(0, 1);
  Array3D<int> occ(64, 64, 64);
  for (int i = 0; i < occ.GetDim(0); i++)
    for (int j = 0; j < occ.GetDim(1); j++)
      for (int k = 0; k < occ.GetDim(2); k++) {
        occ(i, j, k) = dis(gen);
      }

  StopWatch watch;
  watch.Tic();
  Vector3i cl, cr;
  int max_volume;
  LargestCuboidSolver(occ).Solve(cl, cr, max_volume);
  LOGI("Largest cuboid has corner at ({},{},{})-({},{},{}) with volume {}",
       cl.x(), cl.y(), cl.z(), cr.x(), cr.y(), cr.z(), max_volume);
  LOGI("Took {} sec(s)", watch.Toc().count());

  /*
  Vector3i gold_cl, gold_cr;
  int gold_max_volume;
  LargestCuboidSolver(occ).SolveBruteForce(gold_cl, gold_cr, gold_max_volume);

  LOGI(
      "(Gold) Largest cuboid has corner at ({},{},{})-({},{},{}) with volume "
      "{}",
      gold_cl.x(), gold_cl.y(), gold_cl.z(), gold_cr.x(), gold_cr.y(),
      gold_cr.z(), gold_max_volume);

  assert(gold_max_volume == max_volume);
  */
}
}  // namespace
ShortcutController::ShortcutController(GlobalController& global_controller) {}

void ShortcutController::DrawMenu() {
  if (ImGui::MenuItem("Test Largest Cuboid")) {
    TestLargestCuboid();
  }
}
}  // namespace hex
