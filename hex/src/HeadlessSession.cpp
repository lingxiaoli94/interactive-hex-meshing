#include "HeadlessSession.h"

#include <torch/torch.h>
#include <vkoo/st/components/Camera.h>
#include <vkoo/st/components/Light.h>

#include "logging.h"
#include "models/HexConvention.h"
#include "utility/PathManager.h"

namespace hex {
namespace {
// HexMeshingApp takes these from the window it created. There is no window
// here, and nothing is projected to a screen, but the scene still carries a
// camera whose aspect ratio the arc-ball script initialises from -- so use the
// same nominal size vkoo::Application reports when headless.
constexpr uint32_t kNominalWidth = 1280;
constexpr uint32_t kNominalHeight = 720;
}  // namespace

HeadlessSession::HeadlessSession()
    : settings_{PathManager::GetWorkspacePath() / "config.yml", 60} {
  // Same seed as HexMeshingApp: the optimizers draw from the global torch RNG,
  // so seeding must not depend on which owner started the process.
  torch::manual_seed(42);

  // Initialize HexConvention early to prevent race conditions caused by lazy
  // intialization.
  HexConvention::Initialize();

  SetupScene();

  global_controller_ = std::make_unique<GlobalController>(
      settings_, /*device=*/nullptr, *scene_, kNominalWidth, kNominalHeight);

  LOGI("Headless session: no renderer, no window, no Vulkan object.");
}

HeadlessSession::~HeadlessSession() {
  // The controller holds scene nodes, so it must go first -- the same ordering
  // vkoo::Application's destructor relies on.
  global_controller_.reset();
  scene_.reset();
}

void HeadlessSession::SetupScene() {
  auto root = std::make_unique<vkoo::st::Node>();
  auto camera_node = std::make_unique<vkoo::st::Node>();

  auto& camera = camera_node->CreateComponent<vkoo::st::Camera>(
      45.0f,
      static_cast<float>(kNominalWidth) / static_cast<float>(kNominalHeight),
      0.1f, 100.f);

  root->AddChild(std::move(camera_node));
  scene_ = std::make_unique<vkoo::st::Scene>(std::move(root));
  scene_->SetActiveCameraPtr(&camera);

  // Lights are plain scene components with no GPU resource behind them. They
  // are created because Settings describes them and because a later GUI-less
  // consumer of the same scene would otherwise see a different graph; nothing
  // in the script path reads them.
  for (auto& spec : settings_.lights) {
    auto light_node = std::make_unique<vkoo::st::Node>();
    auto& light_comp = light_node->CreateComponent<vkoo::st::Light>();
    light_comp.SetType(spec.type);
    light_comp.SetProperties(spec.properties);
    scene_->GetRoot().AddChild(std::move(light_node));
  }
}
}  // namespace hex
