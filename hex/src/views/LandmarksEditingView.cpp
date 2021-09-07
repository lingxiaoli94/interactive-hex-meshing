#include "LandmarksEditingView.h"

#include <vkoo/core/PrimitiveFactory.h>
#include <vkoo/st/components/Mesh.h>
#include <vkoo/st/components/Tracing.h>

#include "vkoo/st/hittables/Ray.h"
#include "vkoo/st/hittables/Sphere.h"

namespace hex {
namespace {
const float kRadius = 0.01f;
const glm::vec4& kDefaultColor = {1.0f, 1.0f, 0.0f, 1.0f};
const glm::vec4& kHighlightColor = {1.0f, 0.0f, 0.2f, 0.5f};
}

LandmarksEditingView::LandmarksEditingView(vkoo::Device& device,
                                           vkoo::st::Node& parent_node) {
  sphere_vbo_ = vkoo::PrimitiveFactory::CreateSphere(device, kRadius, 24, 24);
  default_material_.colors["diffuse_color"] = kDefaultColor;
  highlight_material_.colors["diffuse_color"] = kHighlightColor;

  auto wrapper_node = std::make_unique<vkoo::st::Node>();
  wrapper_node_ = wrapper_node.get();
  parent_node.AddChild(std::move(wrapper_node));
}

vkoo::st::Node* LandmarksEditingView::GetWrapperNode() const {
  return wrapper_node_;
}

bool LandmarksEditingView::IntersectWithRay(const Ray& world_ray,
                                            size_t& hit_id) {
  bool intersected = false;
  vkoo::st::HitRecord hit_record{};
  for (auto& kv : id_to_node_) {
    auto node = kv.second;
    auto tracing_component = node->GetComponentPtr<vkoo::st::Tracing>();
    assert(tracing_component);
    vkoo::st::Ray local_ray = world_ray.ToGlm();
    local_ray.ApplyTransform(node->GetTransform().GetWorldToLocalMatrix());
    if (tracing_component->GetHittable().Intersect(local_ray, hit_record)) {
      hit_id = kv.first;
      intersected = true;
    }
  }
  return intersected;
}

void LandmarksEditingView::AddLandmark(size_t id, const Vector3f& position) {
  assert(!id_to_node_.count(id));

  auto node = std::make_unique<vkoo::st::Node>();
  node->GetTransform().SetPosition(ToGlm(position));
  auto& mesh = node->CreateComponent<vkoo::st::Mesh>(
      sphere_vbo_, sphere_vbo_->GetIndexCount());
  mesh.SetMaterial(default_material_);

  node->CreateComponent<vkoo::st::Tracing>(
      std::make_shared<vkoo::st::Sphere>(kRadius));

  auto node_ptr = node.get();
  wrapper_node_->AddChild(std::move(node));

  id_to_node_[id] = node_ptr;
}

void LandmarksEditingView::UpdatePositions(
    const std::vector<Vector3f> positions) {
  for (auto& kv : id_to_node_) {
    kv.second->GetTransform().SetPosition(ToGlm(positions[kv.first]));
  }
}

void LandmarksEditingView::MoveLandmark(size_t id,
                                        const Vector3f& new_position) {
  auto itr = id_to_node_.find(id);
  assert(itr != id_to_node_.end());
  itr->second->GetTransform().SetPosition(ToGlm(new_position));
}

void LandmarksEditingView::HighlightLandmark(const Vector3f& position) {
  if (highlight_node_) {
    wrapper_node_->RemoveChild(highlight_node_);
  }

  auto node = std::make_unique<vkoo::st::Node>();
  node->GetTransform().SetPosition(ToGlm(position));
  auto& mesh = node->CreateComponent<vkoo::st::Mesh>(
      sphere_vbo_, sphere_vbo_->GetIndexCount());
  mesh.SetMaterial(highlight_material_);
  mesh.SetTransparent(true);
  mesh.SetAllowDepthTesting(false);

  highlight_node_ = node.get();
  wrapper_node_->AddChild(std::move(node));
}

void LandmarksEditingView::RemoveHighlightLandmark() {
  if (highlight_node_ != nullptr) {
    wrapper_node_->RemoveChild(highlight_node_);
    highlight_node_ = nullptr;
  }
}

void LandmarksEditingView::RemoveLandmark(size_t id) {
  auto itr = id_to_node_.find(id);
  assert(itr != id_to_node_.end());
  wrapper_node_->RemoveChild(itr->second);
  id_to_node_.erase(itr);
}
}  // namespace hex
