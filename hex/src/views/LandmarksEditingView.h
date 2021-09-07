#pragma once

#include "common.h"

#include <vkoo/core/Device.h>
#include <vkoo/core/VertexObject.h>
#include <vkoo/st/Material.h>
#include <vkoo/st/Node.h>

#include "models/Ray.h"

namespace hex {
class LandmarksEditingView {
 public:
  LandmarksEditingView(vkoo::Device& device, vkoo::st::Node& parent_node);
  void AddLandmark(size_t id, const Vector3f& position);
  void HighlightLandmark(const Vector3f& position);
  void RemoveLandmark(size_t id);
  void RemoveHighlightLandmark();
  void MoveLandmark(size_t id, const Vector3f& new_position);
  void UpdatePositions(const std::vector<Vector3f> positions);
  bool IntersectWithRay(const Ray& world_ray, size_t& hit_id);
  vkoo::st::Node* GetWrapperNode() const;

 private:
  vkoo::st::Node* wrapper_node_{nullptr};

  std::shared_ptr<vkoo::VertexObject> sphere_vbo_;
  std::unordered_map<size_t, vkoo::st::Node*> id_to_node_;
  vkoo::st::Node* highlight_node_{nullptr};

  vkoo::st::Material default_material_;
  vkoo::st::Material highlight_material_;
};
}  // namespace hex
