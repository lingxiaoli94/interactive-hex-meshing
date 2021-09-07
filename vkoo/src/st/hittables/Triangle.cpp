#include "vkoo/st/hittables/Triangle.h"

#include "vkoo/st/hittables/Plane.h"

namespace vkoo {
namespace st {
Triangle::Triangle(const glm::vec3& p0, const glm::vec3& p1,
                   const glm::vec3& p2, const glm::vec3& n0,
                   const glm::vec3& n1, const glm::vec3& n2) {
  positions_.push_back(p0);
  positions_.push_back(p1);
  positions_.push_back(p2);

  normals_.push_back(n0);
  normals_.push_back(n1);
  normals_.push_back(n2);
}

Triangle::Triangle(const std::vector<glm::vec3>& positions,
                   const std::vector<glm::vec3>& normals) {
  if (positions.size() != 3 || normals.size() != 3)
    throw std::runtime_error("Wrong vector size in Triangle constructor!");
  positions_ = positions;
  normals_ = normals;
}

bool Triangle::Intersect(const Ray& ray, HitRecord& record) const {
  glm::vec3 O = positions_[0];
  glm::vec3 v1 = positions_[1] - O;
  glm::vec3 v2 = positions_[2] - O;
  glm::vec3 n = glm::normalize(glm::cross(v1, v2));

  Plane plane(n, glm::dot(O, n));
  HitRecord new_record{record};
  if (!plane.Intersect(ray, new_record)) return false;

  glm::vec3 p = ray.At(new_record.time) - O;

  glm::mat3 A(n, v1, v2);
  glm::vec3 x = glm::inverse(A) * p;
  x[0] = 1 - x[1] - x[2];  // barycentric coordinates
  if (x[0] < 0 || x[0] > 1 || x[1] < 0 || x[1] > 1 || x[2] < 0 || x[2] > 1) {
    return false;
  }
  // Only update record after passing all intersection tests.
  record = new_record;
  // Interpolating normals.
  record.normal = glm::normalize(x[0] * normals_[0] + x[1] * normals_[1] +
                                 x[2] * normals_[2]);
  return true;
}
}  // namespace st
}  // namespace vkoo
