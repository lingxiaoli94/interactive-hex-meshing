#include "vkoo/core/PrimitiveFactory.h"

#include "vkoo/utils.h"

namespace vkoo {
std::unique_ptr<VertexObject> PrimitiveFactory::CreateSphere(Device& device,
                                                             float r,
                                                             size_t slices,
                                                             size_t stacks) {
  std::vector<glm::vec3> positions;
  std::vector<glm::vec3> normals;
  std::vector<uint32_t> indices;

  float phi_step = kPi * 2 / slices;
  float theta_step = kPi / stacks;

  for (size_t vi = 0; vi < stacks; vi++) {  // vertical loop
    float theta = vi * theta_step;
    float z = r * cosf(theta);
    for (size_t hi = 0; hi < slices; hi++) {  // horizontal loop
      float phi = hi * phi_step;
      glm::vec3 p(r * cosf(phi) * sinf(theta), r * sinf(phi) * sinf(theta), z);
      positions.push_back(p);
      normals.push_back(glm::normalize(p));
    }
  }

  for (size_t vi = 0; vi < stacks; vi++)
    for (size_t hi = 0; hi < slices; hi++) {
      auto t1 = (uint32_t)(vi * slices + hi);
      auto t2 = (uint32_t)(vi * slices + hi + 1);
      auto t3 = (uint32_t)((vi + 1) * slices + hi + 1);
      auto t4 = (uint32_t)((vi + 1) * slices + hi);
      indices.insert(indices.end(), {t1, t3, t2});
      indices.insert(indices.end(), {t1, t4, t3});
    }

  auto vertex_object = std::make_unique<VertexObject>(device);
  vertex_object->Update("position", positions);
  vertex_object->Update("normal", normals);
  vertex_object->UpdateIndices(indices);
  return vertex_object;
}

std::unique_ptr<VertexObject> PrimitiveFactory::CreateLineSegment(
    Device& device, const glm::vec3& p, const glm::vec3& q) {
  auto vertex_object = std::make_unique<VertexObject>(device);
  vertex_object->Update("position", std::vector<glm::vec3>{p, q});

  return vertex_object;
}

std::unique_ptr<VertexObject> PrimitiveFactory::CreateQuad(Device& device) {
  auto vertex_object = std::make_unique<VertexObject>(device);
  vertex_object->Update(
      "position",
      std::vector<glm::vec3>{{-1, -1, 0}, {1, -1, 0}, {1, 1, 0}, {-1, 1, 0}});
  vertex_object->Update(
      "normal",
      std::vector<glm::vec3>{{0, 0, 1}, {0, 0, 1}, {0, 0, 1}, {0, 0, 1}});
  vertex_object->UpdateIndices({0, 1, 2, 0, 2, 3});

  return vertex_object;
}

std::unique_ptr<VertexObject> PrimitiveFactory::CreateCube(Device& device) {
  auto vertex_object = std::make_unique<VertexObject>(device);
  std::vector<glm::vec3> positions;
  for (int z = -1; z <= 1; z += 2)
    for (int y = -1; y <= 1; y += 2)
      for (int x = -1; x <= 1; x += 2) {
        positions.emplace_back(static_cast<float>(x), static_cast<float>(y),
                               static_cast<float>(z));
      }
  std::vector<uint32_t> indices{0, 2, 3, 0, 3, 1, 1, 3, 7, 1, 7, 5,
                                4, 5, 7, 4, 7, 6, 0, 6, 2, 0, 4, 6,
                                2, 7, 3, 2, 6, 7, 0, 1, 5, 0, 5, 4};

  // Create discontinuous normals by duplicating positions.
  std::vector<glm::vec3> new_positions;
  std::vector<glm::vec3> normals;
  std::vector<uint32_t> new_indices;

  for (size_t i = 0; i < indices.size(); i += 3) {
    auto& a = positions[indices[i]];
    auto& b = positions[indices[i + 1]];
    auto& c = positions[indices[i + 2]];

    auto normal = glm::normalize(glm::cross(b - a, c - a));

    new_positions.push_back(a);
    new_positions.push_back(b);
    new_positions.push_back(c);
    normals.push_back(normal);
    normals.push_back(normal);
    normals.push_back(normal);
    new_indices.push_back(i);
    new_indices.push_back(i + 1);
    new_indices.push_back(i + 2);
  }

  vertex_object->Update("position", new_positions);
  vertex_object->Update("normal", normals);
  vertex_object->UpdateIndices(new_indices);

  return vertex_object;
}

std::unique_ptr<VertexObject> PrimitiveFactory::CreateCylinder(
    Device& device, float r, float h, size_t num_sides) {
  auto positions = std::make_unique<std::vector<glm::vec3>>();
  auto normals = std::make_unique<std::vector<glm::vec3>>();
  auto indices = std::make_unique<std::vector<uint32_t>>();

  float step = 2 * kPi / num_sides;

  for (size_t face = 0; face < num_sides; face++) {
    float lx = r * cosf(face * step);
    float lz = r * sinf(face * step);
    positions->emplace_back(lx, 0.0f, lz);
    positions->emplace_back(lx, h, lz);

    normals->emplace_back(cosf(face * step), 0.0f, sinf(face * step));
    normals->emplace_back(cosf(face * step), 0.0f, sinf(face * step));
  }
  for (size_t face = 0; face < num_sides; ++face) {
    unsigned int i1 = (unsigned int)face * 2;
    unsigned int i2;
    if (face == num_sides - 1) {
      i2 = 1;
    } else {
      i2 = i1 + 3;
    }
    unsigned int i3 = i1 + 1;

    indices->insert(indices->end(), {i1, i2, i3});
    if (face == num_sides - 1) {
      i2 = 0;
      i3 = 1;
    } else {
      i2 = i1 + 2;
      i3 = i1 + 3;
    }
    indices->insert(indices->end(), {i1, i2, i3});
  }
  auto vertex_object = std::make_unique<VertexObject>(device);
  vertex_object->Update("position", *positions);
  vertex_object->Update("normal", *normals);
  vertex_object->UpdateIndices(*indices);

  return vertex_object;
}
}  // namespace vkoo
