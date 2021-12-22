#include <geomlib/autograd/GeneralizedProjection.h>

#include <iostream>

void test_generalized_triangle_projection() {
  /*
  float points_data[] = {0.10798142, 0.09435397, 0.8972397};
  float vertices_data[] = {-0.14192927,  -0.4436557, 0.19722219,
                           -0.099712044, -0.4427007, 0.154986,
                           -0.11019023,  -0.4492986, 0.21198432};
  int64_t faces_data[] = {0, 1, 2};

  auto points =
      torch::from_blob(points_data, {1, 3}, torch::kFloat32).clone().cuda();
  auto vertices =
      torch::from_blob(vertices_data, {3, 3}, torch::kFloat32).clone().cuda();
  auto faces =
      torch::from_blob(faces_data, {1, 3}, torch::kInt64).clone().cuda();

  using namespace torch::indexing;
  auto v0 = vertices.index({faces.index({Slice(), 0})});
  auto e1 = vertices.index({faces.index({Slice(), 1})}) - v0;
  auto e2 = vertices.index({faces.index({Slice(), 2})}) - v0;
  auto e1_dot_e2 = (e1 * e2).sum(-1);    // |F|
  auto e1_norm_sqr = (e1 * e1).sum(-1);  // |F|
  auto e2_norm_sqr = (e2 * e2).sum(-1);  // |F|

  std::cout << "e1: " << e1 << std::endl;
  std::cout << "e1_dot_e2: " << e1_dot_e2 << std::endl;

  auto grad_w1 =
      ((e1_dot_e2 / e2_norm_sqr).unsqueeze(1) * (-e2) + e1) /
      (e1_norm_sqr - e1_dot_e2.square() / e2_norm_sqr).unsqueeze(1);  // |F|x3
  auto grad_w2 =
      ((e1_dot_e2 / e1_norm_sqr).unsqueeze(1) * (-e1) + e2) /
      (e2_norm_sqr - e1_dot_e2.square() / e1_norm_sqr).unsqueeze(1);  // |F|x3
  std::cout << "grad_w1: " << grad_w1 << std::endl;
  std::cout << "grad_w2: " << grad_w2 << std::endl;

  points.requires_grad_();
  auto result =
      geomlib::GeneralizedTriangleProjection<3>::apply(points, vertices, faces);

  auto result_dists = result[0];
  auto result_idxs = result[1];
  auto result_w0 = result[2];
  auto result_w1 = result[3];
  auto result_w2 = result[4];

  std::cout << "dists: " << result_dists << std::endl;
  std::cout << "idxs: " << result_idxs << std::endl;
  std::cout << "w0: " << result_w0 << std::endl;
  std::cout << "w1: " << result_w1 << std::endl;
  std::cout << "w2: " << result_w2 << std::endl;

  // result_dists.sum().backward();

  // std::cout << "derivative of result_dists: " << points.grad() << std::endl;
  */
}

void test_generalized_tetrahedron_projection() {
  /* float points_data[] = {1.5, 0.6, 0.0}; */
  /* float vertices_data[] = {1, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0}; */
  float points_data[] = {0, 0, 1};
  float vertices_data[] = {0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0};
  int64_t tets_data[] = {0, 1, 2, 3};

  auto points =
      torch::from_blob(points_data, {1, 3}, torch::kFloat32).clone().cuda();
  auto vertices =
      torch::from_blob(vertices_data, {4, 3}, torch::kFloat32).clone().cuda();
  auto tets = torch::from_blob(tets_data, {1, 4}, torch::kInt64).clone().cuda();
  auto result = geomlib::GeneralizedTetrahedronProjection<3>::apply(
      points, vertices, tets);
  auto result_dists = result[0];
  auto result_idxs = result[1];
  auto result_weights = result[2];
  std::cout << "output dists: " << result_dists << std::endl;
  std::cout << "output idxs: " << result_idxs << std::endl;
  std::cout << "output weights: " << result_weights << std::endl;
}

void test_matmul() {
  float M1_data[] = {0.1006,  0.1433,  0.1490,  0.2601,  0.0913,  0.1482,
                     0.1157,  0.3593,  0.2491,  0.2436,  0.0140,  0.0665,
                     0.2882,  0.3012,  0.4259,  0.3674,  0.6799,  -0.6787,
                     -0.6441, -0.6262, -0.6778, -0.6801, -0.8964, -0.7123};
  float M2_data[] = {0, 0,   0, 0, 0, 0, 0,   0,  -20, 0,  0, 0,
                     0, -20, 0, 0, 0, 0, -20, 20, 20,  20, 0, 0};
  auto M1 = torch::from_blob(M1_data, {3, 8}, torch::kFloat32).clone().cuda();
  auto M2 = torch::from_blob(M2_data, {8, 3}, torch::kFloat32).clone().cuda();
  auto J_matmul = torch::matmul(M1, M2);
  std::cout << "J_matmul: \n" << J_matmul << std::endl;
  auto J_mm = torch::mm(M1, M2);
  std::cout << "J_mm: \n" << J_mm << std::endl;
  std::cout << "J_mm_cpu:"
            << torch::matmul(M1.detach().cpu(), M2.detach().cpu());
}

int main() {
  // test_generalized_triangle_projection();
  test_generalized_tetrahedron_projection();
  /* test_matmul(); */
}
