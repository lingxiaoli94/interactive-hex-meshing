// geomlib unit tests.
//
// Every case is parameterized on a torch::Device and runs on CPU in *every*
// build, so the CPU path -- which is the only path a -DHEX_ENABLE_CUDA=OFF build
// has -- is covered rather than skipped. A CUDA-enabled build additionally runs
// the same cases on CUDA (proving the shared CPU implementation works against
// CUDA-enabled LibTorch too) plus the mixed-device negative cases, which need a
// second device to exist at all.
//
// Deliberately no test framework: geomlib has no test dependency today and the
// point is a check that runs in seconds, not a new third-party build input.

#include <geomlib/autograd/GeneralizedProjection.h>
#include <geomlib/generalized_projection.h>
#include <geomlib/generalized_projection_info.h>
#include <geomlib/point_tet_mesh_test.h>

#include <torch/torch.h>

#include <cmath>
#include <functional>
#include <iostream>
#include <string>
#include <vector>

namespace {

// ---------------------------------------------------------------- harness ---

int g_checks = 0;
int g_failures = 0;
std::string g_context;

void Fail(const std::string& what, const std::string& detail) {
  ++g_failures;
  std::cout << "  FAIL [" << g_context << "] " << what << ": " << detail
            << std::endl;
}

void Expect(bool ok, const std::string& what) {
  ++g_checks;
  if (!ok) Fail(what, "expected to hold");
}

void ExpectNear(double got, double want, double tol, const std::string& what) {
  ++g_checks;
  if (!(std::abs(got - want) <= tol)) {
    Fail(what, "got " + std::to_string(got) + ", want " + std::to_string(want) +
                   " (tol " + std::to_string(tol) + ")");
  }
}

// Only the mixed-device suite needs this, and that suite needs two devices.
//
// The message is matched, not just the fact of a throw: some mixed-device
// combinations would raise from inside torch's own indexing anyway, so a bare
// "something threw" assertion would keep passing even if geomlib's device
// guards were deleted -- and the combinations torch does *not* catch would
// segfault instead of raising.
[[maybe_unused]] void ExpectThrowsContaining(const std::function<void()>& fn,
                                             const std::string& needle,
                                             const std::string& what) {
  ++g_checks;
  try {
    fn();
  } catch (const std::exception& e) {
    const std::string message = e.what();
    if (message.find(needle) == std::string::npos) {
      Fail(what, "threw, but the message did not contain \"" + needle +
                     "\"; got: " + message);
    }
    return;
  }
  Fail(what, "expected an exception, none was thrown");
}

// ------------------------------------------------------------- tensor help ---

// Source data is always double so that the float32 and float64 variants of a
// case start from the same exact values.
torch::Tensor Real(const std::vector<double>& data, torch::IntArrayRef shape,
                   torch::Device device, torch::Dtype dtype) {
  return torch::tensor(data, torch::TensorOptions().dtype(torch::kFloat64))
      .reshape(shape)
      .to(dtype)
      .to(device)
      .contiguous();
}

torch::Tensor Index(const std::vector<int64_t>& data, torch::IntArrayRef shape,
                    torch::Device device, torch::Dtype dtype) {
  return torch::tensor(data, torch::TensorOptions().dtype(torch::kInt64))
      .reshape(shape)
      .to(dtype)
      .to(device)
      .contiguous();
}

// Reads one element regardless of which device the tensor lives on.
double At(const torch::Tensor& t, int64_t i) {
  return t.detach().to(torch::kCPU).contiguous().reshape({-1})[i].item<double>();
}

double ToleranceFor(torch::Dtype dtype) {
  return dtype == torch::kFloat32 ? 1e-5 : 1e-9;
}

std::string Label(torch::Device device, torch::Dtype dtype) {
  return device.str() + "/" + (dtype == torch::kFloat32 ? "f32" : "f64");
}

// The unit tetrahedron with vertex 0 at the origin. Keeping v0 at the origin
// matters: the tetrahedron kernel measures its interior distance from `p`
// rather than from `p - v0`, so an origin v0 keeps the expectations exact.
const std::vector<double> kUnitTetVertices{0, 0, 0, 1, 0, 0,
                                           0, 1, 0, 0, 0, 1};
// The triangle v0=(0,0,0), v1=(1,0,0), v2=(0,1,0): e1 and e2 are orthonormal,
// so w1/w2 are just the x/y coordinates and every expectation below is exact.
const std::vector<double> kUnitTriangleVertices{0, 0, 0, 1, 0, 0, 0, 1, 0};

// ------------------------------------------------------------------ cases ---

void TestPointTetMeshTest(torch::Device device, torch::Dtype dtype) {
  g_context = "PointTetMeshTest " + Label(device, dtype);

  auto vertices = Real(kUnitTetVertices, {4, 3}, device, dtype);
  auto tets = Index({0, 1, 2, 3}, {1, 4}, device, torch::kInt32);
  // One point well inside the tet, one well outside it.
  auto points = Real({0.1, 0.1, 0.1, 2, 2, 2}, {2, 3}, device, dtype);

  auto signs = geomlib::PointTetMeshTest(points, vertices, tets);

  Expect(signs.dtype() == torch::kInt32, "result keeps the tets dtype (int32)");
  Expect(signs.device() == points.device(), "result stays on the input device");
  Expect(signs.numel() == 2, "one sign per point");
  // Convention: a non-zero winding number means inside, reported as -1.
  ExpectNear(At(signs, 0), -1, 0, "interior point is -1");
  ExpectNear(At(signs, 1), 1, 0, "exterior point is +1");
}

void TestTriangleProjection(torch::Device device, torch::Dtype dtype) {
  g_context = "ComputeGeneralizedTriangleProjection<3> " + Label(device, dtype);
  const double tol = ToleranceFor(dtype);

  auto vertices = Real(kUnitTriangleVertices, {3, 3}, device, dtype);
  auto faces = Index({0, 1, 2}, {1, 3}, device, torch::kInt64);
  geomlib::TriangularProjectionInfo info{vertices, faces};

  // Point 0 sits one unit above the triangle's interior; point 1 is off the
  // far end of e1, so it clamps onto vertex 1.
  auto points = Real({0.25, 0.25, 1.0, 2.0, 0.0, 0.0}, {2, 3}, device, dtype);

  auto result = geomlib::ComputeGeneralizedTriangleProjection<3>(points, info);
  Expect(result.size() == 4, "returns dists, idxs, w1, w2");
  Expect(result[1].dtype() == torch::kInt64, "face indices are int64");
  Expect(result[0].device() == points.device(), "results stay on the device");

  // Distances are squared.
  ExpectNear(At(result[0], 0), 1.0, tol, "interior: squared distance");
  ExpectNear(At(result[1], 0), 0, 0, "interior: nearest face index");
  ExpectNear(At(result[2], 0), 0.25, tol, "interior: w1");
  ExpectNear(At(result[3], 0), 0.25, tol, "interior: w2");

  ExpectNear(At(result[0], 1), 1.0, tol, "clamped: squared distance");
  ExpectNear(At(result[2], 1), 1.0, tol, "clamped: w1 lands on vertex 1");
  ExpectNear(At(result[3], 1), 0.0, tol, "clamped: w2");
}

void TestTetrahedronProjection(torch::Device device, torch::Dtype dtype) {
  g_context =
      "ComputeGeneralizedTetrahedronProjection<3> " + Label(device, dtype);
  const double tol = ToleranceFor(dtype);

  auto vertices = Real(kUnitTetVertices, {4, 3}, device, dtype);
  auto tets = Index({0, 1, 2, 3}, {1, 4}, device, torch::kInt64);
  // Point 0 is inside (distance 0); point 1 is outside, nearest to vertex 1.
  auto points = Real({0.1, 0.1, 0.1, 2.0, 0.0, 0.0}, {2, 3}, device, dtype);

  auto result =
      geomlib::ComputeGeneralizedTetrahedronProjection<3>(points, vertices, tets);
  Expect(result.size() == 3, "returns dists, idxs, weights");
  Expect(result[1].dtype() == torch::kInt64, "tet indices are int64");
  Expect(result[2].size(1) == 3, "three barycentric weights per point");

  ExpectNear(At(result[0], 0), 0.0, tol, "interior: squared distance is 0");
  ExpectNear(At(result[2], 0), 0.1, tol, "interior: weight along e1");
  ExpectNear(At(result[2], 1), 0.1, tol, "interior: weight along e2");
  ExpectNear(At(result[2], 2), 0.1, tol, "interior: weight along e3");

  ExpectNear(At(result[0], 1), 1.0, tol, "exterior: squared distance");
}

// Runs one backward pass through the autograd wrapper and checks dOutput/dp.
// A fresh `points` leaf per call keeps the gradients from accumulating.
void CheckTriangleGradient(torch::Device device, torch::Dtype dtype,
                           size_t output_index, double gx, double gy, double gz,
                           const std::string& what) {
  const double tol = ToleranceFor(dtype);

  auto vertices = Real(kUnitTriangleVertices, {3, 3}, device, dtype);
  auto faces = Index({0, 1, 2}, {1, 3}, device, torch::kInt64);
  geomlib::TriangularProjectionInfo info{vertices, faces};

  // Projects strictly inside the triangle, so the smooth barycentric branch is
  // the one being differentiated.
  auto points = Real({0.25, 0.25, 1.0}, {1, 3}, device, dtype);
  points.set_requires_grad(true);

  // Outputs are {dists, idxs, w0, w1, w2}.
  auto result = geomlib::GeneralizedTriangleProjection<3>::apply(points, info);
  result.at(output_index).sum().backward();

  auto grad = points.grad();
  ++g_checks;
  if (!grad.defined()) {
    Fail(what, "no gradient propagated to points");
    return;
  }
  ExpectNear(At(grad, 0), gx, tol, what + " d/dx");
  ExpectNear(At(grad, 1), gy, tol, what + " d/dy");
  ExpectNear(At(grad, 2), gz, tol, what + " d/dz");
}

void TestTriangleProjectionGradients(torch::Device device, torch::Dtype dtype) {
  g_context = "GeneralizedTriangleProjection<3> backward " + Label(device, dtype);

  // For this triangle grad_w1 = (1,0,0) and grad_w2 = (0,1,0) exactly, so
  // grad_w0 = -grad_w1 - grad_w2.
  CheckTriangleGradient(device, dtype, 3, 1, 0, 0, "dw1/dp");
  CheckTriangleGradient(device, dtype, 4, 0, 1, 0, "dw2/dp");
  CheckTriangleGradient(device, dtype, 2, -1, -1, 0, "dw0/dp");
}

void RunDeviceSuite(torch::Device device) {
  std::cout << "== device: " << device.str() << std::endl;
  for (auto dtype : {torch::kFloat32, torch::kFloat64}) {
    TestPointTetMeshTest(device, dtype);
    TestTriangleProjection(device, dtype);
    TestTetrahedronProjection(device, dtype);
    TestTriangleProjectionGradients(device, dtype);
  }
}

#if HEX_ENABLE_CUDA
// Every dispatcher picks a device from `points` and then dereferences the other
// tensors as raw pointers, so a mismatch must raise rather than hand a device
// pointer to a host loop. Needs two real devices, hence CUDA-build-only.
void RunMixedDeviceSuite() {
  g_context = "mixed-device inputs";
  std::cout << "== mixed-device negative cases" << std::endl;

  // The substring every geomlib device guard emits (CHECK_SAME_DEVICE in
  // geomlib/common.cuh). Requiring it means these cases fail if the guard is
  // ever removed, even where torch would coincidentally raise on its own.
  const std::string kSameDeviceMessage = "must be on the same device as";

  const auto cpu = torch::Device(torch::kCPU);
  const auto cuda = torch::Device(torch::kCUDA);
  const auto f32 = torch::kFloat32;

  auto points_cpu = Real({0.1, 0.1, 0.1}, {1, 3}, cpu, f32);
  auto points_cuda = Real({0.1, 0.1, 0.1}, {1, 3}, cuda, f32);

  auto tet_vertices_cpu = Real(kUnitTetVertices, {4, 3}, cpu, f32);
  auto tet_vertices_cuda = Real(kUnitTetVertices, {4, 3}, cuda, f32);
  auto tets_i32_cpu = Index({0, 1, 2, 3}, {1, 4}, cpu, torch::kInt32);
  auto tets_i64_cpu = Index({0, 1, 2, 3}, {1, 4}, cpu, torch::kInt64);
  auto tets_i64_cuda = Index({0, 1, 2, 3}, {1, 4}, cuda, torch::kInt64);

  ExpectThrowsContaining(
      [&] {
        geomlib::PointTetMeshTest(points_cpu, tet_vertices_cuda, tets_i32_cpu);
      },
      kSameDeviceMessage,
      "PointTetMeshTest rejects cpu points with cuda vertices");
  ExpectThrowsContaining(
      [&] {
        geomlib::PointTetMeshTest(points_cuda, tet_vertices_cuda, tets_i32_cpu);
      },
      kSameDeviceMessage,
      "PointTetMeshTest rejects cuda points with cpu tets");

  ExpectThrowsContaining(
      [&] {
        geomlib::ComputeGeneralizedTetrahedronProjection<3>(
            points_cpu, tet_vertices_cuda, tets_i64_cpu);
      },
      kSameDeviceMessage,
      "tetrahedron projection rejects cpu points with cuda vertices");
  ExpectThrowsContaining(
      [&] {
        geomlib::ComputeGeneralizedTetrahedronProjection<3>(
            points_cpu, tet_vertices_cpu, tets_i64_cuda);
      },
      kSameDeviceMessage,
      "tetrahedron projection rejects cpu points with cuda tets");

  // The cached projection data is the easiest thing to get wrong: `info` is
  // built once and reused, so it can outlive the device its points are on.
  auto tri_vertices_cuda = Real(kUnitTriangleVertices, {3, 3}, cuda, f32);
  auto faces_cuda = Index({0, 1, 2}, {1, 3}, cuda, torch::kInt64);
  geomlib::TriangularProjectionInfo info_cuda{tri_vertices_cuda, faces_cuda};
  ExpectThrowsContaining(
      [&] {
        geomlib::ComputeGeneralizedTriangleProjection<3>(points_cpu, info_cuda);
      },
      kSameDeviceMessage,
      "triangle projection rejects cpu points with cuda projection info");

  auto tri_vertices_cpu = Real(kUnitTriangleVertices, {3, 3}, cpu, f32);
  auto faces_cpu = Index({0, 1, 2}, {1, 3}, cpu, torch::kInt64);
  geomlib::TriangularProjectionInfo info_cpu{tri_vertices_cpu, faces_cpu};
  ExpectThrowsContaining(
      [&] {
        geomlib::ComputeGeneralizedTriangleProjection<3>(points_cuda, info_cpu);
      },
      kSameDeviceMessage,
      "triangle projection rejects cuda points with cpu projection info");
}
#endif  // HEX_ENABLE_CUDA

}  // namespace

int main() {
#if HEX_ENABLE_CUDA
  std::cout << "geomlib tests (build: HEX_ENABLE_CUDA=1)" << std::endl;
#else
  std::cout << "geomlib tests (build: HEX_ENABLE_CUDA=0, CPU-only)" << std::endl;
#endif

  // Always exercised, in every build configuration.
  RunDeviceSuite(torch::Device(torch::kCPU));

#if HEX_ENABLE_CUDA
  if (torch::cuda::is_available()) {
    RunDeviceSuite(torch::Device(torch::kCUDA));
    RunMixedDeviceSuite();
  } else {
    std::cout << "SKIP: CUDA-enabled build but no usable CUDA device; "
                 "CUDA and mixed-device cases not run."
              << std::endl;
  }
#else
  std::cout << "SKIP: CPU-only build has no second device, so the CUDA and "
               "mixed-device cases do not apply."
            << std::endl;
#endif

  std::cout << (g_failures == 0 ? "PASS" : "FAIL") << ": " << g_checks
            << " checks, " << g_failures << " failure(s)" << std::endl;
  return g_failures == 0 ? 0 : 1;
}
