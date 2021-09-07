#include "common.h"

namespace hex {
class HexConvention {
 public:
  HexConvention(HexConvention&) = delete;
  HexConvention& operator=(const HexConvention&) = delete;

  static const std::vector<Vector3i>& GetCorners();
  static const std::vector<Eigen::MatrixXf>& GetCornerJacobians();
  static const std::unordered_map<Vector3i, size_t, Vector3iHasher>&
  GetCornerToIdMap();
  static const Eigen::MatrixXf& GetPrincipleAxes();

  static const std::vector<Vector3i>& GetFaceOffsets();
  static const std::unordered_map<Vector3i, size_t, Vector3iHasher>&
  GetFaceOffsetToId();
  static const std::vector<std::vector<size_t>>& GetFaceCornerIds();
  static const std::vector<std::vector<size_t>>& GetOppositeFaceCornerIds();

  // Initialize should be called once at the start.
  static void Initialize();

 private:
  HexConvention() = default;

  static HexConvention& GetInstance();
  void BuildCorners();

  // 8 corners of a standard hex with coordinates in {0, 1}.
  std::unique_ptr<std::vector<Vector3i>> corners_;

  // This maps Vector3i (e.g. (0,0,0) -> 0, (1,1,1) -> 7) to corner id.
  std::unique_ptr<std::unordered_map<Vector3i, size_t, Vector3iHasher>>
      corner_to_id_;

  // A list of 8x3 matrices Z's so that if X is a 8x3 matrix encoding the
  // positions of a hex, then X^T Z is a 3x3 matrix of the Hessian.
  std::unique_ptr<std::vector<Eigen::MatrixXf>> corner_jacobians_;

  // A 8x3 matrix that extracts the principal axes of a hex.
  std::unique_ptr<Eigen::MatrixXf> principal_axes_;

  // List of 6 face offsets.
  std::unique_ptr<std::vector<Vector3i>> face_offsets_;
  // Inverse of face_offsets_;
  std::unique_ptr<std::unordered_map<Vector3i, size_t, Vector3iHasher>>
      face_offset_to_id_;
  // List of 6 quadruples representing corner ids of each face.
  std::unique_ptr<std::vector<std::vector<size_t>>> face_corner_ids_;
  std::unique_ptr<std::vector<std::vector<size_t>>> opposite_face_corner_ids_;
};
}  // namespace hex
