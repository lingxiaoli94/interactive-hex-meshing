#pragma once

#include "common.h"

namespace hex {
const int kMaxNameLength = 32;

struct FixedLengthStr {
  FixedLengthStr() = default;
  FixedLengthStr(const std::string& str);
  std::string ToString();
  char buf[kMaxNameLength + 1];
};

struct PolycubeInfo {
  void Delete(int k);
  void ChangeName(int k, const std::string& new_name);
  void Push(const std::string& str = "");
  void ToggleLock(int k);
  int GetPosition(int k);  // get position of an index in ordering
  void ChangePosition(int k, int i);
  void MoveItem(int k, int delta);

  std::vector<FixedLengthStr> names;  // name of each cuboid
  std::vector<int> ordering;  // ordering of cuboids, a list of cuboid indices
  std::vector<int> locked;         // whether the cuboid is locked
};
}  // namespace hex
