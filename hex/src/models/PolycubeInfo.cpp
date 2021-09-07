#include "PolycubeInfo.h"

#include "logging.h"

namespace hex {
FixedLengthStr::FixedLengthStr(const std::string& str) {
  int n = std::min((int)str.size(), kMaxNameLength);
  std::memcpy(buf, str.data(), n * sizeof(char));
  buf[n] = '\0';
}

std::string FixedLengthStr::ToString() { return std::string(buf); }

void PolycubeInfo::Delete(int k) {
  assert(k < names.size() && names.size() == ordering.size());

  names.erase(names.begin() + k);
  std::vector<int> new_ordering;
  for (int i = 0; i < static_cast<int>(ordering.size()); i++) {
    if (ordering[i] < k) {
      new_ordering.push_back(ordering[i]);
    } else if (ordering[i] > k) {
      new_ordering.push_back(ordering[i] - 1);
    }
  }
  ordering = new_ordering;

  locked.erase(locked.begin() + k);
}

void PolycubeInfo::MoveItem(int k, int delta) {
  if (delta == 0) {
    return;
  }
  int pos = GetPosition(k);
  int new_pos = std::min(std::max(0, pos + delta), (int)ordering.size() - 1);

  ordering.erase(ordering.begin() + pos);
  ordering.insert(ordering.begin() + new_pos, k);
}

void PolycubeInfo::ChangePosition(int k, int i) {
  int pos = GetPosition(k);
  ordering.erase(ordering.begin() + pos);
  ordering.insert(ordering.begin() + i, k);
}

int PolycubeInfo::GetPosition(int k) {
  int pos = -1;
  for (int i = 0; i < static_cast<int>(ordering.size()); i++) {
    if (ordering[i] == k) {
      pos = i;
    }
  }
  return pos;
}

void PolycubeInfo::Push(const std::string& str) {
  FixedLengthStr fixed_str(str.empty() ? fmt::format("Cuboid {}", names.size())
                                       : str);
  names.push_back(fixed_str);  // default name

  // By default not locking.
  locked.push_back(0);

  // Add the newly created cuboid to the end of the list.
  ordering.push_back(names.size() - 1);
}

void PolycubeInfo::ToggleLock(int k) {
  assert(0 <= k && k < locked.size());
  locked[k] ^= 1;
}

void PolycubeInfo::ChangeName(int k, const std::string& new_name) {
  names.at(k) = new_name;
}
}  // namespace hex
