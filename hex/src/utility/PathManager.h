#pragma once

#include "common.h"

#include <filesystem>

namespace hex {
class PathManager {
 public:
  PathManager(PathManager&) = delete;
  PathManager& operator=(const PathManager&) = delete;
  static void SetWorkspacePath(const std::string& workspace_path);
  static std::filesystem::path GetWorkspacePath();
  static std::filesystem::path GetImportPath();

 private:
  PathManager();
  static PathManager& GetInstance();
  std::filesystem::path workspace_path_;
  std::filesystem::path import_path_;
};
}  // namespace hex
