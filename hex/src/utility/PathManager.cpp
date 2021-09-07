#include "PathManager.h"

#include <filesystem>

#include "logging.h"

namespace fs = std::filesystem;

namespace hex {
PathManager& PathManager::GetInstance() {
  static PathManager instance;
  return instance;
}

PathManager::PathManager() {
  workspace_path_ = fs::current_path();
  LOGI("workspace path: {}", workspace_path_.string());
  import_path_ = IMPORT_PATH;
}

void PathManager::SetWorkspacePath(const std::string& workspace_path) {
  GetInstance().workspace_path_ = workspace_path;
}

fs::path PathManager::GetWorkspacePath() {
  return GetInstance().workspace_path_;
}

fs::path PathManager::GetImportPath() { return GetInstance().import_path_; }
}  // namespace hex
