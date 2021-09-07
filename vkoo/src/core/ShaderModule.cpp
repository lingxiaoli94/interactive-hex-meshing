#include "vkoo/core/ShaderModule.h"

#include "vkoo/core/Device.h"
#include "vkoo/core/GLSLCompiler.h"
#include "vkoo/core/SPIRVReflection.h"
#include "vkoo/utils.h"
#include "vkoo/logging.h"

namespace vkoo {
namespace {
std::string ReadTextFile(const std::string& filename) {
  std::vector<std::string> data;
  std::ifstream file;
  file.open(filename, std::ios::in);
  if (!file.is_open()) {
    throw std::runtime_error("Failed to open file: " + filename);
  }

  return std::string{(std::istreambuf_iterator<char>(file)),
                     (std::istreambuf_iterator<char>())};
}

inline std::vector<std::string> PrecompileShaders(const std::string& source) {
  std::vector<std::string> final_file;

  auto lines = Split(source, '\n');

  for (auto& line : lines) {
    if (line.find("#include \"") == 0) {
      std::string include_path = line.substr(10);
      size_t last_quote = include_path.find("\"");
      if (!include_path.empty() && last_quote != std::string::npos) {
        include_path = include_path.substr(0, last_quote);
      }

      // Include paths are relative to the base shader directory.
      auto include_file =
          PrecompileShaders(GetShaderPath() + "/" + include_path);
      for (auto& include_file_line : include_file) {
        final_file.push_back(include_file_line);
      }
    } else {
      final_file.push_back(line);
    }
  }

  return final_file;
}

inline std::vector<uint8_t> ConvertToBytes(std::vector<std::string>& lines) {
  std::vector<uint8_t> bytes;

  for (auto& line : lines) {
    line += "\n";
    std::vector<uint8_t> line_bytes(line.begin(), line.end());
    bytes.insert(bytes.end(), line_bytes.begin(), line_bytes.end());
  }

  return bytes;
}
}  // namespace

ShaderModule::ShaderModule(Device& device, VkShaderStageFlagBits stage,
                           const ShaderSource& shader_source,
                           const ShaderVariant& shader_variant)
    : device_(device), stage_(stage), entry_point_(shader_source.entry_point) {
  std::string source = ReadTextFile(shader_source.glsl_file);
  std::string info_log;

  auto precompiled_source = PrecompileShaders(source);
  if (!GLSLCompiler::CompileToSPIRV(stage, ConvertToBytes(precompiled_source),
                                    entry_point_, shader_variant, spirv_,
                                    info_log)) {
    throw std::runtime_error(
        fmt::format("Shader compilation failed for {}.\nInfo: {}",
                    shader_source.glsl_file, info_log));
  }

  if (!SPIRVReflection::ReflectShaderResources(stage, spirv_, resources_)) {
    throw std::runtime_error(fmt::format(
        "Cannot reflect shader resources of {}!", shader_source.glsl_file));
  }
  std::hash<std::string> hasher{};
  hash_id_ = hasher(std::string{spirv_.cbegin(), spirv_.cend()});
}

VkShaderStageFlagBits ShaderModule::GetStage() const { return stage_; }

const std::string& ShaderModule::GetEntryPoint() const { return entry_point_; }

void ShaderVariant::AddDef(const std::string& def) {
  processes_.push_back("D" + def);
  preamble_.append("#define " + def + "\n");

  UpdateHashId();
}

void ShaderVariant::AddUndef(const std::string& def) {
  processes_.push_back("U" + def);
  preamble_.append("#undef" + def + "\n");

  UpdateHashId();
}

void ShaderVariant::UpdateHashId() {
  std::hash<std::string> hasher{};
  hash_id_ = hasher(preamble_);
}

const std::string& ShaderVariant::GetPreamble() const { return preamble_; }

const std::vector<std::string>& ShaderVariant::GetProcesses() const {
  return processes_;
}

size_t ShaderVariant::GetHashId() const { return hash_id_; }

void ShaderVariant::Clear() {
  preamble_.clear();
  processes_.clear();
  UpdateHashId();
}

ShaderVariant::ShaderVariant() { UpdateHashId(); }
}  // namespace vkoo
