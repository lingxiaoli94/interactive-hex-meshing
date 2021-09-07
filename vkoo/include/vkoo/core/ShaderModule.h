#pragma once

#include "vkoo/common.h"

namespace vkoo {
class Device;
enum class ShaderResourceType {
  Input,
  InputAttachment,
  Output,
  Image,
  ImageSampler,
  ImageStorage,
  Sampler,
  BufferUniform,
  BufferStorage,
  PushConstant,
  SpecializationConstant,
  All
};

enum class ShaderResourceMode { Static, Dynamic, UpdateAfterBind };

struct ShaderResourceQualifiers {
  enum : uint32_t {
    None = 0,
    NonReadable = 1,
    NonWritable = 2,
  };
};

struct ShaderResource {
  VkShaderStageFlags stages;
  ShaderResourceType type;
  ShaderResourceMode mode;
  uint32_t set;
  uint32_t binding;
  uint32_t location;
  uint32_t input_attachment_index;
  uint32_t vec_size;
  uint32_t columns;
  uint32_t array_size;
  uint32_t offset;
  uint32_t size;
  uint32_t constant_id;
  uint32_t qualifiers;
  std::string name;
};

struct ShaderSource {
  std::string glsl_file;
  std::string entry_point;
};

class ShaderVariant {
 public:
  ShaderVariant();
  void AddDef(const std::string& def);
  void AddUndef(const std::string& def);
  const std::string& GetPreamble() const;
  const std::vector<std::string>& GetProcesses() const;
  size_t GetHashId() const;
  void Clear();

 private:
  void UpdateHashId();

  std::string preamble_;
  std::vector<std::string> processes_;
  size_t hash_id_;
};

class ShaderModule {
 public:
  ShaderModule(Device& device, VkShaderStageFlagBits stage,
               const ShaderSource& shader_source,
               const ShaderVariant& shader_variant);
  ShaderModule(ShaderModule&& other) = default;

  const std::vector<ShaderResource>& GetResources() const { return resources_; }
  VkShaderStageFlagBits GetStage() const;
  const std::string& GetEntryPoint() const;
  const std::vector<uint32_t>& GetBinary() const { return spirv_; }
  size_t GetHashID() const { return hash_id_; }

 private:
  [[maybe_unused]] Device& device_;
  VkShaderStageFlagBits stage_;
  std::string entry_point_;
  std::vector<uint32_t> spirv_;
  std::vector<ShaderResource> resources_;

  size_t hash_id_;
};
}  // namespace vkoo
