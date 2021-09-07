#pragma once

#include "vkoo/common.h"

namespace vkoo {
class PhysicalDevice;

class Instance {
 public:
  Instance(const std::string& application_name,
           const std::vector<const char*>& extensions,
           bool enable_validation_layer);

  ~Instance();

  VkInstance GetHandle() const { return handle_; }
  PhysicalDevice& GetSuitableGPU();

 private:
  bool CheckValidationLayerSupport();
  void PopulateDebugMessengerCreateInfo(
      VkDebugUtilsMessengerCreateInfoEXT& create_info);
  void QueryGPUs();

  VkInstance handle_;

  bool enable_validation_layer_;
  std::vector<const char*> enabled_extensions_;
  const char* kValidationLayerName = "VK_LAYER_KHRONOS_validation";
  VkDebugUtilsMessengerEXT debug_messenger_;
  std::vector<std::unique_ptr<PhysicalDevice>> gpus_;
};
}  // namespace vkoo