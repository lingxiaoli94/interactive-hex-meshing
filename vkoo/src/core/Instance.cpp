#include "vkoo/core/Instance.h"

#include "vkoo/core/PhysicalDevice.h"

namespace vkoo {
namespace {
static VKAPI_ATTR VkBool32 VKAPI_CALL
DebugCallback(VkDebugUtilsMessageSeverityFlagBitsEXT message_severity,
              VkDebugUtilsMessageTypeFlagsEXT message_type,
              const VkDebugUtilsMessengerCallbackDataEXT* p_callback_data,
              void* pUserData) {
  std::cerr << "Validation layer: " << p_callback_data->pMessage << std::endl;
  throw std::runtime_error("Ugh!");

  return VK_FALSE;
}
}  // namespace

Instance::Instance(const std::string& application_name,
                   const std::vector<const char*>& required_extensions,
                   bool enable_validation_layer)
    : enable_validation_layer_(enable_validation_layer),
      enabled_extensions_(required_extensions) {
  if (enable_validation_layer_ && !CheckValidationLayerSupport()) {
    throw std::runtime_error("Validation layers requested but not available!");
  }

  VkApplicationInfo app_info{};
  app_info.sType = VK_STRUCTURE_TYPE_APPLICATION_INFO;
  app_info.pApplicationName = application_name.c_str();
  app_info.pEngineName = "No Engine";
  app_info.apiVersion = VK_API_VERSION_1_2;

  VkInstanceCreateInfo create_info{};
  create_info.sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO;
  create_info.pApplicationInfo = &app_info;

  if (enable_validation_layer_) {
    enabled_extensions_.push_back(VK_EXT_DEBUG_UTILS_EXTENSION_NAME);
  }
  create_info.enabledExtensionCount =
      static_cast<uint32_t>(enabled_extensions_.size());
  create_info.ppEnabledExtensionNames = enabled_extensions_.data();

  VkDebugUtilsMessengerCreateInfoEXT debug_create_info;
  if (enable_validation_layer_) {
    create_info.enabledLayerCount = 1;
    create_info.ppEnabledLayerNames = &kValidationLayerName;

    PopulateDebugMessengerCreateInfo(debug_create_info);
    create_info.pNext = &debug_create_info;
  } else {
    create_info.enabledLayerCount = 0;
    create_info.pNext = nullptr;
  }

  VK_CHECK(vkCreateInstance(&create_info, nullptr, &handle_));

  if (enable_validation_layer_) {
    auto func = (PFN_vkCreateDebugUtilsMessengerEXT)vkGetInstanceProcAddr(
        handle_, "vkCreateDebugUtilsMessengerEXT");
    if (func != nullptr) {
      VK_CHECK(func(handle_, &debug_create_info, nullptr, &debug_messenger_));
    } else {
      throw std::runtime_error(
          "Cannot find function vkCreateDebugUtilsMessengerEXT!");
    }
  }

  QueryGPUs();
}

Instance::~Instance() {
  if (debug_messenger_ != VK_NULL_HANDLE) {
    auto func = (PFN_vkDestroyDebugUtilsMessengerEXT)vkGetInstanceProcAddr(
        handle_, "vkDestroyDebugUtilsMessengerEXT");
    if (func != nullptr) {
      func(handle_, debug_messenger_, nullptr);
    }
  }
}

bool Instance::CheckValidationLayerSupport() {
  uint32_t layer_count;
  vkEnumerateInstanceLayerProperties(&layer_count, nullptr);

  std::vector<VkLayerProperties> available_layers(layer_count);
  vkEnumerateInstanceLayerProperties(&layer_count, available_layers.data());
  for (const auto& layer_properties : available_layers) {
    if (strcmp(kValidationLayerName, layer_properties.layerName) == 0)
      return true;
  }

  return false;
}

void Instance::PopulateDebugMessengerCreateInfo(
    VkDebugUtilsMessengerCreateInfoEXT& create_info) {
  create_info = {};
  create_info.sType = VK_STRUCTURE_TYPE_DEBUG_UTILS_MESSENGER_CREATE_INFO_EXT;
  create_info.messageSeverity =
      // VK_DEBUG_UTILS_MESSAGE_SEVERITY_VERBOSE_BIT_EXT |
      VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT |
      VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT;
  create_info.messageType = VK_DEBUG_UTILS_MESSAGE_TYPE_GENERAL_BIT_EXT |
                            VK_DEBUG_UTILS_MESSAGE_TYPE_VALIDATION_BIT_EXT |
                            VK_DEBUG_UTILS_MESSAGE_TYPE_PERFORMANCE_BIT_EXT;
  create_info.pfnUserCallback = DebugCallback;
}

void Instance::QueryGPUs() {
  uint32_t device_count = 0;
  vkEnumeratePhysicalDevices(handle_, &device_count, nullptr);

  if (device_count == 0) {
    throw std::runtime_error("Failed to find GPUs with Vulkan support!");
  }

  std::vector<VkPhysicalDevice> devices(device_count);
  vkEnumeratePhysicalDevices(handle_, &device_count, devices.data());

  for (auto device : devices) {
    gpus_.push_back(std::make_unique<PhysicalDevice>(*this, device));
  }
}

PhysicalDevice& Instance::GetSuitableGPU() {
  assert(!gpus_.empty() && "No physical devices were found on the system.");

  for (auto& gpu : gpus_) {
    if (gpu->GetProperties().deviceType ==
        VK_PHYSICAL_DEVICE_TYPE_DISCRETE_GPU) {
      return *gpu;
    }
  }

  return *gpus_[0];
}

}  // namespace vkoo
