#include "vkoo/st/Image.h"

#include "vkoo/utils.h"

#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>

namespace vkoo {
namespace st {
Image::Image(std::vector<uint8_t>&& data, std::vector<Mipmap>&& mipmaps,
             VkFormat format)
    : data_{std::move(data)}, mipmaps_{mipmaps}, format_{format} {}

void Image::CreateVkImageAndView(Device& device,
                                 VkImageViewType image_view_type,
                                 VkImageCreateFlags flags) {
  vk_image_ = std::make_unique<core::Image>(
      device, GetExtent(), format_,
      VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT,
      VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, VK_SAMPLE_COUNT_1_BIT,
      VK_IMAGE_TILING_OPTIMAL, flags);

  vk_image_view_ =
      std::make_unique<core::ImageView>(*vk_image_, image_view_type);
}

void Image::UploadDataToGPU(Device& device) {
  auto& command_buffer = device.RequestCommandBuffer();
  command_buffer.Begin(VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT);

  core::Buffer staging_buffer{device, data_.size(),
                              VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                              VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
                                  VK_MEMORY_PROPERTY_HOST_COHERENT_BIT};
  staging_buffer.Update(data_);
  {
    ImageMemoryBarrier memory_barrier{};
    memory_barrier.old_layout = VK_IMAGE_LAYOUT_UNDEFINED;
    memory_barrier.new_layout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
    memory_barrier.src_access_mask = 0;
    memory_barrier.dst_access_mask = VK_ACCESS_TRANSFER_WRITE_BIT;
    memory_barrier.src_stage_mask = VK_PIPELINE_STAGE_HOST_BIT;
    memory_barrier.dst_stage_mask = VK_PIPELINE_STAGE_TRANSFER_BIT;

    command_buffer.InsertImageMemoryBarrier(*vk_image_view_, memory_barrier);
  }

  // Create a buffer image copy for every mip level.
  auto& mipmaps = mipmaps_;

  std::vector<VkBufferImageCopy> buffer_copy_regions(mipmaps.size());

  for (size_t i = 0; i < mipmaps.size(); i++) {
    auto& mipmap = mipmaps[i];
    auto& copy_region = buffer_copy_regions[i];

    copy_region.bufferOffset = mipmap.offset;
    copy_region.imageSubresource = vk_image_view_->GetSubresourceLayers();

    copy_region.imageSubresource.mipLevel = mipmap.level;
    copy_region.imageExtent.width = mipmap.extent.width;
    copy_region.imageExtent.height = mipmap.extent.height;
    copy_region.imageExtent.depth = 1;
  }

  command_buffer.CopyBufferToImage(staging_buffer, *vk_image_,
                                   buffer_copy_regions);

  {
    ImageMemoryBarrier memory_barrier{};
    memory_barrier.old_layout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
    memory_barrier.new_layout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
    memory_barrier.src_access_mask = VK_ACCESS_TRANSFER_WRITE_BIT;
    memory_barrier.dst_access_mask = VK_ACCESS_SHADER_READ_BIT;
    memory_barrier.src_stage_mask = VK_PIPELINE_STAGE_TRANSFER_BIT;
    memory_barrier.dst_stage_mask = VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT;

    command_buffer.InsertImageMemoryBarrier(*vk_image_view_, memory_barrier);
  }
  command_buffer.End();

  auto& queue = device.GetQueueByFlags(VK_QUEUE_GRAPHICS_BIT, 0);

  queue.Submit(command_buffer, device.RequestFence());

  device.GetFencePool().Wait();
  device.GetFencePool().Reset();
  device.GetCommandPool().ResetPool();
  device.WaitIdle();
}

std::unique_ptr<Image> Image::Load(const std::string& image_path) {
  std::vector<uint8_t> data = ReadBinaryFile(image_path);
  int width;
  int height;
  int comp;
  int req_comp = 4;

  auto data_buffer = reinterpret_cast<const stbi_uc*>(data.data());
  auto data_size = static_cast<int>(data.size());

  auto raw_data = stbi_load_from_memory(data_buffer, data_size, &width, &height,
                                        &comp, req_comp);

  if (!raw_data) {
    throw std::runtime_error{"Failed to load " + image_path + ": " +
                             stbi_failure_reason()};
  }

  Mipmap mipmap{};
  mipmap.extent.width = width;
  mipmap.extent.height = height;

  std::vector<uint8_t> image_data{raw_data,
                                  raw_data + width * height * req_comp};
  auto image = std::make_unique<Image>(std::move(image_data),
                                       std::vector<Mipmap>{mipmap});

  stbi_image_free(raw_data);

  return image;
}
}  // namespace st
}  // namespace vkoo
