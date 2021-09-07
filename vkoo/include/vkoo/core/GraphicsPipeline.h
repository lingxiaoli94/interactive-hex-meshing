#pragma once

#include "PipelineState.h"

namespace vkoo {
class GraphicsPipeline {
 public:
  GraphicsPipeline(Device& device, PipelineState& pipeline_state);
  ~GraphicsPipeline();
  GraphicsPipeline(GraphicsPipeline&& other);

  VkPipeline GetHandle() const { return handle_; }

 private:
  Device& device_;
  VkPipeline handle_;
  PipelineState state_;
};
}  // namespace vkoo
