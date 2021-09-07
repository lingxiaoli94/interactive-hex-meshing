#include "vkoo/core/InputEvent.h"

namespace vkoo {
InputEvent::InputEvent(EventSource source) : source_{source} {}

KeyInputEvent::KeyInputEvent(int code, KeyAction action)
    : InputEvent{EventSource::Keyboard}, code_{code}, action_{action} {}

MouseButtonInputEvent::MouseButtonInputEvent(MouseButton button,
                                             MouseAction action, float x_pos,
                                             float y_pos)
    : InputEvent{EventSource::Mouse},
      button_{button},
      action_{action},
      x_pos_{x_pos},
      y_pos_{y_pos} {}

ScrollInputEvent::ScrollInputEvent(double x_offset, double y_offset)
    : InputEvent{EventSource::Scroll},
      x_offset_{x_offset},
      y_offset_{y_offset} {}
}  // namespace vkoo
