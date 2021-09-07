#pragma once

namespace vkoo {

enum class EventSource { Keyboard, Mouse, Scroll };

class InputEvent {
 public:
  InputEvent(EventSource source);
  EventSource GetSource() const { return source_; }

 private:
  EventSource source_;
};

enum class KeyAction { Down, Up, Repeat, Unknown };

class KeyInputEvent : public InputEvent {
 public:
  KeyInputEvent(int code, KeyAction action);
  int GetCode() const { return code_; }
  KeyAction GetAction() const { return action_; }

 private:
  int code_;
  KeyAction action_;
};

enum class MouseButton { Left, Right, Middle, Unknown };
enum class MouseAction { Down, Up, Move, Unknown };

class MouseButtonInputEvent : public InputEvent {
 public:
  MouseButtonInputEvent(MouseButton button, MouseAction action, float x_pos,
                        float y_pos);
  MouseButton GetButton() const { return button_; }
  MouseAction GetAction() const { return action_; }
  float GetXPos() const { return x_pos_; }
  float GetYPos() const { return y_pos_; }

 private:
  MouseButton button_;
  MouseAction action_;
  float x_pos_;
  float y_pos_;
};

class ScrollInputEvent : public InputEvent {
 public:
  ScrollInputEvent(double x_offset, double y_offset);
  double GetXOffset() const { return x_offset_; }
  double GetYOffset() const { return y_offset_; }

 private:
  double x_offset_;
  double y_offset_;
};

}  // namespace vkoo
