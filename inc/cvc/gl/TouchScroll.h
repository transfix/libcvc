#ifndef CVC_GL_TOUCH_SCROLL_H
#define CVC_GL_TOUCH_SCROLL_H

// Scrolling with a finger: what a phone's lists do and ImGui does not. A finger that drags across
// a window's content moves the content with it, the scroll bar untouched, and one that lifts while
// moving leaves the content gliding at the finger's speed, slowed by friction until it stops or
// meets an end. A press on content that is still gliding stops it and is not a tap.
//
// A drag is a scroll when it starts on a window's empty space or text, or on a widget a tap works
// (a button, a selectable, a checkbox, a text field the press would only start editing), once it
// has gone past ImGui's drag threshold: the widget's press is then let go, so it does not fire.
// Whatever already takes a finger's drag keeps it: a slider or a drag widget, a field being
// edited, a window's title bar, its resize grips and borders, a scroll bar, a table's column
// border, and an item that says so with keepTouchDrag (a scene view, the transfer function
// editor, a widget whose document handles on_drag:). The window scrolled is the innermost one
// under the finger that scrolls along the drag's main axis, as the mouse wheel's is.
//
// The glide: its speed v falls as v(t) = v0 * exp(-friction * t), so a fling at v0 covers
// v0 / friction before it stops. friction is per second: 2 is iOS's normal deceleration (0.998 of
// the speed kept each millisecond), 10 its fast one; 0 glides on to the end of the content.
//
// ImGuiOverlay runs one in touch mode, for every window it draws. Its settings are the state keys
// "<scene>.viewers.<viewer>.ui.touch_scroll" and ".ui.scroll_friction" (UiSettings).

namespace cvc {
namespace gl {

// The glide alone: a speed that friction slows. Units are the caller's (px and seconds).
class KineticScroll {
public:
  void fling(float speed); // starts gliding at `speed` (either sign)
  void stop();
  bool gliding() const { return m_speed != 0.0f; }
  float speed() const { return m_speed; }
  // Advances `dt` seconds and returns the distance covered (the speed's sign). The speed then
  // falls by exp(-friction * dt); under `stopSpeed` the glide is over.
  float step(float dt, float friction, float stopSpeed);

private:
  float m_speed = 0.0f;
};

// A finger's velocity from its recent positions: the change across the samples of the last
// `window` seconds, so a finger held still before it lifts has none.
class VelocityTracker {
public:
  static constexpr int kSamples = 16;
  void reset() { m_count = 0; }
  void add(double t, float x, float y);
  // px/s at time `now`, over the samples from `now - window` on (and the one just before, when
  // that leaves fewer than two).
  void velocity(double now, float window, float &vx, float &vy) const;

private:
  struct Sample {
    double t;
    float x, y;
  };
  Sample m_samples[kSamples] = {};
  int m_count = 0; // samples held, newest at (m_count - 1) % kSamples
};

// The gesture, on the current ImGui context.
class TouchScroll {
public:
  struct Settings {
    bool enabled = true;   // a finger's drag scrolls (ui.touch_scroll)
    float friction = 2.0f; // per second: how fast a glide slows (ui.scroll_friction)
  };
  static constexpr float kMaxFriction = 1000.0f;

  void setSettings(Settings s); // friction is clamped to [0, kMaxFriction]
  Settings settings() const { return m_settings; }

  // Each frame, on the current context: beginFrame after ImGui::NewFrame and before the
  // widgets (it moves the content of a drag or a glide, and catches a press on gliding content),
  // endFrame after the widgets and before ImGui::Render (it judges whether a finger's drag is a
  // scroll). Both read the left button as the finger.
  void beginFrame();
  void endFrame();
  // Drops a drag (letting the pointer go) or a glide. Call between NewFrame and Render.
  void reset();

  bool dragging() const { return m_phase == Phase::Dragging; }
  bool gliding() const { return m_phase == Phase::Gliding; }

private:
  enum class Phase { Idle, Pressed, Dragging, Gliding };
  void take(unsigned window, int axis); // the finger's drag scrolls `window`

  Settings m_settings;
  Phase m_phase = Phase::Idle;
  bool m_decided = false;       // this press's drag has been judged
  unsigned m_pressWindow = 0;   // the window under the press (ImGuiID)
  unsigned m_activeAtPress = 0; // what was active as the finger came down (a field being edited)
  unsigned m_target = 0;        // the window scrolled
  int m_axis = 1;               // 0 x, 1 y
  float m_anchorPointer = 0.0f; // the pointer and the scroll when the drag became a scroll
  float m_anchorScroll = 0.0f;
  float m_scroll = 0.0f;  // the glide's scroll, unrounded
  float m_applied = 0.0f; // the scroll last asked of the window
  KineticScroll m_glide;
  VelocityTracker m_tracker;
};

// The item just drawn takes a finger's drag itself, so a drag that starts on it is never a scroll
// (a scene view's orbit, a canvas, a widget with on_drag:). Call it after the item, each frame; it
// does nothing unless the item is active.
void keepTouchDrag();

} // namespace gl
} // namespace cvc

#endif // CVC_GL_TOUCH_SCROLL_H
