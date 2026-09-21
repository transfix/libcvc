#ifndef CVC_GL_SETTINGS_H
#define CVC_GL_SETTINGS_H

#include <cvc/core/state_object.h>
#include <functional>
#include <string>

namespace cvc {
class app;

namespace gl {

// Small state_objects for settings that live on classes which are not
// themselves state_objects (SceneGraph owns the scene tree; ImGuiOverlay is a
// render hook). Rather than convert those, each holds one of these and the
// settings become reachable from cvc::state like everything else.
//
// Each takes an `apply` callback invoked whenever the state changes, so a write
// from a script, a config file or a replicated peer drives the real object.

// Shadow-map settings for a SceneGraph: "<scene prefix>.shadows".
// Keys: enabled, resolution, interval.
class ShadowSettings : public cvc::state_object<ShadowSettings> {
public:
  struct Values {
    bool enabled = false;
    int resolution = 1024;
    int interval = 1;
  };
  ShadowSettings(cvc::app &ctx, const std::string &statePath, std::function<void(Values)> apply);

  static std::string sceneStatePath(const std::string &scenePrefix);

  void set(Values v); // writes state (which then applies)
  Values get() const;

protected:
  void handleStateChanged(const std::string &childState) override;

private:
  void seedState();
  Values m_v;
  std::function<void(Values)> m_apply;
};

// UI settings for an ImGuiOverlay: "<scene prefix>.viewers.<viewer>.ui".
// Keys: visible, scale, touch_mode, panels_open, toggle_button.
class UiSettings : public cvc::state_object<UiSettings> {
public:
  struct Values {
    bool visible = true;
    double scale = 1.0;
    bool touchMode = false;
    bool panelsOpen = true;
    bool toggleButton = true;
  };
  UiSettings(cvc::app &ctx, const std::string &statePath, std::function<void(Values)> apply);

  static std::string viewerStatePath(const std::string &scenePrefix, const std::string &viewerName);

  void set(Values v);
  Values get() const;

protected:
  void handleStateChanged(const std::string &childState) override;

private:
  void seedState();
  Values m_v;
  std::function<void(Values)> m_apply;
};

// Layout of a Viewport within its ViewportManager window:
// "<scene prefix>.viewers.<viewer>.layout". Keys: region.x0/y0/x1/y1 (normalized
// VTK y-up), layer, visible. Persisting these is what lets a picture-in-picture
// arrangement be saved and restored (or driven from a script / config / peer),
// alongside the viewport's camera at ".camera" and its active flag at
// "<scene prefix>.active_viewport".
class ViewportLayout : public cvc::state_object<ViewportLayout> {
public:
  struct Values {
    double region[4] = {0.0, 0.0, 1.0, 1.0};
    int layer = 0;
    bool visible = true;
  };
  ViewportLayout(cvc::app &ctx, const std::string &statePath, std::function<void(Values)> apply);

  static std::string viewerStatePath(const std::string &scenePrefix, const std::string &viewerName);

  void set(Values v);
  Values get() const;

protected:
  void handleStateChanged(const std::string &childState) override;

private:
  void seedState();
  Values m_v;
  std::function<void(Values)> m_apply;
  // Suppress our own object->state writes from re-entering handleStateChanged:
  // region is four correlated keys, so a partial read mid-write would corrupt
  // the ones not yet written.
  bool m_writing = false;
};

} // namespace gl
} // namespace cvc

#endif // CVC_GL_SETTINGS_H
