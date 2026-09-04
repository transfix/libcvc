// App-state-tree binding for cvc::volslice, following cvc::volren's
// state_settings pattern exactly: setters write state, handleStateChanged
// re-reads EVERYTHING with all-or-nothing parsing and invokes one apply
// callback, so a script, a UI, and a replicated peer all drive the same
// renderer.
//
// Key map (all under the constructor's statePath, conventionally
// "<prefix>.volslice"):
//
//   quality                        double in [0,1] (slice density; the legacy
//                                  VolumeRover2 quality slider)
//   max_planes                     int (CLAMPED on read; scales quality's
//                                  curve and caps planes/frame at 10x this)
//   near_plane                     double in [0,1] (fraction of the volume
//                                  diagonal cut from the viewer side)
//   interpolation                  int (0 linear, 1 nearest; REJECTED outside)
//   opacity_correction             int 0/1 (spacing-corrected slice alpha --
//                                  the opt-in deviation, see volslice/types.h)
//   tf_auto_domain                 int 0/1
//   window                         "" or "min,max" (raw value window)
//   transfer_function.color        "value,r,g,b" x N   (flat CSV)
//   transfer_function.opacity      "value,a" x N
//
// The transfer-function encoding matches cvcGL VolumeNode and cvc::volren
// verbatim, so one editor drives all three renderers.  The slicer's texture
// sub-cube (slice_params::tex_min/max) is deliberately NOT state-bound: it is
// a data-side crop the owning node computes, not a user setting.
#ifndef CVC_VOLSLICE_STATE_SETTINGS_H
#define CVC_VOLSLICE_STATE_SETTINGS_H

#include <cvc/core/state_object.h>
#include <cvc/volslice/settings.h>
#include <functional>
#include <mutex>
#include <string>

namespace cvc {
class app;

namespace volslice {

class state_settings : public cvc::state_object<state_settings> {
public:
  // `apply` is invoked (synchronously, on the writer's thread) whenever any
  // bound key changes.
  state_settings(cvc::app &ctx, const std::string &statePath,
                 std::function<void(const render_settings &)> apply = {});

  static std::string sceneStatePath(const std::string &prefix); // "<prefix>.volslice"

  // Object -> state (does not invoke `apply`; that is for the other direction).
  void set(const render_settings &s);
  render_settings get() const;

protected:
  void handleStateChanged(const std::string &childState) override;

private:
  void seedState(const render_settings &s);
  bool readAllFromState(render_settings &out) const;

  mutable std::mutex _mutex; // handlers run on writer threads
  render_settings _settings;
  std::function<void(const render_settings &)> _apply;
};

} // namespace volslice
} // namespace cvc

#endif // CVC_VOLSLICE_STATE_SETTINGS_H
