// App-state-tree binding for cvc::volren -- the modern replacement for the
// legacy `.cnf` config file.  Every renderer setting is two-way bound under
// "<prefix>.volren.*" following the cvcGL settings pattern (ShadowSettings):
// setters write state, handleStateChanged re-reads everything and invokes the
// apply callback, so a script, a UI, and a replicated peer all drive the same
// renderer.
//
// Key map (all under the constructor's statePath, conventionally
// state_settings::sceneStatePath(prefix) == "<prefix>.volren"):
//
//   camera.eye | camera.focal | camera.up      "x,y,z"
//   camera.projection                          int (0 perspective, 1 ortho)
//   camera.vfov_degrees | camera.parallel_scale double
//   image.width | image.height                 int
//   background                                 "r,g,b" in [0,1]
//   lights                                     "r,g,b,dx,dy,dz" x N (flat CSV)
//   cut_planes                                 "px,py,pz,nx,ny,nz" x N
//   steps                                      int
//   opacity_cutoff | depth_alpha_threshold     double
//   two_sided_lighting                         int 0/1
//   ambient                                    double
//   threads                                    int
//   supersample                                int (sub-samples per pixel EDGE)
//   shadows.enabled                            int 0/1
//   shadows.lights                             flat CSV of ints; "" = all cast
//   shadows.resolution                         int (light-view raster edge)
//   shadows.strength | .bias_scale             double
//   shadows.slope_scale                        double
//   shadows.min_occluder_opacity               double
//   volumes.count                              int
//   volumes.<i>.shaded | .unshaded             int 0/1
//   volumes.<i>.tf_auto_domain                 int 0/1
//   volumes.<i>.matrix                         16 row-major doubles (the cvcGL
//                                              GraphicsNode "matrix" encoding)
//   volumes.<i>.transfer_function.color        "value,r,g,b" x N
//   volumes.<i>.transfer_function.opacity      "value,a" x N
//   volumes.<i>.window                         "" or "min,max"
//   volumes.<i>.gradient_ramp                  "" or "r0,r1,r2[,plateau]"
//   volumes.<i>.isosurfaces                    "value,opacity,r,g,b,shininess" x N
//
// `shadows.lights` is a SEPARATE index list rather than extra fields on the
// `lights` key: the gradient_ramp trick of accepting 3 or 4 values cannot be
// applied to a repeated list, since 42 values would be both 7 lights of 6
// fields and 6 lights of 7.  An index list is the only unambiguous encoding.
//
// The transfer-function encoding (separate color and opacity point lists,
// flat comma-separated doubles) deliberately matches cvcGL VolumeNode's
// `transfer_function.color`/`.opacity` keys so the same editors drive both
// renderers; state_settings merges the two ramps into the combined
// cvc::volren::transfer_function.  Known encoding limitation: step transfer
// functions (two control points at the SAME scalar) are flattened by the
// merge -- use two distinct scalars an epsilon apart instead.
//
// Thread affinity: handlers run synchronously on whichever thread writes
// state; the snapshot is mutex-guarded so get()/apply_to() are safe from any
// thread, but set()/destruction should happen on one owning thread (the same
// contract as the other cvcGL settings objects).
#ifndef CVC_VOLREN_STATE_SETTINGS_H
#define CVC_VOLREN_STATE_SETTINGS_H

#include <cvc/core/state_object.h>
#include <cvc/volren/camera.h>
#include <cvc/volren/settings.h>
#include <functional>
#include <mutex>
#include <string>
#include <vector>

namespace cvc {
class app;

namespace volren {

class raycaster;

class state_settings : public cvc::state_object<state_settings> {
public:
  struct snapshot {
    volren::camera camera;
    render_settings settings;
    std::vector<volume_settings> volumes;
  };

  // `apply` is invoked (synchronously, on the writer's thread) whenever any
  // bound key changes.
  state_settings(cvc::app &ctx, const std::string &statePath,
                 std::function<void(const snapshot &)> apply = {});

  static std::string sceneStatePath(const std::string &prefix); // "<prefix>.volren"

  // Object -> state (does not invoke `apply`; that is for the other direction).
  void set(const snapshot &s);
  snapshot get() const;

  // Copy the current snapshot's camera/settings onto a raycaster.  Per-volume
  // settings apply by index to the raycaster's registered volumes; extras on
  // either side are ignored.
  void apply_to(raycaster &rc) const;

protected:
  void handleStateChanged(const std::string &childState) override;

private:
  void seedState(const snapshot &s); // writes s to the tree (handlers suppressed by caller)
  bool readAllFromState(snapshot &out) const;

  mutable std::mutex _snapMutex; // guards _snap (handlers run on writer threads)
  snapshot _snap;
  std::function<void(const snapshot &)> _apply;
};

} // namespace volren
} // namespace cvc

#endif // CVC_VOLREN_STATE_SETTINGS_H
