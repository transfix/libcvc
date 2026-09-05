// Renderer-side settings for cvc::volslice -- everything beyond the slice
// geometry itself (slicer.h owns the geometric subset, slice_params).
#ifndef CVC_VOLSLICE_SETTINGS_H
#define CVC_VOLSLICE_SETTINGS_H

#include <cvc/volslice/slicer.h>
#include <cvc/volslice/types.h>

namespace cvc {
namespace volslice {

// The full tunable surface of the slice renderer.  Legacy mapping:
//   slices.quality / .max_planes / .near_plane  setQuality/setMaxPlanes/setNearPlane
//   filter                                      hardcoded GL_LINEAR in every impl
//   opacity_correction                          NEW (deviation, default off; see types.h)
//   tf / window                                 uploadColorMap + the UChar coercion window
// The legacy blend mode is NOT a setting: the library shipped exactly one
// (SRC_ALPHA/ONE_MINUS_SRC_ALPHA; additive existed only as commented-out dead
// code), and that one is what the node renders with.
struct render_settings {
  slice_params slices;
  interpolation filter = defaults::filter;
  bool opacity_correction = defaults::opacity_correction;

  // Transfer function over the RAW value domain (same model and state
  // encoding as cvc::volren and cvcGL VolumeNode -- one editor drives all
  // three renderers).
  transfer_function tf;
  // When true (default), the TF domain follows the volume's data range; when
  // false, `window` below is the domain.
  bool tf_auto_domain = true;
  // Explicit value window [min,max] (the legacy UChar-coercion range).  Both
  // zero means "unset" -- with tf_auto_domain false and no window the node
  // falls back to the volume's data range.
  double window_min = 0.0;
  double window_max = 0.0;
};

} // namespace volslice
} // namespace cvc

#endif // CVC_VOLSLICE_SETTINGS_H
