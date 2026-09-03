// Transfer function for cvc::volren.
//
// Replaces the legacy integer-density `coldentbl` (a uchar RGBA LUT indexed
// by (int)density) with a piecewise-linear ramp over the RAW VALUE DOMAIN,
// which makes Float/Double volumes first-class instead of asserting.  The
// marcher still samples a baked flat LUT, it is just instance-owned and
// value-domain instead of a global byte table.
#ifndef CVC_VOLREN_TRANSFER_FUNCTION_H
#define CVC_VOLREN_TRANSFER_FUNCTION_H

#include <cvc/volren/types.h>

#include <cstddef>
#include <vector>

namespace cvc {
namespace volren {

class baked_transfer_function;

// One control point: an RGBA color (channels in [0,1]) at a raw data value.
struct transfer_point {
  double value = 0.0;
  float r = 0.f, g = 0.f, b = 0.f, a = 0.f;
};

class transfer_function {
public:
  transfer_function() = default;
  explicit transfer_function(std::vector<transfer_point> points);

  // Insert a control point, keeping the list sorted by value.
  void add(const transfer_point &p);
  void clear() { _points.clear(); }
  bool empty() const { return _points.empty(); }
  const std::vector<transfer_point> &points() const { return _points; }

  // Piecewise-linear sample; clamps outside [front.value, back.value].
  // An empty function is fully transparent black.
  rgba_f sample(double value) const;

  // Extent of the control points ([0,1] when empty).
  double domain_min() const;
  double domain_max() const;

  // Convenience: bake this function into the flat LUT the marcher indexes.
  baked_transfer_function bake(double lo, double hi,
                               std::size_t size = defaults::lut_size) const;

private:
  std::vector<transfer_point> _points; // sorted by value
};

// The flat LUT the ray-marcher indexes: `size` RGBA entries uniform over
// [lo, hi], nearest-entry lookup (the legacy (int)density indexing), clamped
// at the ends.
class baked_transfer_function {
public:
  baked_transfer_function() = default;
  baked_transfer_function(const transfer_function &tf, double lo, double hi,
                          std::size_t size = defaults::lut_size);

  bool empty() const { return _lut.empty(); }
  double domain_min() const { return _lo; }
  double domain_max() const { return _hi; }
  std::size_t size() const { return _lut.size(); }

  rgba_f sample(double value) const {
    if (_lut.empty())
      return {};
    double t = (value - _lo) * _inv_width;
    // The inverted test also routes NaN (e.g. a NaN voxel in a Float volume)
    // to entry 0 instead of computing an undefined LUT index.
    if (!(t > 0.0))
      t = 0.0;
    else if (t > 1.0)
      t = 1.0;
    const std::size_t i = static_cast<std::size_t>(t * double(_lut.size() - 1) + 0.5);
    return _lut[i];
  }

private:
  std::vector<rgba_f> _lut;
  double _lo = 0.0, _hi = 1.0, _inv_width = 1.0;
};

// Gradient-magnitude opacity modulation -- the legacy `gradtbl` 2D-transfer-
// function precursor.  factor() is 0 below ramp0, rises linearly to `plateau`
// at ramp1, holds through ramp2, and is 0 above.  Disabled => factor 1.
struct gradient_opacity_ramp {
  bool enabled = false;
  double ramp0 = 0.0, ramp1 = 0.0, ramp2 = 0.0;
  double plateau = defaults::gradient_plateau;

  float factor(double gradient_magnitude) const {
    if (!enabled)
      return 1.0f;
    // The inverted lower test also maps a NaN magnitude to 0.  Note one
    // deliberate deviation from the legacy gradtbl: the legacy table index
    // clamped at 255, so with ramp2 >= 255 there was NO upper cutoff; here
    // magnitudes above ramp2 are 0 -- set ramp2 to a huge value (or
    // infinity) to reproduce the legacy uncapped configuration.
    if (!(gradient_magnitude >= ramp0) || gradient_magnitude > ramp2)
      return 0.0f;
    if (gradient_magnitude >= ramp1)
      return static_cast<float>(plateau);
    const double span = ramp1 - ramp0;
    if (span <= 0.0)
      return static_cast<float>(plateau);
    return static_cast<float>(plateau * (gradient_magnitude - ramp0) / span);
  }
};

} // namespace volren
} // namespace cvc

#endif // CVC_VOLREN_TRANSFER_FUNCTION_H
