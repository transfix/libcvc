#include <algorithm>
#include <cvc/volren/transfer_function.h>

namespace cvc {
namespace volren {

transfer_function::transfer_function(std::vector<transfer_point> points)
    : _points(std::move(points)) {
  std::stable_sort(
      _points.begin(), _points.end(),
      [](const transfer_point &a, const transfer_point &b) { return a.value < b.value; });
}

void transfer_function::add(const transfer_point &p) {
  const auto it = std::upper_bound(
      _points.begin(), _points.end(), p,
      [](const transfer_point &a, const transfer_point &b) { return a.value < b.value; });
  _points.insert(it, p);
}

rgba_f transfer_function::sample(double value) const {
  if (_points.empty())
    return {};
  if (value <= _points.front().value) {
    const transfer_point &p = _points.front();
    return {p.r, p.g, p.b, p.a};
  }
  if (value >= _points.back().value) {
    const transfer_point &p = _points.back();
    return {p.r, p.g, p.b, p.a};
  }
  // First point with .value >= value; its predecessor exists by the guards.
  const auto hi = std::lower_bound(_points.begin(), _points.end(), value,
                                   [](const transfer_point &p, double v) { return p.value < v; });
  const auto lo = hi - 1;
  const double span = hi->value - lo->value;
  const float t = span > 0.0 ? float((value - lo->value) / span) : 0.f;
  return {lo->r + t * (hi->r - lo->r), lo->g + t * (hi->g - lo->g), lo->b + t * (hi->b - lo->b),
          lo->a + t * (hi->a - lo->a)};
}

double transfer_function::domain_min() const {
  return _points.empty() ? 0.0 : _points.front().value;
}

double transfer_function::domain_max() const {
  return _points.empty() ? 1.0 : _points.back().value;
}

baked_transfer_function transfer_function::bake(double lo, double hi, std::size_t size) const {
  return baked_transfer_function(*this, lo, hi, size);
}

baked_transfer_function::baked_transfer_function(const transfer_function &tf, double lo, double hi,
                                                 std::size_t size)
    : _lo(lo), _hi(hi) {
  if (tf.empty() || size < 2 || !(hi > lo)) {
    // Degenerate: stay empty (fully transparent) rather than bake a LUT that
    // would return one value for every input.
    _lo = lo;
    _hi = hi;
    _inv_width = 1.0;
    return;
  }
  _inv_width = 1.0 / (hi - lo);
  _lut.resize(size);
  for (std::size_t i = 0; i < size; ++i) {
    const double v = lo + (hi - lo) * double(i) / double(size - 1);
    _lut[i] = tf.sample(v);
  }
}

} // namespace volren
} // namespace cvc
