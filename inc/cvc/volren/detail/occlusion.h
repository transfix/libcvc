// Ambient occlusion over a SIGNED DISTANCE volume, shared by raycaster.cpp and
// volren_kernels_test.  Header-only so the host and the device transcription in
// raycast.cu have exactly one source of truth for the estimator.
//
// WHY A DISTANCE FIELD MAKES THIS CHEAP.  The honest cost of ambient occlusion
// is a hemisphere integral: many rays, each marched.  A signed distance field
// collapses that to a handful of point samples, because f(q) is the distance
// from q to the NEAREST surface in ANY direction -- so one fetch certifies that
// a whole SPHERE of radius f(q) around q is empty.  Walking a few points out
// along the normal and asking, at each, "is the free sphere here as big as the
// distance I have travelled?" is therefore a genuine neighbourhood measurement
// and not a single-ray one, which is what makes five taps a usable answer where
// five shadow rays would be noise.
//
// The estimator is the standard SDF cone trace (Quilez), with the falloff and
// the normalization pinned below.  It is exact in the two cases that matter:
// an isolated convex surface reads 0 occlusion (f(p + n*h) == h everywhere, so
// every term vanishes), and a point sealed inside geometry reads 1.
#ifndef CVC_VOLREN_DETAIL_OCCLUSION_H
#define CVC_VOLREN_DETAIL_OCCLUSION_H

#include <cstdint>
#include <cvc/volren/detail/sampler.h>
#include <cvc/volren/types.h>

namespace cvc {
namespace volren {
namespace detail {

// The outward unit direction at a sample, in the volume's LOCAL frame, given
// its spline gradient.
//
// spline_gradient_cache returns value-units-per-CELL (index units), so on a
// grid with unequal spans its DIRECTION is not the local-space direction: the
// conversion is a divide by the span, per axis.  The shading normal skips that
// divide -- that is what the legacy renderer did, and correcting it would move
// every pixel of every anisotropic scene, so it stays a documented deviation --
// but the AO cone cannot skip it, because it MARCHES along this vector and
// compares the field against the distance travelled.  A direction wrong by the
// span ratio makes that comparison meaningless rather than merely off-axis.  On
// the cubic grids a distance field is normally sampled on, the two agree
// exactly, so this costs nothing and only ever helps.
inline vec3d local_outward(const grid_sampler &g, const vec3d &grad) {
  return normalized({grad.x / g.span.x, grad.y / g.span.y, grad.z / g.span.z});
}

// The occlusion FRACTION in [0,1] at local point `p` on the `iso` level set,
// looking out along the local unit direction `n` (0 == fully open sky, 1 ==
// sealed).  Callers turn it into a visibility with `1 - strength * fraction`.
//
// `g` must be a signed distance field in LOCAL units, positive outside
// (volume_settings::distance_field); `f(q) - iso` is then the distance from q to
// the rendered isosurface, which is what makes the level set irrelevant -- an
// offset surface at iso == 4 is the level set of an SDF too, so a decorative
// shell occludes exactly like the body.
//
// FALLOFF.  Term i is weighted 1/i, not the reflex 1/2^(i-1).  Geometric
// falloff is the textbook choice and it is wrong for a knob-driven radius: at
// the 16-sample ceiling it gives the outer HALF of the cone 0.006% of the
// answer, so `radius` silently stops meaning anything past a few taps.  1/i
// still puts the near taps in charge -- the first tap outweighs the last by
// `samples` to one -- while leaving the far end able to register a wall.
//
// A tap that leaves the grid is counted as UNOCCLUDED with its full weight
// rather than dropped: dropping it renormalizes the cone onto the taps that
// remain, so a surface near the volume's boundary would darken as its cone left
// the data, which is precisely backwards.
//
// Deterministic by construction: fixed offsets, fixed order, no jitter.
inline float sdf_occlusion(const grid_sampler &g, const vec3d &p, const vec3d &n, double iso,
                           double radius, int samples) {
  // No direction, no cone.  local_outward() returns {0,0,0} for a gradient
  // below normalized()'s epsilon (and NaN for a NaN field), and marching zero
  // distance would sample the SURFACE at every tap -- f - iso == 0 against a
  // travelled distance of h -- which reads as FULLY OCCLUDED and would crush
  // exactly the flat samples that already get no diffuse or specular to black.
  // Failing UNOCCLUDED is the same non-destructive direction the shadow lookup
  // takes when it cannot answer; the inverted test catches NaN too.
  if (!(dot(n, n) > 0.0))
    return 0.f;
  double occ = 0.0, wsum = 0.0;
  for (int i = 1; i <= samples; ++i) {
    const double h = radius * double(i) / double(samples);
    const double w = 1.0 / double(i);
    wsum += w;
    const vec3d q = p + n * h;
    std::int64_t idx[3];
    if (!g.cell_index(q, idx))
      continue; // outside the volume: nothing there to occlude with
    float vals[8], w3[3];
    g.corners(idx[0], idx[1], idx[2], vals);
    g.local_weights(q, idx[0], idx[1], idx[2], w3);
    // How much of this step's clearance the geometry took away.  An exact SDF
    // in open space returns d == h and contributes exactly nothing.
    const double d = double(trilinear(w3, vals)) - iso;
    double frac = (h - d) / h;
    if (!(frac > 0.0)) // inverted: a NaN voxel reads as UNOCCLUDED, not as dark
      frac = 0.0;
    else if (frac > 1.0) // d < 0: the tap is inside geometry
      frac = 1.0;
    occ += w * frac;
  }
  // wsum >= 1 for every samples >= 1; the guard is for a caller that passes 0.
  return wsum > 0.0 ? float(occ / wsum) : 0.f;
}

} // namespace detail
} // namespace volren
} // namespace cvc

#endif // CVC_VOLREN_DETAIL_OCCLUSION_H
