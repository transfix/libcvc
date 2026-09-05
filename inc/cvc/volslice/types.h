// cvc::volslice -- view-aligned slice volume renderer (VolumeRover2 port).
//
// Port of volumerover2's OpenGLVolumeRendering "VolumeLibrary" (Thane/Bajaj,
// UT Austin 2002-2003, arand patches through 2012): the classic back-to-front
// view-aligned-slice compositor that was VolumeRover2's main volume view.
// The port keeps the legacy slicing model exactly -- the plane sweep, the
// 256-case cube-clip table, the arand 2011 slice-count formula -- and drops
// the GL-extension implementation zoo (paletted textures, SGI color tables,
// NV/ARB fragment programs, Cg): on every driver since ~2010 only the ARB
// fragment-program colormapped path and the simple RGBA path ever ran, and
// both reduce to "sample a 3D texture, look the value up in a 256-entry TF".
//
// Math and transfer-function types are cvc::volren's, on purpose: one TF
// model (piecewise-linear ramp over the raw value domain, baked to a flat
// RGBA LUT) across every volume renderer in libcvc, and one mat4 convention.
#ifndef CVC_VOLSLICE_TYPES_H
#define CVC_VOLSLICE_TYPES_H

#include <cvc/core/exception.h>
#include <cvc/volren/transfer_function.h>
#include <cvc/volren/types.h>

namespace cvc {

CVC_DEF_EXCEPTION(volslice_error);

namespace volslice {

// Shared math/TF vocabulary (see header comment).
using volren::baked_transfer_function;
using volren::mat4;
using volren::rgba_f;
using volren::transfer_function;
using volren::transfer_point;
using volren::vec3d;

// How slices are blended, back to front.  The legacy renderer shipped with
// classic alpha blending hardcoded (glBlendFunc(GL_SRC_ALPHA,
// GL_ONE_MINUS_SRC_ALPHA)); additive was present in the sources only as a
// commented-out alternative, kept here as a real option.
enum class blend_mode : int {
  alpha = 0,    // src_alpha / one_minus_src_alpha (legacy shipped behavior)
  additive = 1, // src_alpha / one (emissive, X-ray-like)
};

// 3D texture sampling filter.  Legacy hardcoded GL_LINEAR.
enum class interpolation : int {
  linear = 0,
  nearest = 1,
};

namespace defaults {
// Slice density control in [0,1]; plane count N = 2*(10 + max_planes*q^3)
// (RendererBase::getIntervalWidth, the arand 6-14-2011 formula).  VolumeRover2
// initialized its quality sliders at 0.5.
inline constexpr double quality = 0.5;
// Scale of the quality curve AND the hard cap (10*max_planes) on planes per
// frame (RendererBase.cpp max_polygons).  Legacy default 1000.
inline constexpr int max_planes = 1000;
// Fraction of the volume diagonal cut away from the viewer side of the sweep
// (RendererBase::getNearestDistance).  0 renders the whole volume.
inline constexpr double near_plane = 0.0;
inline constexpr blend_mode blend = blend_mode::alpha;
inline constexpr interpolation filter = interpolation::linear;
// DEVIATION (opt-in): scale each slice's opacity for the actual inter-slice
// spacing (alpha' = 1 - (1-alpha)^(spacing/reference)), so changing `quality`
// changes sharpness instead of overall density.  The legacy renderer had no
// such correction -- its apparent density changed with the quality slider --
// so the faithful default is OFF.
inline constexpr bool opacity_correction = false;
// TF LUT entries; the legacy table was exactly 256 RGBA bytes.
inline constexpr std::size_t lut_size = 256;
} // namespace defaults

namespace limits {
inline constexpr double min_quality = 0.0; // legacy setQuality clamps to [0,1]
inline constexpr double max_quality = 1.0;
inline constexpr int min_max_planes = 1;
// 10*max_planes polygons are stored per frame; 100k slices of a <=6-vertex
// fan is ~7MB of vertex data, far past any visual difference.
inline constexpr int max_max_planes = 10000;
inline constexpr double min_near_plane = 0.0;
inline constexpr double max_near_plane = 1.0;
} // namespace limits

} // namespace volslice
} // namespace cvc

#endif // CVC_VOLSLICE_TYPES_H
