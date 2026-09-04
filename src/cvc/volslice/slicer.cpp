// The slicing engine (see slicer.h): a faithful port of the legacy
// RendererBase plane sweep + ClipCube 256-case cube clipper, minus GL.
//
// Sources ported: volrover/src/VolumeRenderer/{RendererBase,ClipCube}.cpp and
// inc/VolumeRenderer/LookupTables.h (VolumeLibrary, LGPL, UT Austin
// 2002-2003).  The lookup tables are transcribed verbatim; the sweep keeps
// the arand 6-14-2011 slice-count formula and the 10*max_planes cap.
#include <cmath>
#include <cvc/volslice/slicer.h>

namespace cvc {
namespace volslice {

namespace {

// ---- Legacy lookup tables (LookupTables.h, verbatim) -----------------------

// The eight corners of the origin-centered unit cube (scaled by the aspect
// ratio before use).  Corner index bit pattern: bit0=+x, bit1=+y, bit2=+z.
constexpr double kVertCoords[8 * 3] = {
    -0.5, -0.5, -0.5, 0.5, -0.5, -0.5, -0.5, 0.5, -0.5, 0.5, 0.5, -0.5,
    -0.5, -0.5, 0.5,  0.5, -0.5, 0.5,  -0.5, 0.5, 0.5,  0.5, 0.5, 0.5,
};

// Base texture coordinate (0/1) per corner, remapped to the sub-cube range.
constexpr double kTexCoords[8 * 3] = {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0, 1.0, 0.0,
                                      0.0, 0.0, 1.0, 1.0, 0.0, 1.0, 0.0, 1.0, 1.0, 1.0, 1.0, 1.0};

// The two corner indices of each of the 12 cube edges.
constexpr unsigned kEdges[12 * 2] = {0, 1, 1, 3, 3, 2, 2, 0, 4, 5, 5, 7,
                                     7, 6, 6, 4, 0, 4, 1, 5, 3, 7, 2, 6};

// For each of the 256 above/below corner sign patterns: {vertex count, then
// that many edge indices IN FAN ORDER}.  Only patterns a single plane can
// produce are populated; the rest are zero rows (a plane cannot generate the
// marching-cubes ambiguous configurations).
constexpr unsigned kEdgeCases[256][7] = {
    {0, 0, 0, 0, 0, 0, 0},   {3, 3, 0, 8, 0, 0, 0},   {3, 1, 0, 9, 0, 0, 0},
    {4, 3, 1, 9, 8, 0, 0},   {3, 2, 11, 3, 0, 0, 0},  {4, 2, 11, 8, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0},   {5, 8, 11, 2, 1, 9, 0},  {3, 2, 10, 1, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0},   {4, 2, 10, 9, 0, 0, 0},  {5, 9, 10, 2, 3, 8, 0},
    {4, 10, 11, 3, 1, 0, 0}, {5, 8, 11, 10, 1, 0, 0}, {5, 9, 10, 11, 3, 0, 0},
    {4, 10, 11, 8, 9, 0, 0}, {3, 7, 4, 8, 0, 0, 0},   {4, 3, 7, 4, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0},   {5, 7, 3, 1, 9, 4, 0},   {0, 0, 0, 0, 0, 0, 0},
    {5, 0, 2, 11, 7, 4, 0},  {0, 0, 0, 0, 0, 0, 0},   {6, 1, 9, 4, 7, 11, 2},
    {0, 0, 0, 0, 0, 0, 0},   {0, 0, 0, 0, 0, 0, 0},   {0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0},   {0, 0, 0, 0, 0, 0, 0},   {0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0},   {5, 9, 10, 11, 7, 4, 0}, {3, 5, 9, 4, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0},   {4, 1, 5, 4, 0, 0, 0},   {5, 3, 1, 5, 4, 8, 0},
    {0, 0, 0, 0, 0, 0, 0},   {0, 0, 0, 0, 0, 0, 0},   {0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0},   {0, 0, 0, 0, 0, 0, 0},   {0, 0, 0, 0, 0, 0, 0},
    {5, 0, 2, 10, 5, 4, 0},  {6, 3, 8, 4, 5, 10, 2},  {0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0},   {0, 0, 0, 0, 0, 0, 0},   {5, 8, 11, 10, 5, 4, 0},
    {4, 5, 7, 8, 9, 0, 0},   {5, 5, 7, 3, 0, 9, 0},   {5, 1, 5, 7, 8, 0, 0},
    {4, 3, 1, 5, 7, 0, 0},   {0, 0, 0, 0, 0, 0, 0},   {0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0},   {5, 1, 5, 7, 11, 2, 0},  {0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0},   {0, 0, 0, 0, 0, 0, 0},   {5, 5, 7, 3, 2, 10, 0},
    {0, 0, 0, 0, 0, 0, 0},   {0, 0, 0, 0, 0, 0, 0},   {0, 0, 0, 0, 0, 0, 0},
    {4, 10, 11, 7, 5, 0, 0}, {3, 6, 11, 7, 0, 0, 0},  {0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0},   {0, 0, 0, 0, 0, 0, 0},   {4, 2, 6, 7, 3, 0, 0},
    {5, 0, 2, 6, 7, 8, 0},   {0, 0, 0, 0, 0, 0, 0},   {0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0},   {0, 0, 0, 0, 0, 0, 0},   {0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0},   {5, 7, 3, 1, 10, 6, 0},  {6, 1, 0, 8, 7, 6, 10},
    {0, 0, 0, 0, 0, 0, 0},   {5, 9, 10, 6, 7, 8, 0},  {4, 6, 11, 8, 4, 0, 0},
    {5, 4, 6, 11, 3, 0, 0},  {0, 0, 0, 0, 0, 0, 0},   {0, 0, 0, 0, 0, 0, 0},
    {5, 4, 6, 2, 3, 8, 0},   {4, 2, 6, 4, 0, 0, 0},   {0, 0, 0, 0, 0, 0, 0},
    {5, 4, 6, 2, 1, 9, 0},   {0, 0, 0, 0, 0, 0, 0},   {0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0},   {0, 0, 0, 0, 0, 0, 0},   {0, 0, 0, 0, 0, 0, 0},
    {5, 4, 6, 10, 1, 0, 0},  {0, 0, 0, 0, 0, 0, 0},   {4, 10, 6, 4, 9, 0, 0},
    {0, 0, 0, 0, 0, 0, 0},   {0, 0, 0, 0, 0, 0, 0},   {0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0},   {0, 0, 0, 0, 0, 0, 0},   {0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0},   {0, 0, 0, 0, 0, 0, 0},   {0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0},   {0, 0, 0, 0, 0, 0, 0},   {0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0},   {0, 0, 0, 0, 0, 0, 0},   {0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0},   {5, 8, 11, 6, 5, 9, 0},  {6, 3, 0, 9, 5, 6, 11},
    {0, 0, 0, 0, 0, 0, 0},   {5, 3, 1, 5, 6, 11, 0},  {0, 0, 0, 0, 0, 0, 0},
    {5, 0, 2, 6, 5, 9, 0},   {0, 0, 0, 0, 0, 0, 0},   {4, 2, 6, 5, 1, 0, 0},
    {0, 0, 0, 0, 0, 0, 0},   {0, 0, 0, 0, 0, 0, 0},   {0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0},   {0, 0, 0, 0, 0, 0, 0},   {0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0},   {3, 10, 6, 5, 0, 0, 0},  {3, 10, 6, 5, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0},   {0, 0, 0, 0, 0, 0, 0},   {0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0},   {0, 0, 0, 0, 0, 0, 0},   {0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0},   {4, 2, 6, 5, 1, 0, 0},   {0, 0, 0, 0, 0, 0, 0},
    {5, 0, 2, 6, 5, 9, 0},   {0, 0, 0, 0, 0, 0, 0},   {5, 3, 1, 5, 6, 11, 0},
    {0, 0, 0, 0, 0, 0, 0},   {6, 3, 0, 9, 5, 6, 11},  {5, 8, 11, 6, 5, 9, 0},
    {0, 0, 0, 0, 0, 0, 0},   {0, 0, 0, 0, 0, 0, 0},   {0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0},   {0, 0, 0, 0, 0, 0, 0},   {0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0},   {0, 0, 0, 0, 0, 0, 0},   {0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0},   {0, 0, 0, 0, 0, 0, 0},   {0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0},   {0, 0, 0, 0, 0, 0, 0},   {0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0},   {4, 10, 6, 4, 9, 0, 0},  {0, 0, 0, 0, 0, 0, 0},
    {5, 4, 6, 10, 1, 0, 0},  {0, 0, 0, 0, 0, 0, 0},   {0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0},   {0, 0, 0, 0, 0, 0, 0},   {0, 0, 0, 0, 0, 0, 0},
    {5, 4, 6, 2, 1, 9, 0},   {0, 0, 0, 0, 0, 0, 0},   {4, 2, 6, 4, 0, 0, 0},
    {5, 4, 6, 2, 3, 8, 0},   {0, 0, 0, 0, 0, 0, 0},   {0, 0, 0, 0, 0, 0, 0},
    {5, 4, 6, 11, 3, 0, 0},  {4, 6, 11, 8, 4, 0, 0},  {5, 9, 10, 6, 7, 8, 0},
    {0, 0, 0, 0, 0, 0, 0},   {6, 1, 0, 8, 7, 6, 10},  {5, 7, 3, 1, 10, 6, 0},
    {0, 0, 0, 0, 0, 0, 0},   {0, 0, 0, 0, 0, 0, 0},   {0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0},   {0, 0, 0, 0, 0, 0, 0},   {0, 0, 0, 0, 0, 0, 0},
    {5, 0, 2, 6, 7, 8, 0},   {4, 2, 6, 7, 3, 0, 0},   {0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0},   {0, 0, 0, 0, 0, 0, 0},   {3, 6, 11, 7, 0, 0, 0},
    {4, 10, 11, 7, 5, 0, 0}, {0, 0, 0, 0, 0, 0, 0},   {0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0},   {5, 5, 7, 3, 2, 10, 0},  {0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0},   {0, 0, 0, 0, 0, 0, 0},   {5, 1, 5, 7, 11, 2, 0},
    {0, 0, 0, 0, 0, 0, 0},   {0, 0, 0, 0, 0, 0, 0},   {0, 0, 0, 0, 0, 0, 0},
    {4, 3, 1, 5, 7, 0, 0},   {5, 1, 5, 7, 8, 0, 0},   {5, 5, 7, 3, 0, 9, 0},
    {4, 5, 7, 8, 9, 0, 0},   {5, 8, 11, 10, 5, 4, 0}, {0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0},   {0, 0, 0, 0, 0, 0, 0},   {6, 3, 8, 4, 5, 10, 2},
    {5, 0, 2, 10, 5, 4, 0},  {0, 0, 0, 0, 0, 0, 0},   {0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0},   {0, 0, 0, 0, 0, 0, 0},   {0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0},   {5, 3, 1, 5, 4, 8, 0},   {4, 1, 5, 4, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0},   {3, 5, 9, 4, 0, 0, 0},   {5, 9, 10, 11, 7, 4, 0},
    {0, 0, 0, 0, 0, 0, 0},   {0, 0, 0, 0, 0, 0, 0},   {0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0},   {0, 0, 0, 0, 0, 0, 0},   {0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0},   {6, 1, 9, 4, 7, 11, 2},  {0, 0, 0, 0, 0, 0, 0},
    {5, 0, 2, 11, 7, 4, 0},  {0, 0, 0, 0, 0, 0, 0},   {5, 7, 3, 1, 9, 4, 0},
    {0, 0, 0, 0, 0, 0, 0},   {4, 3, 7, 4, 0, 0, 0},   {3, 7, 4, 8, 0, 0, 0},
    {4, 10, 11, 8, 9, 0, 0}, {5, 9, 10, 11, 3, 0, 0}, {5, 8, 11, 10, 1, 0, 0},
    {4, 10, 11, 3, 1, 0, 0}, {5, 9, 10, 2, 3, 8, 0},  {4, 2, 10, 9, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0},   {3, 2, 10, 1, 0, 0, 0},  {5, 8, 11, 2, 1, 9, 0},
    {0, 0, 0, 0, 0, 0, 0},   {4, 2, 11, 8, 0, 0, 0},  {3, 2, 11, 3, 0, 0, 0},
    {4, 3, 1, 9, 8, 0, 0},   {3, 1, 0, 9, 0, 0, 0},   {3, 3, 0, 8, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0}};

// ---- Legacy ClipCube, in ratio space ---------------------------------------

double clampd(double v, double lo, double hi) { return v < lo ? lo : (v > hi ? hi : v); }

// One plane/cube intersection.  `ratio` is the aspect-normalized cube extent,
// `tex` the 8 corners' sub-cube texcoords, `n` the unit plane normal and `d`
// the swept plane offset (signed distance = n.p + d).  Appends the fan to
// `out` and returns true if the plane cuts the cube (ClipCube::clipPlane).
bool clip_plane(slice_geometry &out, const vec3d &ratio, const double tex[8 * 3], const vec3d &n,
                double d) {
  double dist[8];
  unsigned caseIndex = 0;
  for (unsigned c = 0; c < 8; ++c) {
    const double px = kVertCoords[c * 3 + 0] * ratio.x;
    const double py = kVertCoords[c * 3 + 1] * ratio.y;
    const double pz = kVertCoords[c * 3 + 2] * ratio.z;
    dist[c] = n.x * px + n.y * py + n.z * pz + d;
    if (dist[c] > 0.0)
      caseIndex |= 1u << c;
  }

  const unsigned count = kEdgeCases[caseIndex][0];
  if (count == 0)
    return false;

  const auto first = static_cast<std::uint32_t>(out.vertices());
  for (unsigned c = 0; c < count; ++c) {
    const unsigned edge = kEdgeCases[caseIndex][c + 1];
    const unsigned v1 = kEdges[edge * 2 + 0], v2 = kEdges[edge * 2 + 1];
    const double total = std::fabs(dist[v1]) + std::fabs(dist[v2]);
    const double alpha = total != 0.0 ? std::fabs(dist[v1]) / total : 0.0;

    for (int axis = 0; axis < 3; ++axis) {
      const double r = axis == 0 ? ratio.x : (axis == 1 ? ratio.y : ratio.z);
      out.positions.push_back(static_cast<float>(kVertCoords[v1 * 3 + axis] * r * (1.0 - alpha) +
                                                 kVertCoords[v2 * 3 + axis] * r * alpha));
      out.texcoords.push_back(
          static_cast<float>(tex[v1 * 3 + axis] * (1.0 - alpha) + tex[v2 * 3 + axis] * alpha));
    }
  }
  out.fan_offset.push_back(first);
  out.fan_count.push_back(count);
  return true;
}

} // namespace

vec3d view_plane_normal(const mat4 &local_to_clip) {
  // The near/view plane of the clip frustum pulled back into local space:
  // coefficients are (row 4 + row 3) of the combined matrix -- the classic
  // viewcull.c identity the legacy getViewPlane() used, with the matrix an
  // explicit argument instead of GL state.
  const auto &m = local_to_clip.m;
  vec3d n{m[12] + m[8], m[13] + m[9], m[14] + m[10]};
  const double len = std::sqrt(n.x * n.x + n.y * n.y + n.z * n.z);
  if (len == 0.0)
    throw volslice_error("view_plane_normal: degenerate projection matrix");
  return {n.x / len, n.y / len, n.z / len};
}

slice_geometry compute_slices(const mat4 &local_to_clip, const box3d &box,
                              const slice_params &params) {
  slice_geometry out;

  // Box -> legacy ratio space (see slicer.h "Space conventions").
  const vec3d dim{box.max.x - box.min.x, box.max.y - box.min.y, box.max.z - box.min.z};
  const double longest = std::fmax(dim.x, std::fmax(dim.y, dim.z));
  if (longest <= 0.0)
    return out;
  const vec3d ratio{dim.x / longest, dim.y / longest, dim.z / longest};
  const vec3d center{(box.min.x + box.max.x) * 0.5, (box.min.y + box.max.y) * 0.5,
                     (box.min.z + box.max.z) * 0.5};

  // ratio -> local is translate(center) * scale(longest); fold it into the
  // matrix so the plane extraction happens in ratio space, exactly where the
  // legacy algorithm ran.
  mat4 ratio_to_local;
  ratio_to_local.m = {longest, 0, 0,       center.x, 0, longest, 0, center.y,
                      0,       0, longest, center.z, 0, 0,       0, 1};
  const mat4 ratio_to_clip = local_to_clip * ratio_to_local;

  const vec3d n = view_plane_normal(ratio_to_clip);

  // Sub-cube texcoords per corner (ClipCube::setTextureSubCube).
  double tex[8 * 3];
  for (int i = 0; i < 8; ++i) {
    tex[i * 3 + 0] = kTexCoords[i * 3 + 0] < 0.5 ? params.tex_min.x : params.tex_max.x;
    tex[i * 3 + 1] = kTexCoords[i * 3 + 1] < 0.5 ? params.tex_min.y : params.tex_max.y;
    tex[i * 3 + 2] = kTexCoords[i * 3 + 2] < 0.5 ? params.tex_min.z : params.tex_max.z;
  }

  // The legacy sweep bounds and interval (RendererBase::getFurthestDistance /
  // getNearestDistance / getIntervalWidth, arand 6-14-2011 formula), all in
  // ratio units.
  const double quality = clampd(params.quality, limits::min_quality, limits::max_quality);
  const int max_planes = params.max_planes < limits::min_max_planes
                             ? limits::min_max_planes
                             : (params.max_planes > limits::max_max_planes ? limits::max_max_planes
                                                                           : params.max_planes);
  const double near_frac =
      clampd(params.near_plane, limits::min_near_plane, limits::max_near_plane);

  const double diagonal = std::sqrt(ratio.x * ratio.x + ratio.y * ratio.y + ratio.z * ratio.z);
  const double furthest = 0.5 * diagonal;
  const double nearest = -0.5 * diagonal + near_frac * diagonal;

  const double min_ratio = std::fmin(ratio.x, std::fmin(ratio.y, ratio.z));
  const double N = 2.0 * (10.0 + max_planes * quality * quality * quality);
  double interval = min_ratio / N;

  const int max_polygons = 10 * max_planes;
  if ((furthest - nearest) / interval > max_polygons)
    interval = (furthest - nearest) / max_polygons;

  // Back to front: d from +furthest down to nearest (RendererBase::
  // computePolygons -- the first plane emitted is the farthest from the eye).
  for (double d = furthest; d > nearest; d -= interval)
    clip_plane(out, ratio, tex, n, d);

  // Map ratio-space vertices into the caller's local box.
  for (std::size_t i = 0; i < out.positions.size(); i += 3) {
    out.positions[i + 0] = static_cast<float>(out.positions[i + 0] * longest + center.x);
    out.positions[i + 1] = static_cast<float>(out.positions[i + 1] * longest + center.y);
    out.positions[i + 2] = static_cast<float>(out.positions[i + 2] * longest + center.z);
  }
  out.plane_spacing = interval * longest;
  return out;
}

} // namespace volslice
} // namespace cvc
