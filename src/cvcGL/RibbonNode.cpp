/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.

  libcvc is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include <algorithm>
#include <cmath>
#include <cvc/gl/RibbonNode.h>
#include <stdexcept>
#include <string>
#include <vtkOpenGLRenderWindow.h>
#include <vtkOpenGLState.h>
#include <vtkRenderer.h>
#include <vtkTextureObject.h>
#include <vtk_glad.h>

namespace cvc {
namespace gl {

// The centres' values on the GPU: an R32F texture kWidth texels wide, centre k
// at texel (k % kWidth, k / kWidth), read with texelFetch (WebGL2 does not
// promise linear filtering of float textures). Guarded by the ribbon's mutex.
struct RibbonValueTexture {
  static constexpr std::size_t kWidth = 1024;
  vtkSmartPointer<vtkTextureObject> tex;
  vtkOpenGLRenderWindow *context = nullptr; // compared, never dereferenced
  unsigned rows = 0;                        // the texture's height as made
  bool fullPending = true;
  std::size_t lo = 0, hi = 0; // centres [lo, hi) written since the last upload
  RibbonNode::ValueStats stats;

  static std::size_t rowsFor(std::size_t capacity) {
    return std::max<std::size_t>(1, (capacity + kWidth - 1) / kWidth);
  }
  void mark(std::size_t first, std::size_t end) {
    if (hi <= lo) {
      lo = first;
      hi = end;
    } else {
      lo = std::min(lo, first);
      hi = std::max(hi, end);
    }
  }
  // Render thread, `window` current: make the texture (first use, another
  // context, a growth) or upload the centres written since, as at most three
  // sub-images: a partial first row, the whole rows, a partial last row.
  void prepare(vtkOpenGLRenderWindow *window, const std::vector<float> &values) {
    const auto want = static_cast<unsigned>(values.size() / kWidth);
    if (!tex || context != window || tex->GetHandle() == 0 || rows != want) {
      tex = vtkSmartPointer<vtkTextureObject>::New();
      context = window;
      fullPending = true;
    }
    if (fullPending) {
      tex->SetContext(window);
      tex->SetMinificationFilter(vtkTextureObject::Nearest);
      tex->SetMagnificationFilter(vtkTextureObject::Nearest);
      tex->SetWrapS(vtkTextureObject::ClampToEdge);
      tex->SetWrapT(vtkTextureObject::ClampToEdge);
      tex->SetInternalFormat(GL_R32F); // a normalised default would clamp values to [0, 1]
      tex->SetFormat(GL_RED);
      tex->SetDataType(GL_FLOAT);
      tex->Create2DFromRaw(static_cast<unsigned>(kWidth), want, 1, VTK_FLOAT,
                           const_cast<float *>(values.data()));
      rows = want;
      fullPending = false;
      lo = hi = 0;
      ++stats.fullUploads;
      stats.bytes += values.size() * sizeof(float);
      return;
    }
    if (hi <= lo)
      return;
    tex->Activate();
    glBindBuffer(GL_PIXEL_UNPACK_BUFFER, 0);
    window->GetState()->vtkglPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    const auto sub = [&](std::size_t first, std::size_t w, std::size_t h) {
      glTexSubImage2D(GL_TEXTURE_2D, 0, static_cast<GLint>(first % kWidth),
                      static_cast<GLint>(first / kWidth), static_cast<GLsizei>(w),
                      static_cast<GLsizei>(h), GL_RED, GL_FLOAT, values.data() + first);
      ++stats.uploadCalls;
    };
    std::size_t a = lo;
    const std::size_t b = std::min(hi, values.size());
    if (a % kWidth != 0) { // the first row's tail
      const std::size_t end = std::min(b, (a / kWidth + 1) * kWidth);
      sub(a, end - a, 1);
      a = end;
    }
    if (b - a >= kWidth) { // whole rows, in one call
      const std::size_t full = (b - a) / kWidth;
      sub(a, kWidth, full);
      a += full * kWidth;
    }
    if (a < b) // the last row's head
      sub(a, b - a, 1);
    tex->Deactivate();
    ++stats.uploads;
    stats.bytes += (b - lo) * sizeof(float);
    lo = hi = 0;
  }
};

namespace {
constexpr float kClipOpen = 1.0e30f; // a clip bound past any centre index
constexpr float kDegenerate = 1.0e-12f;

// The value-colour shader (setValueRamp): the centre-line position cvcRibbonL
// (declared by the clip code) picks the value between two centres' texels,
// and the ramp's stops, uniforms each, colour it.
std::string rampDeclarations() {
  std::string s = "//VTK::Color::Dec\nuniform highp sampler2D cvcRibbonV;\n"
                  "uniform float cvcRibbonRampN;\n";
  for (std::size_t i = 0; i < RibbonNode::kMaxRampStops; ++i)
    s += "uniform float cvcRibbonRampV" + std::to_string(i) + ";\nuniform vec3 cvcRibbonRampC" +
         std::to_string(i) + ";\n";
  return s;
}
std::string rampImpl() {
  std::string s = "//VTK::Color::Impl\n"
                  "  if (cvcRibbonRampN > 0.5) {\n"
                  "    ivec2 cvcSz = textureSize(cvcRibbonV, 0);\n"
                  "    int cvcLast = cvcSz.x * cvcSz.y - 1;\n"
                  "    float cvcL = max(cvcRibbonL, 0.0);\n"
                  "    int cvcK0 = int(floor(cvcL));\n"
                  "    float cvcF = cvcL - float(cvcK0);\n"
                  "    cvcK0 = clamp(cvcK0, 0, cvcLast);\n"
                  "    int cvcK1 = min(cvcK0 + 1, cvcLast);\n"
                  "    ivec2 cvcT0 = ivec2(cvcK0 % cvcSz.x, cvcK0 / cvcSz.x);\n"
                  "    ivec2 cvcT1 = ivec2(cvcK1 % cvcSz.x, cvcK1 / cvcSz.x);\n"
                  "    float cvcVa = texelFetch(cvcRibbonV, cvcT0, 0).r;\n"
                  "    float cvcVb = texelFetch(cvcRibbonV, cvcT1, 0).r;\n"
                  "    float cvcV = mix(cvcVa, cvcVb, cvcF);\n"
                  "    vec3 cvcC = cvcRibbonRampC0;\n";
  for (std::size_t i = 1; i < RibbonNode::kMaxRampStops; ++i) {
    const std::string a = std::to_string(i - 1), b = std::to_string(i);
    s += "    if (cvcRibbonRampN > " + std::to_string(i) + ".5 && cvcV > cvcRibbonRampV" + a +
         ") cvcC = mix(cvcRibbonRampC" + a + ", cvcRibbonRampC" + b +
         ", clamp((cvcV - cvcRibbonRampV" + a + ") / max(cvcRibbonRampV" + b + " - cvcRibbonRampV" +
         a + ", 1e-20), 0.0, 1.0));\n";
  }
  s += "    ambientColor = cvcC;\n"
       "    diffuseColor = vec3(0.0);\n"
       "  }\n";
  return s;
}

// Unit 2-D direction from (ax, ay) to (bx, by); false if they coincide.
bool unitDir(float ax, float ay, float bx, float by, float &dx, float &dy) {
  dx = bx - ax;
  dy = by - ay;
  const float len2 = dx * dx + dy * dy;
  if (len2 <= kDegenerate)
    return false;
  const float inv = 1.0f / std::sqrt(len2);
  dx *= inv;
  dy *= inv;
  return true;
}
} // namespace

StreamingLayout RibbonNode::ribbonLayout(std::size_t centers, const cvc::bounding_box &bounds,
                                         StreamingMapperKind kind) {
  StreamingLayout l;
  l.capacity_points = 2 * centers;
  l.reserved_bounds = bounds;
  l.mapper = kind;
  l.triangles.reserve(centers > 1 ? 2 * (centers - 1) : 0);
  for (std::size_t k = 0; k + 1 < centers; ++k) {
    const auto l0 = static_cast<std::uint32_t>(2 * k), r0 = l0 + 1, l1 = l0 + 2, r1 = l0 + 3;
    l.triangles.push_back({{l0, r0, r1}});
    l.triangles.push_back({{l0, r1, l1}});
  }
  return l;
}

RibbonNode::RibbonNode(cvc::app &ctx, const std::string &statePath, const std::string &name,
                       std::size_t capacityCenters, float halfWidth,
                       const cvc::bounding_box &reservedBounds, StreamingMapperKind mapper)
    : StreamingGeometryNode(
          ctx, statePath, name,
          ribbonLayout(std::max<std::size_t>(capacityCenters, 2), reservedBounds, mapper)),
      m_capacity(std::max<std::size_t>(capacityCenters, 2)), m_half(halfWidth) {
  // A flat strip on the ground casts nothing worth a shadow bake, and a cast
  // shadow would freeze at the first bake (streamed writes bump no MTime).
  // setCastsShadow(true) opts back in.
  setCastsShadow(false);
  m_centers.assign(3 * m_capacity, 0.0f);
  m_halfs.assign(m_capacity, m_half);
  m_values.assign(RibbonValueTexture::rowsFor(m_capacity) * RibbonValueTexture::kWidth, 0.0f);
  m_arc.assign(m_capacity, 0.0);

  // The fractional cut of setVisibleCenters: the centre-line position of every
  // fragment, L = pointId / 2 interpolated across each segment's quad, against
  // two uniforms. The discard goes AFTER VTK's uniform-flow code (derivatives).
  // Internal replacements: a caller's add*ShaderReplacement on the same anchor
  // composes with them, and clearShaderReplacements() leaves them in place.
  addInternalVertexShaderReplacement("//VTK::PositionVC::Dec",
                                     "//VTK::PositionVC::Dec\nout float cvcRibbonL;\n");
  addInternalVertexShaderReplacement("//VTK::PositionVC::Impl",
                                     std::string("cvcRibbonL = float(") + pointIdGLSL() +
                                         " / 2);\n  //VTK::PositionVC::Impl\n");
  addInternalFragmentShaderReplacement("//VTK::PositionVC::Dec",
                                       "//VTK::PositionVC::Dec\nin float cvcRibbonL;\n"
                                       "uniform float cvcRibbonClipLo;\n"
                                       "uniform float cvcRibbonClipHi;\n");
  addInternalFragmentShaderReplacement(
      "//VTK::UniformFlow::Impl",
      "//VTK::UniformFlow::Impl\n"
      "  if (cvcRibbonL < cvcRibbonClipLo || cvcRibbonL > cvcRibbonClipHi) discard;\n");
  std::lock_guard<std::mutex> lock(m_ribbonMutex);
  updateWindowLocked(); // nothing assigned: draws nothing
}

RibbonNode::~RibbonNode() = default;

std::size_t RibbonNode::centerCount() const {
  std::lock_guard<std::mutex> lock(m_ribbonMutex);
  return m_count;
}

std::size_t RibbonNode::centerCapacity() const {
  std::lock_guard<std::mutex> lock(m_ribbonMutex);
  return m_capacity;
}

float RibbonNode::halfWidth() const {
  std::lock_guard<std::mutex> lock(m_ribbonMutex);
  return m_half;
}

double RibbonNode::arcLength() const {
  std::lock_guard<std::mutex> lock(m_ribbonMutex);
  return m_count ? m_arc[m_count - 1] : 0.0;
}

double RibbonNode::centerAtArcLength(double s) const {
  std::lock_guard<std::mutex> lock(m_ribbonMutex);
  if (m_count < 2 || s <= 0.0)
    return 0.0;
  const double total = m_arc[m_count - 1];
  if (s >= total)
    return static_cast<double>(m_count - 1);
  // First centre whose cumulative length exceeds s; s lies on the segment before it.
  const auto it = std::upper_bound(m_arc.begin(), m_arc.begin() + m_count, s);
  const std::size_t k = static_cast<std::size_t>(it - m_arc.begin()); // 1..count-1
  const double a = m_arc[k - 1], b = m_arc[k];
  const double f = b > a ? (s - a) / (b - a) : 0.0;
  return static_cast<double>(k - 1) + f;
}

float RibbonNode::centerHalfWidth(std::size_t k) const {
  std::lock_guard<std::mutex> lock(m_ribbonMutex);
  return k < m_count ? m_halfs[k] : 0.0f;
}

float RibbonNode::centerValue(std::size_t k) const {
  std::lock_guard<std::mutex> lock(m_ribbonMutex);
  return k < m_count ? m_values[k] : 0.0f;
}

std::vector<float> RibbonNode::centerVertices(std::size_t k) const {
  std::lock_guard<std::mutex> lock(m_ribbonMutex);
  std::vector<float> out(6, 0.0f);
  if (k < m_count)
    computeVerticesLocked(k, out.data());
  return out;
}

void RibbonNode::computeVerticesLocked(std::size_t k, float out[6]) const {
  const float *p = &m_centers[3 * k];
  float inx = 0, iny = 0, outx = 0, outy = 0;
  const bool hasIn = k > 0 && unitDir(p[-3], p[-2], p[0], p[1], inx, iny);
  const bool hasOut = k + 1 < m_count && unitDir(p[0], p[1], p[3], p[4], outx, outy);
  float mx = 1.0f, my = 0.0f, scale = 1.0f;
  if (hasIn && hasOut) {
    // Mitre: offset along the bisector, stretched so each edge stays halfWidth
    // from its segment; capped at 2x (a hairpin would otherwise spike out).
    float bx = inx + outx, by = iny + outy;
    const float blen = std::sqrt(bx * bx + by * by);
    if (blen > 1.0e-6f) {
      mx = bx / blen;
      my = by / blen;
      const float c = mx * outx + my * outy; // cos of half the turn
      scale = 1.0f / std::max(c, 0.5f);
    } else { // a full reversal: no bisector, square it off on the way out
      mx = outx;
      my = outy;
    }
  } else if (hasOut) {
    mx = outx;
    my = outy;
  } else if (hasIn) {
    mx = inx;
    my = iny;
  }
  const float ox = -my * m_halfs[k] * scale, oy = mx * m_halfs[k] * scale;
  out[0] = p[0] + ox; // left
  out[1] = p[1] + oy;
  out[2] = p[2];
  out[3] = p[0] - ox; // right
  out[4] = p[1] - oy;
  out[5] = p[2];
}

void RibbonNode::writeCentersLocked(std::size_t first, std::size_t last) {
  if (last < first || last >= m_count)
    return;
  const std::size_t n = last - first + 1;
  m_scratch.resize(6 * n);
  for (std::size_t k = first; k <= last; ++k)
    computeVerticesLocked(k, &m_scratch[6 * (k - first)]);
  writePoints(2 * first, m_scratch.data(), 2 * n);
}

void RibbonNode::fitBoundsLocked(std::size_t first, std::size_t last) {
  if (last < first || last >= m_count)
    return;
  // A mitred vertex sits up to 2x its centre's half width off it (computeVerticesLocked).
  double half = 0.0;
  const float *p = &m_centers[3 * first];
  double lo[3] = {p[0], p[1], p[2]}, hi[3] = {p[0], p[1], p[2]};
  for (std::size_t k = first; k <= last; ++k) {
    p = &m_centers[3 * k];
    for (int i = 0; i < 3; ++i) {
      lo[i] = std::min(lo[i], static_cast<double>(p[i]));
      hi[i] = std::max(hi[i], static_cast<double>(p[i]));
    }
    half = std::max(half, std::fabs(static_cast<double>(m_halfs[k])));
  }
  const double pad = 2.0 * half;
  // Only ever grows, so a track that drives out of its box keeps all of it in
  // view: what VTK culls and clips by is this box, never the points.
  growReservedBounds(
      cvc::bounding_box(lo[0] - pad, lo[1] - pad, lo[2], hi[0] + pad, hi[1] + pad, hi[2]));
}

void RibbonNode::growLocked(std::size_t minCenters) {
  if (minCenters <= m_capacity)
    return;
  const std::size_t cap = std::max(minCenters, 2 * m_capacity);
  m_centers.resize(3 * cap, 0.0f);
  m_halfs.resize(cap, m_half);
  m_values.resize(RibbonValueTexture::rowsFor(cap) * RibbonValueTexture::kWidth, 0.0f);
  m_arc.resize(cap, 0.0);
  m_capacity = cap;
  if (m_tex)
    m_tex->fullPending = true; // a taller texture, made again whole
  relayout(ribbonLayout(cap, reservedBounds(), mapperKind()));
}

void RibbonNode::updateWindowLocked() {
  const std::size_t n = m_count;
  const double first = std::max(0.0, m_visFirst);
  const double last = std::min(m_visLast, n ? static_cast<double>(n - 1) : 0.0);
  if (n < 2 || !(last > first)) {
    setDrawRange(0, 0);
    return;
  }
  // Whole part -> the triangle range (segment s = triangles 2s, 2s + 1) ...
  std::size_t segFirst = static_cast<std::size_t>(std::floor(first));
  std::size_t segLast = static_cast<std::size_t>(std::ceil(last)) - 1;
  segFirst = std::min(segFirst, n - 2);
  segLast = std::min(std::max(segLast, segFirst), n - 2);
  setDrawRange(2 * segFirst, 2 * (segLast - segFirst + 1));
  // ... fraction -> the fragment clip.
  setUniform("cvcRibbonClipLo", static_cast<float>(first));
  setUniform("cvcRibbonClipHi",
             m_visLast >= static_cast<double>(n - 1) ? kClipOpen : static_cast<float>(last));
}

void RibbonNode::arcFromLocked(std::size_t first) {
  if (first == 0 && m_count)
    m_arc[0] = 0.0;
  for (std::size_t k = std::max<std::size_t>(first, 1); k < m_count; ++k) {
    const float *p = &m_centers[3 * k];
    const double dx = p[0] - p[-3], dy = p[1] - p[-2], dz = p[2] - p[-1];
    m_arc[k] = m_arc[k - 1] + std::sqrt(dx * dx + dy * dy + dz * dz);
  }
}

void RibbonNode::markValuesLocked(std::size_t first, std::size_t last) {
  if (m_tex && last >= first)
    m_tex->mark(first, last + 1);
}

std::size_t RibbonNode::append(float x, float y, float z) {
  const float p[3] = {x, y, z};
  std::lock_guard<std::mutex> lock(m_ribbonMutex);
  const std::size_t k = m_count;
  replaceFromLocked(k, p, 1, nullptr, nullptr);
  return k;
}

void RibbonNode::assign(const float *xyz, std::size_t nCenters) {
  if (nCenters == 0) {
    clearCenters();
    return;
  }
  if (!xyz)
    throw std::invalid_argument("RibbonNode::assign: null xyz");
  std::lock_guard<std::mutex> lock(m_ribbonMutex);
  m_visFirst = 0.0;
  m_visLast = kToEnd;
  replaceFromLocked(0, xyz, nCenters, nullptr, nullptr); // one sub-range upload of 2n points
}

void RibbonNode::replaceFrom(std::size_t first, const float *xyz, std::size_t n,
                             const float *halfWidths, const float *values) {
  if (n && !xyz)
    throw std::invalid_argument("RibbonNode::replaceFrom: null xyz");
  std::lock_guard<std::mutex> lock(m_ribbonMutex);
  replaceFromLocked(first, xyz, n, halfWidths, values);
}

void RibbonNode::replaceFromLocked(std::size_t first, const float *xyz, std::size_t n,
                                   const float *halfWidths, const float *values) {
  first = std::min(first, m_count);
  const std::size_t count = first + n;
  growLocked(count);
  if (n) {
    std::copy(xyz, xyz + 3 * n, m_centers.begin() + static_cast<std::ptrdiff_t>(3 * first));
    if (halfWidths)
      std::copy(halfWidths, halfWidths + n, m_halfs.begin() + static_cast<std::ptrdiff_t>(first));
    else
      std::fill_n(m_halfs.begin() + static_cast<std::ptrdiff_t>(first), n, m_half);
    if (values)
      std::copy(values, values + n, m_values.begin() + static_cast<std::ptrdiff_t>(first));
    else
      std::fill_n(m_values.begin() + static_cast<std::ptrdiff_t>(first), n, 0.0f);
  }
  m_count = count;
  arcFromLocked(first);
  if (n) {
    fitBoundsLocked(first, count - 1);
    // The new centres, plus the one before re-mitred now that its outgoing
    // direction is known (an append: 4 vertices, 2 for the very first centre).
    writeCentersLocked(first == 0 ? 0 : first - 1, count - 1);
    markValuesLocked(first, count - 1);
  } else if (first > 0) {
    writeCentersLocked(first - 1, first - 1); // truncated: the new last centre loses its exit
  }
  updateWindowLocked();
}

void RibbonNode::setVisibleCenters(double first, double last) {
  std::lock_guard<std::mutex> lock(m_ribbonMutex);
  m_visFirst = first;
  m_visLast = last;
  updateWindowLocked();
}

void RibbonNode::clearCenters() {
  std::lock_guard<std::mutex> lock(m_ribbonMutex);
  m_count = 0;
  m_visFirst = 0.0;
  m_visLast = kToEnd;
  updateWindowLocked();
}

void RibbonNode::setHalfWidth(float halfWidth) {
  std::lock_guard<std::mutex> lock(m_ribbonMutex);
  m_half = halfWidth;
  std::fill(m_halfs.begin(), m_halfs.end(), halfWidth);
  if (m_count) {
    fitBoundsLocked(0, m_count - 1);
    writeCentersLocked(0, m_count - 1);
  }
}

void RibbonNode::setValueRamp(const std::vector<RampStop> &stops) {
  std::vector<RampStop> ramp(stops.begin(), stops.begin() + static_cast<std::ptrdiff_t>(std::min(
                                                                stops.size(), kMaxRampStops)));
  std::stable_sort(ramp.begin(), ramp.end(),
                   [](const RampStop &a, const RampStop &b) { return a.value < b.value; });
  bool install = false;
  {
    std::lock_guard<std::mutex> lock(m_ribbonMutex);
    m_ramp = ramp;
    if (!ramp.empty() && !m_tex)
      m_tex = std::make_unique<RibbonValueTexture>(); // every value goes up at the next draw
    install = !ramp.empty() && !m_rampShader;
    m_rampShader = m_rampShader || install;
  }
  if (install) { // once: the shader code, which the stop count then switches on and off
    addInternalFragmentShaderReplacement("//VTK::Color::Dec", rampDeclarations());
    addInternalFragmentShaderReplacement("//VTK::Color::Impl", rampImpl());
    setSpecular(0.0);
  }
  setUniform("cvcRibbonRampN", static_cast<float>(ramp.size()));
  for (std::size_t i = 0; i < kMaxRampStops; ++i) {
    // Unused stops repeat the last, so a stale one can never show.
    const RampStop s = ramp.empty() ? RampStop{} : ramp[std::min(i, ramp.size() - 1)];
    setUniform("cvcRibbonRampV" + std::to_string(i), s.value);
    setUniform("cvcRibbonRampC" + std::to_string(i), s.r, s.g, s.b);
  }
}

std::vector<RibbonNode::RampStop> RibbonNode::valueRamp() const {
  std::lock_guard<std::mutex> lock(m_ribbonMutex);
  return m_ramp;
}

RibbonNode::ValueStats RibbonNode::valueStats() const {
  std::lock_guard<std::mutex> lock(m_ribbonMutex);
  return m_tex ? m_tex->stats : ValueStats{};
}

void RibbonNode::beforeDraw(vtkRenderer *renderer) {
  StreamingGeometryNode::beforeDraw(renderer);
  vtkOpenGLRenderWindow *window =
      renderer ? vtkOpenGLRenderWindow::SafeDownCast(renderer->GetRenderWindow()) : nullptr;
  std::lock_guard<std::mutex> lock(m_ribbonMutex);
  if (m_tex && window && !m_ramp.empty())
    m_tex->prepare(window, m_values);
}

void RibbonNode::updateShaderProgram(vtkShaderProgram *program) {
  StreamingGeometryNode::updateShaderProgram(program);
  vtkTextureObject *tex = nullptr;
  {
    std::lock_guard<std::mutex> lock(m_ribbonMutex);
    if (m_tex)
      tex = m_tex->tex;
  }
  if (tex)
    bindShaderTextures(program, {{"cvcRibbonV", tex}});
}

} // namespace gl
} // namespace cvc
