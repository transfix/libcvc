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
#include <vtkActor.h>
#include <vtkShaderProperty.h>

namespace cvc {
namespace gl {

namespace {
constexpr float kClipOpen = 1.0e30f; // a clip bound past any centre index
constexpr float kDegenerate = 1.0e-12f;

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
  m_centers.assign(3 * m_capacity, 0.0f);
  m_arc.assign(m_capacity, 0.0);

  // The fractional cut of setVisibleCenters: the centre-line position of every
  // fragment, L = pointId / 2 interpolated across each segment's quad, against
  // two uniforms. The discard goes AFTER VTK's uniform-flow code (derivatives).
  // Set straight on the shader property: the node is not attached yet.
  vtkShaderProperty *sp = actor()->GetShaderProperty();
  sp->AddVertexShaderReplacement("//VTK::PositionVC::Dec", true,
                                 "//VTK::PositionVC::Dec\nout float cvcRibbonL;\n", false);
  sp->AddVertexShaderReplacement("//VTK::PositionVC::Impl", true,
                                 std::string("cvcRibbonL = float(") + pointIdGLSL() +
                                     " / 2);\n  //VTK::PositionVC::Impl\n",
                                 false);
  sp->AddFragmentShaderReplacement("//VTK::PositionVC::Dec", true,
                                   "//VTK::PositionVC::Dec\nin float cvcRibbonL;\n"
                                   "uniform float cvcRibbonClipLo;\n"
                                   "uniform float cvcRibbonClipHi;\n",
                                   false);
  sp->AddFragmentShaderReplacement(
      "//VTK::UniformFlow::Impl", true,
      "//VTK::UniformFlow::Impl\n"
      "  if (cvcRibbonL < cvcRibbonClipLo || cvcRibbonL > cvcRibbonClipHi) discard;\n",
      false);
  std::lock_guard<std::mutex> lock(m_ribbonMutex);
  updateWindowLocked(); // nothing assigned: draws nothing
}

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
  const float ox = -my * m_half * scale, oy = mx * m_half * scale;
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

void RibbonNode::growLocked(std::size_t minCenters) {
  if (minCenters <= m_capacity)
    return;
  const std::size_t cap = std::max(minCenters, 2 * m_capacity);
  m_centers.resize(3 * cap, 0.0f);
  m_arc.resize(cap, 0.0);
  m_capacity = cap;
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

std::size_t RibbonNode::append(float x, float y, float z) {
  std::lock_guard<std::mutex> lock(m_ribbonMutex);
  const std::size_t k = m_count;
  if (k == m_capacity)
    growLocked(k + 1);
  float *p = &m_centers[3 * k];
  p[0] = x;
  p[1] = y;
  p[2] = z;
  if (k == 0) {
    m_arc[0] = 0.0;
  } else {
    const double dx = x - p[-3], dy = y - p[-2], dz = z - p[-1];
    m_arc[k] = m_arc[k - 1] + std::sqrt(dx * dx + dy * dy + dz * dz);
  }
  m_count = k + 1;
  // The new centre, plus the previous one re-mitred now that its outgoing
  // direction is known: 4 vertices (2 for the very first centre).
  writeCentersLocked(k == 0 ? 0 : k - 1, k);
  updateWindowLocked();
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
  growLocked(nCenters);
  std::copy(xyz, xyz + 3 * nCenters, m_centers.begin());
  m_arc[0] = 0.0;
  for (std::size_t k = 1; k < nCenters; ++k) {
    const float *p = &m_centers[3 * k];
    const double dx = p[0] - p[-3], dy = p[1] - p[-2], dz = p[2] - p[-1];
    m_arc[k] = m_arc[k - 1] + std::sqrt(dx * dx + dy * dy + dz * dz);
  }
  m_count = nCenters;
  m_visFirst = 0.0;
  m_visLast = kToEnd;
  writeCentersLocked(0, nCenters - 1); // one sub-range upload of 2n points
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
  if (m_count)
    writeCentersLocked(0, m_count - 1);
}

} // namespace gl
} // namespace cvc
