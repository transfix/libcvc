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
#include <cvc/gl/DrapedLinkNode.h>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>
#include <vtkActor.h>
#include <vtkOpenGLRenderWindow.h>
#include <vtkProperty.h>
#include <vtkRenderer.h>

namespace cvc {
namespace gl {

namespace {
StreamingLayout linkLayout(const std::shared_ptr<HeightFieldTexture> &heights, int stations,
                           StreamingMapperKind kind) {
  if (!heights)
    throw std::invalid_argument("DrapedLinkNode: null height field");
  if (stations < 2)
    throw std::invalid_argument("DrapedLinkNode: need at least 2 stations");
  StreamingLayout l;
  l.capacity_points = 2 * static_cast<std::size_t>(stations);
  l.mapper = kind;
  for (int k = 0; k + 1 < stations; ++k) {
    const auto l0 = static_cast<std::uint32_t>(2 * k), r0 = l0 + 1, l1 = l0 + 2, r1 = l0 + 3;
    l.triangles.push_back({{l0, r0, r1}});
    l.triangles.push_back({{l0, r1, l1}});
  }
  l.reserved_bounds = heights->extent(); // refined by refreshReservedBounds()
  return l;
}

// Vertex shader: the template vertex (t, side, 0) -> its draped world point.
std::string drapeDeclarations() {
  return std::string("//VTK::PositionVC::Dec\n") + HeightFieldTexture::glsl() +
         "uniform vec3 cvcLinkA;\n"
         "uniform vec3 cvcLinkB;\n"
         "uniform float cvcLinkHalf;\n"
         "uniform float cvcLinkLift;\n"
         "vec4 cvc_link_drape(vec4 tmpl) {\n"
         "  vec2 d = cvcLinkB.xy - cvcLinkA.xy;\n"
         "  float len = length(d);\n"
         "  vec2 dir = len > 1e-6 ? d / len : vec2(1.0, 0.0);\n"
         "  vec2 xy = mix(cvcLinkA.xy, cvcLinkB.xy, tmpl.x) + vec2(-dir.y, dir.x) * (tmpl.y * "
         "cvcLinkHalf);\n"
         "  return vec4(xy, cvc_hf_height(xy) + cvcLinkLift, 1.0);\n"
         "}\n";
}
} // namespace

DrapedLinkNode::DrapedLinkNode(cvc::app &ctx, const std::string &statePath, const std::string &name,
                               std::shared_ptr<HeightFieldTexture> heights, int stations,
                               StreamingMapperKind mapper)
    : StreamingGeometryNode(ctx, statePath, name, linkLayout(heights, stations, mapper)),
      m_heights(std::move(heights)), m_stations(stations) {
  // The template, written once: centre k at t = k / (stations - 1), left side
  // +1, right side -1 (the same left/right convention as RibbonNode).
  std::vector<float> tmpl(6 * static_cast<std::size_t>(stations));
  for (int k = 0; k < stations; ++k) {
    const float t = static_cast<float>(k) / static_cast<float>(stations - 1);
    float *v = &tmpl[6 * static_cast<std::size_t>(k)];
    v[0] = t;
    v[1] = 1.0f;
    v[2] = 0.0f;
    v[3] = t;
    v[4] = -1.0f;
    v[5] = 0.0f;
  }
  writePoints(0, tmpl.data(), 2 * static_cast<std::size_t>(stations));

  // The drape goes in at //VTK::Clip::Impl, the earliest point after the
  // low-memory mapper declares vertexMC (//VTK::CustomBegin::Impl): the #define
  // then re-points the clip-plane distances, VTK's position code and the
  // shadow baker at the draped point. Internal replacements (GeometryNode), so
  // a caller's replacements compose with them instead of evicting them.
  addInternalVertexShaderReplacement("//VTK::PositionVC::Dec", drapeDeclarations());
  addInternalVertexShaderReplacement("//VTK::Clip::Impl",
                                     "vec4 cvcLinkMC = cvc_link_drape(vertexMC);\n"
                                     "#define vertexMC cvcLinkMC\n"
                                     "  //VTK::Clip::Impl\n");
  // Flat, unlit colour from uniforms (after VTK declares the colour terms).
  addInternalFragmentShaderReplacement("//VTK::Color::Dec",
                                       "//VTK::Color::Dec\nuniform vec3 cvcLinkColor;\n"
                                       "uniform float cvcLinkOpacity;\n");
  addInternalFragmentShaderReplacement("//VTK::Color::Impl", "//VTK::Color::Impl\n"
                                                             "  ambientColor = cvcLinkColor;\n"
                                                             "  diffuseColor = vec3(0.0);\n"
                                                             "  opacity = cvcLinkOpacity;\n");
  setSpecular(0.0); // no highlight on a flat overlay

  setUniform(HeightFieldTexture::originName(), static_cast<float>(m_heights->x0()),
             static_cast<float>(m_heights->y0()), 0.0f);
  setUniform(HeightFieldTexture::spacingName(), static_cast<float>(m_heights->dx()),
             static_cast<float>(m_heights->dy()), 0.0f);
  setUniform("cvcLinkA", 0.0f, 0.0f, 0.0f);
  setUniform("cvcLinkB", 0.0f, 0.0f, 0.0f);
  m_seenGeneration = m_heights->generation();
  setStyle(1.0f, 0.5f, 0.2f, 0.6f, 1.0f, 1.0f);
}

void DrapedLinkNode::setEndpoints(float x0, float y0, float x1, float y1) {
  {
    std::lock_guard<std::mutex> lock(m_linkMutex);
    m_a[0] = x0;
    m_a[1] = y0;
    m_b[0] = x1;
    m_b[1] = y1;
  }
  setUniform("cvcLinkA", x0, y0, 0.0f);
  setUniform("cvcLinkB", x1, y1, 0.0f);
}

void DrapedLinkNode::setStyle(float halfWidth, float lift, float r, float g, float b,
                              float opacity) {
  bool translucent = opacity < 1.0f, crossed = false;
  {
    std::lock_guard<std::mutex> lock(m_linkMutex);
    m_half = halfWidth;
    m_lift = lift;
    crossed = translucent != m_translucent;
    m_translucent = translucent;
  }
  setUniform("cvcLinkHalf", halfWidth);
  setUniform("cvcLinkLift", lift);
  setUniform("cvcLinkColor", r, g, b);
  setUniform("cvcLinkOpacity", opacity);
  // Width and lift move the derived box; a box the caller pinned stays put.
  stageDerivedBounds(derivedBounds());
  if (crossed) {
    // Route the actor to the translucent pass (or back). A property change, so
    // only when crossing 1, never per frame.
    runOnMainThread(
        [this, translucent]() { actor()->GetProperty()->SetOpacity(translucent ? 0.99 : 1.0); });
  }
}

cvc::bounding_box DrapedLinkNode::derivedBounds() const {
  float half, lift;
  {
    std::lock_guard<std::mutex> lock(m_linkMutex);
    half = m_half;
    lift = m_lift;
  }
  const cvc::bounding_box e = m_heights->extent();
  const double pad = std::fabs(half) + 1.0;
  return cvc::bounding_box(e.minx - pad, e.miny - pad, e.minz + std::min(0.0f, lift) - 1.0,
                           e.maxx + pad, e.maxy + pad, e.maxz + std::max(0.0f, lift) + 1.0);
}

void DrapedLinkNode::refreshReservedBounds() {
  m_seenGeneration = m_heights->generation();
  unpinReservedBounds();
  stageDerivedBounds(derivedBounds());
}

void DrapedLinkNode::beforeComputeBounds() {
  // Heights changed since the box was derived (terrain loaded after the link
  // was made, say): re-derive it now, before VTK culls by it. Here rather than
  // in beforeDraw because a link whose stale box is out of view is never drawn.
  const std::uint64_t gen = m_heights->generation();
  if (gen == m_seenGeneration.load())
    return;
  m_seenGeneration = gen;
  setDerivedBoundsNow(derivedBounds());
}

void DrapedLinkNode::centerAt(double t, double out[3]) const {
  float a[2], b[2], lift;
  {
    std::lock_guard<std::mutex> lock(m_linkMutex);
    a[0] = m_a[0];
    a[1] = m_a[1];
    b[0] = m_b[0];
    b[1] = m_b[1];
    lift = m_lift;
  }
  t = std::min(1.0, std::max(0.0, t));
  const double s = t * (m_stations - 1);
  const int k = std::min(static_cast<int>(std::floor(s)), m_stations - 2);
  const double f = s - k;
  auto station = [&](int i, double p[3]) {
    const double u = static_cast<double>(i) / (m_stations - 1);
    p[0] = a[0] + (b[0] - a[0]) * u;
    p[1] = a[1] + (b[1] - a[1]) * u;
    p[2] = m_heights->sample(p[0], p[1]) + lift;
  };
  double p0[3], p1[3];
  station(k, p0);
  station(k + 1, p1);
  for (int i = 0; i < 3; ++i)
    out[i] = p0[i] + (p1[i] - p0[i]) * f;
}

void DrapedLinkNode::beforeDraw(vtkRenderer *renderer) {
  if (renderer)
    m_heights->prepare(vtkOpenGLRenderWindow::SafeDownCast(renderer->GetRenderWindow()));
}

void DrapedLinkNode::updateShaderProgram(vtkShaderProgram *program) {
  if (vtkTextureObject *tex = m_heights->texture())
    bindShaderTextures(program, {{HeightFieldTexture::samplerName(), tex}});
}

} // namespace gl
} // namespace cvc
