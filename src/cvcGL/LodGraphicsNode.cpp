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
#include <cvc/gl/GeometryNode.h>
#include <cvc/gl/LodGraphicsNode.h>
#include <vtkCamera.h>
#include <vtkProp.h>
#include <vtkRenderer.h>

namespace cvc {
namespace gl {

namespace {

// One container of a mesh: the same storage (copies of one cvc::geometry share
// it, copy-on-write), or equal contents.
template <class C> bool same_array(const C &a, const C &b) { return &a == &b || a == b; }

// Same drawable content -- everything GeometryNode turns into buffers.
bool same_mesh(const cvc::geometry &a, const cvc::geometry &b) {
  return a.get_geometry_type() == b.get_geometry_type() &&
         same_array(a.const_points(), b.const_points()) &&
         same_array(a.const_tris(), b.const_tris()) &&
         same_array(a.const_quads(), b.const_quads()) &&
         same_array(a.const_lines(), b.const_lines()) &&
         same_array(a.const_normals(), b.const_normals()) &&
         same_array(a.const_colors(), b.const_colors()) && same_array(a.const_uvs(), b.const_uvs());
}

} // namespace

void lod_stats::reset() {
  nodes = 0;
  hidden = 0;
  changes = 0;
  std::fill(rung_nodes.begin(), rung_nodes.end(), 0);
  drawn_tris = 0;
  full_tris = 0;
}

LodGraphicsNode::LodGraphicsNode(cvc::app &ctx, const std::string &statePath,
                                 const std::string &name)
    : GraphicsNode(ctx, statePath, name) {}

LodGraphicsNode::~LodGraphicsNode() = default;

void LodGraphicsNode::clearRungs(std::size_t keep) {
  // Detach through removeGraphicsChild so each rung's actor leaves the renderer
  // and its node dies with the last reference -- a re-populate must not leave
  // the old ladder behind, still registered and still drawing.
  while (m_rungs.size() > keep) {
    std::shared_ptr<GeometryNode> r = m_rungs.back();
    m_rungs.pop_back();
    removeGraphicsChild(r);
  }
  m_worldError.resize(m_rungs.size());
  m_tris.resize(m_rungs.size());
  if (m_rungs.empty())
    m_base.reset();
}

void LodGraphicsNode::addRung(const cvc::geometry &g, double worldError) {
  auto child = addGraphicsChild<GeometryNode>("lod" + std::to_string(m_rungs.size()));
  child->setGeometry(g);
  if (m_style)
    m_style(*child); // before the rung can draw, so every rung looks the same
  if (m_rungs.empty())
    m_base = std::make_unique<cvc::geometry>(g); // shares g's storage
  m_rungs.push_back(child);
  m_worldError.push_back(worldError);
  m_tris.push_back(g.num_tris());
}

void LodGraphicsNode::setPyramid(const cvc::lod::mesh_pyramid &pyr) {
  clearRungs();
  m_current = -1;
  for (std::size_t k = 0; k < pyr.rungs.size(); ++k)
    addRung(pyr.rungs[k], k < pyr.world_error_m.size() ? pyr.world_error_m[k] : 0.0);
  applyRungVisibility(); // rung 0 (finest) draws until the first select()
}

void LodGraphicsNode::setBase(const cvc::geometry &finest) {
  clearRungs();
  m_current = -1;
  addRung(finest, 0.0);
  applyRungVisibility();
}

bool LodGraphicsNode::appendRungs(const cvc::lod::mesh_pyramid &pyr) {
  if (pyr.rungs.empty())
    return false;
  if (!m_base || !same_mesh(*m_base, pyr.rungs[0])) {
    setPyramid(pyr);
    return false;
  }
  clearRungs(1); // keep rung 0: its node, its style, its uploaded buffers
  if (!pyr.world_error_m.empty())
    m_worldError[0] = pyr.world_error_m[0];
  for (std::size_t k = 1; k < pyr.rungs.size(); ++k)
    addRung(pyr.rungs[k], k < pyr.world_error_m.size() ? pyr.world_error_m[k] : 0.0);
  if (m_current >= rungCount())
    m_current = rungCount() - 1;
  applyRungVisibility();
  return true;
}

void LodGraphicsNode::setRungStyle(RungStyle style) {
  m_style = std::move(style);
  if (!m_style)
    return;
  for (auto &r : m_rungs)
    m_style(*r);
}

double LodGraphicsNode::rungError(int k) const {
  return (k >= 0 && k < static_cast<int>(m_worldError.size())) ? m_worldError[k] : 0.0;
}

std::uint64_t LodGraphicsNode::rungTriangles(int k) const {
  return (k >= 0 && k < static_cast<int>(m_tris.size())) ? m_tris[k] : 0;
}

std::shared_ptr<GeometryNode> LodGraphicsNode::rung(int k) const {
  return (k >= 0 && k < rungCount()) ? m_rungs[k] : nullptr;
}

int LodGraphicsNode::activeRung() const {
  if (m_rungs.empty())
    return -1;
  return m_current >= 0 ? m_current : 0;
}

cvc::bounding_box LodGraphicsNode::getBoundingBox() const {
  if (m_rungs.empty())
    return cvc::bounding_box();
  return m_rungs.front()->getBoundingBox(); // rung 0 = the full-detail extent
}

vtkProp *LodGraphicsNode::getProp() {
  return nullptr; // prop-less container; the visible rung is a child GeometryNode
}

bool LodGraphicsNode::isRung(const SceneNode *child) const {
  for (const auto &r : m_rungs)
    if (static_cast<const SceneNode *>(r.get()) == child)
      return true;
  return false;
}

void LodGraphicsNode::propagateVisible(bool visible) {
  // The base would show or hide every rung together, and showing them together
  // is the bug: exactly one rung may draw. Re-derive the rung actors from this
  // node's (now updated) visibility instead. Children that are not rungs keep
  // the ordinary propagation.
  for (auto &child : m_children)
    if (!isRung(child.get()))
      child->setVisible(visible);
  runOnMainThread([this]() { applyRungVisibility(); });
}

void LodGraphicsNode::applyRungVisibility() {
  const bool drawable = isVisibleInHierarchy();
  const int active = activeRung();
  const int n = rungCount();
  for (int i = 0; i < n; ++i) {
    // SetVisibility, not Add/RemoveViewProp: the rung stays registered and keeps
    // its GPU buffers (see the header). vtkSetMacro only bumps the MTime on a
    // real change, so an unchanged frame costs the shadow bake nothing.
    if (vtkProp *p = m_rungs[i]->prop())
      p->SetVisibility(drawable && i == active ? 1 : 0);
  }
}

int LodGraphicsNode::select(const cvc::lod::view_params &view) {
  const int n = rungCount();
  if (n == 0)
    return -1;
  const cvc::bounding_box bb = getWorldBoundingBox();
  const double centre[3] = {0.5 * (bb.minx + bb.maxx), 0.5 * (bb.miny + bb.maxy),
                            0.5 * (bb.minz + bb.maxz)};
  const double dx = bb.maxx - bb.minx, dy = bb.maxy - bb.miny, dz = bb.maxz - bb.minz;
  const double radius = 0.5 * std::sqrt(dx * dx + dy * dy + dz * dz);
  const double dist = cvc::lod::bound_distance_m(centre, radius, view);
  // m_current < 0 is "no history": the first choice is made without hysteresis.
  m_current = cvc::lod::select_rung(dist, m_worldError.data(), n, m_current, view);
  applyRungVisibility();
  return m_current;
}

int LodGraphicsNode::setRung(int k) {
  const int n = rungCount();
  if (n == 0)
    return -1;
  m_current = std::clamp(k, 0, n - 1);
  applyRungVisibility();
  return m_current;
}

cvc::lod::view_params make_view_params(vtkRenderer *renderer, const cvc::lod::view_params &base) {
  cvc::lod::view_params v = base;
  if (!renderer)
    return v;
  vtkCamera *cam = renderer->GetActiveCamera();
  if (!cam)
    return v;
  cam->GetPosition(v.eye);

  // The renderer's viewport in pixels -- its share of the window, not the
  // window -- so each view of a split layout budgets against its own height.
  // {0, 0} without a window, in which case base's height stands.
  const int *size = renderer->GetSize();
  const double w = size ? size[0] : 0.0;
  const double h = size ? size[1] : 0.0;
  if (h > 0.0)
    v.viewport_h_px = h;

  if (cam->GetParallelProjection()) {
    // ParallelScale is half the view HEIGHT in world units.
    const double scale = cam->GetParallelScale();
    v.ortho_px_per_m = scale > 0.0 ? v.viewport_h_px / (2.0 * scale) : 0.0;
    return v;
  }
  v.ortho_px_per_m = 0.0;
  constexpr double kPi = 3.14159265358979323846;
  double t = std::tan(0.5 * cam->GetViewAngle() * kPi / 180.0);
  // ViewAngle is vertical unless UseHorizontalViewAngle says otherwise; the
  // selection math wants the vertical half-angle, so scale by the aspect.
  if (cam->GetUseHorizontalViewAngle() && w > 0.0 && h > 0.0)
    t *= h / w;
  v.tan_half_fov = t;
  return v;
}

} // namespace gl
} // namespace cvc
