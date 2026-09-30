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

#include <cmath>
#include <cvc/gl/GeometryNode.h>
#include <cvc/gl/LodGraphicsNode.h>

namespace cvc {
namespace gl {

LodGraphicsNode::LodGraphicsNode(cvc::app &ctx, const std::string &statePath,
                                 const std::string &name)
    : GraphicsNode(ctx, statePath, name) {}

LodGraphicsNode::~LodGraphicsNode() = default;

void LodGraphicsNode::setPyramid(const cvc::lod::mesh_pyramid &pyr) {
  m_rungs.clear();
  m_worldError.clear();
  m_current = 0;
  for (std::size_t k = 0; k < pyr.rungs.size(); ++k) {
    auto child = addGraphicsChild<GeometryNode>("lod" + std::to_string(k));
    child->setGeometry(pyr.rungs[k]);
    child->setVisible(k == 0); // rung 0 (finest) shows until the first select()
    m_rungs.push_back(child);
    m_worldError.push_back(k < pyr.world_error_m.size() ? pyr.world_error_m[k] : 0.0);
  }
}

double LodGraphicsNode::rungError(int k) const {
  return (k >= 0 && k < static_cast<int>(m_worldError.size())) ? m_worldError[k] : 0.0;
}

cvc::bounding_box LodGraphicsNode::getBoundingBox() const {
  if (m_rungs.empty())
    return cvc::bounding_box();
  return m_rungs.front()->getBoundingBox(); // rung 0 = the full-detail extent
}

vtkProp *LodGraphicsNode::getProp() {
  return nullptr; // prop-less container; the visible rung is a child GeometryNode
}

int LodGraphicsNode::select(const cvc::lod::view_params &view) {
  const int n = static_cast<int>(m_rungs.size());
  if (n == 0)
    return 0;
  const cvc::bounding_box bb = getWorldBoundingBox();
  const double centre[3] = {0.5 * (bb.minx + bb.maxx), 0.5 * (bb.miny + bb.maxy),
                            0.5 * (bb.minz + bb.maxz)};
  const double dx = bb.maxx - bb.minx, dy = bb.maxy - bb.miny, dz = bb.maxz - bb.minz;
  const double radius = 0.5 * std::sqrt(dx * dx + dy * dy + dz * dz);
  const double dist = cvc::lod::bound_distance_m(centre, radius, view);
  const int k = cvc::lod::select_rung(dist, m_worldError.data(), n, m_current, view);
  for (int i = 0; i < n; ++i)
    m_rungs[i]->setVisible(i == k);
  m_current = k;
  return k;
}

} // namespace gl
} // namespace cvc
