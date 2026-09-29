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

// LodGraphicsNode.h -- a scene-graph node that owns a mesh LOD ladder and draws
// exactly one rung, chosen each frame by camera distance.
//
// It holds one GeometryNode child per rung (rung 0 = finest) plus the pyramid's
// monotone world_error_m ladder, and is itself a prop-less container. select(view)
// runs the pure math in <cvc/lod/select.h> -- bound-nearest distance to the node's
// world bounding sphere, then the hysteretic select_rung over the ladder -- and
// sets exactly the chosen rung's child visible. A SceneGraph-level pass (or the
// caller) invokes select() per frame; nothing here draws, so the selection is
// testable with no GL context.
//
// LOD is a render proxy only: a rung is never read by a nav/material/RF path.

#ifndef __CVC_GL_LOD_GRAPHICS_NODE_H__
#define __CVC_GL_LOD_GRAPHICS_NODE_H__

#include <cvc/gl/GraphicsNode.h>
#include <cvc/lod/pyramid.h>
#include <cvc/lod/select.h>
#include <memory>
#include <vector>

namespace cvc {
namespace gl {

class GeometryNode;

class LodGraphicsNode : public GraphicsNode {
public:
  LodGraphicsNode(cvc::app &ctx, const std::string &statePath, const std::string &name = "lod");
  ~LodGraphicsNode() override;

  // Populate from a built mesh pyramid: one GeometryNode child per rung, and the
  // ladder select() uses. Replaces any prior rungs. Rung 0 starts visible.
  void setPyramid(const cvc::lod::mesh_pyramid &pyr);

  int rungCount() const { return static_cast<int>(m_rungs.size()); }
  int selectedRung() const { return m_current; }
  double rungError(int k) const;

  // Choose the affordable rung for `view` and show only it. Returns the rung
  // index (0 = finest). A no-op returning 0 when empty.
  int select(const cvc::lod::view_params &view);

  // Untransformed extent of rung 0 (the full-detail mesh) -- what distance and
  // culling should see regardless of which rung currently draws.
  cvc::bounding_box getBoundingBox() const override;

protected:
  vtkProp *getProp() override; // prop-less container; the rungs are children

private:
  std::vector<std::shared_ptr<GeometryNode>> m_rungs;
  std::vector<double> m_worldError; // monotone; matches m_rungs
  int m_current = 0;
};

} // namespace gl
} // namespace cvc

#endif // __CVC_GL_LOD_GRAPHICS_NODE_H__
