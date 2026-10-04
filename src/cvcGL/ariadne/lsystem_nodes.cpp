/*
  Copyright 2007-2011 The University of Texas at Austin

        Authors: Joe Rivera <transfix@ices.utexas.edu>
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

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

// lsystem_nodes.cpp — the custom scene node realizers (see lsystem_nodes.h). Thin cvcGL wrappers:
// they read a node's props, call the cvc::lsys generators, and wire the resulting mesh/field into a
// GeometryNode/VolumeNode (+ a per-frame tick). The procedural generation itself lives in
// cvc::lsys.

#include <cmath>
#include <cvc/ariadne/scene.h>
#include <cvc/ariadne/value.h>
#include <cvc/core/app.h>
#include <cvc/core/world_clock.h>
#include <cvc/geometry/geometry.h>
#include <cvc/gl/GeometryNode.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/ariadne/lsystem_nodes.h>
#include <cvc/gl/ariadne/scene_realize.h>
#include <cvc/lsys/forest.h>
#include <memory>
#include <string>
#include <vector>

namespace cvc {
namespace gl {
namespace ariadne {
namespace {

namespace ari = cvc::ariadne;

// Sample a realized heightfield GeometryNode: make_heightfield lays out a regular row-major res×res
// grid, so a nearest-cell lookup gives the ground height (and band colour) at (x, y). Ported from
// the demo so the forest plants on the island's actual relief.
struct TerrainHeights {
  const cvc::geometry *geom = nullptr;
  int res = 0;
  double minx = 0, miny = 0, step = 1;
  bool ok() const { return geom && res > 1; }
  void build(const cvc::geometry *g) {
    geom = g;
    if (!g || g->const_points().empty())
      return;
    const std::size_t n = g->const_points().size();
    res = static_cast<int>(std::lround(std::sqrt(double(n))));
    if (res < 2 || std::size_t(res) * res != n) {
      res = 0;
      return;
    }
    double maxx = -1e30, maxy = -1e30;
    minx = miny = 1e30;
    for (const auto &p : g->const_points()) {
      minx = std::min(minx, p[0]);
      maxx = std::max(maxx, p[0]);
      miny = std::min(miny, p[1]);
      maxy = std::max(maxy, p[1]);
    }
    step = (maxx - minx) / (res - 1);
  }
  std::size_t index(double x, double y) const {
    const int i = std::max(0, std::min(res - 1, int(std::lround((x - minx) / step))));
    const int j = std::max(0, std::min(res - 1, int(std::lround((y - miny) / step))));
    return std::size_t(j) * res + i;
  }
  double at(double x, double y) const { return ok() ? geom->const_points()[index(x, y)][2] : 0.0; }
};

// Flatten a mesh's points into a flat x,y,z buffer (the wind tick's working buffer).
std::vector<double> flatten_points(const cvc::geometry &g) {
  std::vector<double> b(g.const_points().size() * 3);
  for (std::size_t v = 0; v < g.const_points().size(); ++v) {
    b[v * 3] = g.const_points()[v][0];
    b[v * 3 + 1] = g.const_points()[v][1];
    b[v * 3 + 2] = g.const_points()[v][2];
  }
  return b;
}

// The pine needle colour (the needle-LINES node is single-coloured).
constexpr double C_NEEDLE_R = 0.137, C_NEEDLE_G = 0.557, C_NEEDLE_B = 0.137;

// type: forest_trees — scatter a forest of the demo's two species on a ground node, via
// cvc::lsys::grow_forest. props: count, seed, ground, sea_level, span, species(mix|pine|branchy),
// the branchy knobs (length/radius/levels/branches/scale), pine_scale, and wind (0 disables sway).
std::shared_ptr<GraphicsNode> realize_forest_trees(SceneGraph &sg, const ari::SceneNode &n,
                                                   GraphicsNode *parent, RealizedScene &out,
                                                   std::vector<std::string> *warnings) {
  const auto warn = [&](const std::string &m) {
    if (warnings)
      warnings->push_back("ari: forest_trees '" + n.id + "': " + m);
  };

  cvc::lsys::forest_params fp;
  fp.seed = static_cast<std::uint64_t>(n.props.num("seed", 1337.0));
  fp.count = static_cast<int>(n.props.num("count", 60.0));
  fp.span = n.props.num("span", 100.0);
  fp.sea_level = n.props.num("sea_level", 0.5);
  const std::string species = n.props.str("species").empty() ? "mix" : n.props.str("species");
  fp.species = species == "pine"      ? cvc::lsys::species_mix::pine
               : species == "branchy" ? cvc::lsys::species_mix::branchy
                                      : cvc::lsys::species_mix::mix;
  fp.scale = n.props.num("scale", 1.0);
  fp.length = n.props.num("length", 6.0);
  fp.radius = n.props.num("radius", 0.7);
  fp.levels = static_cast<int>(n.props.num("levels", 4.0));
  fp.branches = static_cast<int>(n.props.num("branches", 3.0));
  fp.pine_scale = n.props.num("pine_scale", 1.35);
  const double wind = n.props.num("wind", 1.0);

  const std::string ground = n.props.str("ground").empty() ? "terrain" : n.props.str("ground");
  auto heights = std::make_shared<TerrainHeights>();
  if (auto gnode = std::dynamic_pointer_cast<GeometryNode>(sg.getGraphics(ground)))
    heights->build(gnode->getGeometry());
  if (!heights->ok())
    warn("ground node '" + ground + "' is not a realized heightfield — planting on a flat plane");
  const cvc::lsys::height_fn height_at = [heights](double x, double y) {
    return heights->ok() ? heights->at(x, y) : 0.0;
  };

  cvc::geometry wood, needle;
  auto windData = std::make_shared<cvc::lsys::forest_wind>();
  const cvc::lsys::forest_result res =
      cvc::lsys::grow_forest(fp, height_at, wood, needle, wind != 0.0 ? windData.get() : nullptr);
  if (wood.const_points().empty()) {
    warn("no trees planted (no dry land above sea_level in the scatter span)");
    return nullptr;
  }

  std::shared_ptr<GraphicsNode> gn =
      parent ? parent->createChild<GeometryNode>(n.id, wood) : sg.addGraphics(n.id, wood);
  auto woodNode = std::dynamic_pointer_cast<GeometryNode>(gn);
  if (woodNode)
    woodNode->setUseSingleColor(false); // per-vertex wood colour

  std::shared_ptr<GeometryNode> needleNode;
  if (!needle.const_points().empty()) {
    needleNode = std::dynamic_pointer_cast<GeometryNode>(sg.addGraphics(n.id + "_needles", needle));
    if (needleNode) {
      needleNode->setRenderMode(cvc::gl::GeometryRenderMode::LINES);
      needleNode->setUseSingleColor(true);
      needleNode->setColor(C_NEEDLE_R, C_NEEDLE_G, C_NEEDLE_B);
    }
  }

  // Wind: re-pose the pines into the merged buffers each frame (route C — two buffer uploads for
  // the whole forest). World (simulation) time from the scene clock, so pausing the sim freezes the
  // sway. The branchy trees have no sway records and sit untouched at bind pose.
  if (wind != 0.0 && !windData->trees.empty() && woodNode) {
    auto woodBind = std::make_shared<std::vector<double>>(flatten_points(wood));
    auto needleBind = std::make_shared<std::vector<double>>(flatten_points(needle));
    auto workW = std::make_shared<std::vector<double>>(*woodBind);
    auto workN = std::make_shared<std::vector<double>>(*needleBind);
    cvc::app *app = &sg.appContext();
    std::weak_ptr<GeometryNode> ww = woodNode, wn = needleNode;
    out.custom_ticks.push_back([=](vtkRenderer *) {
      auto w = ww.lock();
      if (!w)
        return;
      const double t = app->world_clock().t();
      *workW = *woodBind;
      *workN = *needleBind;
      cvc::lsys::repose_forest(*windData, t, wind, *workW, *workN);
      w->updateVertices(*workW);
      if (auto nn = wn.lock())
        nn->updateVertices(*workN);
    });
  }
  (void)res;
  return gn;
}

} // namespace

void register_lsystem_node_types() {
  register_scene_node_type("forest_trees", realize_forest_trees);
}

} // namespace ariadne
} // namespace gl
} // namespace cvc
