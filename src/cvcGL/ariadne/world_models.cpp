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

// world_models.cpp — see world_models.h.

#include <cmath>
#include <cvc/core/app.h>
#include <cvc/gl/GeometryNode.h>
#include <cvc/gl/LodGraphicsNode.h>
#include <cvc/gl/ariadne/world_models.h>
#include <cvc/model/model.h>
#include <cvc/model/model_file_io.h>
#include <cvc/world/model_assets.h>
#include <exception>
#include <filesystem>
#include <map>
#include <mutex>

namespace cvc {
namespace gl {
namespace ariadne {
namespace {

// The app's loaded meshes, keyed by LOD 0's path; held weakly, so a mesh lives as long as its
// instances do.
struct mesh_cache {
  static constexpr const char *app_slot = "cvc.gl.ariadne.model_meshes";
  std::mutex m;
  std::map<std::string, std::weak_ptr<model_mesh>> by_file;
};

// The asset's files: its own directory when its LOD 0 is there, else the catalog's entry's.
cvc::world::model_asset located(cvc::app &app, const cvc::world::model_asset &a) {
  std::error_code ec;
  if (!a.dir.empty() && std::filesystem::is_regular_file(a.path(0), ec))
    return a;
  cvc::world::model_asset found;
  if (cvc::world::model_catalog_of(app).find(a.name, found))
    return found;
  return a;
}

// After the rung's geometry is set (setRungStyle's order): a textured rung samples its texture, a
// far rung (bake_lods' vertex-coloured one) its per-vertex colours.
void style_model(GeometryNode &g, bool soft) {
  g.setUseSingleColor(false);
  g.setAmbient(soft ? 0.32 : 0.4);
  g.setDiffuse(soft ? 0.78 : 0.85);
  g.setSpecular(0.04);
}

} // namespace

cvc::geometry model_frame_to_world(cvc::geometry g) {
  for (auto &p : g.points()) {
    const double in[3] = {p[0], p[1], p[2]};
    double out[3];
    cvc::world::model_to_world(in, out);
    p[0] = out[0];
    p[1] = out[1];
    p[2] = out[2];
  }
  if (g.const_normals().size() == g.const_points().size())
    for (auto &n : g.normals()) {
      const double in[3] = {n[0], n[1], n[2]};
      double out[3];
      cvc::world::model_to_world(in, out);
      n[0] = out[0];
      n[1] = out[1];
      n[2] = out[2];
    }
  return g;
}

std::shared_ptr<model_mesh> load_model_mesh(cvc::app &app, const cvc::world::model_asset &asset,
                                            std::string *error) {
  const cvc::world::model_asset a = located(app, asset);
  const std::string key = a.path(0);
  if (key.empty()) {
    if (error)
      *error = "model '" + a.name + "' has no LODs";
    return nullptr;
  }
  mesh_cache &cache = app.slot<mesh_cache>();
  {
    std::lock_guard<std::mutex> g(cache.m);
    if (auto hit = cache.by_file[key].lock())
      return hit;
  }
  auto mesh = std::make_shared<model_mesh>();
  mesh->name = a.name;
  for (std::size_t k = 0; k < a.lods.size(); ++k) {
    try {
      cvc::model m = cvc::read_model(app, a.path(int(k)));
      cvc::geometry geo = model_frame_to_world(m.merged());
      if (geo.num_tris() == 0)
        throw std::runtime_error("it has no triangles");
      if (geo.const_normals().size() != geo.const_points().size())
        geo.compute_normals();
      cvc::image tex;
      for (const auto &mat : m.materials)
        if (mat.has_base_color_texture()) {
          tex = mat.base_color_texture;
          break;
        }
      mesh->pyr.rungs.push_back(std::move(geo));
      mesh->pyr.world_error_m.push_back(k == 0 ? 0.0 : a.lods[k].world_error_m);
      mesh->textures.push_back(std::move(tex));
    } catch (const std::exception &e) {
      if (k == 0) {
        if (error)
          *error = "model '" + a.name + "': " + a.path(0) + ": " + e.what();
        return nullptr;
      }
      break; // the ladder ends at the last rung that reads
    }
  }
  mesh->donors.resize(mesh->pyr.rungs.size());
  std::lock_guard<std::mutex> g(cache.m);
  std::weak_ptr<model_mesh> &slot = cache.by_file[key];
  if (auto raced = slot.lock()) // another job read it meanwhile: share that one
    return raced;
  slot = mesh;
  return mesh;
}

void model_instance_matrix(const cvc::world::generated_world::model_instance &m, double out[16]) {
  const double c = std::cos(m.yaw) * m.scale, s = std::sin(m.yaw) * m.scale;
  const double M[16] = {c, -s, 0, m.x, s, c, 0, m.y, 0, 0, m.scale, m.z, 0, 0, 0, 1};
  for (int i = 0; i < 16; ++i)
    out[i] = M[i];
}

std::shared_ptr<LodGraphicsNode>
make_model_instance(GraphicsNode &parent, const std::string &name, model_mesh &mesh,
                    const cvc::world::generated_world::model_instance &m, bool soft) {
  auto n = parent.addGraphicsChild<LodGraphicsNode>(name);
  n->setVisible(false);
  double M[16];
  model_instance_matrix(m, M);
  n->setTransform(M);
  n->setRungStyle([soft](GeometryNode &g) { style_model(g, soft); });
  cvc::lod::mesh_pyramid pyr = mesh.pyr; // the rungs share the mesh's buffers (copy-on-write)
  for (double &e : pyr.world_error_m)
    e *= m.scale; // a larger copy's error is larger by its scale
  n->setPyramid(pyr);
  for (int k = 0; k < n->rungCount() && k < int(mesh.donors.size()); ++k) {
    std::shared_ptr<GeometryNode> g = n->rung(k);
    if (!g)
      continue;
    std::shared_ptr<GeometryNode> donor = mesh.donors[k].lock();
    if (donor) {
      g->shareArraysFrom(donor);
      if (!mesh.textures[k].empty()) {
        g->setUseSingleColor(false);
        g->shareTexture(donor);
      }
    } else {
      if (!mesh.textures[k].empty()) {
        g->setUseSingleColor(false);
        g->setTexture(mesh.textures[k], false);
      }
      mesh.donors[k] = g;
    }
  }
  return n;
}

} // namespace ariadne
} // namespace gl
} // namespace cvc
