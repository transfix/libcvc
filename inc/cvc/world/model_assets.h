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

// model_assets.h — textured model assets (a building, a tree, a prop) and the catalog of them.
//
// An asset is a glb at full resolution (LOD 0) and coarser glbs beside it, described by a sidecar
// `<name>.asset.json` (schema "cvc-model-asset/1", what cy-pca/graphics-assets' tools/bake_lods.py
// writes and the building-*, tree-* and prop-* cvcpkg packages install to
// share/cvc-scenes/shared/):
//
//   { "schema": "cvc-model-asset/1", "name": "PalmTree", "kind": "tree", "tags": ["palm"],
//     "units": "m", "up": "+y", "front": "+z", "origin": "bottom-centre",
//     "bounds": {"min": [x, y, z], "max": [x, y, z]},
//     "lods": [{"file": "PalmTree.glb", "triangles": 27838, "world_error_m": 0},
//              {"file": "PalmTree.lod1.glb", "triangles": 9180, "world_error_m": 0.012}, ...] }
//
// The glbs are glTF's frame (+Y up, metres). The world is +Z up with a building's front along its
// local +Y (cvc/world/sites.h, lot::yaw), so an asset is turned into the world frame by
// model_to_world: glTF (x, y, z) with its front along +Z goes to (-x, z, y). model_asset records
// its extents in the world frame. LOD is a render proxy: placement and physics read the bounds, and
// each LOD's world_error_m (the sampled Hausdorff distance from LOD 0, in metres) is what
// cvc::lod::world_error_for_switch_radius turns into the distance it is drawn from.
//
// GL-free. The catalog finds assets by scanning directories for *.asset.json; an app has one
// (model_catalog_of), filled on first use from model_search_dirs(app).

#ifndef CVC_WORLD_MODEL_ASSETS_H
#define CVC_WORLD_MODEL_ASSETS_H

#include <cstdint>
#include <map>
#include <mutex>
#include <string>
#include <vector>

namespace cvc {
class app;
namespace world {

enum class model_kind : int { building = 0, tree, prop, vehicle };
const char *model_kind_name(model_kind k); // "building", "tree", "prop", "vehicle"
bool model_kind_from_name(const std::string &n, model_kind &out); // false for an unknown name

struct model_lod {
  std::string file; // relative to the asset's directory
  std::uint64_t triangles = 0;
  double world_error_m = 0.0; // 0 for LOD 0; non-decreasing down the ladder
  int texture_px = 0;         // its base colour's size, 0 when unknown
};

struct model_asset {
  std::string name;
  model_kind kind = model_kind::prop;
  std::vector<std::string> tags;
  std::string dir; // where the sidecar and its glbs are (empty: a catalog entry with no files)
  // Extents in the world frame (model_to_world applied): half-widths across the front (x) and
  // front to back (y), its height, and where its bottom is (0 for an origin at bottom-centre).
  double half_x = 0.0, half_y = 0.0, height = 0.0, base_z = 0.0;
  double centre_x = 0.0, centre_y = 0.0; // the footprint's centre, in the world frame
  std::vector<model_lod> lods;           // lods[0] is full resolution

  bool has_tag(const std::string &t) const;
  // The file of `lod` (clamped to the ladder) as a path: dir/file. Empty with no LODs.
  std::string path(int lod = 0) const;
};

// The model frame to the world frame: glTF (x, y, z), +Y up and front +Z, to world (-x, z, y),
// +Z up and front +Y. A proper rotation (no mirror).
inline void model_to_world(const double in[3], double out[3]) {
  out[0] = -in[0];
  out[1] = in[2];
  out[2] = in[1];
}

// Parse a sidecar. `dir` is recorded as the asset's directory. Throws std::runtime_error naming
// `what` for malformed JSON, a schema other than cvc-model-asset/1, an unknown kind, no name, no
// LODs, bounds that are not finite and ordered, or world errors that decrease down the ladder.
model_asset parse_model_asset(const std::string &json_text, const std::string &dir,
                              const std::string &what = "model asset");
// Read and parse the sidecar at `path` (its directory becomes the asset's).
model_asset read_model_asset(const std::string &path);
// The asset as sidecar JSON (what parse_model_asset reads back; the bounds in the glTF frame).
std::string model_asset_json(const model_asset &a);

// The assets by name. Adding a name already there replaces it (a later directory overrides an
// earlier one). Thread-safe.
class model_catalog {
public:
  void add(const model_asset &a);
  // Every *.asset.json directly in `dir` (not recursive) that parses, in file-name order; returns
  // how many were added. A sidecar that does not parse is skipped and its error kept (errors()).
  std::size_t scan(const std::string &dir);
  bool find(const std::string &name, model_asset &out) const;
  bool contains(const std::string &name) const;
  std::vector<std::string> names() const; // sorted
  std::vector<std::string> errors() const;
  std::size_t size() const;
  void clear();

private:
  mutable std::mutex m_;
  std::map<std::string, model_asset> assets_;
  std::vector<std::string> errors_;
};

// Where an app looks for assets, lowest priority first (a later directory's asset replaces an
// earlier one's of the same name): share/cvc-scenes/shared under the executable's prefix, then
// under $CONDA_PREFIX, $CVCPKG_PREFIX and $CVC_PREFIX, then $CVC_NAV_BUNDLE/../shared, then each
// directory in $CVC_MODEL_PATH (an OS path list, the first listed winning). The same places
// cvc::ariadne::find_vehicle_model looks, in the same precedence. Directories that do not exist are
// left out.
std::vector<std::string> model_search_dirs(cvc::app &app);
// The app's catalog: scanned from model_search_dirs on first use. Add to it to make an asset
// known without a file on disk (tests, a host's own models).
model_catalog &model_catalog_of(cvc::app &app);

} // namespace world
} // namespace cvc

#endif // CVC_WORLD_MODEL_ASSETS_H
