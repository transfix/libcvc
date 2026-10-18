// pycvc_world.i — SWIG surface for the world's one representation (G4b of
// docs/roadmap/WORLD-GEOMETRY-ROADMAP.md). Python reads the same terrain, buildings
// and surfaces as the renderer, the physics world and the navigator do in C++:
//
//   city = pycvc.load_city_bundle(dir, land_cover=True)
//   field = city.terrain()              # a pycvc.height_field (a pycvc.geometry)
//   field.height(x, y)                  # the mesh's height: the triangle under (x, y)
//   field.heights()                     # numpy (rows, cols) float32, row 0 at min y
//   world = city.world()                # a pycvc.world_geometry
//   world.names(), world.item_geometry("buildings"), world.bounds()
//
// NOT a standalone module: %include'd by pycvc.i after cvc::geometry (height_field
// derives from it) and after pycvc_nav.i (land cover is cvc::nav's provider).
//
// SWIG APPROACH:
//  * The classes are DECLARED here by hand, as the subset Python gets, and every
//    call goes to the real libcvc headers (#included below). height_field.h has
//    vector-valued constructors, deleted operators and a protected write guard;
//    world_geometry and city_bundle hold shared_ptr<const geometry> and
//    std::function members. None of those marshal, and %ignore-ing them member by
//    member is brittle (an %ignore also swallows an %extend of the same name).
//  * height_field derives from geometry here as in C++, so a height field is a
//    pycvc.geometry: num_vertices(), num_triangles() and extents() work. Its mesh
//    follows the grid: vertices() is a read-only view, clear() clears both,
//    load() is refused, and the mesh editors (add_*, merge, ...) raise through the
//    C++ write guard. mesh() returns a plain pycvc.geometry COPY, for drawing or
//    editing.
//  * Arrays go in as any sequence or numpy array, and come back as numpy COPIES
//    (read-only; set_heights / set_height change the field).
//  * The world's items come back as copy-on-write copies (item_geometry, terrain):
//    the same arrays as the C++ objects, never a re-read file.

%{
#include <cvc/geometry/height_field.h>
#include <cvc/nav/land_cover.h>
#include <cvc/world/city_bundle.h>
#include <cvc/world/world_geometry.h>
#include <cmath>
#include <limits>
#include <memory>
#include <string>

namespace pycvc_world {

// Any sequence or array of numbers as floats (float64 input is narrowed).
// None or null: empty.
std::vector<float> floats(PyObject *o, const char *what) {
  if (!o || o == Py_None)
    return {};
  PyArrayObject *a = reinterpret_cast<PyArrayObject *>(
      PyArray_FROMANY(o, NPY_FLOAT, 0, 2, NPY_ARRAY_C_CONTIGUOUS | NPY_ARRAY_FORCECAST));
  if (!a) {
    PyErr_Clear();
    throw std::invalid_argument(std::string(what) + ": expected numbers (a sequence or array)");
  }
  const float *p = static_cast<const float *>(PyArray_DATA(a));
  std::vector<float> out(p, p + PyArray_SIZE(a));
  Py_DECREF(a);
  return out;
}

// Any sequence or array of numbers as doubles, with the array's shape.
std::vector<double> doubles(PyObject *o, const char *what, std::vector<long> &shape) {
  PyArrayObject *a = reinterpret_cast<PyArrayObject *>(
      PyArray_FROMANY(o, NPY_DOUBLE, 0, 0, NPY_ARRAY_C_CONTIGUOUS | NPY_ARRAY_FORCECAST));
  if (!a) {
    PyErr_Clear();
    throw std::invalid_argument(std::string(what) + ": expected numbers (a sequence or array)");
  }
  shape.assign(PyArray_DIMS(a), PyArray_DIMS(a) + PyArray_NDIM(a));
  const double *p = static_cast<const double *>(PyArray_DATA(a));
  std::vector<double> out(p, p + PyArray_SIZE(a));
  Py_DECREF(a);
  return out;
}

// Surface ids (0..65535) from any sequence or array of integers. None: empty.
std::vector<cvc::face_material_t> ids(PyObject *o, const char *what) {
  if (!o || o == Py_None)
    return {};
  PyArrayObject *a = reinterpret_cast<PyArrayObject *>(
      PyArray_FROMANY(o, NPY_INT64, 0, 2, NPY_ARRAY_C_CONTIGUOUS));
  if (!a) {
    PyErr_Clear();
    throw std::invalid_argument(std::string(what) + ": expected integers (a sequence or array)");
  }
  const std::int64_t *p = static_cast<const std::int64_t *>(PyArray_DATA(a));
  const npy_intp n = PyArray_SIZE(a);
  std::vector<cvc::face_material_t> out;
  out.reserve(static_cast<size_t>(n));
  for (npy_intp i = 0; i < n; ++i) {
    if (p[i] < 0 || p[i] > 65535) {
      Py_DECREF(a);
      throw std::invalid_argument(std::string(what) + ": a surface id is outside 0..65535");
    }
    out.push_back(static_cast<cvc::face_material_t>(p[i]));
  }
  Py_DECREF(a);
  return out;
}

// A numpy copy of `values` with `shape`.
template <class T> pycvc::ArrayView copy_of(std::vector<T> values, std::vector<long> shape,
                                             pycvc::DType dtype) {
  auto owned = std::make_shared<std::vector<T>>(std::move(values));
  pycvc::ArrayView v;
  v.dtype = dtype;
  v.writable = false;
  v.shape = std::move(shape);
  v.data = owned->empty() ? nullptr : owned->data();
  v.owner = owned;
  return v;
}

const cvc::world::geometry_item &item(const cvc::world::world_geometry &w,
                                      const std::string &name) {
  const cvc::world::geometry_item *i = w.find(name);
  if (!i)
    throw std::invalid_argument("world_geometry: no item called '" + name + "'");
  return *i;
}

cvc::world::geometry_item placed(const std::string &name,
                                 std::shared_ptr<const cvc::geometry> geometry, int surface,
                                 const std::vector<double> &position,
                                 const std::vector<double> &rotation, int roles) {
  if (surface < 0 || surface > 65535)
    throw std::invalid_argument("world_geometry: surface id outside 0..65535");
  if (roles < 0 || roles > cvc::world::role_all)
    throw std::invalid_argument("world_geometry: roles must be a combination of 1, 2 and 4");
  if (!position.empty() && position.size() != 3)
    throw std::invalid_argument("world_geometry: position is (x, y, z)");
  if (!rotation.empty() && rotation.size() != 4)
    throw std::invalid_argument("world_geometry: rotation is a quaternion (w, x, y, z)");
  cvc::world::geometry_item i{name, std::move(geometry),
                              static_cast<std::uint16_t>(surface)};
  if (!position.empty())
    i.position = {{position[0], position[1], position[2]}};
  if (!rotation.empty())
    i.rotation = {{rotation[0], rotation[1], rotation[2], rotation[3]}};
  i.roles = static_cast<std::uint8_t>(roles);
  return i;
}

PyObject *surface_dict(const cvc::world::surface_class &c) {
  return Py_BuildValue(
      "{s:i,s:s,s:f,s:O,s:(fff),s:s,s:f,s:f,s:f,s:f}", "id", static_cast<int>(c.id), "name",
      c.name, "rho", static_cast<double>(c.rho), "hard", c.hard ? Py_True : Py_False, "albedo",
      static_cast<double>(c.albedo[0]), static_cast<double>(c.albedo[1]),
      static_cast<double>(c.albedo[2]), "rf_material", c.rf_material, "veg_density",
      static_cast<double>(c.veg_density), "mu", static_cast<double>(c.mu), "restitution",
      static_cast<double>(c.restitution), "rolling_resistance",
      static_cast<double>(c.rolling_resistance));
}

} // namespace pycvc_world
%}

// ── cvc::height_field ───────────────────────────────────────────────────────

namespace cvc {
class height_field : public geometry {
public:
  height_field();
  height_field(const height_field &other);
  int rows() const;
  int cols() const;
  int cell_rows() const;
  int cell_cols() const;
  double x0() const;
  double y0() const;
  double dx() const;
  double dy() const;
  double max_x() const;
  double max_y() const;
  // Sample (r, c)'s height; NaN for a hole.
  float height_at(int r, int c) const;
  unsigned short cell_material(int r, int c) const;
  void set_height(int r, int c, float h);
  void set_cell_material(int r, int c, unsigned short id);
  // The mesh's height at (x, y): the plane of the triangle under the point (each
  // cell split south-west to north-east, as the physics engine splits it). NaN
  // outside the grid or over a hole.
  double height(double x, double y) const;
  // The surface id of the cell under (x, y); 0 outside the grid.
  unsigned short material(double x, double y) const;
  // The mesh vertex of sample (r, c), or -1 for a hole.
  std::int64_t vertex_of(int r, int c) const;
  std::int64_t first_face_of(int r, int c) const;
};
} // namespace cvc

%extend cvc::height_field {
  // rows x cols samples (each at least 2), sample (r, c) at (x0 + c*dx, y0 + r*dy).
  // `heights`: rows*cols numbers, row 0 (min y) first, or a (rows, cols) array; None
  // for all zero. `cell_materials`: (rows-1)*(cols-1) surface ids, or None.
  height_field(int rows, int cols, double x0, double y0, double dx, double dy,
               PyObject *heights = nullptr, PyObject *cell_materials = nullptr) {
    return new cvc::height_field(rows, cols, x0, y0, dx, dy,
                                 pycvc_world::floats(heights, "height_field: heights"),
                                 pycvc_world::ids(cell_materials, "height_field: cell_materials"));
  }

  // The heights as a read-only numpy (rows, cols) float32 copy, row 0 at min y.
  pycvc::ArrayView heights() const {
    return pycvc_world::copy_of($self->heights(), {(long)$self->rows(), (long)$self->cols()},
                                pycvc::DType::Float32);
  }
  // The cell surface ids as a read-only numpy (rows-1, cols-1) uint16 copy (empty
  // when every cell is surface 0).
  pycvc::ArrayView cell_materials() const {
    const auto &m = $self->cell_materials();
    std::vector<long> shape{0};
    if (!m.empty())
      shape = {(long)$self->cell_rows(), (long)$self->cell_cols()};
    return pycvc_world::copy_of(std::vector<cvc::face_material_t>(m.begin(), m.end()), shape,
                                pycvc::DType::UInt16);
  }
  void set_heights(PyObject *heights) {
    $self->set_heights(pycvc_world::floats(heights, "height_field.set_heights"));
  }
  void set_cell_materials(PyObject *ids) {
    $self->set_cell_materials(pycvc_world::ids(ids, "height_field.set_cell_materials"));
  }

  // height(x, y) at many points in one call: xs and ys of one shape in, a float64
  // array of that shape out.
  pycvc::ArrayView heights_at(PyObject *xs, PyObject *ys) const {
    std::vector<long> sx, sy;
    const std::vector<double> x = pycvc_world::doubles(xs, "height_field.heights_at: xs", sx);
    const std::vector<double> y = pycvc_world::doubles(ys, "height_field.heights_at: ys", sy);
    if (x.size() != y.size())
      throw std::invalid_argument("height_field.heights_at: xs and ys differ in size");
    std::vector<double> h(x.size());
    for (size_t i = 0; i < x.size(); ++i)
      h[i] = $self->height(x[i], y[i]);
    return pycvc_world::copy_of(std::move(h), sx, pycvc::DType::Float64);
  }
  // The upward unit normal of the triangle under (x, y), as a tuple; NaNs where
  // height is NaN.
  PyObject *normal(double x, double y) const {
    const auto n = $self->normal(x, y);
    return Py_BuildValue("(ddd)", (double)n[0], (double)n[1], (double)n[2]);
  }
  // The cell (r, c) under (x, y), or None outside the grid.
  PyObject *cell_of(double x, double y) const {
    int r = 0, c = 0;
    if (!$self->cell_of(x, y, r, c))
      Py_RETURN_NONE;
    return Py_BuildValue("(ii)", r, c);
  }
  // (min_x, min_y, max_x, max_y): the grid's extent.
  PyObject *bounds2d() const {
    return Py_BuildValue("(dddd)", $self->x0(), $self->y0(), $self->max_x(), $self->max_y());
  }
  // The mesh as a plain, editable pycvc.geometry: the field's own triangles. What a
  // renderer draws. Its vertices are its own (pycvc.geometry.vertices() writes in
  // place), the other arrays are shared until written.
  cvc::geometry mesh() const {
    cvc::geometry g(*$self);
    g.points(); // detach the vertices: writes through g's view never reach the field
    return g;
  }

  // pycvc.geometry's in-place edits, made safe for a mesh that follows its grid:
  // vertices() is READ-ONLY (edit the heights, or a mesh() copy); clear() empties
  // the grid and the mesh together; load() is refused (read into a plain
  // pycvc.geometry). The mesh's other editors raise through the write guard.
  pycvc::ArrayView vertices() {
    cvc::geometry::points_ptr_t container = $self->points_ptr();
    pycvc::ArrayView v;
    v.dtype = pycvc::DType::Float64;
    v.writable = false;
    v.shape = {static_cast<long>(container->size()), 3};
    v.data = container->empty() ? nullptr : &(*container)[0][0];
    v.owner = std::shared_ptr<void>(container.get(), [container](void *) {});
    return v;
  }
  void clear() { $self->clear(); }
  void load(const std::string &path) {
    throw std::logic_error("height_field.load('" + path +
                           "'): the mesh follows the grid; read the file into a plain "
                           "pycvc.geometry, or build the grid with a loader");
  }
}

// ── cvc::world::world_geometry ──────────────────────────────────────────────

namespace cvc {
namespace world {
class world_geometry {
public:
  world_geometry();
};
} // namespace world
} // namespace cvc

%feature("kwargs") cvc::world::world_geometry::add_terrain;
%feature("kwargs") cvc::world::world_geometry::add_mesh;

%extend cvc::world::world_geometry {
  // Add a height field (a copy sharing its arrays). `surface` covers the cells with
  // no cell material; `roles` is render 1 | collide 2 | navigate 4.
  void add_terrain(const std::string &name, const cvc::height_field &field, int surface = 0,
                   int roles = 7) {
    $self->add(pycvc_world::placed(name, std::make_shared<cvc::height_field>(field), surface, {},
                                   {}, roles));
  }
  // Add a mesh (a copy sharing its arrays) at position (x, y, z) and rotation
  // (w, x, y, z).
  void add_mesh(const std::string &name, const cvc::geometry &mesh, int surface = 0,
                const std::vector<double> &position = std::vector<double>(),
                const std::vector<double> &rotation = std::vector<double>(), int roles = 7) {
    $self->add(pycvc_world::placed(name, std::make_shared<cvc::geometry>(mesh), surface, position,
                                   rotation, roles));
  }

  // The items' names, in order.
  std::vector<std::string> names() const {
    std::vector<std::string> out;
    for (const auto &i : $self->items())
      out.push_back(i.name);
    return out;
  }
  bool contains(const std::string &name) const { return $self->find(name) != nullptr; }
  // "height_field" or "mesh".
  std::string kind(const std::string &name) const {
    return pycvc_world::item(*$self, name).height_field() ? "height_field" : "mesh";
  }
  // The item's geometry: for a height field, its mesh. A copy sharing the arrays.
  cvc::geometry item_geometry(const std::string &name) const {
    return *pycvc_world::item(*$self, name).geometry;
  }
  // The item `name` as a height field, or the first height field when `name` is
  // empty. Raises when there is none.
  cvc::height_field terrain(const std::string &name = std::string()) const {
    std::shared_ptr<const cvc::height_field> f =
        name.empty() ? $self->terrain() : pycvc_world::item(*$self, name).height_field();
    if (!f)
      throw std::invalid_argument(name.empty() ? "world_geometry: no height field"
                                               : "world_geometry: '" + name +
                                                     "' is not a height field");
    return *f;
  }
  bool has_terrain() const { return $self->terrain() != nullptr; }
  int surface_of(const std::string &name) const { return pycvc_world::item(*$self, name).surface; }
  int roles_of(const std::string &name) const { return pycvc_world::item(*$self, name).roles; }
  PyObject *position_of(const std::string &name) const {
    const auto &p = pycvc_world::item(*$self, name).position;
    return Py_BuildValue("(ddd)", p[0], p[1], p[2]);
  }
  PyObject *rotation_of(const std::string &name) const {
    const auto &q = pycvc_world::item(*$self, name).rotation;
    return Py_BuildValue("(dddd)", q[0], q[1], q[2], q[3]);
  }

  // (min_x, min_y, min_z, max_x, max_y, max_z) of every item and volume, placement
  // applied; None when the world is empty.
  PyObject *bounds() const {
    const cvc::world::world_bounds b = $self->bounds();
    if (b.empty())
      Py_RETURN_NONE;
    return Py_BuildValue("(dddddd)", b.min[0], b.min[1], b.min[2], b.max[0], b.max[1], b.max[2]);
  }
  // Every surface id some face of some item uses, ascending.
  std::vector<unsigned long> surfaces_used() const {
    const auto used = $self->surfaces_used();
    return std::vector<unsigned long>(used.begin(), used.end());
  }

  // The surface table: one class per id, the same table every consumer reads.
  int surface_count() const { return static_cast<int>($self->surfaces().size()); }
  int surface_id(const std::string &name) const { return $self->surfaces().by_name(name); }
  // A surface class as a dict: id, name, rho, hard, albedo, rf_material,
  // veg_density, mu, restitution, rolling_resistance.
  PyObject *surface(int id) const {
    if (id < 0 || id >= static_cast<int>($self->surfaces().size()))
      throw std::invalid_argument("world_geometry.surface: no surface " + std::to_string(id));
    return pycvc_world::surface_dict($self->surfaces()[static_cast<std::uint16_t>(id)]);
  }

  // The volumes' names (water, snow, wind), in order.
  std::vector<std::string> volume_names() const {
    std::vector<std::string> out;
    for (const auto &v : $self->volumes())
      out.push_back(v.name);
    return out;
  }
  int __len__() const { return static_cast<int>($self->items().size()); }
}

// ── cvc::world::load_city_bundle ────────────────────────────────────────────

namespace cvc {
namespace world {
struct city_bundle {
  std::string dir;
  double min_x, min_y, max_x, max_y; // the terrain's extent
  std::string satellite;             // satellite.png's path, or empty
  bool buildings_loaded;
  std::string buildings_error; // why the buildings are missing, when they are
  bool land_cover_loaded;
};
} // namespace world
} // namespace cvc

%extend cvc::world::city_bundle {
  // The bundle's world: "terrain" and, when loaded, "buildings". A copy sharing the
  // geometry.
  cvc::world::world_geometry world() const { return $self->geometry; }
  // The terrain height field (a copy sharing its arrays).
  cvc::height_field terrain() const {
    const auto f = $self->geometry.terrain();
    if (!f)
      throw std::runtime_error("city_bundle: no terrain");
    return *f;
  }
  PyObject *bounds2d() const {
    return Py_BuildValue("(dddd)", $self->min_x, $self->min_y, $self->max_x, $self->max_y);
  }
}

%feature("kwargs") pycvc::load_city_bundle;

%inline %{
namespace pycvc {
// Load a city scene bundle (terrain.json, buildings.glb, land-cover masks) with
// the C++ loader (cvc::world::load_city_bundle): the terrain comes back row 0 at
// min y, flipped once from terrain.json's north-first rows, and the buildings z up
// and unlifted, exactly as the C++ demos, physics and navigator read them.
//   land_cover: classify the bundle's masks and imagery into the terrain's cell
//     surfaces (cvc::nav's provider);
//   terrain_rows: "north_first" (the bundles' own order) or "south_first";
//   buildings_up: "z" (the bundles) or "y" (a standard glTF).
// Raises when terrain.json is missing or malformed; missing buildings leave
// buildings_loaded False with the reason in buildings_error.
cvc::world::city_bundle load_city_bundle(const std::string &dir, bool buildings = true,
                                         bool prefer_flat = false, bool land_cover = false,
                                         const std::string &terrain_rows = "north_first",
                                         const std::string &buildings_up = "z",
                                         const std::string &building_surface = "building_concrete",
                                         const std::string &terrain_surface = "dirt") {
  cvc::world::city_bundle_options o;
  o.buildings = buildings;
  o.prefer_flat = prefer_flat;
  if (terrain_rows == "north_first")
    o.terrain_rows = cvc::world::city_bundle_options::row_order::north_first;
  else if (terrain_rows == "south_first")
    o.terrain_rows = cvc::world::city_bundle_options::row_order::south_first;
  else
    throw std::invalid_argument("load_city_bundle: terrain_rows is 'north_first' or 'south_first'");
  if (buildings_up == "z")
    o.buildings_up = cvc::world::city_bundle_options::up_axis::z;
  else if (buildings_up == "y")
    o.buildings_up = cvc::world::city_bundle_options::up_axis::y;
  else
    throw std::invalid_argument("load_city_bundle: buildings_up is 'z' or 'y'");
  o.building_surface = building_surface;
  o.terrain_surface = terrain_surface;
  if (land_cover)
    o.land_cover = cvc::nav::land_cover_provider(cvc::world::surface_registry::builtin());
  return cvc::world::load_city_bundle(dir, o);
}

// terrain.json's grid size and extent alone, as a dict (rows, cols, min_x, min_y,
// max_x, max_y): what a reader of the bundle's other rasters lays them over.
PyObject *read_terrain_extent(const std::string &dir) {
  const cvc::world::terrain_extent e = cvc::world::read_terrain_extent(dir);
  return Py_BuildValue("{s:i,s:i,s:d,s:d,s:d,s:d}", "rows", e.rows, "cols", e.cols, "min_x",
                       e.min_x, "min_y", e.min_y, "max_x", e.max_x, "max_y", e.max_y);
}
} // namespace pycvc
%}
