// pycvc_geometry.cpp — Geometry facade implementation (the only TU that
// includes libcvc).
#include "pycvc_geometry.h"

#include <cvc/core/app.h>
#include <cvc/core/types.h>
#include <cvc/geometry/geometry.h>
#include <cvc/geometry/geometry_file_io.h>
#include <stdexcept>

namespace pycvc {

namespace {
// Process-wide app context (thread pool, state root). Mirrors pycvc_volume.cpp
// and the CLI's `static cvc::app app;`. The mesh-processing methods
// (smoothing/quality_improve) need an app: smoothing takes one explicitly, and
// quality_improve reaches through cvc::geometry::ctx() internally — so the
// facade's geometry is constructed WITH this context (a default-constructed
// cvc::geometry has a null ctx, which quality_improve would dereference).
cvc::app &ctx() {
  static cvc::app app;
  return app;
}
} // namespace

Geometry::Geometry() : geom_(std::make_shared<cvc::geometry>(ctx())) {}
Geometry::~Geometry() = default;

std::size_t Geometry::add_vertex(double x, double y, double z) {
  cvc::geometry::point_t p;
  p[0] = x;
  p[1] = y;
  p[2] = z;
  geom_->points().push_back(p);
  return geom_->points().size() - 1;
}

void Geometry::add_triangle(std::size_t a, std::size_t b, std::size_t c) {
  cvc::geometry::tri_t t;
  t[0] = a;
  t[1] = b;
  t[2] = c;
  geom_->tris().push_back(t);
}

void Geometry::add_line(std::size_t a, std::size_t b) {
  cvc::geometry::line_t l;
  l[0] = a;
  l[1] = b;
  geom_->lines().push_back(l);
}

void Geometry::add_vertices(const std::vector<double> &xyz) {
  if (xyz.size() % 3 != 0)
    throw std::invalid_argument("add_vertices: length must be a multiple of 3");
  auto &pts = geom_->points();
  pts.reserve(pts.size() + xyz.size() / 3);
  for (std::size_t i = 0; i + 2 < xyz.size(); i += 3) {
    cvc::geometry::point_t p;
    p[0] = xyz[i];
    p[1] = xyz[i + 1];
    p[2] = xyz[i + 2];
    pts.push_back(p);
  }
}

void Geometry::add_triangles(const std::vector<unsigned long> &ijk) {
  if (ijk.size() % 3 != 0)
    throw std::invalid_argument("add_triangles: length must be a multiple of 3");
  auto &tris = geom_->tris();
  tris.reserve(tris.size() + ijk.size() / 3);
  for (std::size_t i = 0; i + 2 < ijk.size(); i += 3) {
    cvc::geometry::tri_t t;
    t[0] = ijk[i];
    t[1] = ijk[i + 1];
    t[2] = ijk[i + 2];
    tris.push_back(t);
  }
}

void Geometry::add_lines(const std::vector<unsigned long> &ab) {
  if (ab.size() % 2 != 0)
    throw std::invalid_argument("add_lines: length must be a multiple of 2");
  auto &lines = geom_->lines();
  lines.reserve(lines.size() + ab.size() / 2);
  for (std::size_t i = 0; i + 1 < ab.size(); i += 2) {
    cvc::geometry::line_t l;
    l[0] = ab[i];
    l[1] = ab[i + 1];
    lines.push_back(l);
  }
}

void Geometry::set_colors(const std::vector<double> &rgb) {
  if (rgb.size() != geom_->points().size() * 3)
    throw std::invalid_argument("set_colors: length must equal 3 * num_vertices()");
  auto &colors = geom_->colors();
  colors.clear();
  colors.reserve(rgb.size() / 3);
  for (std::size_t i = 0; i + 2 < rgb.size(); i += 3) {
    cvc::geometry::color_t c;
    c[0] = rgb[i];
    c[1] = rgb[i + 1];
    c[2] = rgb[i + 2];
    colors.push_back(c);
  }
}

std::size_t Geometry::num_vertices() const { return geom_->points().size(); }
std::size_t Geometry::num_triangles() const { return geom_->tris().size(); }
std::size_t Geometry::num_lines() const { return geom_->lines().size(); }

// Build a zero-copy view over one of cvc::geometry's shared containers, pinning
// the SPECIFIC container shared_ptr (not the whole geometry). cvc::geometry
// already copy-on-writes each container (points()/colors() -> pre_write ->
// make_unique detaches when the shared_ptr is not unique). By owning that exact
// shared_ptr, the view participates in that refcount: a later append/clear/load
// COW-detaches to a fresh copy and this block is RETIRED to the view — a valid,
// decoupled snapshot — instead of being reallocated out from under it. Read via
// the non-detaching *_ptr() accessor so merely creating a view doesn't itself
// trigger a detach (two views before any mutation still alias one buffer).
template <class SharedPtr> static ArrayView view_over(const SharedPtr &container) {
  auto &vec = *container; // std::vector<boost::array<double,3>> — 3*N contiguous
  ArrayView v;
  v.dtype = DType::Float64;
  v.writable = true;
  v.shape = {static_cast<long>(vec.size()), 3};
  v.data = vec.empty() ? nullptr : &vec[0][0];
  // boost::shared_ptr -> std::shared_ptr<void> keep-alive alias: the no-op
  // deleter captures a copy of `container`, holding the block for the view's
  // lifetime (its stored pointer is just an owner tag).
  v.owner = std::shared_ptr<void>(container.get(), [container](void *) { /* keep-alive */ });
  return v;
}

ArrayView Geometry::vertices() { return view_over(geom_->points_ptr()); }

ArrayView Geometry::vertex_colors() { return view_over(geom_->colors_ptr()); }

void Geometry::compute_normals() { geom_->compute_normals(); }
// Rebuild WITH the process ctx (not a null-ctx default geometry) so the
// facade's "geom_ always carries a valid ctx" invariant holds for
// quality_improve() after a clear().
void Geometry::clear() { *geom_ = cvc::geometry(ctx()); }

// geometry::read() preserves the existing _ctx across the read (unlike the
// free read_geometry(), which returns a null-ctx geometry), keeping the ctx
// invariant so quality_improve() works after load().
void Geometry::load(const std::string &filename) { geom_->read(filename); }
void Geometry::save(const std::string &filename) const { cvc::write_geometry(*geom_, filename); }

// ── Mesh processing ────────────────────────────────────────────────
void Geometry::smoothing(double delta, bool fix_boundary, bool geometric_flow) {
  // cvc::geometry::smoothing(app&, delta, fix_boundary, perturb_1,
  // geometric_flow, smoothing_enabled, perturb_2). We expose the three most
  // useful knobs and keep the perturbation passes off (their defaults).
  geom_->smoothing(ctx(), static_cast<float>(delta), fix_boundary, /*perturb_1=*/false,
                   geometric_flow, /*smoothing_enabled=*/true, /*perturb_2=*/false);
}

void Geometry::quality_improve(int iterations, int method) {
#ifdef CVC_ENABLE_MESHER
  geom_->quality_improve(iterations, static_cast<cvc::improvement_method>(method));
#else
  (void)iterations;
  (void)method;
  throw std::runtime_error("quality_improve: this libcvc build has the mesher disabled "
                           "(CVC_ENABLE_MESHER)");
#endif
}

cvc::geometry &Geometry::native() { return *geom_; }
const cvc::geometry &Geometry::native() const { return *geom_; }

} // namespace pycvc
