// pycvc_geometry.h — Python-facing facade over cvc::geometry.
//
// Deliberately SWIG-safe: it forward-declares cvc::geometry (a pimpl-style
// shared_ptr member) instead of including libcvc's heavy headers, so the
// SWIG parser only sees std::string / std::vector / primitive signatures.
// The C++ .cpp is the only translation unit that includes libcvc.
//
// This is the general-purpose geometry builder for the pycvc bindings; it
// knows nothing about any downstream project. Consumers (e.g. an embedded
// interpreter inside volrover3) reach the underlying cvc::geometry through
// native() in C++ — that accessor is %ignore'd on the Python side.
#pragma once

#include "pycvc_buffer.h"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace cvc {
class geometry;
}

namespace pycvc {

class Geometry {
public:
  Geometry();
  ~Geometry();

  // ── Incremental builders (convenient) ──────────────────────────────
  // Append a vertex; returns its index. Indices use size_t (== uint64_t on
  // LP64) so SWIG maps them to plain Python ints without stdint quirks.
  std::size_t add_vertex(double x, double y, double z);
  void add_triangle(std::size_t a, std::size_t b, std::size_t c);
  void add_line(std::size_t a, std::size_t b);

  // ── Bulk builders (fast path; flat row-major arrays) ───────────────
  // xyz length must be a multiple of 3; ijk of 3; ab of 2; rgb of 3 and
  // equal to 3 * num_vertices().
  void add_vertices(const std::vector<double> &xyz);
  void add_triangles(const std::vector<unsigned long> &ijk);
  void add_lines(const std::vector<unsigned long> &ab);
  void set_colors(const std::vector<double> &rgb);

  std::size_t num_vertices() const;
  std::size_t num_triangles() const;
  std::size_t num_lines() const;

  // Zero-copy numpy views (no data copy; numpy keeps the geometry alive via
  // a shared_ptr in the returned array's base). Writable: mutating the numpy
  // array edits the mesh in place.
  ArrayView vertices();      // (num_vertices, 3) float64
  ArrayView vertex_colors(); // (num_vertices, 3) float64; empty if unset

  void compute_normals();
  void clear();

  // ── File I/O (via cvc::geometry_file_io: .off/.raw/.rawc/…) ─────────
  void load(const std::string &filename);
  void save(const std::string &filename) const;

  // ── C++-only bridge for host apps (SWIG-ignored) ───────────────────
  cvc::geometry &native();
  const cvc::geometry &native() const;

private:
  std::shared_ptr<cvc::geometry> geom_;
};

} // namespace pycvc
