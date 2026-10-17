/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::height_field — terrain as a regular grid of heights, and as the
// triangle mesh of that grid, in one object.
//
// The world's one representation (docs/roadmap/WORLD-GEOMETRY-ROADMAP.md): the
// renderer, the physics layer, the navigator and the loaders share these
// objects instead of keeping their own copies. A height_field IS a
// cvc::geometry, so everything that takes a geometry (a GeometryNode, the
// codecs, LOD, picking) takes it unchanged.
//
//   cvc::height_field t(rows, cols, x0, y0, dx, dy, heights);  // metres, z up
//   t.set_cell_materials(ids);       // surface ids, one per cell
//   double z = t.height(x, y);       // the same surface the mesh draws
//   sg.addGraphics("terrain", t);    // a plain geometry to the renderer
//
// Frame: SI metres, z up, world coordinates. Sample (r, c) sits at
// (x0 + c*dx, y0 + r*dy): columns run along +x, rows along +y, so row 0 is the
// minimum y. Samples are corners, not cell centres: a grid of rows x cols samples
// has (rows-1) x (cols-1) cells and spans (cols-1)*dx by (rows-1)*dy. A loader
// whose source runs north-first flips it on the way in; no consumer flips.
//
// The grid is authoritative. The geometry arrays hold its mesh:
//   - one vertex per sample that is not a hole (NaN), with an upward normal from
//     the grid's slope and a texture coordinate over the extent;
//   - two triangles per cell, split along the diagonal from (r, c) to
//     (r+1, c+1), wound counter-clockwise seen from above; a cell with a hole
//     at any corner has none;
//   - each triangle carries its cell's surface material (face_materials()).
// height(x, y) is that mesh's height (the plane of the triangle under the
// point, not a bilinear blend), so the renderer, physics and nav agree.
//
// Edits go through the grid. set_heights() and set_cell_materials() rebuild the
// mesh; set_height() and set_cell_material() update it in place (a sample that
// becomes or stops being a hole rebuilds it). Writing the mesh arrays directly
// (points(), tris(), face_materials(), ...) throws cvc::read_only_error, since
// the grid would silently go stale; per-vertex colours and other per-vertex
// attributes stay writable, and a rebuild keeps them while the vertex count is
// unchanged. Copy a height_field into a plain cvc::geometry for an ordinary,
// editable mesh: the arrays are shared copy-on-write.
//
// Reading the mesh: geometry's non-const accessors (points(), tris(), ...) are
// writes (they detach copy-on-write), so on a height_field they throw too. Read
// through a const reference or the const_ accessors (const_points(), ...).
//
// Large grids: the mesh costs about 70 bytes per sample. A DEM too large to mesh
// whole belongs in LOD tiles, built from this grid.

#ifndef __CVC_GEOMETRY_HEIGHT_FIELD_H__
#define __CVC_GEOMETRY_HEIGHT_FIELD_H__

#include <boost/shared_ptr.hpp>
#include <cstdint>
#include <cvc/geometry/geometry.h>
#include <vector>

namespace cvc {

class height_field : public geometry {
public:
  // An empty height field (no samples, no mesh).
  height_field();
  // rows x cols samples (each at least 2), sample (r, c) at (x0 + c*dx, y0 + r*dy).
  // `heights` is row-major, row 0 first, rows*cols long (empty: all zero);
  // `cell_materials` is (rows-1)*(cols-1) surface ids or empty (all 0). Throws
  // cvc::geometry_error for a bad size or a spacing that is not finite and > 0.
  height_field(int rows, int cols, double x0, double y0, double dx, double dy,
               std::vector<float> heights = {}, std::vector<face_material_t> cell_materials = {});
  // The same grid given by its extent (corner samples on the bounds).
  static height_field from_bounds(int rows, int cols, double min_x, double min_y, double max_x,
                                  double max_y, std::vector<float> heights = {});

  height_field(const height_field &other);
  height_field &operator=(const height_field &other);
  // A height_field is not assigned from a plain geometry: that would replace the
  // mesh and leave the grid behind. Copy the other way round instead.
  height_field &operator=(const geometry &) = delete;
  // Reading a file into a height_field would replace its mesh the same way: read
  // into a plain geometry, or build the grid with a loader.
  height_field &read(const std::string &) = delete;
  ~height_field() override;

  // --- the grid ---------------------------------------------------------------

  int rows() const { return _rows; }
  int cols() const { return _cols; }
  int cell_rows() const { return _rows > 0 ? _rows - 1 : 0; }
  int cell_cols() const { return _cols > 0 ? _cols - 1 : 0; }
  double x0() const { return _x0; }
  double y0() const { return _y0; }
  double dx() const { return _dx; }
  double dy() const { return _dy; }
  double max_x() const { return _x0 + _dx * cell_cols(); }
  double max_y() const { return _y0 + _dy * cell_rows(); }

  // Row-major heights, row 0 first; NaN marks a hole.
  const std::vector<float> &heights() const { return *_heights; }
  float height_at(int r, int c) const;
  // One surface id per cell, row-major; empty means every cell is material 0.
  const std::vector<face_material_t> &cell_materials() const { return *_cell_materials; }
  face_material_t cell_material(int r, int c) const;

  // --- edits ------------------------------------------------------------------

  void set_heights(std::vector<float> heights);              // rebuilds the mesh
  void set_cell_materials(std::vector<face_material_t> ids); // rebuilds the mesh
  void set_height(int r, int c, float h);                    // in place
  void set_cell_material(int r, int c, face_material_t id);  // in place
  // Regenerate the mesh from the grid (the setters above call it as needed).
  void rebuild();
  // Empty the grid and the mesh. Hides geometry::clear(), which would empty
  // only the mesh.
  height_field &clear();

  // --- queries ----------------------------------------------------------------

  // The mesh's height at (x, y): the plane of the triangle under the point.
  // NaN outside the grid or over a cell with a hole.
  double height(double x, double y) const;
  // That triangle's upward unit normal; (NaN, NaN, NaN) where height is NaN.
  vector_t normal(double x, double y) const;
  // The cell under (x, y); false outside the grid. A point on the last row or
  // column of samples belongs to the last cell.
  bool cell_of(double x, double y, int &r, int &c) const;
  // The surface id of the cell under (x, y); 0 outside the grid.
  face_material_t material(double x, double y) const;

  // The mesh vertex of sample (r, c), or -1 for a hole.
  std::int64_t vertex_of(int r, int c) const;
  // The first of cell (r, c)'s two triangles, or -1 for a cell with no mesh.
  std::int64_t first_face_of(int r, int c) const;

protected:
  void pre_write(ARRAY_TYPE at) override;

private:
  void check(int r, int c, const char *what) const;
  void rebuild_vertex_normal(int r, int c);
  vector_t grid_normal(int r, int c) const;

  int _rows = 0, _cols = 0;
  double _x0 = 0.0, _y0 = 0.0, _dx = 1.0, _dy = 1.0;
  boost::shared_ptr<std::vector<float>> _heights;
  boost::shared_ptr<std::vector<face_material_t>> _cell_materials;
  // Derived with the mesh: sample -> vertex, cell -> first face (0xffffffff: none).
  boost::shared_ptr<std::vector<std::uint32_t>> _vertex;
  boost::shared_ptr<std::vector<std::uint32_t>> _first_face;
  bool _writing = false; // the height field itself is writing its mesh
};

} // namespace cvc

#endif // __CVC_GEOMETRY_HEIGHT_FIELD_H__
