// pycvc_volume.h — Python-facing facade over cvc::volume (a scalar field
// sampled on a regular grid, with a spatial bounding box).
//
// This is the RF / signed-distance / risk-field path: build a 3D scalar
// grid in Python (e.g. from numpy), hand it to a host as a cvc::volume for
// GPU ray-cast rendering. SWIG-safe like pycvc_geometry.h (forward-declares
// cvc::volume; only the .cpp includes libcvc).
#pragma once

#include "pycvc_buffer.h"

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

namespace cvc {
class volume;
}

namespace pycvc {

class Volume {
public:
  Volume();
  ~Volume();

  // Build a Float volume from a flat, row-major (x fastest, then y, then z)
  // scalar grid of nx*ny*nz values, over the object-space box
  // [minx,maxx] x [miny,maxy] x [minz,maxz].
  void set_float_grid(const std::vector<double> &values, unsigned long nx, unsigned long ny,
                      unsigned long nz, double minx, double miny, double minz, double maxx,
                      double maxy, double maxz);

  unsigned long xdim() const;
  unsigned long ydim() const;
  unsigned long zdim() const;

  // Voxel value at grid index (i, j, k).
  double value(unsigned long i, unsigned long j, unsigned long k) const;
  double min_value() const;
  double max_value() const;

  double xmin() const;
  double xmax() const;
  double ymin() const;
  double ymax() const;
  double zmin() const;
  double zmax() const;

  // Zero-copy numpy view of the voxel grid (no data copy; numpy keeps the
  // volume alive via a shared_ptr in the array's base). Shape (nz, ny, nx),
  // float32, writable. Requires a Float volume (as built by set_float_grid).
  ArrayView grid();

  // File I/O (.rawiv and friends, via cvc::volume_file_io).
  void load(const std::string &filename);
  void save(const std::string &filename) const;

  // C++-only bridge for host apps (SWIG-ignored).
  cvc::volume &native();
  const cvc::volume &native() const;

private:
  std::shared_ptr<cvc::volume> vol_;
};

} // namespace pycvc
