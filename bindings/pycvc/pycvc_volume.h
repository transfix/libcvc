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
  //
  // Works for BOTH host and GPU data: libcvc stores GPU voxels in CUDA
  // *unified* (managed) memory, so data_ptr() is host-accessible either way
  // — numpy views it directly and CUDA migrates pages on host access. (If a
  // GPU kernel is concurrently writing, the caller must synchronize first.)
  ArrayView grid();

  // ── Filters (denoise / enhance; mutate the grid IN PLACE) ───────────
  // Thin wrappers over cvc::voxels' filters (cvc::volume extends voxels).
  // Each mutates *this in place and returns nothing. On a CUDA-enabled
  // volume (enable_cuda()) the underlying C++ methods auto-dispatch to their
  // GPU kernels, so a filter "just works" on GPU-resident data — no special
  // casing here. NOTE: enable_cuda()/disable_cuda() reallocate the buffer, so
  // any prior grid() numpy view dangles after a filter that migrates residency
  // (filters themselves do not migrate; enable/disable do). Re-fetch grid()
  // after enable_cuda()/disable_cuda().
  //
  // Edge-preserving denoise: Gaussian weighting in both space and intensity.
  void bilateral_filter(double radiometric_sigma = 200.0, double spatial_sigma = 1.5,
                        unsigned int filter_radius = 2);
  // Zeyun's contrast enhancement; `resistor` in [0, 1] (clamped otherwise).
  void contrast_enhancement(double resistor = 0.95);
  // Zeyun's anisotropic diffusion: denoise while preserving edges.
  void anisotropic_diffusion(unsigned int iterations = 20);
  // Dr. Zhang's GDTV filter. `neighbours == 0` selects the 6-neighbour
  // stencil; nonzero selects the 26-neighbour stencil.
  void gdtv_filter(double q, double lambda, unsigned int iterations, unsigned int neighbours = 0);

  // GPU/CUDA support. on_gpu() is False on CUDA-disabled libcvc builds or
  // when the voxels are host-resident. When True, cuda_ptr() is the
  // device-accessible pointer to the same unified buffer grid() views —
  // exposed to cupy/torch via __cuda_array_interface__ (see pycvc.i) for
  // on-device zero-copy without host migration.
  bool on_gpu() const;
  unsigned long long cuda_ptr() const;

  // Move this volume's voxels into CUDA unified memory (enable_cuda) or back
  // to host (disable_cuda). After enable_cuda(), on_gpu() is True and the
  // same buffer is both host-accessible (grid()/numpy) and device-accessible
  // (cuda_ptr()/__cuda_array_interface__). No-op alternative to check first:
  // cuda_available() reports whether this build+machine can use the GPU.
  static bool cuda_available();
  void enable_cuda(int device = -1);
  void disable_cuda();
  bool using_cuda() const;

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
