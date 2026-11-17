/*
  Copyright 2003, 2026 The University of Texas at Austin

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

// cvc::gradient_vector_diffusion: anisotropic gradient vector diffusion, §2.1 (Eqs. 2-3) of
//
//   C. Bajaj, Z. Yu, M. Auer, "Volumetric feature extraction and visualization of tomographic
//   molecular imaging", Journal of Structural Biology 144(1-2):132-143, 2003,
//   doi:10.1016/j.jsb.2003.09.037,
//
// as VolumeRover's SegMed implements it (transfix/volrover src/Segmentation/SegMed/GVF.cpp,
// gvfflow(); SecStruct and SegMonomer carry copies). Each iteration moves every vector towards its
// six face neighbours:
//
//   u' = u + sum over the six neighbours n of  g_n (u_n - u),
//   g_n = exp(k (c . s_n / (|c| |s_n|) - 1))  (Eq. 3; 0 when s_n is zero),
//
// the six g_n divided by their sum, where c and s_n are the voxel's and the neighbour's vectors of
// the weighing field: the field being diffused (the paper, SecStruct), or the field before the
// first iteration (SegMed). A voxel whose weighing vector is zero takes its six neighbours' mean.
// Past the volume's edge a neighbour is the voxel itself. Vectors along one direction weigh each
// other most, so the field smooths along features and keeps its edges. The paper's data term
// (Eq. 2's second term) is left out, as SegMed and SecStruct leave it out.
//
// critical_point_at() classifies a voxel of a field by its 26 neighbours: a maximum where every
// neighbour's vector points towards it (a sink: the gradient of a volume sinks at the volume's
// maxima), a minimum where every one points away (a source). GenSeg's paper mode
// (seed_mode::critical_points, cvc/volume/segmentation.h) seeds its fronts from them; SecStruct,
// SegMonomer and the paper's §2.3 skeletons use the same field.
//
// Resumable (docs/roadmap/VOLROVER-LIBRARY-PORTS-ROADMAP.md §1.9):
// gradient_vector_diffusion_stepper updates a voxel per unit; gradient_vector_diffusion() is the
// stepper at budget 0, and gradient_vector_diffusion_job() its result_job. Arithmetic is in float,
// as the legacy code's, with exp() in double, so a field is the same bit for bit on every platform.

#ifndef CVC_VOLUME_GRADIENT_VECTOR_DIFFUSION_H
#define CVC_VOLUME_GRADIENT_VECTOR_DIFFUSION_H

#include <cstddef>
#include <cstdint>
#include <cvc/core/result_job.h>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
#ifndef SWIG
#include <boost/describe/enum.hpp>
#endif

namespace cvc {

// What is wrong with a diffusion's input: the message says what.
class gradient_vector_diffusion_error : public std::runtime_error {
public:
  using std::runtime_error::runtime_error;
};

// A vector per voxel of an nx x ny x nz grid, x fastest: component c of voxel (i, j, k) is
// c[(k * ny + j) * nx + i].
struct vector_field {
  int nx = 0, ny = 0, nz = 0;
  std::vector<float> x, y, z;

  vector_field() = default;
  vector_field(int nx_, int ny_, int nz_)
      : nx(nx_), ny(ny_), nz(nz_), x(count(nx_, ny_, nz_)), y(x.size()), z(x.size()) {}
  std::size_t size() const { return count(nx, ny, nz); }
  std::size_t index(int i, int j, int k) const {
    return (std::size_t(k) * std::size_t(ny) + std::size_t(j)) * std::size_t(nx) + std::size_t(i);
  }

private:
  static std::size_t count(int a, int b, int c) {
    return a > 0 && b > 0 && c > 0 ? std::size_t(a) * std::size_t(b) * std::size_t(c) : 0;
  }
};

// The field whose angles weigh the neighbours.
enum class diffusion_weights {
  current, // the field being diffused, so the weights follow it (the paper, SecStruct)
  initial, // the field before the first iteration, every iteration (SegMed)
};
#ifndef SWIG
BOOST_DESCRIBE_ENUM(diffusion_weights, current, initial)
#endif

// The defaults are SegMed's (gvfflow: 10 iterations, Kapa = 2). A param_table
// (cvc/volume/gradient_vector_diffusion_params.h) gives the state keys and validate().
struct gradient_vector_diffusion_params {
  int iterations = 10;
  double k = 2.0; // Eq. 3's k: how much the angle between two vectors lowers their weight
  diffusion_weights weights = diffusion_weights::current;
  // Divide the result by its longest vector's length, so the longest has length 1 (SegMed). A
  // field of zero vectors stays zero.
  bool normalize = true;
};

// `initial` diffused by `params`. Throws gradient_vector_diffusion_error when a component's size is
// not the grid's, a component is not finite (NaN or infinite), or `params` is not valid
// (params_validate).
vector_field gradient_vector_diffusion(vector_field initial,
                                       const gradient_vector_diffusion_params &params = {});

// gradient_vector_diffusion, a slice at a time (§1.9). Phases: "checking" (a unit per voxel),
// "diffusing" (a unit per voxel per iteration, each about a microsecond) and "normalising" (two
// units per voxel). A failure is a gradient_vector_diffusion_error thrown from step() and rethrown
// by later steps and take(). cancel() is the one thread-safe member: the step running (or the
// next) ends the stepper with no result. Not thread-safe otherwise, but it may move between threads
// between steps.
class gradient_vector_diffusion_stepper {
public:
  explicit gradient_vector_diffusion_stepper(vector_field initial,
                                             gradient_vector_diffusion_params params = {});
  ~gradient_vector_diffusion_stepper();
  gradient_vector_diffusion_stepper(gradient_vector_diffusion_stepper &&) noexcept;
  gradient_vector_diffusion_stepper &operator=(gradient_vector_diffusion_stepper &&) noexcept;

  bool step(std::size_t budget); // true once done; budget 0 runs to the end
  bool done() const;
  const char *phase() const; // as listed above, "done", or "failed"
  double progress() const;   // in [0, 1], monotone; 1 once done
  std::string status() const;
  void cancel();
  std::size_t units() const; // the units done so far, for a caller running it inside its own
  vector_field take();       // once done; moved out, so call it once

private:
  struct impl;
  std::unique_ptr<impl> _impl;
};

std::unique_ptr<result_job<vector_field>>
gradient_vector_diffusion_job(vector_field initial, gradient_vector_diffusion_params params = {});

// The kind of a critical point.
enum class critical_kind : std::uint8_t {
  none = 0,    // not a maximum or a minimum (a saddle, or no critical point)
  minimum = 1, // a source: no neighbour's vector points towards the voxel
  maximum = 2, // a sink: no neighbour's vector points away from it
};
#ifndef SWIG
BOOST_DESCRIBE_ENUM(critical_kind, none, minimum, maximum)
#endif

// The kind of voxel (i, j, k) of `f`, by SegMed's test (GVF.cpp): for each of its neighbours n in
// the volume (up to 26), d_n = (n - (i, j, k)) . f(n), which is negative where f(n) points towards
// the voxel. A minimum has no d_n < 0, a maximum no d_n > 0 (and some d_n < 0); a voxel whose own
// vector is zero and every d_n zero is neither (a flat region). (i, j, k) must be in the grid.
critical_kind critical_point_at(const vector_field &f, int i, int j, int k);

} // namespace cvc

#endif // CVC_VOLUME_GRADIENT_VECTOR_DIFFUSION_H
