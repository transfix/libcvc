/*
  Copyright 2026 The University of Texas at Austin

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

// cvc::class_surfaces: the surface of each class of a label volume (a segmentation's labels,
// cvc::seeded_segmentation::labels), for the 3D view of VolumeGridRover's Segmentation tab
// (docs/roadmap/VOLROVER-LIBRARY-PORTS-ROADMAP.md §4, P9: "each class's surface shows in the 3D
// view, as an isosurface of its mask").
//
// Each class's surface is the isosurface at 1/2 of its mask (1 on its voxels, 0 elsewhere), by
// marching cubes over every cell of the voxel grid at once: a cell whose eight corners hold several
// classes adds a piece to each, so the classes are surfaced in one pass whatever their number (252
// on a capsid's capsomers). Vertices sit at the midpoints of the grid edges the masks change
// across, shared between the triangles of a class (an indexed mesh), so neighbouring classes meet
// on the same vertices. With `close`, the volume is taken as surrounded by no class, so every
// surface is closed (a class touching the volume's faces is capped half a voxel past them, where
// the slice planes' quads end). Taubin's lambda|mu smoothing (`smoothing` passes of each) then
// takes away the voxel steps without shrinking the classes (every vertex kept within that box half
// a voxel past the outer voxels), and each vertex gets its normal (the
// area-weighted mean of its triangles' normals). Triangles wind counter-clockwise seen from
// outside the class, so the normals point out of it.
//
//   cvc::class_surface_params p;     // param_table: state keys, CLI flags, DSL dict, pycvc
//   p.smoothing = 10;
//   cvc::class_surface_result r = cvc::class_surfaces(seg.labels, p);
//   for (std::size_t n = 0; n < r.labels.size(); ++n)
//     show(r.surfaces[n], colour_of(r.labels[n]));
//
// Positions are in the volume's object space (voxel i along x at minx + i * spacing, as
// cvc::volume and cvc/volume/volume_slice.h place them), so the surfaces sit where the volume's
// slices are drawn. Resumable (§1.9): class_surfaces_stepper runs every phase a unit at a time;
// class_surfaces() is it at budget 0, and class_surfaces_job() its result_job. No statics, no
// globals (§1.3); deterministic: the same labels and parameters give the same meshes, bit for bit.

#ifndef CVC_VOLUME_CLASS_SURFACES_H
#define CVC_VOLUME_CLASS_SURFACES_H

#include <cstddef>
#include <cvc/core/param_table.h>
#include <cvc/core/result_job.h>
#include <cvc/geometry/geometry.h>
#include <cvc/volume/volume.h>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace cvc {

// What is wrong with class_surfaces' input: the message says what, for example
// "class_surfaces: voxel (3, 4, 5) holds 2.5, which is not a label".
class class_surfaces_error : public std::runtime_error {
public:
  using std::runtime_error::runtime_error;
};

struct class_surface_params {
  // Passes of Taubin's smoothing, each a lambda step (shrinking) and a mu step (growing back);
  // 0 leaves the voxel steps.
  int smoothing = 10;
  double lambda = 0.5;
  double mu = -0.53;
  // The volume taken as surrounded by no class, so every surface closes (else a class is open where
  // it meets the volume's faces).
  bool close = true;
  // The label of no class (seeded_segmentation::unlabelled), which gets no surface.
  int background = 255;
  // Only these labels (empty: every label in the volume but the background).
  std::vector<int> labels;
};

#ifndef SWIG
template <> struct param_table<class_surface_params> : param_fields<class_surface_params> {
  using P = class_surface_params;
  static constexpr double none = std::numeric_limits<double>::quiet_NaN();
  static constexpr auto fields = std::tuple{
      field("smoothing", &P::smoothing, range(0, 1000),
            doc("passes of Taubin's smoothing (a lambda and a mu step each): the voxel steps "
                "go, the classes keep their size; 0 keeps the steps")),
      field("lambda", &P::lambda, range(0.0, 1.0),
            doc("Taubin's lambda: each vertex moves this far towards its neighbours' mean")),
      field("mu", &P::mu, range(-1.0, 0.0),
            doc("Taubin's mu (below -lambda): the step back out that keeps the volume")),
      field("close", &P::close,
            doc("cap each class where it meets the volume's faces, so every surface is closed")),
      field("background", &P::background,
            doc("the label of no class, which gets no surface (a segmentation's 255)")),
      field("labels", &P::labels,
            doc("only these labels get a surface (none: every label but the background)")),
  };
  static void check(const P &p, std::vector<std::string> &problems) {
    if (p.smoothing > 0 && !(p.mu < -p.lambda))
      problems.push_back("mu: must be below -lambda (Taubin), or smoothing 0");
  }
};
#endif // SWIG

struct class_surface_result {
  // The labels that have a surface, ascending, and each one's: triangles (outward, as above), its
  // vertices' positions and normals. A label is listed when it has a voxel in the volume (and is
  // one of params.labels, when that names any) and its mask changes somewhere (without close, a
  // label filling the whole volume has no surface, and a warning says so).
  std::vector<int> labels;
  std::vector<geometry> surfaces;
  // Per label: how many voxels it has.
  std::vector<std::size_t> voxels;
  std::vector<std::string> warnings;

  // The surface of `label`, or null when it has none.
  const geometry *surface_of(int label) const;
  std::size_t triangle_count() const;
};

// The surfaces of the classes of `labels`, a volume whose voxels are integer labels (any voxel
// type; a segmentation's labels are UChar). Throws class_surfaces_error for an undefined voxel type,
// a voxel that is not an integer (NaN, infinite, 2.5) or is out of int's range, and parameters that
// are not valid (params_validate). Warnings, not errors: a label of params.labels the volume does
// not hold, a label with no surface.
class_surface_result class_surfaces(const volume &labels, const class_surface_params &params = {});

// class_surfaces, a slice at a time (§1.9). Phases: "checking" (the parameters), "reading" (each
// voxel's label, a unit per voxel), "extracting" (marching cubes, a unit per cell, and one more per
// class a cell adds triangles to), "smoothing" (each class's neighbours, a unit per triangle, then
// each pass, a unit per vertex) and "normals" (a unit per triangle, then per vertex). Every unit
// costs about a microsecond at most, whatever the volume's size. A failure is a
// class_surfaces_error thrown from step() (and rethrown by later steps and take()). cancel() is the
// one thread-safe member: the step running (or the next) ends the stepper with no result; a
// result_job around it ends "cancelled". Not thread-safe otherwise, but it may move between
// threads between steps. The stepper holds a copy-on-write handle on the volume, which the caller
// must not change while it runs.
class class_surfaces_stepper {
public:
  class_surfaces_stepper(const volume &labels, class_surface_params params = {});
  ~class_surfaces_stepper();
  class_surfaces_stepper(class_surfaces_stepper &&) noexcept;
  class_surfaces_stepper &operator=(class_surfaces_stepper &&) noexcept;

  bool step(std::size_t budget); // true once done; budget 0 runs to the end
  bool done() const;
  const char *phase() const; // as listed above, "done", or "failed"
  double progress() const;   // in [0, 1], monotone; 1 once done
  std::string status() const;
  void cancel();
  class_surface_result take(); // once done; moved out, so call it once

private:
  struct impl;
  std::unique_ptr<impl> _impl;
};

std::unique_ptr<result_job<class_surface_result>>
class_surfaces_job(const volume &labels, class_surface_params params = {});

} // namespace cvc

#endif // CVC_VOLUME_CLASS_SURFACES_H
