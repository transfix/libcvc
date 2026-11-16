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

// cvc::segment_seeded: seeded segmentation of a volume by fast marching, ported from VolumeRover's
// GenSeg (transfix/volrover src/Segmentation/GenSeg) per
// docs/roadmap/VOLROVER-LIBRARY-PORTS-ROADMAP.md §5. It implements §2.2 of
//
//   C. Bajaj, Z. Yu, M. Auer, "Volumetric feature extraction and visualization of tomographic
//   molecular imaging", Journal of Structural Biology 144(1-2):132-143, 2003,
//   doi:10.1016/j.jsb.2003.09.037,
//
// which GenSeg's authors ask you to cite when you use it. The method:
//   1. the volume is rescaled to 0..255, and its gradient magnitude (a 3x3x3 operator) too;
//   2. automatic seeds (seed_mode::local_maxima): the local maxima above t_high of the volume
//      smoothed twice by a 7-point average;
//   3. each point class's points start a front of its class at time 0;
//   4. the automatic seeds are classified: a threshold sweeps down from 255 to t_low, and each seed
//      takes the class whose front reaches it first through voxels above the threshold;
//   5. every labelled voxel then starts a front, and the fronts advance by fast marching through
//      voxels at or above t_low, a step into a voxel costing 0.01 exp(-gamma |grad I|) (|grad I|
//      on 0..255), so fronts slow at edges. A front stops where it meets another class's front, or
//      a voxel below t_low.
//
// The result has none of the legacy library's process-wide state (its ~25 globals), reads and
// writes no file, and leaves nothing in the current directory: its labels and per-class volumes
// come back as volumes. Its labels match the legacy library's voxel for voxel
// (src/cvc/tests/segmentation_test.cpp, against tools/volrover-goldens).
//
// Resumable (§1.9): segment_stepper runs every phase a unit at a time; segment_seeded() is the
// stepper at budget 0, and segment_seeded_job() its result_job. Parameters: a param_table
// (cvc/volume/segmentation_params.h) gives the state keys, CLI flags, DSL dict and pycvc keywords.

#ifndef CVC_VOLUME_SEGMENTATION_H
#define CVC_VOLUME_SEGMENTATION_H

#include <array>
#include <cstddef>
#include <cvc/core/result_job.h>
#include <cvc/volume/point_classes.h>
#include <cvc/volume/volume.h>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
#ifndef SWIG
#include <boost/describe/enum.hpp>
#endif

namespace cvc {

// What is wrong with a segmentation's input: the message says what, for example
// "segment_seeded: seed (12, 40, 7) of class 'membrane' is outside the 64x64x32 volume".
class segmentation_error : public std::runtime_error {
public:
  using std::runtime_error::runtime_error;
};

// The automatic seeds added to the point classes' own.
enum class seed_mode {
  none,         // only the point classes' points
  local_maxima, // GenSeg's: the local maxima above t_high of the volume smoothed twice
};
#ifndef SWIG
BOOST_DESCRIBE_ENUM(seed_mode, none, local_maxima)
#endif

// The defaults are GenSeg's; VolumeGridRover's Segmentation tab had no defaults for the
// thresholds, which the user had to give.
struct seeded_segmentation_params {
  // Voxels below t_low stop every front. Automatic seeds are the smoothed volume's local maxima
  // above t_high. In the voxels' units, or with normalized_thresholds on the legacy 0..255 scale
  // the volume is rescaled to (its minimum 0, its maximum 255). Thresholds in data units are
  // rescaled the same way, so they are compared on that scale.
  double t_low = 0.0;
  double t_high = 0.0;
  bool normalized_thresholds = false;
  // A step into a voxel costs 0.01 exp(-gamma |grad I|), |grad I| rescaled to 0..255: the speed
  // F = exp(gamma |grad I|) of the paper, which slows fronts at edges for gamma < 0.
  double gamma = -0.12;
  seed_mode automatic = seed_mode::local_maxima;
  // The point classes of this variable and timestep seed the run, in their order: class n of
  // the result is the n-th of them (a class with no points labels nothing).
  int variable = 0;
  int timestep = 0;
  // Also make a volume per class (seeded_segmentation::classes).
  bool class_volumes = true;
};

struct seeded_segmentation {
  static constexpr unsigned char unlabelled = 255;

  explicit seeded_segmentation(app &ctx) : labels(ctx, dimension(1, 1, 1), UChar) {}

  // UChar, the input's dimensions and bounding box: each voxel's class (0, 1, ...), or
  // `unlabelled`.
  volume labels;
  // Per class, unless class_volumes is off: the input's voxels where the label is that class, 0
  // elsewhere, in the input's voxel type (the legacy <prefix>_subunit_<n>.rawiv, which held the
  // rescaled values instead).
  std::vector<volume> classes;
  // The voxels where a front stopped: next to a voxel of another class or below t_low (the
  // legacy <prefix>_indices.txt), in x-fastest order. Each is labelled with its own class.
  std::vector<std::array<int, 3>> border;
  // Per class: its name, and how many voxels it labels.
  std::vector<std::string> class_names;
  std::vector<std::size_t> class_voxels;
  // The automatic seeds found (but for those under a point class's seed), and how many of them
  // the threshold sweep gave a class (the rest were left to the final march, which may still
  // reach them).
  std::size_t automatic_seeds = 0;
  std::size_t classified_seeds = 0;
  // The thresholds as the run used them, on the 0..255 scale.
  double t_low_normalized = 0.0;
  double t_high_normalized = 0.0;
  std::vector<std::string> warnings;
};

// The segmentation of `vol` seeded by the classes of `params.variable` and `params.timestep` in
// `seeds`. Throws segmentation_error when no class has a point, a point lies outside the volume,
// there are more than 254 classes, the volume is constant or has a voxel that is not finite (NaN
// or infinite), its voxel type is undefined, or `params` is not valid (params_validate).
// Warnings, not errors: a seed below t_low (it labels only itself and its neighbours), a voxel
// that is a point of two classes (the later class has it), a class with no points.
seeded_segmentation segment_seeded(const volume &vol, const point_classes &seeds,
                                   const seeded_segmentation_params &params = {});

// segment_seeded, a slice at a time (§1.9). Phases: "checking" (the parameters and seeds, a unit
// per seed point), "normalising" (reading the voxels, the rescale and the gradient, a unit per
// voxel), "seeding" (the smoothing and the local-maximum scan, a unit per voxel), "classifying"
// (the threshold sweep: a unit per voxel visited, popped from a heap or stepped through),
// "marching" (the final fast marching, a unit per voxel popped) and "writing" (the labels, the
// border and the class volumes, a unit per voxel). Every unit costs about a microsecond at most,
// whatever the volume's size, but for allocating each output volume (filling its bytes with
// zeros). A failure is a segmentation_error thrown from step() (and rethrown by later steps and
// take()). cancel() is the one thread-safe member: the step running (or the next) ends the
// stepper with no result; a result_job around it ends "cancelled". Not thread-safe otherwise,
// but it may move between threads between steps. Constructing one only stores its input (the
// volume shares its voxels with the caller's copy, which must not change while the stepper
// runs).
class segment_stepper {
public:
  segment_stepper(const volume &vol, point_classes seeds, seeded_segmentation_params params = {});
  ~segment_stepper();
  segment_stepper(segment_stepper &&) noexcept;
  segment_stepper &operator=(segment_stepper &&) noexcept;

  bool step(std::size_t budget); // true once done; budget 0 runs to the end
  bool done() const;
  const char *phase() const; // as listed above, "done", or "failed"
  double progress() const;   // in [0, 1], monotone; 1 once done
  std::string status() const;
  void cancel();
  seeded_segmentation take(); // once done; moved out, so call it once

private:
  struct impl;
  std::unique_ptr<impl> _impl;
};

std::unique_ptr<segment_stepper> segment_seeded_stepper(const volume &vol, point_classes seeds,
                                                        seeded_segmentation_params params = {});
std::unique_ptr<result_job<seeded_segmentation>>
segment_seeded_job(const volume &vol, point_classes seeds, seeded_segmentation_params params = {});

} // namespace cvc

#endif // CVC_VOLUME_SEGMENTATION_H
